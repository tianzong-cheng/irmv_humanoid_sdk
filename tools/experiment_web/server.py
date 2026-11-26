"""The local HTTP service of the experiment workbench.

The service binds loopback only, serves the static workbench, and forwards data
questions to the bounded bridge pool. It never starts, stops or signals an SDK
process, never exposes a path from a browser, and never offers a robot command
endpoint. Layout writes are atomically persisted; data reads are read-only.
"""

from __future__ import annotations

import json
import mimetypes
import secrets
import threading
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from .bridge import BridgePool
from .catalog import CatalogError, paginate, valid_run_id
from .layout_store import LayoutError, LayoutStore
from .live_store import LiveError, LiveReceiver, LiveStore

_MAX_BODY_BYTES = 1024 * 1024
_MAX_ACTIVE_REQUESTS = 8
_MAX_TOKENS = 64
_COOKIE_NAME = "irmv_web_token"


class ExperimentWebServer:
    def __init__(
        self,
        records_root: Path,
        data_dir: Path,
        reader: Path,
        assets: Path,
        port: int = 0,
        telemetry_socket: Path | None = None,
    ) -> None:
        self.records_root = Path(records_root)
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.reader = Path(reader)
        self.assets = Path(assets)
        self.pool = BridgePool(self.reader, self.records_root)
        self.layouts = LayoutStore(self.data_dir)
        self.live = LiveStore(self.records_root)
        self.live_socket_path = (
            Path(telemetry_socket)
            if telemetry_socket is not None
            else self.data_dir / "live.sock"
        )
        self.live_receiver = LiveReceiver(self.live, self.live_socket_path)
        self.live_receiver.start()
        self._slots = threading.BoundedSemaphore(_MAX_ACTIVE_REQUESTS)
        self._tokens: deque[str] = deque()
        self._token_set: set[str] = set()
        self._token_lock = threading.Lock()
        self._http = ThreadingHTTPServer(("127.0.0.1", port), _Handler)
        self._http.daemon_threads = True
        self._http.app = self  # type: ignore[attr-defined]

    @property
    def port(self) -> int:
        return self._http.server_address[1]

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}/"

    def enter(self, timeout: float = 0.2) -> bool:
        return self._slots.acquire(timeout=timeout)

    def leave(self) -> None:
        self._slots.release()

    def issue_token(self) -> str:
        token = secrets.token_hex(16)
        with self._token_lock:
            self._tokens.append(token)
            self._token_set.add(token)
            while len(self._tokens) > _MAX_TOKENS:
                expired = self._tokens.popleft()
                self._token_set.discard(expired)
        return token

    def valid_token(self, token: str | None) -> bool:
        if not token:
            return False
        with self._token_lock:
            return token in self._token_set

    def serve_forever(self) -> None:
        try:
            self._http.serve_forever(poll_interval=0.2)
        finally:
            self.live_receiver.close()
            self.pool.close()
            self._http.server_close()

    def shutdown(self) -> None:
        self._http.shutdown()


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict) -> None:
    body = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode(
        "utf-8"
    )
    try:
        handler.send_response(status)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()
        handler.wfile.write(body)
    except (BrokenPipeError, ConnectionResetError):
        # The browser aborted a superseded query. The request is gone, not the
        # service; there is nobody to receive an error.
        pass


def _error(
    handler: BaseHTTPRequestHandler,
    status: int,
    message: str,
    request_id: str | None = None,
) -> None:
    payload = {"ok": False, "error": message}
    if request_id is not None:
        payload["request_id"] = request_id
    _json_response(handler, status, payload)


def _query_params(handler: BaseHTTPRequestHandler) -> dict[str, list[str]]:
    return parse_qs(urlsplit(handler.path).query, keep_blank_values=True)


def _one(params: dict[str, list[str]], key: str) -> str | None:
    values = params.get(key)
    return values[0] if values else None


def _limit(params: dict[str, list[str]]) -> int:
    raw = _one(params, "limit")
    if raw is None:
        return 100
    if not raw.isdigit():
        raise CatalogError("limit is not a decimal integer")
    value = int(raw)
    if value < 1:
        raise CatalogError("limit must be positive")
    return min(value, 100)


def _u64_param(params: dict[str, list[str]], key: str, default: int) -> int:
    return _int_param(params, key, default, 0, 2**64 - 1)


def _int_param(
    params: dict[str, list[str]], key: str, default: int, minimum: int, maximum: int
) -> int:
    raw = _one(params, key)
    if raw is None:
        return default
    if not raw.isdigit():
        raise CatalogError(f"{key} is not a decimal integer")
    value = int(raw)
    if not minimum <= value <= maximum:
        raise CatalogError(f"{key} must be {minimum}..{maximum}")
    return value


def _signal_ids(params: dict[str, list[str]]) -> list[str]:
    values = params.get("signal_ids", [])
    result: list[str] = []
    for value in values:
        if value.startswith("["):
            try:
                loaded = json.loads(value)
            except json.JSONDecodeError as error:
                raise CatalogError(f"signal_ids is not valid JSON: {error}") from error
            if not isinstance(loaded, list) or not all(
                isinstance(item, str) for item in loaded
            ):
                raise CatalogError("signal_ids is not an array of strings")
            result.extend(loaded)
        else:
            result.extend(item for item in value.split(",") if item)
    if not result:
        raise CatalogError("signal_ids is required")
    if len(result) > 128:
        raise CatalogError("signal_ids holds more than 128 signals")
    return result


class _Handler(BaseHTTPRequestHandler):
    server_version = "irmv-experiment-web/1"
    # Admission is connection-scoped: close after each response so idle browser
    # keep-alives cannot hold every request slot and prevent module/API loading.
    protocol_version = "HTTP/1.0"

    @property
    def app(self) -> ExperimentWebServer:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, format: str, *args: object) -> None:
        return

    def handle(self) -> None:
        if not self.app.enter():
            _error(self, HTTPStatus.SERVICE_UNAVAILABLE, "the service is busy")
            return
        try:
            super().handle()
        finally:
            self.app.leave()

    # -- authentication -----------------------------------------------------

    def _cookie_token(self) -> str | None:
        header = self.headers.get("Cookie", "")
        for part in header.split(";"):
            name, _, value = part.strip().partition("=")
            if name == _COOKIE_NAME:
                return value
        return None

    def _send_token(self) -> None:
        token = self.app.issue_token()
        self.send_header(
            "Set-Cookie",
            f"{_COOKIE_NAME}={token}; HttpOnly; SameSite=Strict; Path=/",
        )

    def _authorized(self) -> bool:
        return self.app.valid_token(self._cookie_token())

    def _host_ok(self) -> bool:
        host = self.headers.get("Host", "")
        port = self.server.app.port
        return host in {f"127.0.0.1:{port}", f"localhost:{port}"}

    def _origin_ok(self) -> bool:
        if not self._host_ok():
            return False
        origin = self.headers.get("Origin")
        host = self.headers.get("Host")
        return bool(origin and host) and origin in {
            f"http://{host}",
            f"https://{host}",
        }

    # -- request helpers ----------------------------------------------------

    def _body(self) -> dict:
        length = self.headers.get("Content-Length")
        if length is None:
            raise CatalogError("a request body is required")
        if not length.isdigit():
            raise CatalogError("Content-Length is not a decimal integer")
        size = int(length)
        if size > _MAX_BODY_BYTES:
            raise CatalogError("the request body is larger than 1 MiB")
        raw = self.rfile.read(size)
        try:
            document = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise CatalogError(
                f"the request body is not valid JSON: {error}"
            ) from error
        if not isinstance(document, dict):
            raise CatalogError("the request body is not a JSON object")
        return document

    def _bridge(self, request: dict) -> dict | None:
        reply = self.app.pool.query(request)
        if reply.get("transport_error"):
            _error(
                self,
                HTTPStatus.BAD_GATEWAY,
                str(reply.get("error", "bridge failure")),
                str(reply.get("request_id", "")),
            )
            return None
        return reply

    def _require_auth(self) -> bool:
        if self._authorized():
            return True
        _error(self, HTTPStatus.UNAUTHORIZED, "the bootstrap cookie is missing")
        return False

    # -- static -------------------------------------------------------------

    def _serve_static(self) -> bool:
        path = unquote(urlsplit(self.path).path)
        if path == "/":
            path = "/index.html"
        if "\x00" in path or ".." in Path(path).parts:
            _error(self, HTTPStatus.BAD_REQUEST, "the path is refused")
            return True
        root = self.app.assets.resolve()
        candidate = (root / path.lstrip("/")).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            _error(self, HTTPStatus.BAD_REQUEST, "the path leaves the assets root")
            return True
        if not candidate.is_file():
            _error(self, HTTPStatus.NOT_FOUND, "the resource does not exist")
            return True
        content = candidate.read_bytes()
        content_type = (
            mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        )
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            if not self._authorized():
                self._send_token()
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            pass
        return True

    # -- routes -------------------------------------------------------------

    def do_GET(self) -> None:
        if not self._host_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the host is refused")
            return
        parsed = urlsplit(self.path)
        if not parsed.path.startswith("/api/"):
            self._serve_static()
            return
        if not self._require_auth():
            return
        try:
            self._api_get(parsed.path, _query_params(self))
        except CatalogError as error:
            _error(self, HTTPStatus.BAD_REQUEST, str(error))
        except (LayoutError, OSError) as error:
            _error(self, HTTPStatus.BAD_REQUEST, str(error))

    def do_POST(self) -> None:
        if not self._host_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the host is refused")
            return
        if not self._require_auth():
            return
        if not self._origin_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the origin is refused")
            return
        content_type = self.headers.get("Content-Type", "")
        if not content_type.startswith("application/json"):
            _error(self, HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "a JSON body is required")
            return
        parsed = urlsplit(self.path)
        try:
            if parsed.path == "/api/layouts":
                document = self._body()
                layout_id = self.app.layouts.save(document)
                _json_response(self, HTTPStatus.CREATED, {"ok": True, "id": layout_id})
                return
            _error(self, HTTPStatus.NOT_FOUND, "the endpoint does not exist")
        except (CatalogError, LayoutError) as error:
            _error(self, HTTPStatus.BAD_REQUEST, str(error))
        except OSError as error:
            _error(self, HTTPStatus.INTERNAL_SERVER_ERROR, str(error))

    def do_PUT(self) -> None:
        if not self._host_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the host is refused")
            return
        if not self._authorized():
            _error(self, HTTPStatus.UNAUTHORIZED, "the bootstrap cookie is missing")
            return
        if not self._origin_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the origin is refused")
            return
        content_type = self.headers.get("Content-Type", "")
        if not content_type.startswith("application/json"):
            _error(self, HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "a JSON body is required")
            return
        parsed = urlsplit(self.path)
        parts = [part for part in parsed.path.split("/") if part]
        if len(parts) != 3 or parts[:2] != ["api", "layouts"]:
            _error(self, HTTPStatus.NOT_FOUND, "the endpoint does not exist")
            return
        try:
            document = self._body()
            layout_id = self.app.layouts.save(document, parts[2])
            _json_response(self, HTTPStatus.OK, {"ok": True, "id": layout_id})
        except (CatalogError, LayoutError) as error:
            _error(self, HTTPStatus.BAD_REQUEST, str(error))
        except OSError as error:
            _error(self, HTTPStatus.INTERNAL_SERVER_ERROR, str(error))

    def do_DELETE(self) -> None:
        if not self._host_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the host is refused")
            return
        if not self._authorized():
            _error(self, HTTPStatus.UNAUTHORIZED, "the bootstrap cookie is missing")
            return
        if not self._origin_ok():
            _error(self, HTTPStatus.FORBIDDEN, "the origin is refused")
            return
        parsed = urlsplit(self.path)
        parts = [part for part in parsed.path.split("/") if part]
        if len(parts) != 3 or parts[:2] != ["api", "layouts"]:
            _error(self, HTTPStatus.NOT_FOUND, "the endpoint does not exist")
            return
        try:
            if self.app.layouts.delete(parts[2]):
                _json_response(self, HTTPStatus.OK, {"ok": True})
            else:
                _error(self, HTTPStatus.NOT_FOUND, "the layout does not exist")
        except LayoutError as error:
            _error(self, HTTPStatus.BAD_REQUEST, str(error))

    def _api_get(self, path: str, params: dict[str, list[str]]) -> None:
        parts = [part for part in path.split("/") if part]
        if parts == ["api", "runs"]:
            reply = self._bridge({"op": "describe"})
            if reply is None:
                return
            if not reply.get("ok"):
                _json_response(self, HTTPStatus.OK, reply)
                return
            runs = reply.get("runs", [])
            page, continuation = paginate(
                runs, _one(params, "continuation"), _limit(params)
            )
            live = self.app.live.CatalogStatus()
            for run in page:
                run["live"] = live.get(run["run_id"], {"active": False, "age_ms": None})
            reply["runs"] = page
            reply.pop("continuation", None)
            if continuation is not None:
                reply["continuation"] = continuation
            _json_response(self, HTTPStatus.OK, reply)
            return
        if parts == ["api", "live", "snapshot"]:
            run_id = _one(params, "run_id")
            if run_id is None or not valid_run_id(run_id):
                raise CatalogError("the run id is not 32 lower-case hex digits")
            signal_ids = _signal_ids(params)
            begin_us = _int_param(params, "begin_us", 0, 0, 2**64 - 1)
            end_us = _int_param(params, "end_us", 2**64 - 1, 1, 2**64 - 1)
            pixel_width = _int_param(params, "pixel_width", 800, 1, 4096)
            after_revision = _one(params, "after_revision")
            revision = None
            if after_revision is not None:
                if not after_revision.isdigit():
                    raise CatalogError("after_revision is not a decimal integer")
                revision = int(after_revision)
            try:
                snapshot = self.app.live.Snapshot(
                    run_id, signal_ids, begin_us, end_us, pixel_width, revision
                )
            except LiveError as error:
                _error(self, HTTPStatus.NOT_FOUND, str(error))
                return
            _json_response(self, HTTPStatus.OK, snapshot)
            return
        if parts == ["api", "layouts"]:
            page, continuation = self.app.layouts.list_page(
                _one(params, "continuation"), _limit(params)
            )
            payload = {"ok": True, "layouts": page}
            if continuation is not None:
                payload["continuation"] = continuation
            _json_response(self, HTTPStatus.OK, payload)
            return
        if len(parts) == 3 and parts[:2] == ["api", "layouts"]:
            document = self.app.layouts.get(parts[2])
            if document is None:
                _error(self, HTTPStatus.NOT_FOUND, "the layout does not exist")
                return
            _json_response(self, HTTPStatus.OK, {"ok": True, "layout": document})
            return
        if len(parts) == 4 and parts[:2] == ["api", "runs"]:
            run_id, operation = parts[2], parts[3]
            if not valid_run_id(run_id):
                _error(
                    self,
                    HTTPStatus.BAD_REQUEST,
                    "the run id is not 32 lower-case hex digits",
                )
                return
            self._run_query(run_id, operation, params)
            return
        _error(self, HTTPStatus.NOT_FOUND, "the endpoint does not exist")

    def _run_query(
        self, run_id: str, operation: str, params: dict[str, list[str]]
    ) -> None:
        request: dict = {"run_id": run_id}
        app_run_id = _one(params, "app_run_id")
        if app_run_id is not None:
            if not valid_run_id(app_run_id):
                _error(
                    self,
                    HTTPStatus.BAD_REQUEST,
                    "the application run id is not 32 lower-case hex digits",
                )
                return
            request["app_run_id"] = app_run_id
        if operation in {"describe", "context"}:
            evidence = _one(params, "simulation_evidence")
            if evidence is not None:
                if evidence not in {"0", "1"}:
                    raise CatalogError("simulation_evidence must be 0 or 1")
                request["simulation_evidence"] = evidence == "1"
        if operation == "describe":
            request["op"] = "describe"
        elif operation == "context":
            request["op"] = "context"
        elif operation == "range":
            request["op"] = "range"
            request["signal_ids"] = _signal_ids(params)
            request["begin_us"] = str(_u64_param(params, "begin_us", 0))
            request["end_us"] = str(_u64_param(params, "end_us", 2**64 - 1))
            request["pixel_width"] = _int_param(params, "pixel_width", 800, 1, 4096)
        elif operation == "cursor":
            request["op"] = "cursor"
            request["signal_ids"] = _signal_ids(params)
            if _one(params, "cursor_us") is None:
                raise CatalogError("cursor_us is required")
            request["cursor_us"] = str(_u64_param(params, "cursor_us", 0))
        elif operation == "events":
            request["op"] = "events"
            request["begin_us"] = str(_u64_param(params, "begin_us", 0))
            request["end_us"] = str(_u64_param(params, "end_us", 2**64 - 1))
            request["max_rows"] = _int_param(params, "max_rows", 50, 1, 100)
            continuation = _one(params, "continuation")
            if continuation is not None:
                request["continuation"] = continuation
        else:
            _error(self, HTTPStatus.NOT_FOUND, "unknown run operation")
            return
        reply = self._bridge(request)
        if reply is None:
            return
        if reply.get("ok"):
            run_id = request["run_id"]
            if operation == "range":
                self.app.live.merge_range(
                    reply,
                    run_id,
                    request.get("signal_ids", []),
                    int(request.get("begin_us", "0")),
                    int(request.get("end_us", str(2**64 - 1))),
                    int(request.get("pixel_width", 800)),
                )
            elif operation == "events":
                self.app.live.merge_events(reply, run_id)
        _json_response(self, HTTPStatus.OK, reply)


__all__ = ["ExperimentWebServer"]
