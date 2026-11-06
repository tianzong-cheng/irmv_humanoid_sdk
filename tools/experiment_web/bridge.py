"""Bounded pool of read-only query bridge processes.

Each bridge is a separate ``web_record_query`` process speaking one JSON object
per line. A bridge handles one request at a time, and the pool holds at most
four of them, so a slow query cannot starve every other view. A bridge that
times out, exits, or answers with invalid JSON is killed and replaced; the
error goes back to its caller with the request id, and no SDK process is ever
signalled by this module.
"""

from __future__ import annotations

import json
import queue
import select
import subprocess
import threading
import uuid
from pathlib import Path
from typing import Any


class BridgeError(RuntimeError):
    pass


class _Bridge:
    def __init__(self, reader: Path, records_root: Path) -> None:
        self._reader = str(reader)
        self._records_root = str(records_root)
        self._process: subprocess.Popen[str] | None = None

    def _start(self) -> subprocess.Popen[str]:
        if self._process is not None and self._process.poll() is None:
            return self._process
        self._process = subprocess.Popen(
            [self._reader, "--records-root", self._records_root],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        return self._process

    def request(self, payload: dict, timeout: float) -> str:
        process = self._start()
        if process.stdin is None or process.stdout is None:
            raise BridgeError("the reader has no stdin/stdout")
        try:
            process.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
            process.stdin.flush()
        except (BrokenPipeError, OSError) as error:
            raise BridgeError(f"the reader is not accepting requests: {error}") from error
        ready, _, _ = select.select([process.stdout], [], [], timeout)
        if not ready:
            raise BridgeError("the reader did not answer within the deadline")
        line = process.stdout.readline()
        if not line:
            raise BridgeError("the reader exited without an answer")
        return line

    def kill(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        try:
            process.kill()
        except OSError:
            pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass
        for stream in (process.stdin, process.stdout):
            try:
                if stream is not None:
                    stream.close()
            except OSError:
                pass


class BridgePool:
    def __init__(
        self,
        reader: Path | str,
        records_root: Path | str,
        timeout: float = 5.0,
        size: int = 4,
    ) -> None:
        if size < 1:
            raise ValueError("the bridge pool size must be positive")
        self._reader = Path(reader)
        self._records_root = Path(records_root)
        self._timeout = timeout
        self._size = size
        self._available: queue.Queue[_Bridge] = queue.Queue()
        self._lock = threading.Lock()
        self._closed = False
        for _ in range(size):
            self._available.put(_Bridge(self._reader, self._records_root))

    def query(self, request: dict[str, Any]) -> dict[str, Any]:
        request = dict(request)
        request_id = request.get("request_id")
        if not isinstance(request_id, str) or not request_id:
            request_id = uuid.uuid4().hex
            request["request_id"] = request_id
        if self._closed:
            return {"request_id": request_id, "ok": False, "transport_error": True, "error": "the service is closing"}
        try:
            bridge = self._available.get(timeout=self._timeout)
        except queue.Empty:
            return {
                "request_id": request_id,
                "ok": False,
                "transport_error": True,
                "error": "all read-only bridges are busy",
            }
        discard = False
        try:
            line = bridge.request(request, self._timeout)
            try:
                reply = json.loads(line)
            except json.JSONDecodeError as error:
                discard = True
                return {
                    "request_id": request_id,
                    "ok": False,
                    "transport_error": True,
                    "error": f"the reader answered invalid JSON: {error}",
                }
            if not isinstance(reply, dict) or reply.get("request_id") != request_id:
                discard = True
                return {
                    "request_id": request_id,
                    "ok": False,
                    "transport_error": True,
                    "error": "the reader answered another request",
                }
            return reply
        except BridgeError as error:
            discard = True
            return {"request_id": request_id, "ok": False, "transport_error": True, "error": str(error)}
        finally:
            if discard:
                bridge.kill()
                bridge = _Bridge(self._reader, self._records_root)
            if not self._closed:
                self._available.put(bridge)

    def close(self) -> None:
        self._closed = True
        while True:
            try:
                bridge = self._available.get_nowait()
            except queue.Empty:
                break
            bridge.kill()


__all__ = ["BridgeError", "BridgePool"]
