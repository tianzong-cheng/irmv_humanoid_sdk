#!/usr/bin/env python3
"""HTTP, bridge and layout acceptance of the local experiment web service.

The records are minimal, valid runs written by the test itself: a runtime run
with an empty committed chunk index and an application run with a JSONL tail.
The bridge then reads them through its public protocol, so the service is tested
against the same bytes a real recorder leaves behind.
"""

from __future__ import annotations

import argparse
import http.client
import json
import os
import select
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from http.cookiejar import CookieJar
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE_ROOT))

from tools.experiment_web.bridge import BridgePool  # noqa: E402

READER: Path | None = None


def runtime_run(root: Path, name: str, boot_id: str = "boot-one") -> Path:
    directory = root / name
    directory.mkdir()
    metadata = {
        "schema_version": 4,
        "config_id": "a" * 64,
        "layout_id": "b" * 64,
        "backend": "fake",
        "control_hz": 500.0,
        "sample_hz": 100,
        "joint_count": 2,
        "joints": ["left_hip", "right_hip"],
        "sdk_git_sha": "server-test-sha",
        "clock": {
            "clock": "linux_clock_monotonic",
            "boot_id": boot_id,
            "available": True,
        },
        "units": {"position": "rad", "velocity": "rad_s", "torque": "Nm"},
        "imu": {"orientation_rpy": {"axis_frame": "unknown"}},
    }
    index = {
        "schema_version": 4,
        "complete": True,
        "chunks": [],
        "gaps": [],
        "faults": {"complete": True, "losses": []},
    }
    (directory / "meta.json").write_text(json.dumps(metadata), encoding="utf-8")
    (directory / "index.json").write_text(json.dumps(index), encoding="utf-8")
    (directory / "sessions.json").write_text(
        json.dumps(
            [
                {
                    "schema_version": 4,
                    "runtime_id": "11" * 16,
                    "session_id": "22" * 16,
                    "producer_id": "33" * 16,
                    "epoch": 1,
                    "stamp_us": 1000,
                    "state_seq": 10,
                }
            ]
        ),
        encoding="utf-8",
    )
    (directory / "faults.jsonl").write_text("", encoding="utf-8")
    return directory


def application_run(
    root: Path, name: str, boot_id: str = "boot-one", with_stream: bool = True
) -> Path:
    directory = root / name
    directory.mkdir()
    metadata = {
        "schema": "irmv.application_velocity",
        "version": 1,
        "sdk_git_sha": "server-test-sha",
        "application_config_id": "c" * 64,
        "config_id": "a" * 64,
        "layout_id": "b" * 64,
        "joint_count": 2,
        "sample_hz": 100,
        "source": "local",
        "clock": {
            "clock": "linux_clock_monotonic",
            "boot_id": boot_id,
            "available": True,
        },
        "units": {"forward": "m_s", "lateral": "m_s", "yaw": "rad_s"},
        "model": {"status": "loaded", "name": "policy.onnx"},
    }
    index = {
        "schema": "irmv.application_velocity",
        "version": 1,
        "complete": True,
        "losses": [],
    }
    (directory / "meta.json").write_text(json.dumps(metadata), encoding="utf-8")
    (directory / "index.json").write_text(json.dumps(index), encoding="utf-8")
    if with_stream:
        lines = []
        for seq, (stamp, forward) in enumerate(((1000, 0.1), (2000, 0.2))):
            lines.append(
                json.dumps(
                    {
                        "schema": "irmv.application_velocity",
                        "version": 1,
                        "host_us": stamp,
                        "stamp_us": stamp,
                        "sample_seq": f"{seq:016x}",
                        "identity_known": True,
                        "runtime_id": "11" * 16,
                        "session_id": "22" * 16,
                        "latest_state_seq": f"{seq + 10:016x}",
                        "forward": forward,
                        "lateral": -0.2,
                        "yaw": 0.3,
                    }
                )
            )
        (directory / "velocity.jsonl").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
    return directory


def layout(name: str = "compare") -> dict:
    return {
        "version": 1,
        "name": name,
        "dock": {"type": "leaf", "chartId": "c1"},
        "charts": [
            {
                "id": "c1",
                "title": "q",
                "curves": [{"signalId": "runtime/left_hip/q", "axisId": "a1"}],
                "axes": [{"id": "a1", "quantity": "joint_position", "unit": "rad"}],
            }
        ],
    }


def start_server(
    reader: Path, records_root: Path, data_dir: Path, assets: Path
) -> tuple[subprocess.Popen, str]:
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(SOURCE_ROOT)
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "tools.experiment_web",
            "--records-root",
            str(records_root),
            "--data-dir",
            str(data_dir),
            "--reader",
            str(reader),
            "--assets",
            str(assets),
            "--port",
            "0",
        ],
        cwd=str(SOURCE_ROOT),
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    assert process.stdout is not None
    ready, _, _ = select.select([process.stdout], [], [], 10.0)
    if not ready:
        process.kill()
        raise AssertionError("the web service did not print its URL")
    line = process.stdout.readline().strip()
    if not line.startswith("http://127.0.0.1:"):
        process.kill()
        raise AssertionError(f"unexpected service output: {line!r}")
    return process, line


def stop_server(process: subprocess.Popen) -> None:
    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)
    for stream in (process.stdout, process.stderr):
        if stream is not None:
            stream.close()


def opener() -> urllib.request.OpenerDirector:
    return urllib.request.build_opener(urllib.request.HTTPCookieProcessor(CookieJar()))


def _decode(payload: bytes) -> object:
    try:
        return json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return payload.decode("utf-8", errors="replace")


def call(
    http: urllib.request.OpenerDirector,
    method: str,
    url: str,
    body: dict | None = None,
    headers: dict | None = None,
) -> tuple[int, object]:
    data = None
    actual_headers = dict(headers or {})
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        actual_headers.setdefault("Content-Type", "application/json")
    request = urllib.request.Request(
        url, data=data, method=method, headers=actual_headers
    )
    try:
        with http.open(request, timeout=10.0) as response:
            return response.status, _decode(response.read())
    except urllib.error.HTTPError as error:
        return error.code, _decode(error.read())


class ServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        assert READER is not None
        cls.temp = tempfile.TemporaryDirectory()
        cls.base = Path(cls.temp.name)
        cls.records = cls.base / "records"
        cls.records.mkdir()
        cls.data = cls.base / "data"
        cls.assets = cls.base / "assets"
        cls.assets.mkdir()
        (cls.assets / "index.html").write_text(
            "<html>workbench</html>", encoding="utf-8"
        )
        (cls.assets / "styles.css").write_text("body{}", encoding="utf-8")
        cls.runtime = runtime_run(cls.records, "run_0001")
        cls.application = application_run(cls.records, "app_run_0001")
        cls.missing_application = application_run(
            cls.records, "app_run_0002", with_stream=False
        )
        cls.process, cls.url = start_server(READER, cls.records, cls.data, cls.assets)
        cls.http = opener()
        status, _ = call(cls.http, "GET", cls.url)
        assert status == 200

    @classmethod
    def tearDownClass(cls) -> None:
        stop_server(cls.process)
        cls.temp.cleanup()

    def catalog(self) -> dict:
        status, body = call(self.http, "GET", f"{self.url}api/runs")
        self.assertEqual(status, 200)
        self.assertTrue(body["ok"])
        return body

    def test_static_and_bootstrap_cookie(self) -> None:
        fresh = opener()
        status, body = call(fresh, "GET", self.url)
        self.assertEqual(status, 200)
        status, body = call(fresh, "GET", f"{self.url}api/runs")
        self.assertEqual(status, 200)

    def test_idle_browser_connections_do_not_block_new_page_resources(self) -> None:
        connections = []
        try:
            for _ in range(8):
                connection = http.client.HTTPConnection(
                    self.url.removeprefix("http://").rstrip("/"), timeout=3
                )
                connections.append(connection)
                connection.request("GET", "/")
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                response.read()
            status, _ = call(opener(), "GET", f"{self.url}styles.css")
            self.assertEqual(status, 200)
        finally:
            for connection in connections:
                connection.close()

    def test_api_without_cookie_is_unauthorized(self) -> None:
        fresh = opener()
        status, body = call(fresh, "GET", f"{self.url}api/runs")
        self.assertEqual(status, 401)
        self.assertFalse(body["ok"])

    def test_catalog_and_pagination(self) -> None:
        catalog = self.catalog()
        names = {run["name"] for run in catalog["runs"]}
        self.assertIn("run_0001", names)
        self.assertIn("app_run_0001", names)
        status, first = call(self.http, "GET", f"{self.url}api/runs?limit=1")
        self.assertEqual(status, 200)
        self.assertEqual(len(first["runs"]), 1)
        self.assertIn("continuation", first)
        status, second = call(
            self.http,
            "GET",
            f"{self.url}api/runs?limit=1&continuation={first['continuation']}",
        )
        self.assertEqual(status, 200)
        self.assertNotEqual(first["runs"][0]["run_id"], second["runs"][0]["run_id"])

    def test_describe_range_cursor_and_matching_app(self) -> None:
        catalog = self.catalog()
        runtime = next(run for run in catalog["runs"] if run["name"] == "run_0001")
        application = next(
            run for run in catalog["runs"] if run["name"] == "app_run_0001"
        )
        status, described = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{runtime['run_id']}/describe",
        )
        self.assertEqual(status, 200)
        self.assertTrue(described["ok"])
        self.assertEqual(described["context"]["kind"], "runtime")
        status, range_reply = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{runtime['run_id']}/range"
            f"?signal_ids=runtime%2Fleft_hip%2Fq&begin_us=0&end_us=5000&pixel_width=10",
        )
        self.assertEqual(status, 200)
        self.assertTrue(range_reply["ok"])
        self.assertEqual(range_reply["series"][0]["points"], [])
        status, app_range = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{runtime['run_id']}/range"
            f"?app_run_id={application['run_id']}"
            "&signal_ids=application%2F%2Fforward"
            "&begin_us=0&end_us=3000&pixel_width=10",
        )
        self.assertEqual(status, 200)
        self.assertEqual(len(app_range["series"][0]["points"]), 2)
        status, cursor = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{runtime['run_id']}/cursor"
            f"?app_run_id={application['run_id']}"
            "&signal_ids=application%2F%2Fforward&cursor_us=2500",
        )
        self.assertEqual(status, 200)
        self.assertEqual(cursor["series"][0]["value"], 0.2)

    def test_missing_application_stream_is_an_error(self) -> None:
        catalog = self.catalog()
        missing = next(run for run in catalog["runs"] if run["name"] == "app_run_0002")
        status, described = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{missing['run_id']}/describe",
        )
        self.assertEqual(status, 200)
        self.assertFalse(described["ok"])
        self.assertIn("cannot open", described["error"])

    def test_refuses_a_non_loopback_host_header(self) -> None:
        host = urllib.parse.urlsplit(self.url).netloc
        status, body = call(
            self.http,
            "GET",
            f"{self.url}api/runs",
            headers={"Host": f"evil.example:{host.split(':')[-1]}"},
        )
        self.assertEqual(status, 403)
        self.assertFalse(body["ok"])

    def test_malformed_range_time_is_a_400_not_a_widened_query(self) -> None:
        catalog = self.catalog()
        runtime = next(run for run in catalog["runs"] if run["name"] == "run_0001")
        status, body = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{runtime['run_id']}/range"
            "?signal_ids=runtime%2Fleft_hip%2Fq&begin_us=abc&end_us=5000",
        )
        self.assertEqual(status, 400)
        self.assertFalse(body["ok"])
        self.assertIn("begin_us", body["error"])

    def test_refuses_path_shaped_ids(self) -> None:
        status, body = call(self.http, "GET", f"{self.url}api/runs/2e2e/describe")
        self.assertEqual(status, 400)
        self.assertFalse(body["ok"])

    def test_layout_round_trip_and_validation(self) -> None:
        origin = {"Origin": self.url.rstrip("/")}
        status, created = call(
            self.http,
            "POST",
            f"{self.url}api/layouts",
            body=layout(),
            headers=origin,
        )
        self.assertEqual(status, 201)
        layout_id = created["id"]
        status, loaded = call(self.http, "GET", f"{self.url}api/layouts/{layout_id}")
        self.assertEqual(status, 200)
        self.assertEqual(loaded["layout"]["dock"], layout()["dock"])
        self.assertEqual(loaded["layout"]["version"], 1)
        status, listed = call(self.http, "GET", f"{self.url}api/layouts")
        self.assertEqual(status, 200)
        self.assertEqual(listed["layouts"][0]["id"], layout_id)
        updated = layout("renamed")
        status, _ = call(
            self.http,
            "PUT",
            f"{self.url}api/layouts/{layout_id}",
            body=updated,
            headers=origin,
        )
        self.assertEqual(status, 200)
        status, loaded = call(self.http, "GET", f"{self.url}api/layouts/{layout_id}")
        self.assertEqual(loaded["layout"]["name"], "renamed")
        status, _ = call(
            self.http, "DELETE", f"{self.url}api/layouts/{layout_id}", headers=origin
        )
        self.assertEqual(status, 200)
        status, _ = call(self.http, "GET", f"{self.url}api/layouts/{layout_id}")
        self.assertEqual(status, 404)

    def test_layout_write_requires_origin_and_json(self) -> None:
        status, body = call(self.http, "POST", f"{self.url}api/layouts", body=layout())
        self.assertEqual(status, 403)
        self.assertFalse(body["ok"])
        status, body = call(
            self.http,
            "POST",
            f"{self.url}api/layouts",
            body=layout(),
            headers={"Origin": self.url.rstrip("/"), "Content-Type": "text/plain"},
        )
        self.assertEqual(status, 415)
        status, body = call(
            self.http,
            "PUT",
            f"{self.url}api/layouts/not-an-id",
            body=layout(),
            headers={"Origin": self.url.rstrip("/")},
        )
        self.assertEqual(status, 400)

    def test_invalid_layout_is_refused(self) -> None:
        document = layout()
        document["dock"] = {
            "type": "split",
            "direction": "row",
            "ratio": 0.99,
            "first": {"type": "empty"},
            "second": {"type": "empty"},
        }
        status, body = call(
            self.http,
            "POST",
            f"{self.url}api/layouts",
            body=document,
            headers={"Origin": self.url.rstrip("/")},
        )
        self.assertEqual(status, 400)
        self.assertFalse(body["ok"])

    def test_events_missing_fault_file_is_an_error(self) -> None:
        catalog = self.catalog()
        runtime = next(run for run in catalog["runs"] if run["name"] == "run_0001")
        (self.runtime / "faults.jsonl").unlink()
        try:
            status, body = call(
                self.http,
                "GET",
                f"{self.url}api/runs/{runtime['run_id']}/events?begin_us=0",
            )
            self.assertEqual(status, 200)
            self.assertFalse(body["ok"])
            self.assertIn("cannot find", body["error"])
        finally:
            (self.runtime / "faults.jsonl").write_text("", encoding="utf-8")


class BridgeFailureTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.records = self.base / "records"
        self.records.mkdir()
        self.assets = self.base / "assets"
        self.assets.mkdir()
        (self.assets / "index.html").write_text("ok", encoding="utf-8")
        self.data = self.base / "data"

    def tearDown(self) -> None:
        self.temp.cleanup()

    def fake_reader(self, body: str) -> Path:
        script = self.base / "fake_reader.py"
        script.write_text(body, encoding="utf-8")
        script.chmod(0o755)
        return script

    def test_invalid_json_from_reader_carries_the_request_id(self) -> None:
        reader = self.fake_reader(
            "#!/usr/bin/env python3\n"
            "import sys\n"
            "for line in sys.stdin:\n"
            "    print('this is not json', flush=True)\n"
        )
        process, url = start_server(reader, self.records, self.data, self.assets)
        try:
            http = opener()
            call(http, "GET", url)
            status, body = call(http, "GET", f"{url}api/runs")
            self.assertEqual(status, 502)
            self.assertFalse(body["ok"])
            self.assertTrue(body["request_id"])
            self.assertIn("invalid JSON", body["error"])
        finally:
            stop_server(process)

    def test_timeout_replaces_the_helper(self) -> None:
        reader = self.fake_reader(
            "#!/usr/bin/env python3\nimport sys, time\ntime.sleep(30)\n"
        )
        pool = BridgePool(reader, self.records, timeout=0.2, size=1)
        started = time.monotonic()
        reply = pool.query({"op": "describe"})
        elapsed = time.monotonic() - started
        pool.close()
        self.assertFalse(reply["ok"])
        self.assertTrue(reply.get("transport_error"))
        self.assertTrue(reply["request_id"])
        self.assertLess(elapsed, 3.0)


class LiveServerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.records = self.base / "records"
        self.records.mkdir()
        self.data = self.base / "data"
        self.assets = self.base / "assets"
        self.assets.mkdir()
        (self.assets / "index.html").write_text("ok", encoding="utf-8")
        self.runtime = runtime_run(self.records, "run_live")
        self.process, self.url = start_server(
            READER, self.records, self.data, self.assets
        )
        self.http = opener()
        call(self.http, "GET", self.url)
        status, catalog = call(self.http, "GET", f"{self.url}api/runs")
        self.run_id = next(
            run["run_id"] for run in catalog["runs"] if run["name"] == "run_live"
        )

    def tearDown(self) -> None:
        stop_server(self.process)
        self.temp.cleanup()

    def send(self, kind: str, transport_seq: int, payload: dict) -> None:
        envelope = {
            "version": 1,
            "emitter_id": "e" * 32,
            "run_path": str(self.runtime),
            "source": "runtime",
            "transport_seq": str(transport_seq),
            "kind": kind,
            "payload": payload,
        }
        client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            client.sendto(
                json.dumps(envelope).encode("utf-8"), str(self.data / "live.sock")
            )
        finally:
            client.close()

    def sample(self, stamp: int, seq: int, q0: float) -> dict:
        return {
            "kind": "sample",
            "stamp_us": str(stamp),
            "state_seq": str(seq),
            "command_present": True,
            "feedback_valid": True,
            "joint_count": 2,
            "q": [q0, 0.0],
            "joint_valid": [True, True],
        }

    def test_snapshot_range_events_and_revision(self) -> None:
        self.send("hello", 1, {})
        self.send("sample", 2, self.sample(1000, 1, 0.25))
        self.send("sample", 3, self.sample(3000, 3, 0.75))
        time.sleep(0.1)
        status, snapshot = call(
            self.http,
            "GET",
            f"{self.url}api/live/snapshot?run_id={self.run_id}"
            "&signal_ids=runtime%2Fleft_hip%2Fq&begin_us=0&end_us=5000",
        )
        self.assertEqual(status, 200)
        self.assertTrue(snapshot["ok"])
        self.assertEqual(len(snapshot["series"][0]["points"]), 2)
        status, catalog = call(self.http, "GET", f"{self.url}api/runs")
        live_run = next(run for run in catalog["runs"] if run["run_id"] == self.run_id)
        self.assertEqual(status, 200)
        self.assertTrue(live_run["live"]["active"])
        self.assertGreaterEqual(live_run["live"]["age_ms"], 0)
        revision = snapshot["revision"]
        status, unchanged = call(
            self.http,
            "GET",
            f"{self.url}api/live/snapshot?run_id={self.run_id}"
            "&signal_ids=runtime%2Fleft_hip%2Fq&begin_us=0&end_us=5000"
            f"&after_revision={revision}",
        )
        self.assertEqual(status, 200)
        self.assertFalse(unchanged["changed"])

        status, live_range = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{self.run_id}/range"
            "?signal_ids=runtime%2Fleft_hip%2Fq&begin_us=0&end_us=5000"
            "&pixel_width=10",
        )
        self.assertEqual(status, 200)
        self.assertTrue(live_range["live"]["active"])
        self.assertEqual(len(live_range["series"][0]["points"]), 2)

        self.send(
            "fault",
            4,
            {
                "record_seq": "9",
                "detected_stamp_us": "1200",
                "state_seq": "10",
                "fault_id": "3",
                "joint_index": 0,
                "reason": 5,
                "condition": 8,
            },
        )
        time.sleep(0.1)
        status, events = call(
            self.http,
            "GET",
            f"{self.url}api/runs/{self.run_id}/events?begin_us=0",
        )
        self.assertEqual(status, 200)
        self.assertEqual(len(events["events"]), 1)
        self.assertEqual(events["events"][0]["record_seq"], "9")
        self.assertTrue(events["live"]["active"])

    def test_service_restart_does_not_join_old_tail(self) -> None:
        self.send("hello", 1, {})
        self.send("sample", 2, self.sample(1000, 1, 0.5))
        time.sleep(0.1)
        stop_server(self.process)
        self.process, self.url = start_server(
            READER, self.records, self.data, self.assets
        )
        self.http = opener()
        call(self.http, "GET", self.url)
        status, snapshot = call(
            self.http,
            "GET",
            f"{self.url}api/live/snapshot?run_id={self.run_id}"
            "&signal_ids=runtime%2Fleft_hip%2Fq&begin_us=0&end_us=5000",
        )
        self.assertEqual(status, 404)
        self.assertFalse(snapshot["ok"])


def main() -> int:
    global READER
    parser = argparse.ArgumentParser()
    parser.add_argument("--reader", required=True)
    args, remaining = parser.parse_known_args()
    READER = Path(args.reader).resolve()
    unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
