#!/usr/bin/env python3
"""Two-process experiment recording and the local web service.

The test starts the real Fake runtime and application on an isolated DDS domain,
lets both write their own run directory, and only then starts the web service.
It also restarts the service, restarts the runtime, starts a recorder-disabled
runtime, and inspects a corrupted run and the two different fault-stream
absences. Every process has a deadline and a cleanup path; no service is ever
signalled by an SDK process and no SDK process is started by the service.

Usage: experiment_web_test.py <source-root> <robot_runtime> <application> <reader>
"""

from __future__ import annotations

import json
import os
import pty
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.parse
import urllib.request
from http.cookiejar import CookieJar
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2]
DOMAIN = 155
DOMAIN_CONFIG = 9


def copy_config(source: Path, target: Path, records: Path, logs: Path,
                enabled: bool) -> Path:
    config = json.loads((source / "config" / "sdk.json").read_text(encoding="utf-8"))
    config["telemetry"]["directory"] = str(logs)
    config["telemetry"]["recorder"] = {
        "enabled": enabled,
        "sample_hz": 100,
        "queue_capacity": 256,
        "directory": str(records) if enabled else None,
    }
    config["dds"]["domain"] = DOMAIN_CONFIG
    config["dds"]["interface"] = "lo"
    target.write_text(json.dumps(config, indent=2), encoding="utf-8")
    return target


def wait_for(predicate, timeout: float, step: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


def stop_process(process: subprocess.Popen, timeout: float = 8.0) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def run_processes(config: Path, runtime: Path, application: Path) -> tuple[int, int]:
    runtime_log = config.parent / "runtime.log"
    application_log = config.parent / "application.log"
    with runtime_log.open("wb") as runtime_out, application_log.open("wb") as app_out:
        runtime_process = subprocess.Popen(
            [str(runtime), "--config", str(config), "--domain", str(DOMAIN)],
            stdin=subprocess.DEVNULL, stdout=runtime_out, stderr=subprocess.STDOUT,
            cwd=str(SOURCE),
        )
        try:
            time.sleep(1.0)
            master, slave = pty.openpty()
            application_process = subprocess.Popen(
                [str(application), "--config", str(config), "--domain", str(DOMAIN)],
                stdin=slave, stdout=app_out, stderr=subprocess.STDOUT,
                cwd=str(SOURCE),
            )
            os.close(slave)
            try:
                time.sleep(3.0)
            finally:
                stop_process(application_process)
                os.close(master)
        finally:
            stop_process(runtime_process)
    return runtime_process.returncode or 0, application_process.returncode or 0


class Service:
    def __init__(self, records: Path, data: Path, reader: Path) -> None:
        self.process = subprocess.Popen(
            [
                sys.executable, "-m", "tools.experiment_web",
                "--records-root", str(records),
                "--data-dir", str(data),
                "--reader", str(reader),
                "--assets", str(SOURCE / "web"),
                "--port", "0",
            ],
            cwd=str(SOURCE), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True,
        )
        assert self.process.stdout is not None
        line = self.process.stdout.readline().strip()
        if not line.startswith("http://127.0.0.1:"):
            self.stop()
            raise AssertionError(f"the service did not start: {line!r}")
        self.url = line.rstrip("/")
        self.opener = urllib.request.build_opener(
            urllib.request.HTTPCookieProcessor(CookieJar())
        )
        with self.opener.open(self.url + "/", timeout=10.0) as response:
            response.read()

    def call(self, method: str, path: str) -> tuple[int, dict]:
        request = urllib.request.Request(self.url + path, method=method)
        try:
            with self.opener.open(request, timeout=10.0) as response:
                return response.status, json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read().decode("utf-8"))

    def stop(self) -> None:
        stop_process(self.process)
        for stream in (self.process.stdout, self.process.stderr):
            if stream is not None:
                stream.close()


class ExperimentWebTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.temp = tempfile.TemporaryDirectory()
        cls.base = Path(cls.temp.name)
        cls.records = cls.base / "records"
        cls.records.mkdir()
        cls.logs = cls.base / "logs"
        cls.data = cls.base / "data"
        cls.config = copy_config(SOURCE, cls.base / "sdk.json", cls.records,
                                 cls.logs, True)
        run_processes(cls.config, RUNTIME, APPLICATION)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.temp.cleanup()

    def runtime_dirs(self) -> list[Path]:
        return sorted(path for path in self.records.iterdir()
                      if path.is_dir() and path.name.startswith("run_"))

    def application_dirs(self) -> list[Path]:
        return sorted(path for path in self.records.iterdir()
                      if path.is_dir() and path.name.startswith("app_run_"))

    def test_recording_precedes_the_service(self) -> None:
        self.assertTrue(self.runtime_dirs(), "the runtime wrote no run")
        self.assertTrue(self.application_dirs(), "the application wrote no run")
        service = Service(self.records, self.data, READER)
        try:
            status, catalog = service.call("GET", "/api/runs")
            self.assertEqual(status, 200)
            names = {run["name"] for run in catalog["runs"]}
            self.assertTrue(any(name.startswith("run_") for name in names))
            self.assertTrue(any(name.startswith("app_run_") for name in names))
            runtime = next(run for run in catalog["runs"]
                           if run["kind"] == "runtime")
            status, described = service.call(
                "GET", f"/api/runs/{runtime['run_id']}/describe")
            self.assertEqual(status, 200)
            signal = next(signal for signal in described["context"]["signals"]
                          if signal["field"] == "q")
            status, ranged = service.call(
                "GET",
                f"/api/runs/{runtime['run_id']}/range"
                f"?signal_ids={urllib.parse.quote(signal['id'], safe='')}"
                "&begin_us=0&end_us=9999999999999999&pixel_width=100",
            )
            self.assertEqual(status, 200)
            self.assertTrue(ranged["ok"])
            self.assertGreater(len(ranged["series"][0]["points"]), 0)
        finally:
            service.stop()

    def test_service_restart_keeps_layouts_and_history(self) -> None:
        service = Service(self.records, self.data, READER)
        layout = {
            "version": 1,
            "name": "integration-layout",
            "dock": {"type": "empty"},
            "charts": [],
        }
        request = urllib.request.Request(
            service.url + "/api/layouts",
            data=json.dumps(layout).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json",
                     "Origin": service.url},
        )
        with service.opener.open(request) as response:
            saved = json.loads(response.read().decode("utf-8"))
        service.stop()
        service = Service(self.records, self.data, READER)
        try:
            status, layouts = service.call("GET", "/api/layouts")
            self.assertEqual(status, 200)
            self.assertIn(saved["id"], [entry["id"] for entry in layouts["layouts"]])
            status, catalog = service.call("GET", "/api/runs")
            self.assertGreaterEqual(len(catalog["runs"]), 2)
        finally:
            service.stop()

    def test_disabled_recorder_and_corrupt_file(self) -> None:
        base = self.base / "disabled"
        base.mkdir()
        records = base / "records"
        records.mkdir()
        config = copy_config(SOURCE, base / "sdk.json", records,
                             base / "logs", False)
        run_processes(config, RUNTIME, APPLICATION)
        self.assertEqual(list(records.iterdir()), [])
        service = Service(self.records, self.data, READER)
        try:
            status, catalog = service.call("GET", "/api/runs")
            names_before = {run["name"] for run in catalog["runs"]}
            self.assertNotIn("run_disabled", names_before)
        finally:
            service.stop()

        # A committed chunk whose payload was edited is named corrupt and
        # contributes no rows; the run itself stays readable.
        runtime = self.runtime_dirs()[0]
        corrupt = self.records / "run_corrupt_copy"
        shutil.copytree(runtime, corrupt)
        chunk = next(corrupt.glob("chunk_*.bin"))
        data = bytearray(chunk.read_bytes())
        data[-1] ^= 0x5A
        chunk.write_bytes(data)
        service = Service(self.records, self.data, READER)
        try:
            status, catalog = service.call("GET", "/api/runs")
            entry = next(run for run in catalog["runs"]
                         if run["name"] == "run_corrupt_copy")
            status, described = service.call(
                "GET", f"/api/runs/{entry['run_id']}/describe")
            self.assertTrue(described["ok"])
            signal = next(item for item in described["context"]["signals"]
                          if item["field"] == "q")
            status, ranged = service.call(
                "GET",
                f"/api/runs/{entry['run_id']}/range"
                f"?signal_ids={urllib.parse.quote(signal['id'], safe='')}"
                "&begin_us=0&end_us=9999999999999999&pixel_width=10",
            )
            self.assertEqual(status, 200)
            self.assertTrue(ranged["integrity"]["runtime"]["corrupt"])
            self.assertEqual(ranged["series"][0]["points"], [])
        finally:
            service.stop()

    def test_empty_fault_stream_differs_from_missing_file(self) -> None:
        runtime = self.runtime_dirs()[0]
        empty = self.records / "run_empty_faults"
        missing = self.records / "run_missing_faults"
        shutil.copytree(runtime, empty)
        shutil.copytree(runtime, missing)
        (empty / "faults.jsonl").write_text("", encoding="utf-8")
        (missing / "faults.jsonl").unlink()
        service = Service(self.records, self.data, READER)
        try:
            status, catalog = service.call("GET", "/api/runs")
            empty_id = next(run["run_id"] for run in catalog["runs"]
                            if run["name"] == "run_empty_faults")
            missing_id = next(run["run_id"] for run in catalog["runs"]
                              if run["name"] == "run_missing_faults")
            status, events = service.call(
                "GET", f"/api/runs/{empty_id}/events?begin_us=0")
            self.assertEqual(status, 200)
            self.assertTrue(events["ok"])
            status, events = service.call(
                "GET", f"/api/runs/{missing_id}/events?begin_us=0")
            self.assertEqual(status, 200)
            self.assertFalse(events["ok"])
            self.assertIn("cannot find", events["error"])
        finally:
            service.stop()

    def test_runtime_restart_is_a_new_run(self) -> None:
        before = {path.name for path in self.runtime_dirs()}
        run_processes(self.config, RUNTIME, APPLICATION)
        after = {path.name for path in self.runtime_dirs()}
        self.assertGreater(len(after), len(before))


if __name__ == "__main__":
    if len(sys.argv) < 5:
        print(__doc__, file=sys.stderr)
        raise SystemExit(2)
    SOURCE = Path(sys.argv[1]).resolve()
    RUNTIME = Path(sys.argv[2]).resolve()
    APPLICATION = Path(sys.argv[3]).resolve()
    READER = Path(sys.argv[4]).resolve()
    unittest.main(argv=[sys.argv[0]])
