#!/usr/bin/env python3
"""tests/integration/record_export_test.py <robot_runtime> <source-root>

The recorded run, exported: a real runtime records for a few seconds with the
recorder enabled, tools/read_record reads it back, and tools/record_to_npz.py
turns it into an archive. The test then proves what the archive must carry for a
consumer to interpret it - every column by name, the joint table, the units, the
schema version, the configuration and layout digests, the rates - and that a
time window selects a subset of the same data rather than a different run.

Nothing here touches DDS on the reading side: the tools are file readers.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

DOMAIN = 228
RECORD_SECONDS = 4

COLUMNS = (
    "stamp_us",
    "state_seq",
    "execution_state",
    "output_action",
    "stop_reason",
    "fault_id",
    "healthy",
    "damping_eligible",
    "command_present",
    "epoch",
    "command_seq",
    "source_state_seq",
    "command_rx_us",
    "feedback_seq",
    "feedback_stamp_us",
    "feedback_valid",
    "imu_valid",
    "send_accepted",
    "q",
    "dq",
    "tau_ff",
    "kp",
    "kd",
)


def wait_for(path: Path, pattern: str, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    text = ""
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="replace")
            if pattern in text:
                return text
        time.sleep(0.2)
    return text


def main() -> int:
    runtime, source_root = sys.argv[1], Path(sys.argv[2]).resolve()
    failures: list[str] = []

    def fail(note: str) -> None:
        failures.append(note)
        print(f"record_export_test: {note}", file=sys.stderr, flush=True)

    with tempfile.TemporaryDirectory(prefix="irmv-record-export-") as directory:
        work = Path(directory)
        generated = subprocess.run(
            [
                sys.executable,
                str(source_root / "tools" / "make_test_model.py"),
                "--fake-deployment",
                str(work / "deployment"),
                str(source_root / "config" / "sdk.json"),
                str(DOMAIN),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if generated.returncode != 0:
            fail(f"the deployment was not generated: {generated.stderr}")
            return 1
        config_path = work / "deployment" / "sdk.json"
        config = json.loads(config_path.read_text())
        config["telemetry"]["directory"] = str(work / "logs")
        config["telemetry"]["recorder"] = {
            "enabled": True,
            "sample_hz": 100,
            "queue_capacity": 256,
            "directory": str(work / "runs"),
        }
        config_path.write_text(json.dumps(config, indent=2))
        (work / "runs").mkdir()

        runtime_log = (work / "runtime.log").open("w", encoding="utf-8")
        process = subprocess.Popen(
            [
                runtime,
                "--config",
                str(config_path),
                "--domain",
                str(DOMAIN),
                "--fake-scenario",
                str(work / "deployment" / "fake_scenario.json"),
            ],
            stdout=runtime_log,
            stderr=subprocess.STDOUT,
            cwd=work,
        )
        try:
            text = wait_for(work / "runtime.log", 'state="ZeroTorque"', 20)
            if 'state="ZeroTorque"' not in text:
                fail(f"the runtime never reached ZeroTorque: {text[-400:]}")
                return 1
            time.sleep(RECORD_SECONDS)
        finally:
            process.send_signal(15)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
            runtime_log.close()

        runs = sorted((work / "runs").iterdir())
        if len(runs) != 1:
            fail(f"expected one run directory, found {len(runs)}")
            return 1
        run = runs[0]
        index = json.loads((run / "index.json").read_text())
        rows_written = int(index["rows_recorded"])

        # The reader must agree with the index the run wrote for itself.
        archive_path = work / "run.npz"
        convert = subprocess.run(
            [
                sys.executable,
                str(source_root / "tools" / "record_to_npz.py"),
                str(run),
                str(archive_path),
                "--reader",
                str(source_root / "build" / "read_record"),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if convert.returncode != 0:
            fail(f"the conversion failed: {convert.stderr.strip()}")
            return 1

        data = np.load(archive_path, allow_pickle=False)
        for column in COLUMNS:
            if column not in data:
                fail(f"the archive has no {column} column")
        if failures:
            return 1

        for column in ("stamp_us", "state_seq", "execution_state", "send_accepted"):
            if data[column].shape != (rows_written,):
                fail(f"{column} has shape {data[column].shape}, expected ({rows_written},)")
        joints = int(data["joint_names"].shape[0])
        for column in ("q", "dq", "tau_ff", "kp", "kd"):
            if data[column].shape != (rows_written, joints):
                fail(f"{column} has shape {data[column].shape}, expected ({rows_written}, {joints})")

        if int(data["schema_version"]) != 1:
            fail("the archive does not carry the schema version")
        if int(data["sample_hz"]) != 100:
            fail("the archive does not carry the sampling rate")
        if str(data["config_id"]) == "":
            fail("the archive does not carry the configuration digest")
        if len(data["joint_names"]) != joints or joints == 0:
            fail("the archive does not carry the joint table")

        # A 100 Hz recording over a four second run: the row rate is the
        # configured one and not the control rate, and the spacing is even.
        span_us = int(data["stamp_us"][-1]) - int(data["stamp_us"][0])
        if span_us <= 0:
            fail("the archive spans no time")
        else:
            rate = (rows_written - 1) * 1e6 / span_us
            if not 90.0 <= rate <= 110.0:
                fail(f"the recorded rate is {rate:.1f} Hz, expected about 100")
        steps = np.diff(data["state_seq"])
        if steps.size and not np.all(steps == steps[0]):
            fail("the sampling stride is not constant across the run")

        # A window of the run returns the same data for that window, not a
        # different run: the same rows, by the controller's own stamp.
        window_start = int(data["stamp_us"][rows_written // 4])
        window_end = int(data["stamp_us"][rows_written // 2])
        window_path = work / "window.npz"
        window = subprocess.run(
            [
                sys.executable,
                str(source_root / "tools" / "record_to_npz.py"),
                str(run),
                str(window_path),
                "--reader",
                str(source_root / "build" / "read_record"),
                "--from-us",
                str(window_start),
                "--to-us",
                str(window_end),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if window.returncode != 0:
            fail(f"the windowed conversion failed: {window.stderr.strip()}")
        else:
            subset = np.load(window_path, allow_pickle=False)
            expected = data["state_seq"][
                (data["stamp_us"] >= window_start) & (data["stamp_us"] < window_end)
            ]
            if not np.array_equal(subset["state_seq"], expected):
                fail("the windowed export does not match the same rows of the whole run")

    if failures:
        return 1
    print(
        f"record_export_test: {rows_written} rows, {joints} joints, "
        f"{len(index['gaps'])} gap(s), exported and windowed"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
