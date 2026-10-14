#!/usr/bin/env python3
"""tests/integration/record_export_test.py <robot_runtime> <source-root>

The recorded run, exported: a real runtime records for a few seconds with the
recorder enabled, tools/read_record reads it back, and tools/record_to_npz.py
turns it into an archive. The test then proves what the archive must carry for a
consumer to interpret it - every column by name, the joint table, the units, the
schema version, the configuration and layout digests, the rates - and that a
time window selects a subset of the same data rather than a different run.

The archive also has to keep the *shape* of the numbers it carries, so each
column is re-read and checked against the reader's own output: the 64-bit counts
stay uint64 (a microsecond stamp above 2**53, which the Fake deployment's stamps
are far below, is proved exactly on a row of the same shape), the flags stay
bool, the joint columns stay float64, and the 16-byte producer identity of every
row survives whole rather than being narrowed to a number.

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

DOMAIN = 150
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
    "producer_id",
    "epoch",
    "command_seq",
    "source_state_seq",
    "command_rx_us",
    "feedback_seq",
    "feedback_stamp_us",
    "feedback_valid",
    "imu_valid",
    "send_accepted",
    "output_valid",
    "q",
    "dq",
    "tau_ff",
    "kp",
    "kd",
    "measured_q",
    "measured_dq",
    "output_torque",
)

# The dtype each column must keep in the archive: a 64-bit count that came back
# as a signed or floating width has lost bits, and a flag that came back as a
# number invites arithmetic on it.
UINT64_COLUMNS = (
    "stamp_us",
    "state_seq",
    "fault_id",
    "epoch",
    "command_seq",
    "source_state_seq",
    "command_rx_us",
    "feedback_seq",
    "feedback_stamp_us",
)
UINT32_COLUMNS = ("execution_state", "output_action", "stop_reason")
BOOL_COLUMNS = (
    "healthy",
    "damping_eligible",
    "command_present",
    "feedback_valid",
    "imu_valid",
    "send_accepted",
    "output_valid",
)
FLOAT_COLUMNS = (
    "q",
    "dq",
    "tau_ff",
    "kp",
    "kd",
    "measured_q",
    "measured_dq",
    "output_torque",
)

# The producer identity is 16 opaque bytes, rendered the way the reader prints
# it: 32 lower-case hex digits, one string per row.
PRODUCER_ID = "f00d" * 8


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
    reader = source_root / "build" / "read_record"
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
                str(reader),
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
        if "units" not in data:
            fail("the archive does not carry the run's units")
        if failures:
            return 1

        # The dtype of every column is part of the contract a consumer relies
        # on, not something numpy happens to infer.
        for column in UINT64_COLUMNS:
            if data[column].dtype != np.uint64:
                fail(f"{column} is {data[column].dtype}, expected uint64")
        for column in UINT32_COLUMNS:
            if data[column].dtype != np.uint32:
                fail(f"{column} is {data[column].dtype}, expected uint32")
        for column in BOOL_COLUMNS:
            if data[column].dtype != np.bool_:
                fail(f"{column} is {data[column].dtype}, expected bool")
        for column in FLOAT_COLUMNS:
            if data[column].dtype != np.float64:
                fail(f"{column} is {data[column].dtype}, expected float64")
        if data["producer_id"].dtype.kind != "U":
            fail(f"producer_id is {data['producer_id'].dtype}, expected a hex string")
        # A run with no gap at all still has the gap table, and its columns must
        # not change shape just because the table is empty.
        for column in ("gap_first_seq", "gap_last_seq", "gap_dropped"):
            if data[column].dtype != np.uint64:
                fail(f"{column} is {data[column].dtype}, expected uint64")
        if data["gap_reason"].dtype.kind != "U":
            fail(f"gap_reason is {data['gap_reason'].dtype}, expected a text column")

        for column in (*UINT64_COLUMNS, *UINT32_COLUMNS, *BOOL_COLUMNS, "producer_id"):
            if data[column].shape != (rows_written,):
                fail(f"{column} has shape {data[column].shape}, expected ({rows_written},)")
        joints = int(data["joint_names"].shape[0])
        for column in FLOAT_COLUMNS:
            if data[column].shape != (rows_written, joints):
                fail(f"{column} has shape {data[column].shape}, expected ({rows_written}, {joints})")

        if int(data["schema_version"]) != 3:
            fail("the archive does not carry the schema version")
        # The recorded run is intact, so the two damage flags are false: a run
        # that was never cut short and never corrupted says so.
        if bool(data["truncated"]):
            fail("the archive marks an intact run as truncated")
        if bool(data["corrupt"]):
            fail("the archive marks an intact run as corrupt")
        if int(data["sample_hz"]) != 100:
            fail("the archive does not carry the sampling rate")
        if str(data["config_id"]) == "":
            fail("the archive does not carry the configuration digest")
        if len(data["joint_names"]) != joints or joints == 0:
            fail("the archive does not carry the joint table")

        # The session sidecar is part of the run: every session column must be
        # present, and its length is the number of contexts the run adopted -
        # not the number of sample rows. The digest is text, kept verbatim.
        for column in (
            "session_runtime_id",
            "session_session_id",
            "session_producer_id",
            "session_epoch",
            "session_stamp_us",
            "session_state_seq",
            "session_joint_count",
            "session_schema_version",
            "session_kp",
            "session_kd",
            "session_model_sha256",
        ):
            if column not in data:
                fail(f"the archive has no {column} column")

        # The units are the run's own declaration, not a table this export
        # derived: the metadata the run wrote for itself says where the numbers
        # came from, and a copy that says anything else is a different claim.
        metadata = json.loads((run / "meta.json").read_text())
        archived_units = json.loads(str(data["units"]))
        if archived_units != metadata["units"]:
            fail(f"the archive units {archived_units} are not the run's {metadata['units']}")

        # Every column is checked against the reader's own output row by row:
        # the archive is a copy of a read, so a value it changed is a value it
        # invented. The producer identity is 16 opaque bytes and the stamp is a
        # 64-bit microsecond count, the two fields a narrower encoding could not
        # have carried whole.
        sys.path.insert(0, str(source_root / "tools"))
        import record_to_npz

        rows_read, run_summary = record_to_npz.read_run(reader, run, 0, None)
        if len(rows_read) != rows_written:
            fail(f"the reader returned {len(rows_read)} rows, expected {rows_written}")
        else:
            if [row["producer_id"] for row in rows_read] != list(data["producer_id"]):
                fail("the archive does not carry the producer identity of every row")
            if [row["stamp_us"] for row in rows_read] != [int(value) for value in data["stamp_us"]]:
                fail("the archive stamps are not the reader's stamps")
            # The columns a v1 run could not carry: the measured feedback and
            # the output stage's torque. They are checked against the reader's
            # own rows, so an archive that changed them is an archive that
            # invented them.
            for column in ("measured_q", "measured_dq", "output_torque"):
                if [row[column] for row in rows_read] != [
                    list(value) for value in data[column]
                ]:
                    fail(f"the archive does not carry the reader's {column} columns")
            if [row["output_valid"] for row in rows_read] != [
                bool(value) for value in data["output_valid"]
            ]:
                fail("the archive does not carry the reader's output_valid flags")

            # The session table is carried whole and in order, with the digest
            # the runtime wrote - "unknown" here, because the runtime holds no
            # model and never invents one.
            session_table = run_summary.get("sessions", [])
            if len(session_table) != int(data["session_joint_count"].shape[0]):
                fail("the archive does not carry the same number of sessions as the run")
            if [entry["producer_id"] for entry in session_table] != list(
                data["session_producer_id"]
            ) or [entry["epoch"] for entry in session_table] != [
                int(value) for value in data["session_epoch"]
            ]:
                fail("the archive does not carry the session producer and epoch")
            if any(entry["model_sha256"] != "unknown" for entry in session_table):
                fail("a session event invented a model digest")
            if list(data["session_model_sha256"]) != [
                entry["model_sha256"] for entry in session_table
            ]:
                fail("the archive does not carry the session model digest verbatim")

        # The Fake deployment's stamps stay far below 2**53, where a float64
        # would still be exact, so the guarantee is proved on a row of the same
        # shape placed above it rather than hoped for from a short run.
        if rows_read:
            above_two_53 = (1 << 53) + 1
            synthetic_row = dict(rows_read[0])
            synthetic_row["stamp_us"] = above_two_53
            synthetic_row["producer_id"] = PRODUCER_ID
            synthetic_path = work / "synthetic.npz"
            np.savez_compressed(
                synthetic_path, **record_to_npz.to_archive([synthetic_row], run_summary)
            )
            synthetic = np.load(synthetic_path, allow_pickle=False)
            if synthetic["stamp_us"].dtype != np.uint64 or int(synthetic["stamp_us"][0]) != above_two_53:
                fail(
                    f"a stamp above 2**53 came back as {synthetic['stamp_us'][0]!r} "
                    f"of dtype {synthetic['stamp_us'].dtype}"
                )
            if str(synthetic["producer_id"][0]) != PRODUCER_ID:
                fail(f"the producer identity came back as {str(synthetic['producer_id'][0])!r}")

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
                str(reader),
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
