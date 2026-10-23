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
import shutil
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
    "imu_orientation_rpy",
    "imu_angular_velocity",
    "imu_linear_acceleration",
    "imu_rx_stamp_us",
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
    "measured_tau_est",
    "joint_valid",
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
    "imu_rx_stamp_us",
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
    "measured_tau_est",
)

# The IMU vectors are (rows, 3) float64 and the per-joint validity flag is a
# (rows, joints) bool matrix, so neither shape nor dtype is inferred.
IMU_COLUMNS = (
    "imu_orientation_rpy",
    "imu_angular_velocity",
    "imu_linear_acceleration",
)
JOINT_BOOL_COLUMNS = ("joint_valid",)

# The producer identity is 16 opaque bytes, rendered the way the reader prints
# it: 32 lower-case hex digits, one string per row.
PRODUCER_ID = "f00d" * 8

# The identity the synthetic runtime record authorizes. The application stream
# is built around it so association is exercised on real reader output.
RUNTIME_ID = "ab" * 16
SESSION_ID = "cd" * 16
OTHER_RUNTIME_ID = "ef" * 16
OTHER_SESSION_ID = "12" * 16


def write_json(path: Path, document) -> None:
    path.write_text(json.dumps(document, ensure_ascii=False, indent=1) + "\n")


def make_fault_run(
    source: Path, target: Path, stamps: list[int], joint_count: int
) -> None:
    """Copy a real run and give it a session identity and a fault stream.

    Copying keeps the committed chunks real: the reader still verifies their
    headers, payload lengths and checksums. The sidecar and the index are
    written in the exact schema the recorder publishes.
    """

    shutil.copytree(source, target)
    write_json(
        target / "sessions.json",
        [
            {
                "schema_version": 4,
                "runtime_id": RUNTIME_ID,
                "session_id": SESSION_ID,
                "producer_id": "00" * 16,
                "epoch": 3,
                "stamp_us": stamps[0],
                "state_seq": 1,
                "joint_count": joint_count,
                "kp": [1.0] * joint_count,
                "kd": [0.1] * joint_count,
                "model_sha256": "unknown",
            }
        ],
    )
    message = "synthetic fault"
    faults = []
    for index, (condition, name, joint, observed, limit) in enumerate(
        (
            (2, "JointDisconnected", 0, "NaN", 0.0),
            (3, "JointFeedbackInvalid", None, "+Inf", 1.0),
            (4, "JointFeedbackNonFinite", 1, "-Inf", -1.0),
        )
    ):
        faults.append(
            {
                "schema_version": 4,
                "record_seq": index + 1,
                "runtime_id": RUNTIME_ID,
                "state_seq": f"{index + 1:016x}",
                "fault_id": f"{7:016x}",
                "reason": 2,
                "reason_name": "FeedbackInvalid",
                "condition": condition,
                "condition_name": name,
                "joint_index": joint,
                "error_code": 10 + index,
                "observed_value": observed,
                "limit": limit,
                "sample_stamp_us": stamps[index],
                "detected_stamp_us": stamps[index] + 1,
                "output_action": 0,
                "message": message,
                "message_hex": message.encode("utf-8").hex(),
            }
        )
    (target / "faults.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in faults), encoding="utf-8"
    )
    index = json.loads((target / "index.json").read_text())
    index["faults"] = {
        "complete": True,
        "recorded": len(faults),
        "written": len(faults),
        "queue_dropped": 0,
        "generation_dropped": 0,
        "write_failures": 0,
        "losses": [],
    }
    write_json(target / "index.json", index)


def make_application_run(
    directory: Path, runtime_meta: dict, t0: int
) -> dict:
    """Write one application run with a loss, a gap and a foreign tail."""

    directory.mkdir()
    model_name = "模型-实验.onnx"
    write_json(
        directory / "meta.json",
        {
            "schema": "irmv.application_velocity",
            "version": 1,
            "sdk_git_sha": runtime_meta["sdk_git_sha"],
            "application_config_id": "application-config-digest",
            "config_id": runtime_meta["config_id"],
            "layout_id": runtime_meta["layout_id"],
            "joint_count": runtime_meta["joint_count"],
            "sample_hz": 100,
            "loop_period_us": 20000,
            "sampling": {"paced_by_deadline": False},
            "source": "local",
            "clock": runtime_meta["clock"],
            "units": {
                "forward": "m_s",
                "lateral": "m_s",
                "yaw": "rad_s",
                "time": "us",
            },
            "model": {
                "status": "loaded",
                "name": model_name,
                "path": f"/opt/models/{model_name}",
                "sha256": "a" * 64,
                "contract_sha256": "b" * 64,
            },
        },
    )

    def row(
        seq: int,
        stamp: int,
        *,
        runtime_id=RUNTIME_ID,
        session_id=SESSION_ID,
        unknown=False,
    ):
        return {
            "schema": "irmv.application_velocity",
            "version": 1,
            "host_us": stamp,
            "stamp_us": stamp,
            "sample_seq": f"{seq:016x}",
            "identity_known": not unknown,
            "runtime_id": "" if unknown else runtime_id,
            "session_id": "" if unknown else session_id,
            "latest_state_seq": f"{seq + 1:016x}",
            "forward": 0.01 * (seq + 1),
            "lateral": -0.002 * (seq + 1),
            "yaw": 0.003 * (seq + 1),
        }

    rows = [row(index, t0 + index * 10_000) for index in range(20)]
    # A ten-sample loss (20..29) followed by the next recorded segment at 30.
    rows += [row(index, t0 + index * 10_000) for index in range(30, 40)]
    # A foreign context at the tail, and one row whose identity was never known.
    rows += [
        row(
            index,
            t0 + index * 10_000,
            runtime_id=OTHER_RUNTIME_ID,
            session_id=OTHER_SESSION_ID,
        )
        for index in range(40, 43)
    ]
    rows.append(row(90, t0 + 900_000, unknown=True))
    (directory / "velocity.jsonl").write_text(
        "".join(json.dumps(entry) + "\n" for entry in rows), encoding="utf-8"
    )
    write_json(
        directory / "index.json",
        {
            "schema": "irmv.application_velocity",
            "version": 1,
            "complete": True,
            "write_failures": 0,
            "losses": [
                {
                    "first_sample_seq": f"{20:016x}",
                    "last_sample_seq": f"{29:016x}",
                    "first_stamp_us": t0 + 20 * 10_000,
                    "last_stamp_us": t0 + 29 * 10_000,
                    "dropped": 10,
                    "reason": "queue_full",
                }
            ],
        },
    )
    policy_entry = {
        "schema": "irmv.policy_session",
        "version": 1,
        "host_us": t0,
        "monotonic_us": t0,
        "runtime_id": RUNTIME_ID,
        "session_id": SESSION_ID,
        "producer_process_id": "00" * 16,
        "epoch": 3,
        "source": "local",
        "model": {
            "name": model_name,
            "sha256": "a" * 64,
            "path": f"/opt/models/{model_name}",
            "input": "observation",
            "output": "action",
            "contract_sha256": "b" * 64,
        },
        "gains": [],
    }
    # The policy sidecar is one JSON object per line.
    (directory / "policy_sessions.jsonl").write_text(
        json.dumps(policy_entry, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return {
        "t0": t0,
        "rows": rows,
        "selected": 30,
        "excluded": len(rows) - 30,
        "model_name": model_name,
        "gap_start": t0 + 20 * 10_000,
        "gap_end": t0 + 30 * 10_000,
    }


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
        for column in IMU_COLUMNS:
            if data[column].dtype != np.float64:
                fail(f"{column} is {data[column].dtype}, expected float64")
        for column in JOINT_BOOL_COLUMNS:
            if data[column].dtype != np.bool_:
                fail(f"{column} is {data[column].dtype}, expected bool")
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
        for column in IMU_COLUMNS:
            if data[column].shape != (rows_written, 3):
                fail(f"{column} has shape {data[column].shape}, expected ({rows_written}, 3)")
        for column in JOINT_BOOL_COLUMNS:
            if data[column].shape != (rows_written, joints):
                fail(f"{column} has shape {data[column].shape}, expected ({rows_written}, {joints})")
        if int(data["imu_rx_stamp_us"].shape[0]) != rows_written:
            fail("imu_rx_stamp_us does not have one value per row")

        if int(data["schema_version"]) != 4:
            fail("the archive does not carry the schema version")
        # The recorded run is intact, so the two damage flags are false: a run
        # that was never cut short and never corrupted says so.
        if bool(data["truncated"]):
            fail("the archive marks an intact run as truncated")
        if bool(data["corrupt"]):
            fail("the archive marks an intact run as corrupt")
        if "complete" not in data or not bool(data["complete"]):
            fail("the archive does not mark an intact run as complete")
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

        # The IMU convention the run can state is written down, and what the
        # driver contract does not fix is marked unknown rather than guessed at:
        # the archive must let a consumer see the difference.
        imu_units = {
            "imu_orientation_rpy": "rad",
            "imu_angular_velocity": "rad_s",
            "imu_linear_acceleration": "m_s2",
        }
        for key, unit in imu_units.items():
            if archived_units.get(key) != unit:
                fail(f"the run's units do not declare {key} as {unit}")
        imu_metadata = metadata.get("imu")
        if not isinstance(imu_metadata, dict):
            fail("the run's metadata does not state its IMU conventions")
        if "imu_conventions" not in data:
            fail("the archive does not carry the run's IMU conventions")
            imu_conventions = {}
        else:
            archived_conventions = json.loads(str(data["imu_conventions"]))
            imu_conventions = archived_conventions
            if archived_conventions != imu_metadata:
                fail("the archive IMU conventions are not the run's own declaration")
        # The archive itself - not the run directory it came from - must carry
        # the declared order, units and the conventions marked unknown.
        orientation = imu_conventions.get("orientation_rpy", {})
        if orientation.get("order") != "roll,pitch,yaw" or orientation.get("unit") != "rad":
            fail("the archive does not state RPY as roll,pitch,yaw in rad")
        acceleration = imu_conventions.get("linear_acceleration", {})
        if acceleration.get("unit") != "m_s2":
            fail("the archive does not state linear acceleration in m/s^2")
        if acceleration.get("includes_gravity") != "unknown":
            fail("the archive claims a gravity convention the driver does not define")
        for vector in ("orientation_rpy", "angular_velocity", "linear_acceleration"):
            entry = imu_conventions.get(vector, {})
            if entry.get("axis_frame") != "unknown" or entry.get("axis_order") != "unknown":
                fail(f"the archive states an axis convention for {vector} the driver does not define")

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
            # The schema-4 columns: the IMU snapshot, the receive stamp, the
            # torque estimate and the per-joint validity flags. Each is compared
            # with the reader's own row, so the archive carries what was read.
            for column in IMU_COLUMNS:
                if [row[column] for row in rows_read] != [
                    list(value) for value in data[column]
                ]:
                    fail(f"the archive does not carry the reader's {column} vectors")
            if [row["imu_rx_stamp_us"] for row in rows_read] != [
                int(value) for value in data["imu_rx_stamp_us"]
            ]:
                fail("the archive does not carry the reader's IMU receive stamps")
            if [row["measured_tau_est"] for row in rows_read] != [
                list(value) for value in data["measured_tau_est"]
            ]:
                fail("the archive does not carry the reader's measured_tau_est column")
            if [row["joint_valid"] for row in rows_read] != [
                [bool(value) for value in row] for row in data["joint_valid"]
            ]:
                fail("the archive does not carry the reader's joint_valid flags")

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
            synthetic_row["imu_rx_stamp_us"] = above_two_53 + 2
            synthetic_row["producer_id"] = PRODUCER_ID
            synthetic_row["joint_valid"] = [index % 2 == 0 for index in range(joints)]
            # The reader writes a non-finite sample double as an explicit
            # string. The export must restore it without folding either
            # infinity into NaN.
            synthetic_row["measured_tau_est"] = (
                ["NaN", "+Inf"] + [0.0] * (joints - 2)
            )
            synthetic_row["imu_orientation_rpy"] = ["-Inf", 0.0, 1.0]
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
            if synthetic["imu_rx_stamp_us"].dtype != np.uint64 or int(
                synthetic["imu_rx_stamp_us"][0]
            ) != above_two_53 + 2:
                fail(
                    f"an IMU stamp above 2**53 came back as "
                    f"{synthetic['imu_rx_stamp_us'][0]!r} of dtype "
                    f"{synthetic['imu_rx_stamp_us'].dtype}"
                )
            if synthetic["joint_valid"].dtype != np.bool_ or list(
                synthetic["joint_valid"][0]
            ) != [index % 2 == 0 for index in range(joints)]:
                fail("the per-joint validity flags did not survive the export as booleans")
            if str(synthetic["producer_id"][0]) != PRODUCER_ID:
                fail(f"the producer identity came back as {str(synthetic['producer_id'][0])!r}")
            if not np.isnan(synthetic["measured_tau_est"][0][0]):
                fail("a NaN sample value did not survive the export as NaN")
            if not np.isposinf(synthetic["measured_tau_est"][0][1]):
                fail("a +Inf sample value did not survive the export as +Inf")
            if not np.isneginf(synthetic["imu_orientation_rpy"][0][0]):
                fail("a -Inf IMU value did not survive the export as -Inf")

            # An empty run keeps every column's shape: (0, 3) for the IMU
            # vectors, (0, joints) for the joint matrices and validity block,
            # and a one-dimensional uint64 stamp column.
            empty = record_to_npz.to_archive([], run_summary)
            for column in IMU_COLUMNS:
                if empty[column].shape != (0, 3) or empty[column].dtype != np.float64:
                    fail(f"an empty run exported {column} with shape "
                         f"{empty[column].shape} and dtype {empty[column].dtype}")
            for column in (*FLOAT_COLUMNS, *JOINT_BOOL_COLUMNS):
                expected_dtype = np.bool_ if column in JOINT_BOOL_COLUMNS else np.float64
                if empty[column].shape != (0, joints) or empty[column].dtype != expected_dtype:
                    fail(f"an empty run exported {column} with shape "
                         f"{empty[column].shape} and dtype {empty[column].dtype}")
            if empty["imu_rx_stamp_us"].shape != (0,) or empty["imu_rx_stamp_us"].dtype != np.uint64:
                fail(f"an empty run exported imu_rx_stamp_us with shape "
                     f"{empty['imu_rx_stamp_us'].shape} and dtype "
                     f"{empty['imu_rx_stamp_us'].dtype}")
            # The fault table is independent of the sample rows and keeps its
            # own shape when there is no occurrence at all.
            for column, dtype in (
                ("fault_record_seq", np.uint64),
                ("fault_joint_index", np.int64),
                ("fault_observed_value", np.float64),
                ("fault_condition", np.uint32),
            ):
                if empty[column].shape != (0,) or empty[column].dtype != dtype:
                    fail(
                        f"an empty run exported {column} with shape "
                        f"{empty[column].shape} and dtype {empty[column].dtype}"
                    )
            if empty["fault_condition_name"].dtype.kind != "U":
                fail("an empty run exported fault_condition_name as a non-text column")

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

        # A run killed before its recorder finalizes is unfinished even when
        # every committed chunk reads cleanly. Both exports must carry that
        # fact; a consumer that only sees truncated=false and corrupt=false
        # would otherwise treat the run as whole.
        killed_runs = work / "killed-runs"
        killed_runs.mkdir()
        killed_config = json.loads(config_path.read_text())
        killed_config["telemetry"]["directory"] = str(work / "killed-logs")
        killed_config["telemetry"]["recorder"] = {
            "enabled": True,
            # One second of rows per chunk at the Fake deployment's 500 Hz
            # control rate, so the test does not wait nine seconds for the first
            # committed chunk.
            "sample_hz": 500,
            "queue_capacity": 256,
            "directory": str(killed_runs),
        }
        killed_config_path = work / "killed.json"
        killed_config_path.write_text(json.dumps(killed_config, indent=2))
        killed_log_path = work / "killed-runtime.log"
        with killed_log_path.open("w", encoding="utf-8") as killed_log:
            killed = subprocess.Popen(
                [
                    runtime,
                    "--config",
                    str(killed_config_path),
                    "--domain",
                    str(DOMAIN + 4),
                    "--fake-scenario",
                    str(work / "deployment" / "fake_scenario.json"),
                ],
                stdout=killed_log,
                stderr=subprocess.STDOUT,
                cwd=work,
            )
            killed_run = None
            try:
                text = wait_for(killed_log_path, 'state="ZeroTorque"', 20)
                if 'state="ZeroTorque"' not in text:
                    fail(f"the killed runtime never reached ZeroTorque: {text[-400:]}")
                else:
                    deadline = time.monotonic() + 20
                    while time.monotonic() < deadline:
                        runs = [entry for entry in killed_runs.iterdir()
                                if entry.is_dir()]
                        if runs:
                            index_path = runs[0] / "index.json"
                            try:
                                current = json.loads(index_path.read_text())
                            except (OSError, ValueError):
                                current = {}
                            if int(current.get("rows_recorded", 0)) > 0 and current.get("chunks"):
                                killed_run = runs[0]
                                break
                        time.sleep(0.1)
                    if killed_run is None:
                        fail("the killed run committed no chunk to export")
                    else:
                        # SIGKILL leaves the open run exactly as a crash does:
                        # no final index replacement, no Stop().
                        killed.kill()
                        killed.wait(timeout=10)
                        killed_archive = work / "killed.npz"
                        killed_convert = subprocess.run(
                            [
                                sys.executable,
                                str(source_root / "tools" / "record_to_npz.py"),
                                str(killed_run),
                                str(killed_archive),
                                "--reader",
                                str(reader),
                            ],
                            capture_output=True,
                            text=True,
                            check=False,
                        )
                        if killed_convert.returncode != 0:
                            fail(f"the killed-run conversion failed: {killed_convert.stderr.strip()}")
                        else:
                            unfinished = np.load(killed_archive, allow_pickle=False)
                            if "complete" not in unfinished:
                                fail("the killed-run archive has no completion flag")
                            elif bool(unfinished["complete"]):
                                fail("the killed-run archive marks the run complete")
                            if int(unfinished["stamp_us"].shape[0]) == 0:
                                fail("the killed-run archive exported no committed rows")
            finally:
                if killed.poll() is None:
                    killed.kill()
                    try:
                        killed.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        pass

        # ------------------------------------------------------------------
        # The fault stream and the application's independent velocity stream,
        # exported together. The runtime run is a real committed run with a
        # synthetic session identity and fault sidecar; the application run is
        # written in the exact schema its recorder publishes.
        # ------------------------------------------------------------------
        sample_stamps = [int(value) for value in data["stamp_us"]]
        fault_stamps = [
            sample_stamps[len(sample_stamps) // 4],
            sample_stamps[len(sample_stamps) // 2],
            sample_stamps[3 * len(sample_stamps) // 4],
        ]
        fault_run = work / "fault-run"
        make_fault_run(run, fault_run, fault_stamps, joints)
        fault_meta = json.loads((fault_run / "meta.json").read_text())
        t0 = sample_stamps[0] + 5_000
        app_run = work / "app-run"
        app_fixture = make_application_run(app_run, fault_meta, t0)

        fault_read = subprocess.run(
            [str(reader), "--run", str(fault_run)],
            capture_output=True,
            text=True,
            check=False,
        )
        if fault_read.returncode != 0:
            fail(f"the fault run was not readable: {fault_read.stderr.strip()}")
        else:
            fault_lines = [
                line for line in fault_read.stdout.splitlines() if line.strip()
            ]
            fault_records = [json.loads(line) for line in fault_lines[:-1]]
            fault_summary = json.loads(fault_lines[-1])
            fault_rows = [
                row for row in fault_records if row.get("record_type") == "fault"
            ]
            fault_stats = fault_summary.get("faults") or {}
            if fault_summary.get("record_kind") != "runtime":
                fail("read_record did not dispatch the fault run")
            if fault_stats.get("complete") is not True:
                fail(f"the summary does not mark the fault stream complete: {fault_stats}")
            if int(fault_stats.get("rows_in_range", -1)) != 3:
                fail(f"the summary reports {fault_stats.get('rows_in_range')} fault rows")
            if int(fault_stats.get("missing_occurrences", -1)) != 0:
                fail(f"the summary reports missing fault occurrences: {fault_stats}")
            if len(fault_rows) != 3:
                fail(f"the reader printed {len(fault_rows)} fault rows, expected 3")
            else:
                if [row["condition_name"] for row in fault_rows] != [
                    "JointDisconnected",
                    "JointFeedbackInvalid",
                    "JointFeedbackNonFinite",
                ]:
                    fail("the fault condition names did not survive the reader")
                if [row["joint_index"] for row in fault_rows] != [0, None, 1]:
                    fail(f"the fault joint indices are {[row['joint_index'] for row in fault_rows]}")
                if [row["observed_value"] for row in fault_rows] != [
                    "NaN",
                    "+Inf",
                    "-Inf",
                ]:
                    fail("the reader did not preserve the explicit non-finite fault values")
                if [row["output_action"] for row in fault_rows] != [0, 0, 0]:
                    fail("the fault output actions did not survive the reader")

        app_read = subprocess.run(
            [str(reader), "--run", str(app_run)],
            capture_output=True,
            text=True,
            check=False,
        )
        if app_read.returncode != 0:
            fail(f"the application run was not readable: {app_read.stderr.strip()}")
        else:
            app_lines = [
                line for line in app_read.stdout.splitlines() if line.strip()
            ]
            app_records = [json.loads(line) for line in app_lines[:-1]]
            app_summary = json.loads(app_lines[-1])
            velocity_rows = [
                row for row in app_records if row.get("record_type") == "velocity"
            ]
            policy_rows = [
                row
                for row in app_records
                if row.get("record_type") == "policy_session"
            ]
            if app_summary.get("record_kind") != "application":
                fail("read_record did not dispatch the application run")
            if app_summary.get("complete") is not True or app_summary.get(
                "truncated"
            ):
                fail(f"the application run is not a complete stream: {app_summary}")
            if int(app_summary.get("rows_in_range", -1)) != len(app_fixture["rows"]):
                fail(
                    f"the application reader returned {app_summary.get('rows_in_range')} "
                    f"rows, expected {len(app_fixture['rows'])}"
                )
            if len(velocity_rows) != len(app_fixture["rows"]):
                fail("the velocity rows did not survive the reader")
            if int(app_summary.get("policy_sessions", -1)) != len(policy_rows):
                fail("the policy-session count does not match the printed rows")
            if str(app_summary.get("model", {}).get("name")) != app_fixture["model_name"]:
                fail(
                    "the application metadata lost its UTF-8 model file name: "
                    f"{app_summary.get('model')}"
                )

        # A window of the application sidecar follows the same interval rules
        # as every other stream: a mid-range window returns the sessions in it,
        # and an empty or inverted range returns none rather than the whole
        # sidecar.
        policy_stamp = app_fixture["t0"]
        for label, window_start, window_end, expected_policy in (
            ("mid-range", policy_stamp, policy_stamp + 1, 1),
            ("empty", policy_stamp, policy_stamp, 0),
            ("inverted", policy_stamp + 100, policy_stamp, 0),
        ):
            windowed = subprocess.run(
                [
                    str(reader),
                    "--run",
                    str(app_run),
                    "--from-us",
                    str(window_start),
                    "--to-us",
                    str(window_end),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if windowed.returncode != 0:
                fail(f"the {label} application window failed: {windowed.stderr.strip()}")
                continue
            windowed_lines = [
                line for line in windowed.stdout.splitlines() if line.strip()
            ]
            windowed_records = [json.loads(line) for line in windowed_lines[:-1]]
            windowed_summary = json.loads(windowed_lines[-1])
            returned_policy = [
                row
                for row in windowed_records
                if row.get("record_type") == "policy_session"
            ]
            if len(returned_policy) != expected_policy:
                fail(
                    f"the {label} window returned {len(returned_policy)} policy "
                    f"session(s), expected {expected_policy}"
                )
            if int(windowed_summary.get("policy_sessions", -1)) != expected_policy:
                fail(
                    f"the {label} window summary reports "
                    f"{windowed_summary.get('policy_sessions')} policy session(s), "
                    f"expected {expected_policy}"
                )
            if expected_policy == 0 and int(
                windowed_summary.get("rows_in_range", -1)
            ) != 0:
                fail(f"the {label} window returned velocity rows")

        associated_path = work / "associated.npz"
        associated = subprocess.run(
            [
                sys.executable,
                str(source_root / "tools" / "record_to_npz.py"),
                str(fault_run),
                str(associated_path),
                "--reader",
                str(reader),
                "--application-record",
                str(app_run),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if associated.returncode != 0:
            fail(f"the associated conversion failed: {associated.stderr.strip()}")
        else:
            linked = np.load(associated_path, allow_pickle=False)
            if not bool(linked["association_ok"]):
                fail(
                    "the application run did not associate: "
                    f"{str(linked['association_reason'])}"
                )
            if int(linked["application_rows_total"]) != len(app_fixture["rows"]):
                fail("the archive does not report the application's total row count")
            if int(linked["application_rows_excluded"]) != app_fixture["excluded"]:
                fail(
                    "the archive reports "
                    f"{int(linked['application_rows_excluded'])} excluded rows, "
                    f"expected {app_fixture['excluded']}"
                )
            if int(linked["velocity_stamp_us"].shape[0]) != app_fixture["selected"]:
                fail(
                    "the archive carries "
                    f"{int(linked['velocity_stamp_us'].shape[0])} velocity rows, "
                    f"expected {app_fixture['selected']}"
                )
            for column, dtype in (
                ("velocity_stamp_us", np.uint64),
                ("velocity_sample_seq", np.uint64),
                ("velocity_identity_known", np.bool_),
                ("velocity_latest_state_seq", np.uint64),
                ("velocity_forward", np.float64),
                ("velocity_lateral", np.float64),
                ("velocity_yaw", np.float64),
            ):
                if linked[column].dtype != dtype:
                    fail(
                        f"the archive's {column} is {linked[column].dtype}, "
                        f"expected {dtype}"
                    )
                if linked[column].shape != (app_fixture["selected"],):
                    fail(f"the archive's {column} has shape {linked[column].shape}")
            if set(str(value) for value in linked["velocity_runtime_id"]) != {
                RUNTIME_ID
            }:
                fail("the archive did not keep only the matching runtime identity")
            if not bool(linked["application_complete"]) or bool(
                linked["application_truncated"]
            ):
                fail("the archive does not carry the application completion facts")
            if int(linked["application_sample_hz"]) != 100:
                fail("the archive does not carry the application sample rate")
            if str(linked["application_model_name"]) != app_fixture["model_name"]:
                fail(
                    "the archive lost the UTF-8 model file name: "
                    f"{str(linked['application_model_name'])!r}"
                )
            if str(linked["application_schema"]) != "irmv.application_velocity":
                fail("the archive does not carry the application schema")
            if int(linked["policy_session_count"]) != 1:
                fail("the archive does not carry the policy-session sidecar")
            if "policy_session_json" not in linked:
                fail("the archive has no policy_session_json column")
            elif app_fixture["model_name"] not in str(linked["policy_session_json"][0]):
                fail("the policy-session text lost its UTF-8 model name")

            # Hold-forward is aligned to the runtime rows and bounded to the
            # contiguous segments: the gap between the loss and the next
            # recorded sample is NaN, never the last value carried across it.
            hold = linked["velocity_hold_forward"]
            if hold.shape != (rows_written,) or hold.dtype != np.float64:
                fail(
                    f"velocity_hold_forward is {hold.shape}/{hold.dtype}, "
                    f"expected ({rows_written},)/float64"
                )
            if linked["velocity_hold_lateral"].shape != (rows_written,):
                fail("velocity_hold_lateral is not aligned with the runtime rows")
            if linked["velocity_hold_yaw"].shape != (rows_written,):
                fail("velocity_hold_yaw is not aligned with the runtime rows")
            gap_target = app_fixture["gap_start"] + 50_000
            gap_index = int(
                np.searchsorted(
                    data["stamp_us"], np.uint64(gap_target), side="right"
                )
                - 1
            )
            if 0 <= gap_index < rows_written:
                if not np.isnan(hold[gap_index]):
                    fail(
                        "the held velocity crossed the recorded gap at "
                        f"{int(data['stamp_us'][gap_index])}"
                    )
            finite_target = app_fixture["t0"] + 50_000
            finite_index = int(
                np.searchsorted(
                    data["stamp_us"], np.uint64(finite_target), side="right"
                )
                - 1
            )
            if 0 <= finite_index < rows_written:
                if np.isnan(hold[finite_index]):
                    fail("a runtime row inside a contiguous segment holds no velocity")
                else:
                    applied_seq = (
                        int(data["stamp_us"][finite_index]) - app_fixture["t0"]
                    ) // 10_000
                    expected_forward = 0.01 * (applied_seq + 1)
                    if abs(float(hold[finite_index]) - expected_forward) > 1e-9:
                        fail(
                            "the held forward value is "
                            f"{float(hold[finite_index])}, expected "
                            f"{expected_forward} from the sample in force"
                        )

            # A window that starts between two velocity samples still holds the
            # sample before it: association and hold-forward see the whole
            # application recording, while the exported rows are the window's.
            sample_window_start = app_fixture["t0"] + 55_000
            sample_window_end = app_fixture["t0"] + 85_000
            sample_window_path = work / "sample-window.npz"
            sample_window = subprocess.run(
                [
                    sys.executable,
                    str(source_root / "tools" / "record_to_npz.py"),
                    str(fault_run),
                    str(sample_window_path),
                    "--reader",
                    str(reader),
                    "--application-record",
                    str(app_run),
                    "--from-us",
                    str(sample_window_start),
                    "--to-us",
                    str(sample_window_end),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if sample_window.returncode != 0:
                fail(
                    "the mid-segment window conversion failed: "
                    f"{sample_window.stderr.strip()}"
                )
            else:
                windowed = np.load(sample_window_path, allow_pickle=False)
                if not bool(windowed["association_ok"]):
                    fail(
                        "a window starting between samples lost the association: "
                        f"{str(windowed['association_reason'])}"
                    )
                exported = [int(value) for value in windowed["velocity_stamp_us"]]
                if any(
                    value < sample_window_start or value >= sample_window_end
                    for value in exported
                ):
                    fail("the mid-segment window exported a velocity row out of range")
                if app_fixture["t0"] + 50_000 in exported:
                    fail("the mid-segment window exported the sample before its start")
                first_index = int(
                    np.searchsorted(
                        windowed["stamp_us"],
                        np.uint64(sample_window_start),
                        side="left",
                    )
                )
                if first_index >= int(windowed["stamp_us"].shape[0]):
                    fail("the mid-segment window selected no runtime row")
                else:
                    stamp = int(windowed["stamp_us"][first_index])
                    applied_seq = (stamp - app_fixture["t0"]) // 10_000
                    expected_forward = 0.01 * (applied_seq + 1)
                    held = float(windowed["velocity_hold_forward"][first_index])
                    if np.isnan(held) or abs(held - expected_forward) > 1e-9:
                        fail(
                            "a window starting between samples did not hold the "
                            f"preceding value: {held} != {expected_forward}"
                        )

            # A window narrower than one period may contain no velocity row at
            # all. Association is still a fact about the full recordings, and
            # the runtime rows in it still hold the sample in force.
            narrow_start = app_fixture["t0"] + 51_000
            narrow_end = app_fixture["t0"] + 59_000
            narrow_path = work / "narrow-window.npz"
            narrow = subprocess.run(
                [
                    sys.executable,
                    str(source_root / "tools" / "record_to_npz.py"),
                    str(fault_run),
                    str(narrow_path),
                    "--reader",
                    str(reader),
                    "--application-record",
                    str(app_run),
                    "--from-us",
                    str(narrow_start),
                    "--to-us",
                    str(narrow_end),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if narrow.returncode != 0:
                fail(f"the empty-velocity window failed: {narrow.stderr.strip()}")
            else:
                narrow_data = np.load(narrow_path, allow_pickle=False)
                if not bool(narrow_data["association_ok"]):
                    fail(
                        "a window with no velocity row refused the association: "
                        f"{str(narrow_data['association_reason'])}"
                    )
                if int(narrow_data["velocity_stamp_us"].shape[0]) != 0:
                    fail("the narrow window exported velocity rows")
                if int(narrow_data["application_rows_total"]) != 0:
                    fail("the narrow window reports velocity rows in its total")
                first_index = int(
                    np.searchsorted(
                        narrow_data["stamp_us"],
                        np.uint64(narrow_start),
                        side="left",
                    )
                )
                if first_index >= int(narrow_data["stamp_us"].shape[0]):
                    fail("the narrow window selected no runtime row")
                else:
                    stamp = int(narrow_data["stamp_us"][first_index])
                    applied_seq = (stamp - app_fixture["t0"]) // 10_000
                    expected_forward = 0.01 * (applied_seq + 1)
                    held = float(narrow_data["velocity_hold_forward"][first_index])
                    if np.isnan(held) or abs(held - expected_forward) > 1e-9:
                        fail(
                            "the narrow window did not hold the preceding value: "
                            f"{held} != {expected_forward}"
                        )

            # The fault arrays have their own length, their own dtypes, and the
            # explicit non-finite values the file declared.
            for column, dtype in (
                ("fault_record_seq", np.uint64),
                ("fault_state_seq", np.uint64),
                ("fault_fault_id", np.uint64),
                ("fault_reason", np.uint32),
                ("fault_condition", np.uint32),
                ("fault_joint_index", np.int64),
                ("fault_error_code", np.uint64),
                ("fault_observed_value", np.float64),
                ("fault_limit", np.float64),
                ("fault_sample_stamp_us", np.uint64),
                ("fault_detected_stamp_us", np.uint64),
                ("fault_output_action", np.uint32),
            ):
                if linked[column].dtype != dtype or linked[column].shape != (3,):
                    fail(
                        f"the archive's {column} is {linked[column].dtype}/"
                        f"{linked[column].shape}, expected {dtype}/(3,)"
                    )
            if [int(value) for value in linked["fault_joint_index"]] != [0, -1, 1]:
                fail("joint-less faults did not become the int64 sentinel -1")
            if [str(value) for value in linked["fault_condition_name"]] != [
                "JointDisconnected",
                "JointFeedbackInvalid",
                "JointFeedbackNonFinite",
            ]:
                fail("the archive lost a fault condition name")
            if not np.isnan(linked["fault_observed_value"][0]):
                fail("the archive folded a NaN fault value into another number")
            if not np.isposinf(linked["fault_observed_value"][1]):
                fail("the archive folded +Inf into NaN")
            if not np.isneginf(linked["fault_observed_value"][2]):
                fail("the archive folded -Inf into NaN")
            if [str(value) for value in linked["fault_runtime_id"]] != [
                RUNTIME_ID,
                RUNTIME_ID,
                RUNTIME_ID,
            ]:
                fail("the fault runtime identity did not survive the export")
            # No column is an object array: loading with allow_pickle=False is
            # part of the contract, and a fault detail is never pickled.
            for column in linked.files:
                if linked[column].dtype == np.dtype("O"):
                    fail(f"the archive exported {column} as an object array")

        # An empty application run still exports explicit (0,) columns, and the
        # missing association is reported rather than silently treated as a
        # perfect join.
        empty_app = work / "app-empty"
        empty_app.mkdir()
        write_json(empty_app / "meta.json", json.loads((app_run / "meta.json").read_text()))
        write_json(
            empty_app / "index.json",
            {
                "schema": "irmv.application_velocity",
                "version": 1,
                "complete": True,
                "write_failures": 0,
                "losses": [],
            },
        )
        (empty_app / "velocity.jsonl").write_text("", encoding="utf-8")
        empty_path = work / "empty-associated.npz"
        empty_convert = subprocess.run(
            [
                sys.executable,
                str(source_root / "tools" / "record_to_npz.py"),
                str(fault_run),
                str(empty_path),
                "--reader",
                str(reader),
                "--application-record",
                str(empty_app),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if empty_convert.returncode != 0:
            fail(f"the empty application conversion failed: {empty_convert.stderr.strip()}")
        else:
            empty_linked = np.load(empty_path, allow_pickle=False)
            if bool(empty_linked["association_ok"]):
                fail("an application run with no rows was accepted as associated")
            if "no application" not in str(empty_linked["association_reason"]):
                fail(
                    "the empty association does not name its cause: "
                    f"{str(empty_linked['association_reason'])!r}"
                )
            if int(empty_linked["application_rows_total"]) != 0:
                fail("the empty application run reports rows")
            for column in (
                "velocity_stamp_us",
                "velocity_sample_seq",
                "velocity_identity_known",
                "velocity_runtime_id",
                "velocity_forward",
            ):
                if empty_linked[column].shape != (0,):
                    fail(
                        f"an empty application exported {column} with shape "
                        f"{empty_linked[column].shape}"
                    )
            if not np.all(np.isnan(empty_linked["velocity_hold_forward"])):
                fail("an empty application exported a held value")
            if int(empty_linked["fault_record_seq"].shape[0]) != 3:
                fail("the fault array changed length when the application was empty")

        # Unknown and malformed inputs are refused with a nonzero exit, never
        # reinterpreted. A missing prerequisite is not a pass.
        unknown_runtime = work / "unknown-runtime"
        unknown_runtime.mkdir()
        write_json(unknown_runtime / "meta.json", {"schema_version": 99})
        result = subprocess.run(
            [str(reader), "--run", str(unknown_runtime)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            fail("an unknown runtime schema version was accepted")
        unknown_app = work / "unknown-app"
        unknown_app.mkdir()
        write_json(
            unknown_app / "meta.json",
            {"schema": "irmv.application_velocity", "version": 2},
        )
        result = subprocess.run(
            [str(reader), "--run", str(unknown_app)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            fail("an unknown application schema version was accepted")

        # A declared schema name is authoritative. In particular, an unknown
        # schema that also carries a numeric `schema_version` must not be
        # reinterpreted as a runtime run of this build's chunk format.
        for name, meta_document in (
            ("unknown-schema-plain", {"schema": "irmv.bogus", "version": 1}),
            (
                "unknown-schema-versioned",
                {"schema": "irmv.bogus", "schema_version": 4},
            ),
        ):
            directory = work / name
            directory.mkdir()
            write_json(directory / "meta.json", meta_document)
            result = subprocess.run(
                [str(reader), "--run", str(directory)],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode == 0:
                fail(f"the viewer accepted the unknown schema in {name}")
            elif "irmv.bogus" not in result.stderr:
                fail(
                    f"the viewer's refusal for {name} does not name the schema: "
                    f"{result.stderr.strip()}"
                )
            archive = work / f"{name}.npz"
            export = subprocess.run(
                [
                    sys.executable,
                    str(source_root / "tools" / "record_to_npz.py"),
                    str(directory),
                    str(archive),
                    "--reader",
                    str(reader),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if export.returncode == 0:
                fail(f"the exporter accepted the unknown schema in {name}")
            if archive.exists():
                fail(f"the exporter wrote an archive for the unknown schema in {name}")

        corrupt_app = work / "corrupt-app"
        shutil.copytree(app_run, corrupt_app)
        with (corrupt_app / "velocity.jsonl").open("a", encoding="utf-8") as stream:
            stream.write('{"schema": "irmv.application_velocity", "version": 1\n')
        result = subprocess.run(
            [str(reader), "--run", str(corrupt_app)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            fail("a malformed complete velocity line was accepted")
        elif "record_to_npz" in result.stderr:
            fail("the read tool blamed the converter for a malformed line")

        corrupt_policy = work / "corrupt-policy"
        shutil.copytree(app_run, corrupt_policy)
        with (corrupt_policy / "policy_sessions.jsonl").open(
            "a", encoding="utf-8"
        ) as stream:
            stream.write('{"schema": "irmv.policy_session"\n')
        result = subprocess.run(
            [str(reader), "--run", str(corrupt_policy)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            fail("a malformed complete policy-session line was accepted")

        corrupt_fault = work / "corrupt-fault"
        shutil.copytree(fault_run, corrupt_fault)
        with (corrupt_fault / "faults.jsonl").open("a", encoding="utf-8") as stream:
            stream.write('{"schema_version": 4\n')
        result = subprocess.run(
            [str(reader), "--run", str(corrupt_fault)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            fail("a malformed complete fault line was accepted")

    if failures:
        return 1
    print(
        f"record_export_test: {rows_written} rows, {joints} joints, "
        f"{len(index['gaps'])} gap(s), exported and windowed"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
