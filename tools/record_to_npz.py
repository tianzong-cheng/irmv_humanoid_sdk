#!/usr/bin/env python3
"""Convert one recorded run into a NumPy archive.

    record_to_npz.py <run-directory> <out.npz> [--reader <path-to-read_record>]

The conversion goes through ``tools/read_record``, the same tool an operator
would use: the archive is built from the reader's own output rather than from a
second implementation of the chunk format, so the two cannot disagree about what
a run contains.

What the archive preserves, because a consumer needs all of it to interpret the
numbers: every column by name, the joint names in the order the joint columns
use, the units, the schema version, the configuration and layout digests, the
sampling and control rates, and the gap table - a run's missing samples are part
of its data, not a footnote.

The dtype of a column is part of that contract rather than something numpy
infers. A stamp, a sequence number or a gap count is a 64-bit unsigned count and
stays ``uint64``: a microsecond stamp above 2**53, which a ``float64`` could not
hold to the microsecond, survives exactly. A flag stays ``bool``, the enum
columns stay the ``uint32`` the reader prints, and only the joint columns and
the control rate are floating point. The 16-byte ``producer_id`` is an identity
rather than a number: it crosses as the reader's own 32 lower-case hex digits,
one ``U32`` string per row, so every bit of it survives and two rows are
comparable as text. ``units`` is stored as the JSON text of the units object the
run declared, copied from the reader's summary rather than re-derived, so a
consumer reads exactly what the run said about its own numbers.

The tool only reads: it opens no DDS endpoint and publishes nothing.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

# The scalar columns of one row, as `read_record` prints them, with the dtype
# each one must keep in the archive. Nothing is inferred: numpy's default for a
# list of JSON integers is a signed width, and a stamp the run recorded as
# unsigned must not come back rounded or sign-wrapped.
SCALAR_DTYPES = {
    "stamp_us": np.uint64,
    "state_seq": np.uint64,
    "execution_state": np.uint32,
    "output_action": np.uint32,
    "stop_reason": np.uint32,
    "fault_id": np.uint64,
    "healthy": np.bool_,
    "damping_eligible": np.bool_,
    "command_present": np.bool_,
    "epoch": np.uint64,
    "command_seq": np.uint64,
    "source_state_seq": np.uint64,
    "command_rx_us": np.uint64,
    "feedback_seq": np.uint64,
    "feedback_stamp_us": np.uint64,
    "feedback_valid": np.bool_,
    "imu_valid": np.bool_,
    "send_accepted": np.bool_,
    "output_valid": np.bool_,
}

# The producer identity of one row: 16 opaque bytes, not a count. It crosses as
# the reader's own 32 lower-case hex digits - one string per row - because no
# integer or floating width would hold it whole and an identity is compared, not
# computed with.
PRODUCER_COLUMN = "producer_id"
PRODUCER_DTYPE = np.dtype("<U32")

# The joint columns, each of shape (rows, joints). The first five are the
# applied target; the last three are the measured feedback and the output
# stage's torque of the same tick.
JOINT_COLUMNS = (
    "q",
    "dq",
    "tau_ff",
    "kp",
    "kd",
    "measured_q",
    "measured_dq",
    "output_torque",
)

# The session sidecar's columns. A run's session table is one entry per
# authorization context the runtime adopted, so it is a different length from
# the sample columns and is carried as its own set of arrays rather than folded
# into the row-arrays above.
SESSION_SCALAR_DTYPES = {
    "session_runtime_id": np.dtype("<U32"),
    "session_session_id": np.dtype("<U32"),
    "session_producer_id": np.dtype("<U32"),
    "session_epoch": np.uint64,
    "session_stamp_us": np.uint64,
    "session_state_seq": np.uint64,
    "session_joint_count": np.uint32,
    "session_schema_version": np.uint32,
}
SESSION_JOINT_COLUMNS = ("session_kp", "session_kd")
SESSION_DIGEST_DTYPE = np.dtype("<U64")

GAP_REASONS = {0: "none", 1: "queue_full", 2: "write_failed", 3: "stopped"}

# A reason this build does not know about is still reported as a name, and the
# column is wide enough for every name a run can produce even when the run has
# no gap at all: an empty text column that comes back float64 is the same silent
# narrowing as a stamp that comes back rounded.
GAP_REASON_DTYPE = np.dtype(f"<U{max(len(name) for name in (*GAP_REASONS.values(), 'unknown'))}")


def read_run(reader: Path, run_directory: Path, begin_us: int, end_us: int):
    """Return (rows, summary) for the range, straight from the reader tool."""

    command = [str(reader), "--run", str(run_directory)]
    if begin_us or end_us is not None:
        command += ["--from-us", str(begin_us)]
        if end_us is not None:
            command += ["--to-us", str(end_us)]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise SystemExit(f"record_to_npz: read_record failed: {completed.stderr.strip()}")
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    if not lines:
        raise SystemExit("record_to_npz: the reader printed nothing")
    summary = json.loads(lines[-1])
    rows = [json.loads(line) for line in lines[:-1]]
    if len(rows) != summary["rows_in_range"]:
        raise SystemExit(
            f"record_to_npz: the reader printed {len(rows)} rows but its summary reports "
            f"{summary['rows_in_range']}"
        )
    return rows, summary


def to_archive(rows, summary) -> dict:
    count = len(rows)
    arrays = {}
    for column, dtype in SCALAR_DTYPES.items():
        # An empty run still has the column, so it is built from the dtype
        # alone: its shape says how many rows there are even when there are
        # none.
        arrays[column] = np.array([row[column] for row in rows], dtype=dtype)
    arrays[PRODUCER_COLUMN] = np.array(
        [row[PRODUCER_COLUMN] for row in rows], dtype=PRODUCER_DTYPE
    )
    joint_count = int(summary["joint_count"])
    for column in JOINT_COLUMNS:
        if count == 0:
            arrays[column] = np.zeros((0, joint_count), dtype=np.float64)
            continue
        matrix = np.array([row[column] for row in rows], dtype=np.float64)
        if matrix.shape != (count, joint_count):
            raise SystemExit(
                f"record_to_npz: the {column} column has shape {matrix.shape}, expected "
                f"({count}, {joint_count})"
            )
        arrays[column] = matrix

    # The run's own units, as the object the metadata declared: a missing table
    # is a failure rather than a quietly absent field, because a consumer that
    # cannot tell a radian from a newton-metre cannot use the numbers.
    if "units" not in summary:
        raise SystemExit("record_to_npz: the reader reported no units for this run")
    arrays["units"] = np.array(json.dumps(summary["units"], sort_keys=True))

    arrays["joint_names"] = np.array(summary["joints"])
    arrays["schema_version"] = np.int64(summary["schema_version"])
    arrays["sample_hz"] = np.int64(summary["sample_hz"])
    arrays["control_hz"] = np.float64(summary["control_hz"])
    arrays["config_id"] = np.array(summary["config_id"])
    arrays["layout_id"] = np.array(summary["layout_id"])
    arrays["backend"] = np.array(summary["backend"])
    arrays["truncated"] = np.bool_(summary["truncated"])

    gaps = summary["gaps"]
    arrays["gap_first_seq"] = np.array([gap["first_seq"] for gap in gaps], dtype=np.uint64)
    arrays["gap_last_seq"] = np.array([gap["last_seq"] for gap in gaps], dtype=np.uint64)
    arrays["gap_dropped"] = np.array([gap["dropped"] for gap in gaps], dtype=np.uint64)
    arrays["gap_reason"] = np.array(
        [GAP_REASONS.get(int(gap["reason"]), "unknown") for gap in gaps],
        dtype=GAP_REASON_DTYPE,
    )

    # The session sidecar: one row per authorization context the runtime
    # adopted. It is a different length from the sample rows, so it is carried
    # as its own arrays; a run with no session still has each column, with an
    # empty shape. The digest is text and kept verbatim.
    sessions = summary.get("sessions", [])
    for column, dtype in SESSION_SCALAR_DTYPES.items():
        source = column[len("session_") :]
        arrays[column] = np.array([entry[source] for entry in sessions], dtype=dtype)
    for column in SESSION_JOINT_COLUMNS:
        source = column[len("session_") :]
        if len(sessions) == 0:
            arrays[column] = np.zeros((0, joint_count), dtype=np.float64)
            continue
        matrix = np.array([entry[source] for entry in sessions], dtype=np.float64)
        if matrix.shape != (len(sessions), joint_count):
            raise SystemExit(
                f"record_to_npz: the {column} column has shape {matrix.shape}, expected "
                f"({len(sessions)}, {joint_count})"
            )
        arrays[column] = matrix
    arrays["session_model_sha256"] = np.array(
        [entry["model_sha256"] for entry in sessions], dtype=SESSION_DIGEST_DTYPE
    )
    return arrays


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("out_path", type=Path)
    parser.add_argument(
        "--reader",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "build" / "read_record",
        help="the read_record tool to consume (default: build/read_record)",
    )
    parser.add_argument("--from-us", type=int, default=0)
    parser.add_argument("--to-us", type=int, default=None)
    args = parser.parse_args()

    if not args.reader.exists():
        raise SystemExit(f"record_to_npz: no reader at {args.reader}; build the read_record target")
    rows, summary = read_run(args.reader, args.run_directory, args.from_us, args.to_us)
    arrays = to_archive(rows, summary)
    np.savez_compressed(args.out_path, **arrays)
    print(
        f"record_to_npz: {summary['rows_in_range']} rows, {summary['joint_count']} joints, "
        f"{len(summary['gaps'])} gap(s), schema {summary['schema_version']} -> {args.out_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
