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

The tool only reads: it opens no DDS endpoint and publishes nothing.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

# The scalar columns of one row, as `read_record` prints them.
SCALAR_COLUMNS = (
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
)

# The joint columns, each of shape (rows, joints).
JOINT_COLUMNS = ("q", "dq", "tau_ff", "kp", "kd")

GAP_REASONS = {0: "none", 1: "queue_full", 2: "write_failed", 3: "stopped"}


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
    for column in SCALAR_COLUMNS:
        values = [row[column] for row in rows]
        arrays[column] = np.array(values, dtype=np.float64 if column == "stamp_us" else None)
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
        [GAP_REASONS.get(int(gap["reason"]), "unknown") for gap in gaps]
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
