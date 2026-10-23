#!/usr/bin/env python3
"""Convert one recorded run into a NumPy archive.

    record_to_npz.py <run-directory> <out.npz> [--reader <path-to-read_record>]
                     [--from-us N] [--to-us N]
                     [--application-record <app_run-directory>]

The conversion goes through ``tools/read_record``, the same tool an operator
would use: the archive is built from the reader's own output rather than from a
second implementation of the chunk format, so the two cannot disagree about what
a run contains.

What the archive preserves, because a consumer needs all of it to interpret the
numbers: every column by name, the joint names in the order the joint columns
use, the units, the schema version, the configuration and layout digests, the
sampling and control rates, the truncation and corruption flags the reader set,
the reader's completion flag (a killed run is unfinished even when its committed
chunks read cleanly), and the gap table - a run's missing samples are part of
its data, not a footnote.

The dtype of a column is part of that contract rather than something numpy
infers. A stamp, a sequence number or a gap count is a 64-bit unsigned count and
stays ``uint64``: a microsecond stamp above 2**53, which a ``float64`` could not
hold to the microsecond, survives exactly. A flag stays ``bool`` - including
the per-joint ``joint_valid`` matrix - the enum columns stay the ``uint32`` the
reader prints, and only the joint columns, the IMU vectors and the control rate
are floating point. The 16-byte ``producer_id`` is an identity
rather than a number: it crosses as the reader's own 32 lower-case hex digits,
one ``U32`` string per row, so every bit of it survives and two rows are
comparable as text. ``units`` and ``imu_conventions`` are stored as the JSON
text of the objects the run declared, copied from the reader's summary rather
than re-derived, so a consumer reads exactly what the run said about its own
numbers - including an axis frame or gravity convention it marked unknown.

Fault occurrences are events, not samples: ``fault_*`` arrays have their own
length, never the joint row count, and a joint-less fault becomes the int64
sentinel -1 rather than an object array. With ``--application-record``, the
application's independent velocity stream is associated only when both runs
declare the same monotonic clock and boot id and the application carries the
runtime record's session identity; foreign and unknown-identity rows are
excluded and counted, never merged. The selected samples are exported as
``velocity_*`` arrays, the policy-session sidecar as text, and
``velocity_hold_forward/lateral/yaw`` are aligned to the runtime rows. A held
value is valid only inside a contiguous recorded segment (no indexed loss and
no cadence gap), never before the first sample or past one planned period after
the last; every other runtime row is NaN.

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
    "imu_rx_stamp_us": np.uint64,
    "send_accepted": np.bool_,
    "output_valid": np.bool_,
}

# The producer identity of one row: 16 opaque bytes, not a count. It crosses as
# the reader's own 32 lower-case hex digits - one string per row - because no
# integer or floating width would hold it whole and an identity is compared, not
# computed with.
PRODUCER_COLUMN = "producer_id"
PRODUCER_DTYPE = np.dtype("<U32")

# The IMU vectors, each of shape (rows, 3) in the driver contract's axis
# order. They are floating point like the joint columns, never a string or an
# object: the archive carries the numbers the run recorded.
IMU_COLUMNS = (
    "imu_orientation_rpy",
    "imu_angular_velocity",
    "imu_linear_acceleration",
)

# The joint columns, each of shape (rows, joints). The first five are the
# applied target; then the measured feedback and the output stage's torque of
# the same tick, with the torque estimate appended as the ninth column.
JOINT_COLUMNS = (
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

# The per-joint validity flag, shape (rows, joints), one bool per value. It is
# its own column group because a bool matrix is not a float matrix: numpy would
# silently promote one to the other if the dtype were inferred.
JOINT_BOOL_COLUMNS = ("joint_valid",)

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

# The clock every recorded stamp in both streams must come from, and the two
# facts that make two independent runs comparable. A run whose identity could
# not be read is never merged automatically.
CLOCK_NAME = "linux_clock_monotonic"

# The fault columns. A fault is an event, not a sample: its arrays have their
# own length and are never folded into the joint row count. `joint_index` has
# no value for a fault that is not about one joint, and JSON null becomes the
# int64 sentinel -1 rather than an object array or a missing value.
FAULT_SCALAR_DTYPES = {
    "fault_record_seq": np.uint64,
    "fault_state_seq": np.uint64,
    "fault_fault_id": np.uint64,
    "fault_reason": np.uint32,
    "fault_condition": np.uint32,
    "fault_joint_index": np.int64,
    "fault_error_code": np.uint64,
    "fault_observed_value": np.float64,
    "fault_limit": np.float64,
    "fault_sample_stamp_us": np.uint64,
    "fault_detected_stamp_us": np.uint64,
    "fault_output_action": np.uint32,
}
FAULT_TEXT_COLUMNS = {
    "fault_runtime_id": 32,
    "fault_reason_name": 48,
    "fault_condition_name": 48,
    "fault_message": 512,
    "fault_message_hex": 512,
}

# The velocity columns of the application stream the runtime run was joined to.
# They are independent of the sample rows and are carried only for the rows
# that match the runtime record's own identity.
VELOCITY_SCALAR_DTYPES = {
    "velocity_stamp_us": np.uint64,
    "velocity_sample_seq": np.uint64,
    "velocity_identity_known": np.bool_,
    "velocity_latest_state_seq": np.uint64,
    "velocity_forward": np.float64,
    "velocity_lateral": np.float64,
    "velocity_yaw": np.float64,
}
VELOCITY_TEXT_COLUMNS = {
    "velocity_runtime_id": 32,
    "velocity_session_id": 32,
}


def _read_output(reader: Path, run_directory: Path, begin_us: int, end_us: int):
    """Return (records, summary) for the range, straight from the reader tool."""

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
    records = []
    for line in lines[:-1]:
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as error:
            raise SystemExit(f"record_to_npz: the reader printed a malformed line: {error}")
    return records, summary


def read_run(reader: Path, run_directory: Path, begin_us: int, end_us: int):
    """Return (sample_rows, summary) for a runtime range.

    The runtime output also carries fault occurrences, which are events rather
    than sampled rows; they are not part of the joint row table.
    """

    records, summary = _read_output(reader, run_directory, begin_us, end_us)
    rows = [row for row in records if row.get("record_type", "sample") == "sample"]
    if len(rows) != summary["rows_in_range"]:
        raise SystemExit(
            f"record_to_npz: the reader printed {len(rows)} sample(s) but its summary "
            f"reports {summary['rows_in_range']}"
        )
    return rows, summary


def read_experiment(
    reader: Path,
    run_directory: Path,
    begin_us: int,
    end_us: int,
    application_directory: Path | None = None,
) -> dict:
    """Read the runtime run and, optionally, the application run to join to it.

    The returned mapping has `samples`, `faults`, `summary` and - when an
    application directory was supplied - `application` with its `velocity` rows,
    `policy_sessions` lines and summary. Neither stream is reinterpreted here:
    both come from `read_record`, whose own schema checks are authoritative.
    """

    records, summary = _read_output(reader, run_directory, begin_us, end_us)
    samples = [row for row in records if row.get("record_type", "sample") == "sample"]
    faults = [row for row in records if row.get("record_type") == "fault"]
    if len(samples) != summary["rows_in_range"]:
        raise SystemExit(
            f"record_to_npz: the reader printed {len(samples)} sample(s) but its summary "
            f"reports {summary['rows_in_range']}"
        )
    if "faults" in summary and len(faults) != summary["faults"].get("rows_in_range"):
        raise SystemExit(
            "record_to_npz: the reader printed "
            f"{len(faults)} fault row(s) but its summary reports "
            f"{summary['faults'].get('rows_in_range')}"
        )

    result = {"samples": samples, "faults": faults, "summary": summary}
    if application_directory is None:
        result["application"] = None
        return result

    app_records, app_summary = _read_output(
        reader, application_directory, begin_us, end_us
    )
    velocity = [
        row for row in app_records if row.get("record_type") == "velocity"
    ]
    policy = [
        row for row in app_records if row.get("record_type") == "policy_session"
    ]
    if len(velocity) != app_summary["rows_in_range"]:
        raise SystemExit(
            "record_to_npz: the application reader printed "
            f"{len(velocity)} velocity row(s) but its summary reports "
            f"{app_summary['rows_in_range']}"
        )
    result["application"] = {
        "summary": app_summary,
        "velocity": velocity,
        "policy_sessions": policy,
    }
    return result


def _clock_identity(summary: dict, label: str) -> tuple[str | None, str | None]:
    """Return (clock, boot_id) or (None, reason) for one run's metadata."""

    clock = summary.get("clock")
    if not isinstance(clock, dict):
        return None, f"the {label} metadata carries no clock identity"
    if clock.get("clock") != CLOCK_NAME:
        return None, (
            f"the {label} run declares clock {clock.get('clock')!r}, "
            f"not {CLOCK_NAME!r}"
        )
    if not clock.get("available"):
        return None, f"the {label} clock identity is unavailable"
    boot_id = clock.get("boot_id")
    if not isinstance(boot_id, str) or not boot_id:
        return None, f"the {label} metadata carries no boot id"
    return boot_id, None


def associate(
    runtime_summary: dict, application_summary: dict, application_rows: list[dict]
) -> dict:
    """Select the application rows that may join the runtime run.

    Automatic association is refused - with the reason named - unless both runs
    come from the same monotonic clock and the same boot, the runtime record
    names a session identity, and at least one application row carries that
    identity. Rows of another producer instance, or rows whose identity is not
    known, are excluded and counted: they are never merged and never silently
    dropped.
    """

    runtime_boot, reason = _clock_identity(runtime_summary, "runtime")
    application_boot = None
    if reason is None:
        application_boot, reason = _clock_identity(application_summary, "application")
    if reason is None and runtime_boot != application_boot:
        reason = (
            "the runs come from different boots: "
            f"{runtime_boot!r} and {application_boot!r}"
        )
    if reason is not None:
        return {
            "ok": False,
            "reason": reason,
            "rows": [],
            "excluded": len(application_rows),
            "total": len(application_rows),
        }

    identities: set[tuple[str, str]] = set()
    for session in runtime_summary.get("sessions", []):
        runtime_id = session.get("runtime_id")
        session_id = session.get("session_id")
        if (
            isinstance(runtime_id, str)
            and runtime_id
            and isinstance(session_id, str)
            and session_id
        ):
            identities.add((runtime_id, session_id))
    if not identities:
        return {
            "ok": False,
            "reason": "the runtime record names no session identity",
            "rows": [],
            "excluded": len(application_rows),
            "total": len(application_rows),
        }

    selected: list[dict] = []
    excluded = 0
    for row in application_rows:
        identity = (row.get("runtime_id"), row.get("session_id"))
        if row.get("identity_known") and identity in identities:
            selected.append(row)
        else:
            excluded += 1
    if not selected:
        return {
            "ok": False,
            "reason": (
                "no application velocity row matches the runtime identity "
                "(unknown and foreign identities are never associated)"
            ),
            "rows": [],
            "excluded": excluded,
            "total": len(application_rows),
        }
    return {
        "ok": True,
        "reason": "",
        "rows": selected,
        "excluded": excluded,
        "total": len(application_rows),
    }


def _planned_period_us(sample_hz) -> float:
    try:
        frequency = float(sample_hz)
    except (TypeError, ValueError):
        raise SystemExit(f"record_to_npz: invalid application sample rate {sample_hz!r}")
    if frequency <= 0.0:
        raise SystemExit(f"record_to_npz: invalid application sample rate {sample_hz!r}")
    return 1e6 / frequency


def _velocity_segments(rows, period_us, losses):
    """Split rows into contiguous recorded stretches.

    A segment ends at a cadence gap, an identity change, or an indexed loss
    overlapping the interval to the next row. The returned tuple is
    `(first_index, last_index, end_us)`: `end_us` is the last instant that may
    still be served by the segment's last sample, and it never crosses a loss.
    """

    if not rows:
        return []
    stamps = [int(row["stamp_us"]) for row in rows]
    segments = []
    start = 0
    for index in range(len(rows) - 1):
        a = rows[index]
        b = rows[index + 1]
        a_stamp = stamps[index]
        b_stamp = stamps[index + 1]
        end_us = int(round(a_stamp + period_us))
        broken = False
        if b_stamp <= a_stamp:
            broken = True
        elif b_stamp - a_stamp > period_us * 1.5:
            broken = True
        elif (
            a.get("identity_known") != b.get("identity_known")
            or a.get("runtime_id") != b.get("runtime_id")
            or a.get("session_id") != b.get("session_id")
        ):
            broken = True
        else:
            for loss in losses:
                first = int(loss.get("first_stamp_us", 0))
                last = int(loss.get("last_stamp_us", 0))
                if first < b_stamp and last >= a_stamp:
                    # The value may be held only up to the instant the missing
                    # sample would have been due; it never crosses the loss.
                    end_us = min(end_us, max(first, a_stamp))
                    broken = True
                    break
        if broken:
            segments.append((start, index, end_us))
            start = index + 1
    segments.append(
        (start, len(rows) - 1, int(round(stamps[-1] + period_us)))
    )
    return segments


def hold_forward(runtime_rows, velocity_rows, sample_hz, losses=None) -> dict:
    """Hold the latest velocity sample forward onto the runtime rows.

    The pairing is by monotonic stamp only. A runtime stamp before the first
    sample, inside a gap or a lost span, or more than one planned period past
    the last sample of a segment gets NaN: the export never invents a value for
    time the application did not record.
    """

    losses = losses or []
    count = len(runtime_rows)
    result = {
        key: np.full(count, np.nan, dtype=np.float64)
        for key in ("forward", "lateral", "yaw")
    }
    if count == 0 or not velocity_rows:
        return result
    period_us = _planned_period_us(sample_hz)
    runtime_stamps = np.array(
        [int(row["stamp_us"]) for row in runtime_rows], dtype=np.uint64
    )
    for start, end_index, end_us in _velocity_segments(
        velocity_rows, period_us, losses
    ):
        segment = velocity_rows[start : end_index + 1]
        stamps = np.array([int(row["stamp_us"]) for row in segment], dtype=np.uint64)
        if end_us <= int(stamps[0]):
            continue
        mask = (runtime_stamps >= stamps[0]) & (
            runtime_stamps < np.uint64(end_us)
        )
        if not mask.any():
            continue
        indices = np.flatnonzero(mask)
        positions = (
            np.searchsorted(stamps, runtime_stamps[indices], side="right") - 1
        )
        positions = np.clip(positions, 0, len(stamps) - 1)
        deltas = runtime_stamps[indices].astype(np.int64) - stamps[
            positions
        ].astype(np.int64)
        valid = deltas < period_us
        chosen = indices[valid]
        local = positions[valid]
        if chosen.size == 0:
            continue
        for key in ("forward", "lateral", "yaw"):
            values = np.array(
                [float(segment[index][key]) for index in local],
                dtype=np.float64,
            )
            result[key][chosen] = values
    return result


def _decode_value(value, column: str) -> float:
    """Restore one JSON double column, including its explicit non-finite form.

    The reader writes NaN/+Inf/-Inf as strings because JSON cannot carry them.
    They are restored here, before numpy sees the values, so an infinity never
    crosses as a NaN and a missing value can never be confused with either.
    """

    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    if value == "NaN":
        return float("nan")
    if value == "+Inf":
        return float("inf")
    if value == "-Inf":
        return float("-inf")
    raise SystemExit(
        f"record_to_npz: {column} carries {value!r}, which is not a number or "
        "an explicit non-finite value"
    )


def _text_array(values, width: int):
    return np.array(
        ["" if value is None else str(value) for value in values],
        dtype=np.dtype(f"<U{max(1, width)}"),
    )


def _text_scalar(value, width: int):
    return np.array(
        "" if value is None else str(value), dtype=np.dtype(f"<U{max(1, width)}")
    )


def to_archive(rows, summary, faults=None, application=None) -> dict:
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
        matrix = np.array(
            [
                [_decode_value(value, column) for value in row[column]]
                for row in rows
            ],
            dtype=np.float64,
        )
        if matrix.shape != (count, joint_count):
            raise SystemExit(
                f"record_to_npz: the {column} column has shape {matrix.shape}, expected "
                f"({count}, {joint_count})"
            )
        arrays[column] = matrix

    for column in IMU_COLUMNS:
        if count == 0:
            arrays[column] = np.zeros((0, 3), dtype=np.float64)
            continue
        matrix = np.array(
            [
                [_decode_value(value, column) for value in row[column]]
                for row in rows
            ],
            dtype=np.float64,
        )
        if matrix.shape != (count, 3):
            raise SystemExit(
                f"record_to_npz: the {column} column has shape {matrix.shape}, expected "
                f"({count}, 3)"
            )
        arrays[column] = matrix

    for column in JOINT_BOOL_COLUMNS:
        if count == 0:
            arrays[column] = np.zeros((0, joint_count), dtype=np.bool_)
            continue
        matrix = np.array([row[column] for row in rows], dtype=np.bool_)
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
    # The IMU convention object is copied verbatim for the same reason: an
    # archive moved away from its run directory must still say that an axis
    # frame or gravity convention is unknown, not lose the declaration.
    if "imu_conventions" not in summary:
        raise SystemExit(
            "record_to_npz: the reader reported no IMU conventions for this run"
        )
    arrays["imu_conventions"] = np.array(
        json.dumps(summary["imu_conventions"], sort_keys=True)
    )

    arrays["joint_names"] = np.array(summary["joints"])
    arrays["schema_version"] = np.int64(summary["schema_version"])
    arrays["sample_hz"] = np.int64(summary["sample_hz"])
    arrays["control_hz"] = np.float64(summary["control_hz"])
    arrays["config_id"] = np.array(summary["config_id"])
    arrays["layout_id"] = np.array(summary["layout_id"])
    arrays["backend"] = np.array(summary["backend"])
    # A killed run can have clean committed chunks and no truncation or
    # corruption in the bytes it did write. Completion is its own fact: without
    # it an archive silently presents an unfinished run as a whole one.
    arrays["complete"] = np.bool_(summary["complete"])
    arrays["truncated"] = np.bool_(summary["truncated"])
    # A corrupt chunk is carried as its own column rather than folded into the
    # truncation flag: an archive that says "short" when it means "damaged"
    # tells a consumer the wrong thing about the rows it holds.
    arrays["corrupt"] = np.bool_(summary["corrupt"])

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

    # The fault stream. A fault is an occurrence, not a sampled row: its arrays
    # have their own length, so a run whose sample and fault counts differ is
    # still exported faithfully. A joint-less fault (joint_index null) is the
    # int64 sentinel -1; no object array and no pickle is ever produced.
    faults = faults or []
    for column, dtype in FAULT_SCALAR_DTYPES.items():
        source = column[len("fault_") :]
        values = []
        for fault in faults:
            value = fault.get(source)
            if source == "joint_index" and value is None:
                value = -1
            if source in ("observed_value", "limit"):
                value = _decode_value(value, column)
            values.append(value)
        arrays[column] = np.array(values, dtype=dtype)
    for column, width in FAULT_TEXT_COLUMNS.items():
        source = column[len("fault_") :]
        arrays[column] = _text_array(
            [fault.get(source) for fault in faults], width
        )

    if application is not None:
        app_summary = application["summary"]
        app_rows = application.get("velocity", [])
        association = application.get("association")
        if association is None:
            association = associate(summary, app_summary, app_rows)
        selected = association["rows"]
        hold = application.get("hold")
        if hold is None:
            hold = hold_forward(
                rows,
                selected,
                app_summary.get("sample_hz", 0),
                app_summary.get("losses", []),
            )
        for column, dtype in VELOCITY_SCALAR_DTYPES.items():
            source = column[len("velocity_") :]
            arrays[column] = np.array(
                [row.get(source) for row in selected], dtype=dtype
            )
        for column, width in VELOCITY_TEXT_COLUMNS.items():
            source = column[len("velocity_") :]
            arrays[column] = _text_array(
                [row.get(source) for row in selected], width
            )
        for key in ("forward", "lateral", "yaw"):
            values = np.asarray(hold[key], dtype=np.float64)
            if values.shape != (count,):
                raise SystemExit(
                    f"record_to_npz: the held {key} column has shape "
                    f"{values.shape}, expected ({count},)"
                )
            arrays[f"velocity_hold_{key}"] = values

        arrays["application_schema"] = _text_scalar(
            app_summary.get("schema"), 48
        )
        arrays["application_version"] = np.int64(
            int(app_summary.get("version", app_summary.get("schema_version", 0)))
        )
        arrays["application_config_id"] = _text_scalar(
            app_summary.get("application_config_id"), 64
        )
        arrays["application_runtime_config_id"] = _text_scalar(
            app_summary.get("config_id"), 64
        )
        arrays["application_layout_id"] = _text_scalar(
            app_summary.get("layout_id"), 64
        )
        arrays["application_joint_count"] = np.int64(
            int(app_summary.get("joint_count", 0))
        )
        arrays["application_sample_hz"] = np.int64(
            int(app_summary.get("sample_hz", 0))
        )
        arrays["application_source"] = _text_scalar(
            app_summary.get("source"), 16
        )
        clock = app_summary.get("clock") or {}
        arrays["application_clock"] = _text_scalar(clock.get("clock"), 32)
        arrays["application_boot_id"] = _text_scalar(clock.get("boot_id"), 64)
        model = app_summary.get("model") or {}
        arrays["application_model_status"] = _text_scalar(
            model.get("status"), 32
        )
        arrays["application_model_name"] = _text_scalar(model.get("name"), 256)
        arrays["application_model_sha256"] = _text_scalar(
            model.get("sha256"), 64
        )
        arrays["application_complete"] = np.bool_(
            bool(app_summary.get("complete"))
        )
        arrays["application_truncated"] = np.bool_(
            bool(app_summary.get("truncated"))
        )
        arrays["association_ok"] = np.bool_(bool(association["ok"]))
        arrays["association_reason"] = _text_scalar(
            association.get("reason", ""), 256
        )
        arrays["application_rows_total"] = np.uint64(
            association.get("total", len(app_rows))
        )
        arrays["application_rows_excluded"] = np.uint64(
            association.get("excluded", 0)
        )

        policy_sessions = application.get("policy_sessions", [])
        arrays["policy_session_count"] = np.uint64(len(policy_sessions))
        policy_json = [
            json.dumps(entry, ensure_ascii=False, sort_keys=True)
            for entry in policy_sessions
        ]
        width = max((len(value) for value in policy_json), default=1)
        arrays["policy_session_json"] = np.array(
            policy_json, dtype=np.dtype(f"<U{max(1, width)}")
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
    parser.add_argument(
        "--application-record",
        type=Path,
        default=None,
        help="the application app_run_* directory to associate with the runtime run",
    )
    args = parser.parse_args()

    if not args.reader.exists():
        raise SystemExit(f"record_to_npz: no reader at {args.reader}; build the read_record target")
    experiment = read_experiment(
        args.reader,
        args.run_directory,
        args.from_us,
        args.to_us,
        args.application_record,
    )
    summary = experiment["summary"]
    application = experiment["application"]
    application_payload = None
    if application is not None:
        app_summary = application["summary"]
        association = associate(
            summary, app_summary, application["velocity"]
        )
        hold = hold_forward(
            experiment["samples"],
            association["rows"],
            app_summary.get("sample_hz", summary.get("sample_hz", 0)),
            app_summary.get("losses", []),
        )
        application_payload = {
            "summary": app_summary,
            "velocity": application["velocity"],
            "policy_sessions": application["policy_sessions"],
            "association": association,
            "hold": hold,
        }
        if not association["ok"]:
            # The archive still carries the runtime run, but it says why no
            # application row was merged: an association is never guessed.
            print(
                "record_to_npz: application association refused: "
                f"{association['reason']}",
                file=sys.stderr,
            )

    arrays = to_archive(
        experiment["samples"],
        summary,
        experiment["faults"],
        application_payload,
    )
    np.savez_compressed(args.out_path, **arrays)
    message = (
        f"record_to_npz: {summary['rows_in_range']} rows, "
        f"{summary['joint_count']} joints, {len(summary['gaps'])} gap(s), "
        f"{len(experiment['faults'])} fault(s)"
    )
    if application_payload is not None:
        message += (
            f", {int(arrays['velocity_stamp_us'].shape[0])} velocity row(s), "
            f"association={'ok' if bool(arrays['association_ok']) else 'refused'}"
        )
    print(message + f", schema {summary['schema_version']} -> {args.out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
