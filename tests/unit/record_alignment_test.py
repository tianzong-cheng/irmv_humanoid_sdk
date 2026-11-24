#!/usr/bin/env python3
"""tests/unit/record_alignment_test.py

The association rules between one runtime run and the application's independent
velocity run. They are pure functions of the two readers' outputs, so this test
builds both sides by hand: no DDS, no process, no files.

What is proved here:

* two runs of one boot match by monotonic time even when their phases differ -
  a row index is never the join key;
* an application row with no known identity, a different boot id, or another
  runtime instance never takes part in the automatic merge;
* a file that spans producer A and producer B contributes only the rows whose
  runtime/session identity is the runtime run's own, and the excluded rows are
  counted;
* a held-forward velocity is valid only inside a contiguous recorded segment.
  A gap in the cadence, a lost sample span, and the end of the recording all
  yield NaN rather than a value carried past what was recorded.
"""

from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np

SOURCE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE_ROOT / "tools"))

import record_to_npz  # noqa: E402

RUNTIME_ID_A = "a1" * 16
SESSION_ID_A = "11" * 16
RUNTIME_ID_B = "b2" * 16
SESSION_ID_B = "22" * 16
BOOT_ID = "boot-one"
OTHER_BOOT_ID = "boot-two"

failures: list[str] = []


def check(condition: bool, note: str) -> None:
    if not condition:
        failures.append(note)
        print(f"record_alignment_test: {note}", file=sys.stderr, flush=True)


def runtime_summary(
    *,
    boot_id: str = BOOT_ID,
    clock: str = "linux_clock_monotonic",
    available: bool = True,
    sessions: list[dict] | None = None,
) -> dict:
    if sessions is None:
        sessions = [
            {
                "runtime_id": RUNTIME_ID_A,
                "session_id": SESSION_ID_A,
                "epoch": 7,
                "producer_id": "",
            }
        ]
    return {
        "record_kind": "runtime",
        "schema_version": 4,
        "sample_hz": 100,
        "joint_count": 2,
        "clock": {"clock": clock, "boot_id": boot_id, "available": available},
        "sessions": sessions,
    }


def application_summary(
    *,
    boot_id: str = BOOT_ID,
    clock: str = "linux_clock_monotonic",
    available: bool = True,
) -> dict:
    return {
        "record_kind": "application",
        "schema": "irmv.application_velocity",
        "version": 1,
        "sample_hz": 100,
        "complete": True,
        "truncated": False,
        "losses": [],
        "clock": {"clock": clock, "boot_id": boot_id, "available": available},
    }


def velocity_row(
    stamp_us: int,
    sample_seq: int,
    *,
    runtime_id: str = RUNTIME_ID_A,
    session_id: str = SESSION_ID_A,
    identity_known: bool = True,
    forward: float = 0.1,
    lateral: float = -0.2,
    yaw: float = 0.3,
) -> dict:
    return {
        "record_type": "velocity",
        "stamp_us": stamp_us,
        "sample_seq": sample_seq,
        "identity_known": identity_known,
        "runtime_id": runtime_id if identity_known else "",
        "session_id": session_id if identity_known else "",
        "latest_state_seq": sample_seq + 1,
        "forward": forward,
        "lateral": lateral,
        "yaw": yaw,
    }


def runtime_row(stamp_us: int) -> dict:
    return {"record_type": "sample", "stamp_us": stamp_us}


def hold_values(runtime_rows, velocity_rows, sample_hz=100, losses=None) -> dict:
    return record_to_npz.hold_forward(
        runtime_rows, velocity_rows, sample_hz, losses or []
    )


def close(actual: float, expected: float) -> bool:
    return abs(actual - expected) < 1e-12


# ---------------------------------------------------------------------------
# Phase: the same boot and identity, but the application's samples start at a
# different phase. Only the monotonic stamp pairs a runtime row with a sample.
# ---------------------------------------------------------------------------
rows = [velocity_row(1_000_000, 0, forward=0.1)]
rows.append(velocity_row(1_010_000, 1, forward=0.2))
rows.append(velocity_row(1_020_000, 2, forward=0.3))
association = record_to_npz.associate(runtime_summary(), application_summary(), rows)
check(
    association["ok"],
    f"the same boot/identity did not associate: {association['reason']}",
)
check(association["excluded"] == 0, "the phase case excluded a matching row")
holds = hold_values(
    [
        runtime_row(999_999),
        runtime_row(1_000_000),
        runtime_row(1_015_000),
        runtime_row(1_020_000),
    ],
    rows,
)
check(np.isnan(holds["forward"][0]), "a value was invented before the recording began")
check(
    close(float(holds["forward"][1]), 0.1),
    "the first sample was not held at its own stamp",
)
check(
    close(float(holds["forward"][2]), 0.2),
    "the held value was paired by index, not by stamp",
)
check(
    close(float(holds["forward"][3]), 0.3),
    "an exact sample stamp did not select its own sample",
)
check(
    close(float(holds["lateral"][2]), -0.2),
    "the lateral value was not held with the sample",
)
check(close(float(holds["yaw"][2]), 0.3), "the yaw value was not held with the sample")

# ---------------------------------------------------------------------------
# Refusals: each of these names a different merge the automatic path must not
# make. None of them may silently export a subset as if it were associated.
# ---------------------------------------------------------------------------
for label, summary, app, app_rows, needle in (
    (
        "unknown identity",
        runtime_summary(),
        application_summary(),
        [velocity_row(1_000_000, 0, identity_known=False)],
        "identity",
    ),
    (
        "different boot",
        runtime_summary(),
        application_summary(boot_id=OTHER_BOOT_ID),
        [velocity_row(1_000_000, 0)],
        "boot",
    ),
    (
        "different clock",
        runtime_summary(),
        application_summary(clock="realtime"),
        [velocity_row(1_000_000, 0)],
        "clock",
    ),
    (
        "unknown clock identity",
        runtime_summary(available=False),
        application_summary(available=False),
        [velocity_row(1_000_000, 0)],
        "clock",
    ),
    (
        "different runtime id",
        runtime_summary(),
        application_summary(),
        [velocity_row(1_000_000, 0, runtime_id=RUNTIME_ID_B, session_id=SESSION_ID_B)],
        "no application",
    ),
    (
        "no runtime session",
        runtime_summary(sessions=[]),
        application_summary(),
        [velocity_row(1_000_000, 0)],
        "runtime",
    ),
):
    association = record_to_npz.associate(summary, app, app_rows)
    check(not association["ok"], f"{label} was accepted as an automatic merge")
    check(
        needle in association["reason"],
        f"{label}: the refusal does not name the cause ({association['reason']!r})",
    )
    check(
        association["rows"] == [],
        f"{label}: a refused association still selected rows",
    )
    check(
        association["excluded"] == len(app_rows),
        f"{label}: the refused rows were not counted as excluded",
    )

# ---------------------------------------------------------------------------
# A file that spans A then B: only A's rows may join this runtime's record. The
# B rows are not an error - they are excluded and counted.
# ---------------------------------------------------------------------------
spanned = [
    velocity_row(2_000_000, 0, forward=0.5),
    velocity_row(2_010_000, 1, forward=0.6),
    velocity_row(
        2_020_000, 2, runtime_id=RUNTIME_ID_B, session_id=SESSION_ID_B, forward=9.0
    ),
    velocity_row(
        2_030_000, 3, runtime_id=RUNTIME_ID_B, session_id=SESSION_ID_B, forward=9.0
    ),
]
association = record_to_npz.associate(runtime_summary(), application_summary(), spanned)
check(association["ok"], "the A-only part of a spanning file did not associate")
check(
    [row["runtime_id"] for row in association["rows"]] == [RUNTIME_ID_A, RUNTIME_ID_A],
    "the A/B file kept rows of the other runtime instance",
)
check(association["excluded"] == 2, "the A/B file did not count the excluded rows")
check(association["total"] == 4, "the A/B file did not report its total row count")

# A file that carries only B is refused, not merged from the wrong instance.
association = record_to_npz.associate(
    runtime_summary(),
    application_summary(),
    [row for row in spanned if row["runtime_id"] == RUNTIME_ID_B],
)
check(not association["ok"], "a file of another runtime instance was associated")

# ---------------------------------------------------------------------------
# Hold-forward boundaries. The sample period is 10 ms here.
# ---------------------------------------------------------------------------
losses = [
    {
        "first_sample_seq": 5,
        "last_sample_seq": 9,
        "first_stamp_us": 3_050_000,
        "last_stamp_us": 3_090_000,
        "dropped": 5,
        "reason": "queue_full",
    }
]
lost = [
    velocity_row(3_000_000, 0, forward=1.0),
    velocity_row(3_010_000, 1, forward=1.1),
    velocity_row(3_100_000, 10, forward=1.2),
    velocity_row(3_110_000, 11, forward=1.3),
]
runtime = [
    runtime_row(3_000_000),  # first sample
    runtime_row(3_009_999),  # inside the first sample's planned period
    runtime_row(3_010_000),  # second sample
    runtime_row(3_020_000),  # one period after the second
    runtime_row(3_049_999),  # inside the lost span
    runtime_row(3_050_000),  # first lost sample's deadline
    runtime_row(3_095_000),  # still inside the lost span
    runtime_row(3_100_000),  # the next recorded sample
    runtime_row(3_110_000),  # last recorded sample
    runtime_row(3_120_000),  # one period past the end of the recording
]
held = hold_values(runtime, lost, losses=losses)
check(
    close(float(held["forward"][0]), 1.0), "the first sample was not held at its stamp"
)
check(
    close(float(held["forward"][1]), 1.0),
    "a sample was not held to the next planned deadline",
)
check(close(float(held["forward"][2]), 1.1), "the second sample was not selected")
check(
    np.isnan(held["forward"][3]),
    "a value was extrapolated past one planned sample period",
)
check(
    np.isnan(held["forward"][4]) and np.isnan(held["forward"][5]),
    "a value was held across an indexed loss",
)
check(
    np.isnan(held["forward"][6]),
    "a value was extrapolated across the gap to the next segment",
)
check(close(float(held["forward"][7]), 1.2), "the next segment was not used")
check(close(float(held["forward"][8]), 1.3), "the last sample was not used")
check(
    np.isnan(held["forward"][9]),
    "the last value was extrapolated beyond the recorded range",
)

# A time gap longer than the planned cadence ends the segment even when the
# index carries no loss entry for it.
gapped = [
    velocity_row(4_000_000, 0, forward=2.0),
    velocity_row(4_010_000, 1, forward=2.1),
    velocity_row(4_200_000, 2, forward=2.2),
]
held = hold_values(
    [
        runtime_row(4_015_000),
        runtime_row(4_020_000),
        runtime_row(4_100_000),
        runtime_row(4_200_000),
    ],
    gapped,
)
check(close(float(held["forward"][0]), 2.1), "a cadence-adjacent stamp lost its sample")
check(np.isnan(held["forward"][1]), "a value crossed a cadence gap")
check(np.isnan(held["forward"][2]), "a value was invented inside a cadence gap")
check(close(float(held["forward"][3]), 2.2), "the sample after a gap was not used")

# A cadence gap between two successful samples must not bypass an indexed loss.
# The pair 1.015 s -> 1.040 s is already a gap, but the loss says the 1.020 s
# sample was due: 1.019 s may still hold 1.015 s, 1.022 s must not.
interior_losses = [
    {
        "first_sample_seq": 2,
        "last_sample_seq": 3,
        "first_stamp_us": 1_020_000,
        "last_stamp_us": 1_030_000,
        "dropped": 2,
        "reason": "queue_full",
    }
]
interior_samples = [
    velocity_row(1_000_000, 0, forward=1.9),
    velocity_row(1_015_000, 1, forward=2.0),
    velocity_row(1_040_000, 2, forward=2.2),
]
interior = hold_values(
    [
        runtime_row(1_019_000),
        runtime_row(1_022_000),
        runtime_row(1_040_000),
    ],
    interior_samples,
    losses=interior_losses,
)
check(
    close(float(interior["forward"][0]), 2.0),
    "the sample before an interior loss was not held",
)
check(
    np.isnan(interior["forward"][1]),
    "a cadence gap bypassed an interior loss and held the old value",
)
check(
    close(float(interior["forward"][2]), 2.2),
    "the sample after an interior loss was not used",
)

# Losses are grouped by reason in the index, not sorted by time. Every
# overlapping loss must bound an interior segment, even if an earlier loss
# appears after a later one in the table.
mixed_losses = [
    {
        "first_stamp_us": 1_050_000,
        "last_stamp_us": 1_060_000,
        "reason": "queue_full",
    },
    {
        "first_stamp_us": 1_020_000,
        "last_stamp_us": 1_040_000,
        "reason": "write_failed",
    },
]
mixed_samples = [
    velocity_row(1_000_000, 0, forward=1.9),
    velocity_row(1_015_000, 1, forward=2.0),
    velocity_row(1_080_000, 8, forward=2.2),
]
for losses in (mixed_losses, list(reversed(mixed_losses))):
    mixed = hold_values(
        [
            runtime_row(1_019_999),
            runtime_row(1_020_000),
            runtime_row(1_022_000),
            runtime_row(1_080_000),
        ],
        mixed_samples,
        losses=losses,
    )
    check(close(float(mixed["forward"][0]), 2.0), "mixed losses cut the hold early")
    check(
        np.isnan(mixed["forward"][1]) and np.isnan(mixed["forward"][2]),
        "loss table order hid the earliest missing sample",
    )
    check(
        close(float(mixed["forward"][3]), 2.2),
        "mixed losses hid the sample after recovery",
    )

# The final segment has no following row, so the index's losses are the only
# thing that can bound it. A loss starting inside the last planned period ends
# the hold at that instant: 1.019 s is still served, 1.020 s and 1.022 s are
# not. Without the loss the same 1.022 s row would hold the value.
tail_losses = [
    {
        "first_sample_seq": 1,
        "last_sample_seq": 2,
        "first_stamp_us": 1_020_000,
        "last_stamp_us": 1_030_000,
        "dropped": 2,
        "reason": "queue_full",
    }
]
tail_samples = [velocity_row(1_015_000, 0, forward=7.0)]
tail = hold_values(
    [
        runtime_row(1_019_000),
        runtime_row(1_020_000),
        runtime_row(1_022_000),
    ],
    tail_samples,
    losses=tail_losses,
)
check(
    close(float(tail["forward"][0]), 7.0),
    "the last sample was not held before a future loss",
)
check(
    np.isnan(tail["forward"][1]),
    "a value was held at the first lost sample's deadline",
)
check(
    np.isnan(tail["forward"][2]),
    "a value was held past a loss that started after the last sample",
)
unclamped = hold_values([runtime_row(1_022_000)], tail_samples)
check(
    close(float(unclamped["forward"][0]), 7.0),
    "without a loss the last sample was not held for one planned period",
)

# Several future losses bound the hold at the earliest one, regardless of the
# order the index lists them in.
multiple_losses = [
    {
        "first_sample_seq": 3,
        "last_sample_seq": 4,
        "first_stamp_us": 1_022_000,
        "last_stamp_us": 1_023_000,
        "dropped": 1,
        "reason": "queue_full",
    },
    {
        "first_sample_seq": 1,
        "last_sample_seq": 2,
        "first_stamp_us": 1_019_000,
        "last_stamp_us": 1_021_000,
        "dropped": 2,
        "reason": "queue_full",
    },
]
multiple = hold_values(
    [
        runtime_row(1_018_999),
        runtime_row(1_019_000),
        runtime_row(1_021_000),
    ],
    tail_samples,
    losses=multiple_losses,
)
check(
    close(float(multiple["forward"][0]), 7.0),
    "the held value was cut before the earliest future loss",
)
check(
    np.isnan(multiple["forward"][1]),
    "a later loss won over the earliest future loss",
)
check(
    np.isnan(multiple["forward"][2]),
    "a value was held past the earliest future loss",
)

# A loss exactly at the planned deadline does not change that deadline: one
# microsecond before it is still inside the hold, the deadline itself is not.
deadline_losses = [
    {
        "first_sample_seq": 1,
        "last_sample_seq": 1,
        "first_stamp_us": 1_025_000,
        "last_stamp_us": 1_025_000,
        "dropped": 1,
        "reason": "queue_full",
    }
]
deadline = hold_values(
    [runtime_row(1_024_999), runtime_row(1_025_000)],
    tail_samples,
    losses=deadline_losses,
)
check(
    close(float(deadline["forward"][0]), 7.0),
    "a loss at the planned deadline cut the hold early",
)
check(
    np.isnan(deadline["forward"][1]),
    "a value was held at the planned deadline",
)

# Explicit simulation identity must dominate the host boot identity.

sim_runtime = runtime_summary()
sim_app = application_summary()
sim_clock = dict(
    clock="simulation",
    clock_domain="simulation",
    available=True,
    boot_id=BOOT_ID,
    clock_run_id="12" * 16,
    logical_origin_us=1,
    physics_dt_us=1000,
    control_dt_us=2000,
)
sim_runtime["clock"] = copy.deepcopy(sim_clock)
sim_app["clock"] = copy.deepcopy(sim_clock)
check(
    record_to_npz.associate(sim_runtime, sim_app, [velocity_row(1, 1)])["ok"],
    "same simulation identity did not associate",
)
for key, value in (
    ("clock_run_id", "34" * 16),
    ("logical_origin_us", 2),
    ("physics_dt_us", 500),
    ("control_dt_us", 4000),
    ("clock_domain", "host_monotonic"),
):
    foreign = copy.deepcopy(sim_app)
    foreign["clock"][key] = value
    check(
        not record_to_npz.associate(sim_runtime, foreign, [velocity_row(1, 1)])["ok"],
        "simulation association ignored " + key,
    )
    if key == "clock_run_id":
        check(
            "simulation clock"
            in record_to_npz.associate(sim_runtime, foreign, [velocity_row(1, 1)])[
                "reason"
            ],
            "different simulation runs mislabeled as host boots",
        )
check(
    not record_to_npz.associate(
        sim_runtime, sim_app, [velocity_row(1, 1, runtime_id=RUNTIME_ID_B)]
    )["ok"],
    "simulation association bypassed actual runtime/session identity",
)

if failures:
    print(
        f"record_alignment_test: {len(failures)} failure(s)",
        file=sys.stderr,
        flush=True,
    )
    raise SystemExit(1)

print(
    "record_alignment_test: association by boot/identity/time; "
    "hold-forward bounded to contiguous recorded segments"
)
