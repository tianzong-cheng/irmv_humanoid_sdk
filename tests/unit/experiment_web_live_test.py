#!/usr/bin/env python3
"""Bounded live tails, deduplication and committed-record reconciliation.

The store is fed with envelopes the recorder writes. No wall-clock sleep and no
socket are needed: the tests pass a frozen arrival time where time matters and
assert on keys, coverage and revisions directly.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE_ROOT))

from tools.experiment_web.live_store import (  # noqa: E402
    LiveError,
    LiveStore,
    run_id_for_directory,
)


def runtime_run(root: Path, name: str, boot_id: str = "boot-live") -> Path:
    directory = root / name
    directory.mkdir()
    (directory / "meta.json").write_text(
        json.dumps(
            {
                "schema_version": 4,
                "config_id": "a" * 64,
                "layout_id": "b" * 64,
                "sample_hz": 100,
                "joint_count": 2,
                "joints": ["j1", "j2"],
                "clock": {
                    "clock": "linux_clock_monotonic",
                    "boot_id": boot_id,
                    "available": True,
                },
            }
        ),
        encoding="utf-8",
    )
    return directory


def application_run(root: Path, name: str) -> Path:
    directory = root / name
    directory.mkdir()
    (directory / "meta.json").write_text(
        json.dumps(
            {
                "schema": "irmv.application_velocity",
                "version": 1,
                "config_id": "a" * 64,
                "layout_id": "b" * 64,
                "sample_hz": 100,
                "clock": {
                    "clock": "linux_clock_monotonic",
                    "boot_id": "boot-live",
                    "available": True,
                },
            }
        ),
        encoding="utf-8",
    )
    return directory


def envelope(run_path: Path, source: str, transport_seq: int, kind: str,
             payload: dict, emitter: str = "e" * 32) -> dict:
    return {
        "version": 1,
        "emitter_id": emitter,
        "run_path": str(run_path),
        "source": source,
        "transport_seq": str(transport_seq),
        "kind": kind,
        "payload": payload,
    }


def runtime_sample(stamp: int, seq: int, q0: float = 0.5) -> dict:
    return {
        "kind": "sample",
        "stamp_us": str(stamp),
        "state_seq": str(seq),
        "joint_count": 2,
        "command_present": True,
        "feedback_valid": True,
        "output_valid": True,
        "imu_valid": True,
        "q": [q0, 0.0],
        "dq": [0.0, 0.0],
        "tau_ff": [0.0, 0.0],
        "kp": [1.0, 1.0],
        "kd": [0.1, 0.1],
        "measured_q": [q0, 0.0],
        "measured_dq": [0.0, 0.0],
        "measured_tau_est": [0.0, 0.0],
        "output_torque": [0.0, 0.0],
        "joint_valid": [True, True],
        "imu_orientation_rpy": [0.0, 0.0, 0.0],
        "imu_angular_velocity": [0.0, 0.0, 0.0],
        "imu_linear_acceleration": [0.0, 0.0, 0.0],
    }


def application_sample(stamp: int, seq: int, forward: float = 0.1) -> dict:
    return {
        "kind": "sample",
        "stamp_us": str(stamp),
        "sample_seq": str(seq),
        "identity_known": True,
        "runtime_id": "1" * 32,
        "session_id": "2" * 32,
        "latest_state_seq": str(seq),
        "forward": forward,
        "lateral": -0.2,
        "yaw": 0.3,
    }


class LiveStoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.store = LiveStore(self.root)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_late_join_has_hello_and_the_start_of_the_tail(self) -> None:
        run = runtime_run(self.root, "run_1")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(run, "runtime", 2, "sample", runtime_sample(1000, 1))
        )
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000, 10)
        self.assertTrue(snapshot["ok"])
        self.assertTrue(snapshot["changed"])
        self.assertEqual(snapshot["coverage"]["begin_us"], "1000")
        points = snapshot["series"][0]["points"]
        self.assertEqual(points[0]["t"], "1000")
        self.assertEqual(points[0]["v"], 0.5)
        self.assertEqual(snapshot["identity"]["run_path"], str(run))

    def test_two_arrival_orders_produce_the_same_tail(self) -> None:
        run_a = runtime_run(self.root, "run_a")
        run_b = runtime_run(self.root, "run_b")
        store_b = LiveStore(self.root)
        self.store.ingest(envelope(run_a, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(run_a, "runtime", 2, "sample", runtime_sample(1000, 1, 0.1))
        )
        self.store.ingest(
            envelope(run_a, "runtime", 3, "sample", runtime_sample(2000, 2, 0.2))
        )
        store_b.ingest(envelope(run_b, "runtime", 1, "hello", {}))
        store_b.ingest(
            envelope(run_b, "runtime", 2, "sample", runtime_sample(2000, 2, 0.2))
        )
        store_b.ingest(
            envelope(run_b, "runtime", 3, "sample", runtime_sample(1000, 1, 0.1))
        )
        a = self.store.Snapshot(run_id_for_directory(self.root, run_a),
                                ["runtime/j1/q"], 0, 5000, 10)
        b = store_b.Snapshot(run_id_for_directory(self.root, run_b),
                             ["runtime/j1/q"], 0, 5000, 10)
        self.assertEqual(
            [(p["t"], p["v"]) for p in a["series"][0]["points"]],
            [(p["t"], p["v"]) for p in b["series"][0]["points"]],
        )

    def test_same_sample_is_counted_once(self) -> None:
        run = runtime_run(self.root, "run_dup")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        sample = envelope(run, "runtime", 2, "sample", runtime_sample(1000, 1))
        self.store.ingest(sample)
        revision = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000,
                                       10)["revision"]
        self.store.ingest(sample)
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000, 10)
        self.assertEqual(snapshot["revision"], revision)
        keys = {(p["t"], p["seq"]) for p in snapshot["series"][0]["points"]}
        self.assertEqual(len(keys), len(snapshot["series"][0]["points"]))

    def test_transport_loss_and_out_of_order_samples_do_not_stitch(self) -> None:
        run = runtime_run(self.root, "run_loss")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(run, "runtime", 3, "sample", runtime_sample(2000, 2, 0.2))
        )
        self.store.ingest(
            envelope(run, "runtime", 2, "sample", runtime_sample(1000, 1, 0.1))
        )
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000, 10)
        self.assertEqual(snapshot["transport_losses"], 1)
        stamps = [int(point["t"]) for point in snapshot["series"][0]["points"]]
        self.assertEqual(stamps, sorted(stamps))

    def test_committed_sample_replaces_the_temporary_copy(self) -> None:
        run = runtime_run(self.root, "run_merge")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(run, "runtime", 2, "sample", runtime_sample(1000, 1, 0.1))
        )
        self.store.ingest(
            envelope(run, "runtime", 3, "sample", runtime_sample(3000, 3, 0.3))
        )
        reply = {
            "ok": True,
            "series": [
                {
                    "signal": {"id": "runtime/j1/q"},
                    "points": [
                        {"t": "1000", "seq": "1", "v": 0.1, "valid": True,
                         "segment": 0, "agg": "first"}
                    ],
                }
            ],
            "committed": {"begin_us": "1000", "end_us": "2001",
                          "complete": False},
        }
        merged = self.store.merge_range(
            reply, run_id, ["runtime/j1/q"], 0, 5000, 10
        )
        points = merged["series"][0]["points"]
        keys = [(point["t"], point["seq"]) for point in points]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertEqual([point["t"] for point in points], ["1000", "3000"])
        self.assertTrue(merged["live"]["active"])

    def test_application_source_mismatch_is_refused(self) -> None:
        run = runtime_run(self.root, "run_wrong_source")
        with self.assertRaises(LiveError):
            self.store.ingest(
                envelope(run, "application", 1, "hello", {})
            )

    def test_two_runtime_restarts_are_separate_runs(self) -> None:
        first = runtime_run(self.root, "run_boot_a", "boot-a")
        second = runtime_run(self.root, "run_boot_b", "boot-b")
        first_id = run_id_for_directory(self.root, first)
        second_id = run_id_for_directory(self.root, second)
        self.store.ingest(envelope(first, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(first, "runtime", 2, "sample", runtime_sample(1000, 1))
        )
        self.store.ingest(
            envelope(second, "runtime", 1, "hello", {}),
            now_ms=1.0,
        )
        self.store.ingest(
            envelope(second, "runtime", 2, "sample", runtime_sample(9000, 1)),
            now_ms=2.0,
        )
        first_snapshot = self.store.Snapshot(first_id, ["runtime/j1/q"], 0,
                                             10000, 10)
        second_snapshot = self.store.Snapshot(second_id, ["runtime/j1/q"], 0,
                                              10000, 10)
        self.assertEqual(first_snapshot["coverage"]["begin_us"], "1000")
        self.assertEqual(second_snapshot["coverage"]["begin_us"], "9000")
        self.assertNotEqual(first_snapshot["identity"]["run_path"],
                            second_snapshot["identity"]["run_path"])

    def test_more_than_eight_live_runs_are_refused(self) -> None:
        for index in range(8):
            run = runtime_run(self.root, f"run_{index}")
            self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
            self.store.ingest(envelope(run, "runtime", 2, "sample",
                                       runtime_sample(1000, 1)))
        ninth = runtime_run(self.root, "run_ninth")
        with self.assertRaises(LiveError):
            self.store.ingest(envelope(ninth, "runtime", 1, "hello", {}))
        self.assertEqual(len(self.store.active_runs()), 8)

    def test_tail_is_bounded_in_time_and_names_its_eviction(self) -> None:
        run = runtime_run(self.root, "run_long")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        sequence = 1
        for index in range(20):
            stamp = index * 1_000_000
            self.store.ingest(
                envelope(run, "runtime", sequence + 1, "sample",
                         runtime_sample(stamp, sequence)),
                now_ms=float(index),
            )
            sequence += 1
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 20_000_000,
                                       100)
        self.assertIsNotNone(snapshot["coverage"]["evicted_before_us"])
        self.assertGreater(int(snapshot["coverage"]["evicted_before_us"]), 0)
        self.assertLessEqual(len(snapshot["series"][0]["points"]), 400)

    def test_after_revision_returns_an_empty_change(self) -> None:
        run = runtime_run(self.root, "run_rev")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000, 10)
        same = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000, 10,
                                   after_revision=int(snapshot["revision"]))
        self.assertFalse(same["changed"])

    def test_application_values_come_from_their_own_run(self) -> None:
        run = application_run(self.root, "app_run")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "application", 1, "hello", {}))
        self.store.ingest(
            envelope(run, "application", 2, "sample",
                     application_sample(1000, 0, 0.4))
        )
        snapshot = self.store.Snapshot(run_id, ["application//forward"], 0,
                                       5000, 10)
        self.assertEqual(snapshot["series"][0]["points"][0]["v"], 0.4)

    def test_live_gap_starts_a_new_segment_and_never_joins_commit(self) -> None:
        run = runtime_run(self.root, "run_gap")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(run, "runtime", 2, "sample", runtime_sample(1000, 1))
        )
        self.store.ingest(
            envelope(run, "runtime", 3, "sample", runtime_sample(30000, 2))
        )
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 40000, 100)
        segments = [point["segment"] for point in snapshot["series"][0]["points"]]
        self.assertGreater(segments[-1], segments[0])
        reply = {
            "ok": True,
            "series": [{
                "signal": {"id": "runtime/j1/q"},
                "points": [{"t": "1000", "seq": "1", "v": 0.1,
                            "valid": True, "segment": 0, "agg": "first"}],
            }],
            "committed": {"begin_us": "1000", "end_us": "1001",
                          "complete": False},
        }
        merged = self.store.merge_range(reply, run_id, ["runtime/j1/q"],
                                        0, 40000, 100)
        points = merged["series"][0]["points"]
        self.assertEqual(points[0]["segment"], 0)
        self.assertGreater(points[1]["segment"], 0)

    def test_faults_and_rejections_are_bounded(self) -> None:
        run = runtime_run(self.root, "run_fault_bound")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        for seq in range(2000):
            self.store.ingest(envelope(
                run, "runtime", seq + 2, "fault",
                {"record_seq": str(seq), "detected_stamp_us": str(seq)},
            ))
        snapshot = self.store.Snapshot(run_id, ["runtime/j1/q"], 0, 5000, 10)
        self.assertLessEqual(len(snapshot["faults"]), 1024)
        for index in range(100):
            extra = runtime_run(self.root, f"run_reject_{index}")
            try:
                self.store.ingest(envelope(extra, "runtime", 1, "hello", {}))
            except LiveError:
                pass
        self.assertLessEqual(len(self.store._rejected), 32)

    def test_faults_are_merged_once(self) -> None:
        run = runtime_run(self.root, "run_fault")
        run_id = run_id_for_directory(self.root, run)
        self.store.ingest(envelope(run, "runtime", 1, "hello", {}))
        self.store.ingest(
            envelope(run, "runtime", 2, "fault",
                     {"record_seq": "4", "detected_stamp_us": "1000"})
        )
        reply = {"ok": True, "events": [{"event_id": "4"}]}
        merged = self.store.merge_events(reply, run_id)
        self.assertEqual(len(merged["events"]), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
