#!/usr/bin/env python3
"""tests/integration/fault_recording_test.py <robot_runtime> <read_record>
                                                   <fault_recording_peer> <source-root>
                                                   [--keep-work DIR]

The fault sidecar of a real recorded run, proved on the wire and on disk at the
same time:

* a real runtime runs a fake scenario in which joint 0 disconnects, joint 1
  disconnects while the first fault stays latched, both recover, and joint 0
  disconnects again;
* a real DDS peer subscribes to `irmv/v1/robot_state` for the whole run and
  prints every published fault report;
* the runtime recorder writes the complete occurrences to `faults.jsonl`.

The published report is the first latched fault and never changes: the state
machine latches one identity and its details until Reset. The sidecar is the
complete observation log: it carries the sustained episode once, a second
condition on another joint, and the recurrence after recovery. The test asserts
both facts and the ordering of `record_seq`, so a regression that either
replaces the latched report or starts deduplicating the sidecar is caught.

Nothing here is a machine limit or a real device: the deployment is the Fake
backend and the peer only listens.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# Fixed test domains 150..153 are already taken by the record/application
# tests, and the scenario harness allocates its own entries from the 190..229
# band. 154 is outside both and is unique to this test.
DOMAIN = 154
RUN_DEADLINE_S = 30.0
RECURRENCE_OBSERVE_S = 0.75


def wait_for(path: Path, pattern: str, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    text = ""
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="replace")
            if re.search(pattern, text):
                return text
        time.sleep(0.05)
    return text


def wait_for_fault_rows(run_directory: Path, count: int, timeout_s: float) -> int:
    deadline = time.monotonic() + timeout_s
    rows = 0
    while time.monotonic() < deadline:
        path = run_directory / "faults.jsonl"
        if path.exists():
            rows = len(
                [
                    line
                    for line in path.read_text(
                        encoding="utf-8", errors="replace"
                    ).splitlines()
                    if line.strip()
                ]
            )
            if rows >= count:
                return rows
        time.sleep(0.05)
    return rows


def stop_process(process: subprocess.Popen, timeout_s: float) -> int | None:
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
        try:
            return process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
            return process.returncode
    return process.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_runtime")
    parser.add_argument("read_record")
    parser.add_argument("fault_recording_peer")
    parser.add_argument("source_root", type=Path)
    parser.add_argument("--keep-work", type=Path, default=None)
    args = parser.parse_args()

    source_root = args.source_root.resolve()
    runtime_binary = str(Path(args.robot_runtime).resolve())
    read_record = str(Path(args.read_record).resolve())
    peer_binary = str(Path(args.fault_recording_peer).resolve())

    failures: list[str] = []

    def fail(note: str) -> None:
        failures.append(note)
        print(f"fault_recording_test: {note}", file=sys.stderr, flush=True)

    work = (
        args.keep_work
        if args.keep_work is not None
        else Path(tempfile.mkdtemp(prefix="irmv-fault-record-"))
    )
    work.mkdir(parents=True, exist_ok=True)
    try:
        return run(
            source_root,
            runtime_binary,
            read_record,
            peer_binary,
            work,
            failures,
            fail,
        )
    finally:
        if args.keep_work is None:
            shutil.rmtree(work, ignore_errors=True)


def run(source_root, runtime_binary, read_record, peer_binary, work, failures, fail):
    runs_directory = work / "runs"
    runs_directory.mkdir(parents=True, exist_ok=True)
    document = json.loads((source_root / "config" / "sdk.json").read_text())
    document["telemetry"]["recorder"] = {
        "enabled": True,
        "sample_hz": 100,
        "queue_capacity": 256,
        "directory": str(runs_directory),
    }
    config_path = work / "sdk.json"
    config_path.write_text(json.dumps(document, indent=2) + "\n")

    runtime_log = work / "runtime.log"
    peer_log = work / "peer.log"
    peer_err = work / "peer.err"
    runtime_out = runtime_log.open("w", encoding="utf-8")
    peer_out = peer_log.open("w", encoding="utf-8")
    peer_err_out = peer_err.open("w", encoding="utf-8")
    runtime_process = None
    peer_process = None
    try:
        # The peer starts first so discovery cannot miss the first latched
        # report. It only listens; the runtime publishes when it starts.
        peer_process = subprocess.Popen(
            [
                peer_binary,
                "--config",
                str(config_path),
                "--domain",
                str(DOMAIN),
                "--seconds",
                "45",
                # The scenario's recurrence is at 7.5 s and both joints are
                # healthy again at 8.5 s; the reset must follow both so the
                # runtime can leave Fault into a healthy ZeroTorque.
                "--reset-at-seconds",
                "10",
            ],
            stdout=peer_out,
            stderr=peer_err_out,
            cwd=work,
        )
        runtime_process = subprocess.Popen(
            [
                runtime_binary,
                "--config",
                str(config_path),
                "--domain",
                str(DOMAIN),
                "--fake-scenario",
                str(source_root / "tests" / "support" / "fake_fault_sequence.json"),
            ],
            stdin=subprocess.DEVNULL,
            stdout=runtime_out,
            stderr=subprocess.STDOUT,
            cwd=work,
        )

        text = wait_for(runtime_log, r'state="ZeroTorque"', 20)
        if 'state="ZeroTorque"' not in text:
            fail(f"the runtime never reached ZeroTorque: {text[-800:]}")
            return 1

        deadline = time.monotonic() + 10
        run_directory = None
        while time.monotonic() < deadline:
            candidates = [
                path
                for path in runs_directory.iterdir()
                if path.is_dir() and path.name.startswith("run_")
            ]
            if candidates:
                run_directory = sorted(candidates)[0]
                break
            time.sleep(0.05)
        if run_directory is None:
            fail("the recorder created no runtime run directory")
            return 1

        # The scenario's three occurrences are at 3.0 s, 4.5 s and 7.5 s. Wait
        # for all three lines, then leave enough time for the DDS state of the
        # recurrence to be published and observed.
        rows = wait_for_fault_rows(run_directory, 3, RUN_DEADLINE_S)
        if rows < 3:
            fail(f"the run wrote {rows} fault row(s), expected 3")
        time.sleep(RECURRENCE_OBSERVE_S)
        # Leave Fault only through the protocol's Reset: the peer acquires the
        # session the runtime issues and sends the reset after the scenario's
        # recurrence. Without it the runtime rightly keeps running, and the
        # recorder would be killed instead of finalizing its fault stream.
        peer_text = wait_for(peer_log, r'"event": *"reset_applied"', 25)
        if '"reset_applied"' not in peer_text:
            fail(
                f"the peer did not observe the reset being applied: {peer_text[-800:]}"
            )
    finally:
        if runtime_process is not None:
            code = stop_process(runtime_process, 20)
            if code not in (0, None) and not failures:
                fail(f"the runtime exited with {code} after SIGTERM")
        if peer_process is not None:
            code = stop_process(peer_process, 15)
            if code not in (0, None) and not failures:
                fail(f"the peer exited with {code} after SIGTERM")
        runtime_out.close()
        peer_out.close()
        peer_err_out.close()

    if run_directory is None:
        return 1

    # The index owns the completion marker: an orderly stop finalizes the fault
    # worker, and a fault write failure keeps the marker false.
    index = json.loads((run_directory / "index.json").read_text())
    fault_index = index.get("faults")
    if not isinstance(fault_index, dict):
        fail("the run index has no fault object")
        fault_index = {}
    if fault_index.get("complete") is not True:
        fail(f"the fault stream is not complete: {fault_index}")
    if int(fault_index.get("write_failures", -1)) != 0:
        fail(f"the fault stream recorded write failures: {fault_index}")
    if int(fault_index.get("queue_dropped", -1)) != 0:
        fail(f"the fault stream dropped occurrences: {fault_index}")
    if int(fault_index.get("generation_dropped", -1)) != 0:
        fail(f"the fault observations overflowed: {fault_index}")
    if fault_index.get("losses"):
        fail(f"the fault stream lost a span: {fault_index['losses']}")

    full = subprocess.run(
        [read_record, "--run", str(run_directory)],
        capture_output=True,
        text=True,
        check=False,
    )
    if full.returncode != 0:
        fail(f"read_record failed: {full.stderr.strip()}")
        return 1
    lines = [line for line in full.stdout.splitlines() if line.strip()]
    if not lines:
        fail("read_record printed nothing")
        return 1
    summary = json.loads(lines[-1])
    records = [json.loads(line) for line in lines[:-1]]
    samples = [row for row in records if row.get("record_type") == "sample"]
    faults = [row for row in records if row.get("record_type") == "fault"]

    if summary.get("record_kind") != "runtime":
        fail(
            f"read_record did not dispatch the runtime run: {summary.get('record_kind')}"
        )
    if len(samples) != summary.get("rows_in_range"):
        fail(
            f"read_record printed {len(samples)} sample(s) but reports "
            f"{summary.get('rows_in_range')}"
        )
    fault_summary = summary.get("faults")
    if not isinstance(fault_summary, dict):
        fail("the runtime summary carries no fault statistics")
        fault_summary = {}
    if fault_summary.get("complete") is not True:
        fail(f"the summary does not mark the fault stream complete: {fault_summary}")
    if fault_summary.get("truncated"):
        fail("the summary marks the fault stream truncated")
    if int(fault_summary.get("rows_in_range", -1)) != len(faults):
        fail(
            f"the summary reports {fault_summary.get('rows_in_range')} fault row(s), "
            f"the reader printed {len(faults)}"
        )
    if int(fault_summary.get("missing_occurrences", -1)) != 0:
        fail(f"the summary reports missing fault occurrences: {fault_summary}")

    if len(faults) != 3:
        fail(f"the run recorded {len(faults)} fault row(s), expected 3: {faults}")
    else:
        expected = [
            (1, 0, "JointDisconnected", 2),
            (2, 1, "JointDisconnected", 2),
            (3, 0, "JointDisconnected", 2),
        ]
        for row, (seq, joint, condition, reason) in zip(faults, expected):
            if int(row.get("record_seq", -1)) != seq:
                fail(f"fault record_seq {row.get('record_seq')} is not {seq}")
            if row.get("joint_index") != joint:
                fail(
                    f"fault {seq} names joint {row.get('joint_index')}, "
                    f"expected {joint}"
                )
            if row.get("condition_name") != condition:
                fail(
                    f"fault {seq} has condition {row.get('condition_name')}, "
                    f"expected {condition}"
                )
            if int(row.get("condition", -1)) != 2:
                fail(f"fault {seq} has numeric condition {row.get('condition')}")
            if int(row.get("reason", -1)) != reason:
                fail(f"fault {seq} has reason {row.get('reason')}")
            if row.get("reason_name") != "FeedbackInvalid":
                fail(f"fault {seq} has reason name {row.get('reason_name')}")
            if int(row.get("sample_stamp_us", 0)) <= 0:
                fail(f"fault {seq} carries no feedback sample stamp")
            if int(row.get("detected_stamp_us", 0)) <= 0:
                fail(f"fault {seq} carries no detection stamp")
        sequences = [int(row.get("record_seq", 0)) for row in faults]
        if sequences != sorted(set(sequences)):
            fail(f"fault record_seq is not strictly increasing: {sequences}")
        detected = [int(row.get("detected_stamp_us", 0)) for row in faults]
        if detected != sorted(detected):
            fail(f"fault detections are not in time order: {detected}")
        # Every occurrence names the same latched fault identity: the second
        # condition and the recurrence are observations, not new latches.
        fault_ids = {row.get("fault_id") for row in faults}
        if len(fault_ids) != 1 or 0 in fault_ids:
            fail(f"the fault rows do not share the first latched fault id: {fault_ids}")

    # What DDS published. Every fault state must carry the first latched
    # report, unchanged, including the states after recovery and after the
    # second joint fault.
    peer_lines = []
    for line in peer_log.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            peer_lines.append(json.loads(line))
        except json.JSONDecodeError:
            fail(f"the peer printed a malformed line: {line[:200]}")
    states = [entry for entry in peer_lines if entry.get("event") == "state"]
    if not states:
        fail("the DDS peer observed no robot_state")
    fault_states = [entry for entry in states if int(entry.get("fault_id", 0)) != 0]
    if len(fault_states) < 10:
        fail(f"the DDS peer observed only {len(fault_states)} fault state(s)")
    if faults and fault_states:
        first_published = fault_states[0]
        first_file = faults[0]
        if int(first_published.get("fault_id", 0)) != int(
            first_file.get("fault_id", -1)
        ):
            fail(
                "DDS and the fault file name different fault ids: "
                f"{first_published.get('fault_id')} != {first_file.get('fault_id')}"
            )
        if int(first_published.get("fault_count", 0)) != 1:
            fail(
                "the latched report does not publish exactly one detail: "
                f"{first_published.get('fault_count')}"
            )
        detail = (first_published.get("faults") or [{}])[0]
        for key in (
            "reason",
            "condition",
            "joint_index",
            "error_code",
            "observed_value",
            "limit",
            "sample_stamp_us",
            "output_action",
            "message",
        ):
            if detail.get(key) != first_file.get(key):
                fail(
                    f"the published first fault detail differs at {key}: "
                    f"{detail.get(key)!r} != {first_file.get(key)!r}"
                )
        published = {
            (
                entry.get("fault_id"),
                json.dumps(entry.get("faults"), sort_keys=True),
            )
            for entry in fault_states
        }
        if len(published) != 1:
            fail(
                "the published fault report changed while the fault stayed "
                f"latched: {len(published)} distinct report(s)"
            )
        last_stamp = max(int(entry.get("stamp_us", 0)) for entry in fault_states)
        if faults:
            recurrence = (
                int(faults[2].get("detected_stamp_us", 0)) if len(faults) >= 3 else 0
            )
            if last_stamp < recurrence:
                fail(
                    "no DDS state was observed at or after the recurrence "
                    f"({last_stamp} < {recurrence})"
                )

    if failures:
        return 1
    print(
        f"fault_recording_test: {len(faults)} fault occurrence(s), "
        f"{len(samples)} recorded sample(s), "
        f"{len(fault_states)} latched DDS report(s), "
        f"fault stream complete={fault_summary.get('complete')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
