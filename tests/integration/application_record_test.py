#!/usr/bin/env python3
"""tests/integration/application_record_test.py <application> <robot_runtime> <source-root>
                                                    [--keep-work DIR]

The application-driven recorded run (§11.1, Task 9): a real runtime records a
session the real application supervised, and the run is read back as the
evidence of what the *machine* did.

    Standby -> Damping -> Initializing -> Ready -> (hold) -> Damping -> exit

The recorder is enabled in a copy of the shipped configuration, so both
processes keep the same configuration digest and the recorded `meta.json` names
the deployment that ran. What the run has to show, and what this test asserts,
is the coherence the recorder exists for: every row of the authorized window
carries the command that was in force in the same tick - its identity, its
epoch and its sequence - alongside the state the runtime published for it, the
feedback it used and the output it produced. The authorization context is also
in the run's own `sessions.json`, so a consumer can join a row's epoch to the
context it was authorized under.

Nothing here is a machine limit: the deployment is the shipped Fake one, whose
amplitudes and gains are demonstration values.
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

DOMAIN = 226
RECORD_HOLD_S = 2.0
KEYS = {"damping": b"d", "initialize": b"i", "exit": b"x"}


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application")
    parser.add_argument("robot_runtime")
    parser.add_argument("source_root", type=Path)
    parser.add_argument("--keep-work", type=Path, default=None)
    args = parser.parse_args()

    source_root = args.source_root.resolve()
    application_binary = str(Path(args.application).resolve())
    runtime_binary = str(Path(args.robot_runtime).resolve())
    read_record = Path(runtime_binary).parent / "read_record"
    if not read_record.exists():
        print(f"application_record_test: no read_record next to {runtime_binary}")
        return 1

    failures: list[str] = []

    def fail(note: str) -> None:
        failures.append(note)
        print(f"application_record_test: {note}", file=sys.stderr, flush=True)

    work = (
        args.keep_work
        if args.keep_work is not None
        else Path(tempfile.mkdtemp(prefix="irmv-record-"))
    )
    work.mkdir(parents=True, exist_ok=True)
    try:
        return run(source_root, application_binary, runtime_binary, read_record,
                   work, failures, fail)
    finally:
        if args.keep_work is None:
            shutil.rmtree(work, ignore_errors=True)


def run(source_root, application_binary, runtime_binary, read_record, work,
        failures, fail) -> int:
    # The recorder's own directory, and a configuration that turns it on. It is
    # a copy of the shipped document with one section changed, so the two
    # processes of this run still agree on every other parameter - and on the
    # digest they publish.
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

    application_log = work / "application.log"
    runtime_log = work / "runtime.log"
    application_out = application_log.open("w", encoding="utf-8")
    runtime_out = runtime_log.open("w", encoding="utf-8")
    runtime_process = subprocess.Popen(
        [
            runtime_binary,
            "--config",
            str(config_path),
            "--domain",
            str(DOMAIN),
            "--fake-scenario",
            str(source_root / "tests" / "support" / "fake_static.json"),
        ],
        stdout=runtime_out,
        stderr=subprocess.STDOUT,
        cwd=work,
    )
    application_process = None
    try:
        text = wait_for(runtime_log, r'state="ZeroTorque"', 20)
        if 'state="ZeroTorque"' not in text:
            fail(f"the runtime never reached ZeroTorque: {text[-600:]}")
            return 1

        application_process = subprocess.Popen(
            [
                application_binary,
                "--config",
                str(config_path),
                "--domain",
                str(DOMAIN),
            ],
            stdin=subprocess.PIPE,
            stdout=application_out,
            stderr=subprocess.STDOUT,
            cwd=work,
        )

        def key(name: str) -> None:
            assert application_process is not None
            assert application_process.stdin is not None
            application_process.stdin.write(KEYS[name])
            application_process.stdin.flush()

        def expect(note: str, pattern: str, timeout_s: float = 25) -> str:
            text = application_log.read_text(encoding="utf-8", errors="replace")
            if not re.search(pattern, text):
                text = wait_for(application_log, pattern, timeout_s)
            if not re.search(pattern, text):
                fail(f"{note}: {pattern!r} never appeared; tail: {text[-600:]}")
            return text

        expect("the handover", r"behavior=Standby runtime=ZeroTorque ")
        key("damping")
        expect("the damping stop", r"behavior=Damping runtime=Damping ")
        key("initialize")
        expect("the initialization", r"behavior=Ready runtime=Tracking ")
        # Hold the authorized window long enough to be more than a handful of
        # recorded rows.
        time.sleep(RECORD_HOLD_S)
        key("damping")
        expect("the damping stop", r"behavior=Damping runtime=Damping ")
        key("exit")
        expect("the ordered exit", r"application: exit stop=confirmed", 25)
        application_process.wait(timeout=15)
        if application_process.returncode != 0:
            fail(f"the application exited with {application_process.returncode}")
    finally:
        if application_process is not None and application_process.poll() is None:
            application_process.send_signal(signal.SIGTERM)
            try:
                application_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                application_process.kill()
                application_process.wait(timeout=10)
        if runtime_process.poll() is None:
            runtime_process.send_signal(signal.SIGTERM)
            try:
                runtime_process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                runtime_process.kill()
                runtime_process.wait(timeout=10)
        application_out.close()
        runtime_out.close()

    if failures:
        return 1

    runs = sorted(path for path in runs_directory.iterdir() if path.is_dir())
    if len(runs) != 1:
        fail(f"the recorder created {len(runs)} run directories, expected one")
        return 1
    run_directory = runs[0]

    summary = subprocess.run(
        [str(read_record), "--run", str(run_directory), "--summary-only"],
        capture_output=True, text=True, check=False,
    )
    if summary.returncode != 0:
        fail(f"read_record failed: {summary.stderr.strip()}")
        return 1
    summary_json = json.loads(summary.stdout.splitlines()[-1])
    if summary_json["schema_version"] != 2:
        fail(f"the run declares schema {summary_json['schema_version']}, expected 2")
    if summary_json["rows_in_range"] <= 0:
        fail("the run recorded no rows")
    if summary_json["sample_hz"] != 100 or summary_json["control_hz"] != 500:
        fail(
            "the run declares another rate than the configured one: "
            f"{summary_json['sample_hz']} Hz on {summary_json['control_hz']} Hz"
        )

    # The rows themselves: the authorized window has to be coherent, and the
    # identity of the command in force has to be the one the application sent.
    rows = subprocess.run(
        [str(read_record), "--run", str(run_directory)],
        capture_output=True, text=True, check=False,
    )
    if rows.returncode != 0:
        fail(f"read_record (rows) failed: {rows.stderr.strip()}")
        return 1
    lines = [line for line in rows.stdout.splitlines() if line.strip()]
    records = [json.loads(line) for line in lines[:-1]]
    if len(records) != summary_json["rows_in_range"]:
        fail(
            f"the reader printed {len(records)} rows but its summary reports "
            f"{summary_json['rows_in_range']}"
        )
    tracking = [row for row in records if row["execution_state"] == 3]
    if not tracking:
        fail("no row of the run shows the runtime tracking")
    else:
        producers = {row["producer_id"] for row in tracking}
        if len(producers) != 1 or "" in producers:
            fail(f"the tracking rows do not name one producer: {sorted(producers)}")
        without_command = [
            row for row in tracking if not row["command_present"]
        ]
        if without_command:
            fail(
                f"{len(without_command)} tracking row(s) carry no command in force"
            )
        epochs = {row["epoch"] for row in tracking}
        if 0 in epochs:
            fail("a tracking row carries no authorization epoch")
        sequences = [row["command_seq"] for row in tracking]
        if any(later < earlier for earlier, later in zip(sequences, sequences[1:])):
            fail("the recorded command sequence goes backwards")
        # The snapshot is one tick: the feedback and the output of a row belong
        # to the state that row names, and the output the runtime produced is
        # recorded next to the target it was asked for. The shipped Fake
        # scenario holds its measured position at the initialization target, so
        # the target itself is constant here by construction - what this run
        # proves is that the row carries both sides at all.
        for row in tracking:
            if row["feedback_stamp_us"] == 0 or row["feedback_seq"] == 0:
                fail("a tracking row carries no feedback snapshot")
                break
            if not row["feedback_valid"]:
                fail("a tracking row reports the feedback it used as invalid")
                break
            if not row["output_valid"]:
                fail("a tracking row carries no output of its own tick")
                break
            if not row["measured_q"] or not row["output_torque"]:
                fail("a tracking row carries no measured feedback or output")
                break
            if not row["send_accepted"]:
                fail("a tracking row reports its output as refused by the host")
                break

    # The run's own session table: the authorization context the rows belong to,
    # with the runtime's explicit `unknown` where the model digest would be.
    sessions_path = run_directory / "sessions.json"
    if not sessions_path.exists():
        fail("the run has no sessions.json")
    else:
        sessions = json.loads(sessions_path.read_text())
        if not isinstance(sessions, list) or not sessions:
            fail("sessions.json holds no session event")
        else:
            if not any(int(entry.get("epoch", 0)) != 0 for entry in sessions):
                fail("no session event names an authorization epoch")
            if not any(
                entry.get("model_sha256") == "unknown" for entry in sessions
            ):
                fail(
                    "the runtime invented a model digest instead of writing "
                    "`unknown`"
                )
            row_epochs = {row["epoch"] for row in tracking}
            session_epochs = {int(entry.get("epoch", 0)) for entry in sessions}
            if not row_epochs & session_epochs:
                fail(
                    "no session event names an epoch the rows were recorded "
                    f"under: rows={sorted(row_epochs)} sessions="
                    f"{sorted(session_epochs)}"
                )

    if failures:
        return 1
    print(
        f"application_record_test: {summary_json['rows_in_range']} row(s), "
        f"{summary_json['sample_hz']} Hz on {summary_json['control_hz']} Hz, "
        f"{len(tracking)} tracking row(s), schema "
        f"{summary_json['schema_version']}, sessions "
        f"{len(json.loads(sessions_path.read_text())) if sessions_path.exists() else 0}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
