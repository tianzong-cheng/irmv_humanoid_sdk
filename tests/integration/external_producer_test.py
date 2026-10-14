#!/usr/bin/env python3
"""The external producer acceptance: the real application, the real runtime and
the real example producer, over real DDS, driven through a terminal.

    external_producer_test.py <application> <robot_runtime> <external_producer> <source-root>

What this proves that no unit test can: a producer process which is not this
application registers, is selected, prepares a first frame, is authorized after
the runtime applied the context the application asked for, and then publishes
the target stream itself - under the producer identity and the epoch the runtime
granted, with the sequence continuing from the first frame the application's
Tracking request carried. It also proves the other half of the interface: the
application publishes no frame of that context at all, and a stop ends the
context on both sides.

The deployment is the shipped Fake configuration, so no model, no simulator and
no training environment is involved. Nothing here is a machine limit.
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

DOMAIN = 153
KEYS = {
    "damping": b"d",
    "initialize": b"i",
    "start_locomotion": b"l",
    "forward": b"w",
    "exit": b"x",
}


def wait_for(path: Path, pattern: str, timeout_s: float) -> str:
    """Waits until the file contains the pattern, and returns everything it holds."""

    deadline = time.monotonic() + timeout_s
    text = ""
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="replace")
            if re.search(pattern, text):
                return text
        time.sleep(0.1)
    return text



def targets_of_epoch(trace_path: Path, epoch: int) -> list:
    """The targets of one epoch the runtime really adopted, deduplicated."""

    targets = []
    last = None
    for line in trace_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        target = record.get("target")
        if not record.get("target_present") or not isinstance(target, dict):
            continue
        if int(target.get("epoch", 0)) != epoch:
            continue
        identity = (target.get("epoch"), target.get("command_seq"))
        if identity == last:
            continue
        last = identity
        targets.append(target)
    return targets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application")
    parser.add_argument("robot_runtime")
    parser.add_argument("external_producer")
    parser.add_argument("source_root", type=Path)
    parser.add_argument(
        "--keep-work",
        type=Path,
        default=None,
        help="run in this directory and keep it, instead of a temporary one",
    )
    args = parser.parse_args()
    application = args.application
    runtime = args.robot_runtime
    producer_binary = args.external_producer
    source_root = args.source_root.resolve()
    # The children run in their own working directory: a relative binary path
    # would be resolved against that directory and not against the caller's.
    application = str(Path(application).resolve())
    runtime = str(Path(runtime).resolve())
    producer_binary = str(Path(producer_binary).resolve())
    failures: list[str] = []

    def fail(note: str) -> None:
        # Reported as it happens: a deployment that does not even start has to be
        # diagnosable from the test output alone.
        failures.append(note)
        print(f"external_producer_test: {note}", file=sys.stderr, flush=True)

    work = (
        args.keep_work
        if args.keep_work is not None
        else Path(tempfile.mkdtemp(prefix="irmv-external-"))
    )
    work.mkdir(parents=True, exist_ok=True)
    try:
        return run(application, runtime, producer_binary, source_root, work,
                   failures, fail)
    finally:
        if args.keep_work is None:
            shutil.rmtree(work, ignore_errors=True)


def run(application, runtime, producer_binary, source_root, work, failures, fail):
    runtime_log = work / "runtime.log"
    application_log = work / "application.log"
    runtime_out = runtime_log.open("w", encoding="utf-8")
    application_out = application_log.open("w", encoding="utf-8")

    # The recorder on, in this run's own directory: the external lifecycle is
    # also what the session sidecar has to describe - one line per authorized
    # context, carrying the producer process the runtime gated and the gains its
    # frames declared, joined to the runtime's own session record by the
    # identity they share.
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

    application_process = None
    runtime_process = subprocess.Popen(
        [
            runtime,
            "--config",
            str(config_path),
            "--domain",
            str(DOMAIN),
            "--fake-scenario",
            str(source_root / "tests" / "support" / "fake_static.json"),
            "--output-trace",
            str(work / "trace_run1.jsonl"),
        ],
        stdout=runtime_out,
        stderr=subprocess.STDOUT,
        cwd=work,
    )
    try:
        text = wait_for(runtime_log, r'state="ZeroTorque"', 20)
        if 'state="ZeroTorque"' not in text:
            fail(f"the runtime never reached ZeroTorque: {text[-800:]}")
            return 1

        # The application runs the external source: it opens the producer
        # endpoints of that interface and no model at all.
        application_process = subprocess.Popen(
            [
                application,
                "--config",
                str(config_path),
                "--domain",
                str(DOMAIN),
                "--locomotion-source",
                "external",
            ],
            stdin=subprocess.PIPE,
            stdout=application_out,
            stderr=subprocess.STDOUT,
            cwd=work,
        )

        def app_text() -> str:
            return application_log.read_text(encoding="utf-8", errors="replace")

        def expect(note: str, pattern: str, timeout_s: float = 25) -> str:
            text = app_text()
            if not re.search(pattern, text):
                text = wait_for(application_log, pattern, timeout_s)
            if not re.search(pattern, text):
                fail(f"{note}: {pattern!r} never appeared; tail: {text[-900:]}")
            return text

        def key(name: str) -> None:
            assert application_process is not None
            assert application_process.stdin is not None
            application_process.stdin.write(KEYS[name])
            application_process.stdin.flush()

        expect("the external source startup", r"policy=none source=external")
        expect("the handover", r"behavior=Standby runtime=ZeroTorque ")

        # The behaviour has to be Ready before the loop prepares anything:
        # a preparation is only useful once a locomotion authorization can
        # be asked for.
        key("damping")
        expect("the damping stop", r"behavior=Damping runtime=Damping ")
        key("initialize")
        expect("the initialization", r"behavior=Ready runtime=Tracking ")

        # The producer processes of this run, each with its own log: the happy
        # path uses one, and the phases after it start more (a restart, a
        # replacement, a recovery). Every one of them is killed and closed by
        # the cleanup below, whatever the run did.
        producers: list = []
        producer_index = 0

        def start_producer():
            nonlocal producer_index
            producer_index += 1
            log = work / f"producer_{producer_index}.log"
            log_out = log.open("w", encoding="utf-8")
            process = subprocess.Popen(
                [
                    producer_binary,
                    "--config",
                    str(config_path),
                    "--domain",
                    str(DOMAIN),
                    "--rate",
                    "50",
                ],
                stdout=log_out,
                stderr=subprocess.STDOUT,
                cwd=work,
            )
            producers.append((process, log_out, log))
            return process, log

        # The producer process: it announces itself, is selected, prepares
        # from the request it is given, and publishes nothing yet.
        producer_process, producer_log = start_producer()

        selected = wait_for(application_log, r"producer=selected id=([0-9a-f]{32})", 20)
        matched = re.search(r"producer=selected id=([0-9a-f]{32})", selected)
        if matched is None:
            fail(f"the producer was never selected; tail: {selected[-800:]}")
            return 1
        producer_id = matched.group(1)

        if re.search(r"producer=publishing", producer_log.read_text(encoding="utf-8", errors="replace")):
            fail("the producer published a frame before any authorization")

        prepared = wait_for(application_log, r"producer=prepared prepare_id=\d+", 20)
        if "producer=prepared" not in prepared:
            fail(f"no first frame was prepared; tail: {prepared[-800:]}")
        if re.search(r"producer=failed", prepared):
            fail("the preparation failed")

        # The operator asks for the locomotion behaviour: the application
        # proposes the prepared frame as the first frame of the Tracking
        # request, under the producer identity the runtime will gate.
        requests_before = len(re.findall(r"producer=authorized", app_text()))
        key("start_locomotion")
        authorized = expect(
            "the locomotion authorization",
            r"behavior=Locomotion runtime=Tracking",
            30,
        )
        epoch_match = re.search(r"producer=authorized epoch=(\d+) first_seq=(\d+)", authorized)
        if epoch_match is None:
            fail(f"the producer was never authorized; tail: {authorized[-900:]}")
            return 1
        if len(re.findall(r"producer=authorized", authorized)) != requests_before + 1:
            fail("the producer was authorized more than once for one request")
        epoch = int(epoch_match.group(1))
        first_seq = int(epoch_match.group(2))

        # Hold the intent, so the producer's own stream is what keeps the
        # runtime tracking: a frame that stopped arriving would be the
        # runtime's CommandTimeout, not an upper fault.
        key("forward")
        time.sleep(2.0)
        held = app_text()
        if "cause=LocalPolicyFailed" in held:
            fail("the local policy latched a fault in the external source")
        if "behavior=Locomotion runtime=Fault" in held:
            fail("the upper layer faulted while holding Locomotion")
        if "stop=CommandTimeout" in held or "stop=SourceStateTimeout" in held:
            fail("the runtime stopped the target while the producer was publishing")

        producer_text = producer_log.read_text(encoding="utf-8", errors="replace")
        if not re.search(r"producer=publishing seq=\d+", producer_text):
            fail(f"the producer never published; tail: {producer_text[-900:]}")
        publishing_lines = re.findall(r"producer=publishing seq=(\d+)", producer_text)
        publishing_seqs = [int(value) for value in publishing_lines]
        if publishing_seqs and publishing_seqs[0] <= first_seq:
            fail(
                "the first published frame did not continue the first frame the "
                f"request carried: seq={publishing_seqs[0]} first_seq={first_seq}"
            )

        # The runtime's own trace is where the third party's frames are
        # visible: every adopted target of the authorized epoch has to carry
        # the granted producer, the granted epoch and a strictly increasing
        # sequence, and the last frame of the application's request has to be
        # the first one of them.
        trace_path = work / "trace_run1.jsonl"
        targets = []
        target_q_values = set()
        last_identity = None
        for line in trace_path.read_text(encoding="utf-8", errors="replace").splitlines():
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            target = record.get("target")
            if not record.get("target_present") or not isinstance(target, dict):
                continue
            if int(target.get("epoch", 0)) != epoch:
                continue
            # A target stays the current one over every tick until the next frame
            # arrives: only the transitions are the adopted stream.
            identity = (target.get("epoch"), target.get("command_seq"))
            if identity == last_identity:
                continue
            last_identity = identity
            targets.append(target)
            values = record.get("target_q")
            if isinstance(values, list) and values:
                target_q_values.add(round(float(values[0]), 9))

        if len(targets) < 20:
            fail(
                "the runtime's trace holds only "
                f"{len(targets)} targets of epoch {epoch}: the producer's "
                "stream was not adopted"
            )
        else:
            sequences = [int(t["command_seq"]) for t in targets]
            if sequences[0] != first_seq:
                fail(
                    f"the first adopted target is not the request's first frame: "
                    f"{sequences[0]} != {first_seq}"
                )
            if any(later <= earlier for earlier, later in zip(sequences, sequences[1:])):
                fail("the adopted target sequence is not strictly increasing")
            if sequences[-1] <= first_seq:
                fail("no target of the producer itself was adopted")
            for target in targets:
                if target.get("producer_id") != producer_id:
                    fail(
                        "an adopted target of the authorized epoch names another "
                        f"producer: {target.get('producer_id')} != {producer_id}"
                    )
                    break
            # The intent reached the frames: the walking target moves once
            # the operator commands a forward velocity, which is what makes
            # the velocity lane a tested path and not a formality.
            if len(target_q_values) < 2:
                fail("the producer's targets never moved while a forward intent was held")

        # The stop: the operator leaves the behaviour, the producer is told,
        # and the ordered exit is confirmed as always.
        key("damping")
        expect("the damping stop", r"behavior=Damping runtime=Damping ")
        stopped = wait_for(producer_log, r"producer=stopped", 10)
        if "producer=stopped" not in stopped:
            fail(f"the producer was never told to stop: {stopped[-600:]}")

        # ------------------------------------------------------------------
        # The lifecycle around the happy path: a producer that restarts, a
        # second producer that replaces the first, and a producer that dies
        # while its context is live. Each of them has to leave the application
        # able to authorize again - a producer is a process, and a process is
        # something that stops, restarts and is replaced.
        #
        # Every wait here is "one more than before": the happy path has already
        # printed each of these lines once, so waiting for a pattern to *exist*
        # would pass on the first producer's line.
        # ------------------------------------------------------------------
        def count_app(pattern: str) -> int:
            return len(re.findall(pattern, app_text()))

        def wait_more(note: str, pattern: str, before: int, timeout_s: float) -> str:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                text = app_text()
                if len(re.findall(pattern, text)) > before:
                    return text
                time.sleep(0.05)
            fail(f"{note}: no new {pattern!r} within {timeout_s:.0f} s")
            return app_text()

        def wait_stop(note: str) -> None:
            """The damping key and the transition it must produce, counted."""

            before = count_app(r"behavior=Damping runtime=Damping ")
            key("damping")
            wait_more(note, r"behavior=Damping runtime=Damping ", before, 15)

        def back_to_ready(note: str) -> None:
            before = count_app(r"behavior=Ready runtime=Tracking ")
            key("initialize")
            wait_more(f"{note}: the initialization", r"behavior=Ready runtime=Tracking ",
                      before, 25)

        def authorize(note: str) -> int:
            """The locomotion key, and the epoch it authorized (0 on failure)."""

            states = count_app(r"behavior=Locomotion runtime=Tracking ")
            notices = count_app(r"producer=authorized ")
            key("start_locomotion")
            text = wait_more(f"{note}: the locomotion authorization",
                             r"behavior=Locomotion runtime=Tracking ", states, 30)
            found = re.findall(r"producer=authorized epoch=(\d+) first_seq=(\d+)", text)
            if len(found) != notices + 1:
                fail(f"{note}: the producer was authorized {len(found) - notices} times")
                return 0
            return int(found[-1][0])

        def selected_before(pattern: str) -> int:
            return count_app(pattern)

        # A producer that restarts: the process that is gone cannot authorize
        # anything, and the one that takes its place has to register, be
        # prepared and be authorized again.
        selected = selected_before(r"producer=selected id=")
        producer_process.send_signal(signal.SIGTERM)
        producer_process.wait(timeout=10)
        back_to_ready("the restart")
        producer_process, producer_log = start_producer()
        text = wait_more("the restarted producer", r"producer=selected id=", selected, 25)
        selected_ids = re.findall(r"producer=selected id=([0-9a-f]{32})", text)
        if selected_ids[-1] == producer_id:
            fail("the restarted producer was selected under the old identity")
        prepared = selected_before(r"producer=prepared ")
        wait_more("the restarted preparation", r"producer=prepared ", prepared, 25)
        restart_epoch = authorize("the restart")
        if restart_epoch != 0 and restart_epoch <= epoch:
            fail(
                f"the restart authorized an older epoch than the first context: "
                f"{restart_epoch} <= {epoch}"
            )
        wait_stop("the stop of the restart")

        # Two producers at once: the second replaces the first, the first is
        # told to stop, and the context the application authorizes belongs to
        # the process that is actually selected.
        replaced_log = producer_log
        back_to_ready("the replacement")
        selected = selected_before(r"producer=selected id=")
        replacing_process, _ = start_producer()
        text = wait_more("the replacing producer", r"producer=selected id=", selected, 25)
        replacing_ids = re.findall(r"producer=selected id=([0-9a-f]{32})", text)
        if replacing_ids[-1] in (producer_id, selected_ids[-1]):
            fail(f"the replacing producer was selected under an identity already used: {replacing_ids[-3:]}")
        prepared = selected_before(r"producer=prepared ")
        wait_more("the replacing preparation", r"producer=prepared ", prepared, 25)
        replacement_epoch = authorize("the replacement")
        if replacement_epoch != 0 and replacement_epoch <= restart_epoch:
            fail(
                "the replacement authorized an older epoch than the restart: "
                f"{replacement_epoch} <= {restart_epoch}"
            )
        # The context the application authorized belongs to the process that is
        # actually selected. A replaced producer that is no longer authorized
        # has nothing to hear in a stop notice, so the runtime's own trace - the
        # producer of every adopted frame - is what this checks.
        replacement_targets = targets_of_epoch(work / "trace_run1.jsonl", replacement_epoch)
        if not replacement_targets:
            fail("the replacement authorized a context the runtime never adopted")
        elif any(
            target.get("producer_id") != replacing_ids[-1]
            for target in replacement_targets
        ):
            fail(
                "the authorized context does not name the replacing producer: "
                f"{sorted({target.get('producer_id') for target in replacement_targets})} "
                f"!= {replacing_ids[-1]}"
            )

        # A process that announces itself while a context is authorized must not
        # take the selection from the producer the runtime is gating: the frame
        # the runtime adopted names one process, and the notice that tells a
        # producer to publish has to name the same one. It is not lost either -
        # it announces itself every 500 ms on its own - it is taken once the
        # context has ended and the application is idle in Ready again.
        deferred_process, deferred_log = start_producer()
        live_selections = count_app(r"producer=selected id=")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if count_app(r"producer=selected id=") > live_selections:
                fail(
                    "a producer that announced itself during a live context took "
                    "the selection from the producer the runtime was gating"
                )
            time.sleep(0.05)
        live_targets = targets_of_epoch(work / "trace_run1.jsonl", replacement_epoch)
        if not live_targets:
            fail("the live context stopped being adopted while it was live")
        elif any(target.get("producer_id") != replacing_ids[-1] for target in live_targets):
            fail(
                "the live context started adopting frames of another producer: "
                f"{sorted({target.get('producer_id') for target in live_targets})}"
            )
        if re.search(
            r"producer=publishing",
            deferred_log.read_text(encoding="utf-8", errors="replace"),
        ):
            fail("a producer that was never authorized published a frame")

        # The stop ends the context, and the selection stays where it is while
        # the application is not idle in Ready: a registration is taken from the
        # state a locomotion context is asked from, not from the damping that
        # follows one.
        wait_stop("the stop before the deferred registration")
        parked = count_app(r"producer=selected id=")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if count_app(r"producer=selected id=") > parked:
                fail("a producer was selected while the application was damping")
            time.sleep(0.05)

        back_to_ready("the deferred registration")
        text = wait_more("the deferred producer", r"producer=selected id=", parked, 25)
        deferred_ids = re.findall(r"producer=selected id=([0-9a-f]{32})", text)
        if deferred_ids[-1] in (producer_id, selected_ids[-1], replacing_ids[-1]):
            fail(
                "the deferred producer was selected under an identity already used: "
                f"{deferred_ids[-3:]}"
            )
        prepared = selected_before(r"producer=prepared ")
        wait_more("the deferred preparation", r"producer=prepared ", prepared, 25)
        deferred_epoch = authorize("the deferred registration")
        if deferred_epoch != 0 and deferred_epoch <= replacement_epoch:
            fail(
                "the deferred registration authorized an older epoch than the "
                f"replacement: {deferred_epoch} <= {replacement_epoch}"
            )
        wait_stop("the stop of the deferred registration")
        # The notice follows the revocation by one iteration of the application's
        # loop. The example producer stays alive after it - a killed process is
        # a different case, and this one is told to stop - so the notice is what
        # its own log has to show.
        stopped = wait_for(deferred_log, r"producer=stopped", 10)
        if "producer=stopped" not in stopped:
            fail(f"the deferred producer was never told to stop: {stopped[-400:]}")

        # The producer dies while its context is live: the runtime falls back on
        # its own command deadline, the application follows it without latching
        # anything, and a fresh process can be authorized again.
        falling = count_app(r"behavior=Damping runtime=Damping ")
        replacing_process.send_signal(signal.SIGKILL)
        replacing_process.wait(timeout=10)
        fell_back = wait_more("the fallback", r"behavior=Damping runtime=Damping ",
                              falling, 15)
        if re.search(r"behavior=Fault", fell_back):
            fail("the application faulted when its producer died")
        back_to_ready("the recovery")
        selected = selected_before(r"producer=selected id=")
        producer_process, producer_log = start_producer()
        wait_more("the recovered producer", r"producer=selected id=", selected, 25)
        prepared = selected_before(r"producer=prepared ")
        wait_more("the recovered preparation", r"producer=prepared ", prepared, 25)
        authorize("the recovery")
        wait_stop("the final stop")

        runtime_text = runtime_log.read_text(encoding="utf-8", errors="replace")
        if 'to="Tracking"' not in runtime_text:
            fail(f"the runtime never entered Tracking; tail: {runtime_text[-900:]}")
        if "FaultLatched" in runtime_text:
            fail("the runtime latched a fault")
    finally:
        for process, log_out, _ in producers:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=10)
            log_out.close()
        for process in (application_process,):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=10)
        if runtime_process.poll() is None:
            runtime_process.send_signal(signal.SIGTERM)
            try:
                runtime_process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                runtime_process.send_signal(signal.SIGKILL)
                runtime_process.wait(timeout=10)
        application_out.close()
        runtime_out.close()

    # The session sidecar. The recorder was on for this run, so every
    # authorized context has to have a line of its own - carrying the
    # producer process the runtime gated and the gains its frames declared -
    # and every line has to join the runtime's own session record by the
    # identity the two share. This is what a run's recording is joined to
    # the source that produced it; the runtime's record cannot say it, since
    # the runtime never reads a deployment contract.
    sidecar = work / "runs" / "policy_sessions.jsonl"
    if not sidecar.is_file():
        fail("the session sidecar was not written")
    else:
        events = [
            json.loads(line)
            for line in sidecar.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.strip()
        ]
        if len(events) < 4:
            fail(f"the sidecar holds {len(events)} session(s), not one per authorized context")
        # The runtime's own per-tick trace is where the identity of an adopted
        # context is written while the run happens, so the join is checked
        # against what the runtime published rather than against a file only a
        # clean shutdown produces.
        trace_rows = [
            json.loads(line)
            for line in (work / "trace_run1.jsonl")
            .read_text(encoding="utf-8", errors="replace")
            .splitlines()
            if line.strip()
        ]
        # A trace row carries the identity only while a context is published:
        # the ones before the handover name neither.
        joined = {
            (row["session_id"], int(row["epoch"]), row["producer_id"])
            for row in trace_rows
            if "session_id" in row and "epoch" in row and "producer_id" in row
        }
        names = {joint["name"] for joint in document["robot"]["joints"]}
        for event in events:
            if event.get("schema") != "irmv.policy_session" or event.get("version") != 1:
                fail(f"a sidecar line is not this schema: {event.get('schema')} v{event.get('version')}")
            if event.get("source") != "external":
                fail(f"an external run recorded source={event.get('source')}")
            key = (event["session_id"], int(event["epoch"]), event["producer_process_id"])
            if key not in joined:
                fail(
                    f"the session record {key} is not joined to any runtime session; "
                    f"the recorder directory holds {sorted(path.name for path in (work / 'runs').iterdir())} "
                    f"and the runtime recorded {sorted(joined)}; the run holds "
                    f"{sorted(path.name for run in (work / 'runs').glob('run_*') for path in run.iterdir())}"
                )
            gains = event.get("gains") or []
            if len(gains) != 2 * len(names):
                fail(f"the external session recorded {len(gains)} gain rows, not two per joint")
            for row in gains:
                if row["gain"] not in ("kp", "kd") or row["joint"] not in names:
                    fail(f"a gain row names {row['joint']}.{row['gain']}")
                if row["model"] != row["effective"]:
                    fail("this application overrode a gain of an external source")

    if failures:
        return 1
    print(
        "external_producer_test: Standby -> Ready -> prepare -> authorization -> "
        "producer stream -> stop"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
