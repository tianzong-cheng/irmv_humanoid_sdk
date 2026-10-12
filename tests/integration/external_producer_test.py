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

import json
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

DOMAIN = 229
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


def main() -> int:
    application, runtime, producer_binary, source_root = (
        sys.argv[1],
        sys.argv[2],
        sys.argv[3],
        Path(sys.argv[4]).resolve(),
    )
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

    with tempfile.TemporaryDirectory(prefix="irmv-external-") as directory:
        work = Path(directory)
        producer_log = work / "producer.log"
        runtime_log = work / "runtime.log"
        application_log = work / "application.log"
        producer_out = producer_log.open("w", encoding="utf-8")
        runtime_out = runtime_log.open("w", encoding="utf-8")
        application_out = application_log.open("w", encoding="utf-8")

        producer_process = None
        application_process = None
        runtime_process = subprocess.Popen(
            [
                runtime,
                "--config",
                str(source_root / "config" / "sdk.json"),
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
                    str(source_root / "config" / "sdk.json"),
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

            # The producer process: it announces itself, is selected, prepares
            # from the request it is given, and publishes nothing yet.
            producer_process = subprocess.Popen(
                [
                    producer_binary,
                    "--config",
                    str(source_root / "config" / "sdk.json"),
                    "--domain",
                    str(DOMAIN),
                    "--rate",
                    "50",
                ],
                stdout=producer_out,
                stderr=subprocess.STDOUT,
                cwd=work,
            )

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

            key("exit")
            expect("the ordered exit", r"application: exit stop=confirmed", 25)
            application_process.wait(timeout=15)
            if application_process.returncode != 0:
                fail(f"the application exited with {application_process.returncode}")

            runtime_text = runtime_log.read_text(encoding="utf-8", errors="replace")
            if 'to="Tracking"' not in runtime_text:
                fail(f"the runtime never entered Tracking; tail: {runtime_text[-900:]}")
            if "FaultLatched" in runtime_text:
                fail("the runtime latched a fault")
        finally:
            for process in (application_process, producer_process):
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
            producer_out.close()
            runtime_out.close()
            application_out.close()

    if failures:
        return 1
    print(
        "external_producer_test: Standby -> Ready -> prepare -> authorization -> "
        "producer stream -> stop"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
