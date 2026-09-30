#!/usr/bin/env python3
"""The local policy acceptance: the real application and the real runtime, over
real DDS, driven through a terminal, enter and leave a `Locomotion` behaviour.

    policy_locomotion_test.py <application> <robot_runtime> <source-root>

The deployment is generated (a twelve-joint Fake configuration and a policy ONNX
whose `irmv.deployment` document commands exactly those joints), so this test
needs no simulator, no training environment and no checked-in model. It proves
what the unit tests cannot: that the supervision loop really submits
observations, runs the model in another thread, offers the frame to the
behaviour and publishes it on the execution topic, and that the runtime keeps
tracking it - and that a stop ends it cleanly on both sides.

Nothing here is a machine limit: the model is a two-layer matrix and the gains
are the fake deployment's demonstration values.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

DOMAIN = 227
KEYS = {
    "damping": b"d",
    "initialize": b"i",
    "start_locomotion": b"l",
    "exit": b"x",
}


def wait_for(path: Path, pattern: str, timeout_s: float) -> str:
    """Waits until the file contains the pattern, and returns everything it holds."""

    deadline = time.monotonic() + timeout_s
    text = ""
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="replace")
            if pattern in text:
                return text
        time.sleep(0.1)
    return text


def main() -> int:
    application, runtime, source_root = sys.argv[1], sys.argv[2], Path(sys.argv[3]).resolve()
    failures: list[str] = []

    def fail(note: str) -> None:
        # Reported as it happens: a deployment that does not even start must be
        # diagnosable from the test output alone.
        failures.append(note)
        print(f"policy_locomotion_test: {note}", file=sys.stderr, flush=True)

    with tempfile.TemporaryDirectory(prefix="irmv-policy-") as directory:
        work = Path(directory)
        generated = subprocess.run(
            [
                sys.executable,
                str(source_root / "tools" / "make_test_model.py"),
                "--fake-deployment",
                str(work),
                str(source_root / "config" / "sdk.json"),
                str(DOMAIN),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if generated.returncode != 0:
            print(f"policy_locomotion_test: the deployment was not generated: {generated.stderr}")
            return 1

        config = work / "sdk.json"
        runtime_log = work / "runtime.log"
        application_log = work / "application.log"
        runtime_out = runtime_log.open("w", encoding="utf-8")
        application_out = application_log.open("w", encoding="utf-8")

        runtime_process = subprocess.Popen(
            [
                runtime,
                "--config",
                str(config),
                "--domain",
                str(DOMAIN),
                "--fake-scenario",
                str(work / "fake_scenario.json"),
            ],
            stdout=runtime_out,
            stderr=subprocess.STDOUT,
            cwd=work,
        )
        application_process = None
        try:
            text = wait_for(runtime_log, 'state="ZeroTorque"', 20)
            if 'state="ZeroTorque"' not in text:
                fail(f"the runtime never reached ZeroTorque: {text[-800:]}")
                return 1

            application_process = subprocess.Popen(
                [application, "--config", str(config), "--domain", str(DOMAIN)],
                stdin=subprocess.PIPE,
                stdout=application_out,
                stderr=subprocess.STDOUT,
                cwd=work,
            )

            def key(name: str) -> None:
                application_process.stdin.write(KEYS[name])
                application_process.stdin.flush()

            def expect(note: str, pattern: str, timeout_s: float = 25) -> str:
                text = wait_for(application_log, pattern, timeout_s)
                if pattern not in text:
                    fail(f"{note}: {pattern!r} never appeared; tail: {text[-800:]}")
                return text

            expect("the handover", "behavior=Standby runtime=ZeroTorque ")
            startup = application_log.read_text(encoding="utf-8", errors="replace")
            if "policy=" not in startup or "joints=12" not in startup:
                fail(f"the policy was not loaded from the configured path: {startup[:400]}")

            key("damping")
            expect("the damping stop", "behavior=Damping runtime=Damping ")
            key("initialize")
            expect("the initialization", "behavior=Ready runtime=Tracking ")
            key("start_locomotion")
            locomotion = expect("the locomotion behaviour", "behavior=Locomotion runtime=Tracking ", 30)

            # The runtime must really be tracking the context the upper layer
            # authorized, and it must not have fallen back on its own timeouts.
            runtime_text = runtime_log.read_text(encoding="utf-8", errors="replace")
            # The runtime's own transition line names the state it moved to.
            if 'to="Tracking"' not in runtime_text:
                fail(f"the runtime never entered Tracking; tail: {runtime_text[-900:]}")
            if "behavior=Locomotion runtime=Fault" in locomotion:
                fail("the upper layer faulted while holding Locomotion")

            # Hold the behaviour for more than every target deadline, so a frame
            # that was not refreshed would be reported by the runtime.
            time.sleep(1500 / 1000)
            held = application_log.read_text(encoding="utf-8", errors="replace")
            if "cause=LocalPolicyFailed" in held:
                fail("the local policy failed while holding Locomotion")
            if "stop=CommandTimeout" in held or "stop=SourceStateTimeout" in held:
                fail("the runtime stopped the target while the local policy was running")

            key("exit")
            expect("the ordered exit", "application: exit stop=confirmed", 25)
            application_process.wait(timeout=15)
            if application_process.returncode != 0:
                fail(f"the application exited with {application_process.returncode}")
        finally:
            if application_process is not None and application_process.poll() is None:
                application_process.send_signal(signal.SIGKILL)
                application_process.wait(timeout=10)
            if runtime_process.poll() is None:
                runtime_process.send_signal(signal.SIGTERM)
                try:
                    runtime_process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    runtime_process.send_signal(signal.SIGKILL)
                    runtime_process.wait(timeout=10)
            runtime_out.close()
            application_out.close()

    if failures:
        return 1
    print("policy_locomotion_test: Standby -> Damping -> Initializing -> Ready -> Locomotion -> exit")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
