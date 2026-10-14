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

import hashlib
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

        # The recorder on, and one experimental gain override keyed to this
        # model's own digest: the run is then also the evidence that a recorded
        # context is joined to the model that produced it and to the gains this
        # deployment actually sent, which is what the runtime's own session
        # record cannot say - it never reads a deployment contract.
        model_sha256 = hashlib.sha256((work / "policy.onnx").read_bytes()).hexdigest()
        document = json.loads((work / "sdk.json").read_text(encoding="utf-8"))
        document["telemetry"]["recorder"] = {
            "enabled": True,
            "sample_hz": 100,
            "queue_capacity": 256,
            "directory": str(work / "runs"),
        }
        overridden_joint = document["robot"]["joints"][0]["name"]
        document["application"]["policy_overrides"] = [
            {"model_sha256": model_sha256, "joints": {overridden_joint: {"kp": 60.0}}}
        ]
        (work / "sdk.json").write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        (work / "runs").mkdir(parents=True, exist_ok=True)

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
                "--output-trace",
                str(work / "trace_run1.jsonl"),
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

        # The session sidecar: one line for the context this run
        # authorized, carrying the model that was armed, the contract it
        # declared, the one gain this deployment overrode and the identity
        # it shares with the runtime's own session record.
        sidecar = work / "runs" / "policy_sessions.jsonl"
        if not sidecar.is_file():
            fail("the session sidecar was not written")
        else:
            events = [
                json.loads(line)
                for line in sidecar.read_text(encoding="utf-8", errors="replace").splitlines()
                if line.strip()
            ]
            if len(events) != 1:
                fail(f"the sidecar holds {len(events)} session(s), not one for the one context")
            else:
                event = events[0]
                if event.get("schema") != "irmv.policy_session" or event.get("version") != 1:
                    fail(f"the sidecar line is {event.get('schema')} v{event.get('version')}")
                if event.get("source") != "local":
                    fail(f"a local run recorded source={event.get('source')}")
                model = event.get("model") or {}
                if model.get("sha256") != model_sha256:
                    fail(f"the session names model {model.get('sha256')}, not {model_sha256}")
                if model.get("path") != str((work / "policy.onnx").resolve()):
                    fail(f"the session names the model at {model.get('path')}")
                if model.get("input") != "obs" or model.get("output") != "actions":
                    fail("the session does not name the model's own I/O")
                if len(model.get("contract_sha256") or "") != 64:
                    fail("the session does not name the deployment contract it was joined to")
                gains = event.get("gains") or []
                if len(gains) != 1 or gains[0].get("joint") != overridden_joint:
                    fail(f"the session recorded {gains}, not the one overridden gain")
                elif gains[0].get("gain") != "kp" or gains[0].get("effective") != 60.0:
                    fail(f"the overridden gain is recorded as {gains[0]}")
                elif gains[0].get("model") == gains[0].get("effective"):
                    fail("the session does not record both sides of the override")
                # The runtime's own per-tick trace is where the identity of an
                # adopted context is written while the run happens.
                trace_rows = [
                    json.loads(line)
                    for line in (work / "trace_run1.jsonl")
                    .read_text(encoding="utf-8", errors="replace")
                    .splitlines()
                    if line.strip()
                ]
                # A trace row carries the identity only while a context is
                # published: the ones before the handover name neither.
                joined = {
                    (row["session_id"], int(row["epoch"]))
                    for row in trace_rows
                    if "session_id" in row and "epoch" in row
                }
                if (event["session_id"], int(event["epoch"])) not in joined:
                    fail(
                        "the session record is not joined to any runtime session: "
                        f"{(event['session_id'], event['epoch'])} not in {sorted(joined)}; "
                        f"the recorder directory holds "
                        f"{sorted(path.name for path in (work / 'runs').iterdir())}"
                    )

    if failures:
        return 1
    print("policy_locomotion_test: Standby -> Damping -> Initializing -> Ready -> Locomotion -> exit")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
