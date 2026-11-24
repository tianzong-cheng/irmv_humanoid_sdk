"""Actual built SDK processes, real DDS, test-only deterministic engine. Serial domain223."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
assert (
    "223"
    in json.loads((ROOT / "tests/integration/dds_domains.json").read_text())["reserved"]
)
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests/support/simulation"))
from fake_engine import FakeEngine
from simulation.main import Runner, load_bootstrap


def run(build, delay=0, worker_delay=False, stop_pending=False, rejected_handoff=False):
    with tempfile.TemporaryDirectory(prefix="irmv-t7-") as directory:
        d = Path(directory)
        cfg = json.loads((ROOT / "config/sdk.json").read_text())
        cfg.update(deployment="simulation", backend="sim")
        cfg["dds"]["domain"] = 223
        cfg["telemetry"]["directory"] = str(d)
        cfg["simulation"] = dict(
            socket_directory=str(d),
            engine="mujoco_warp",
            engine_sha256="a" * 64,
            model_path=str(ROOT / "tests/support/simulation/free_base.xml"),
            asset_sha256="b" * 64,
            physics_dt_us=1000,
            base_body="base",
            imu_site="imu",
            weld_name="support",
            initial_base_position=[0, 0, 1],
            initial_base_quaternion=[1, 0, 0, 0],
            initial_joint_position=dict(test_left=0, test_right=0),
            joint_mapping=dict(
                test_left=dict(joint="left", actuator="left"),
                test_right=dict(joint="right", actuator="right"),
            ),
        )
        fixture = json.loads(
            (ROOT / "tests/support/deployment/valid_two_axis.json")
            .read_text()
            .replace('"left"', '"test_left"')
            .replace('"right"', '"test_right"')
        )
        fixture["action"]["default_q"] = [0.0, 0.0]
        fixture["action"]["offset"] = [0.25, 0.25] if rejected_handoff else [0.0, 0.0]
        fixture["action"]["scale"] = [0.0, 0.0]
        fixture["action"]["kp"] = [1.0, 1.0]
        fixture["action"]["kd"] = [1.0, 1.0]
        fixture_path = d / "deployment.json"
        fixture_path.write_text(json.dumps(fixture))
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/make_test_model.py"),
                str(d / "models"),
                str(fixture_path),
            ],
            check=True,
        )
        if worker_delay or stop_pending:
            import importlib.util

            spec = importlib.util.spec_from_file_location(
                "fixture_generator", ROOT / "tools/make_test_model.py"
            )
            generator = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(generator)
            # Real ONNX work; no sleeps or completion injection in production workers.
            generator.save(
                d / "models",
                "delayed.onnx",
                generator.make_model(
                    generator.long_running_graph(
                        fixture["model"]["input"]["dimension"],
                        2,
                        iterations=200000 if stop_pending else 10000,
                    ),
                    fixture,
                ),
            )
        cfg["application"]["policy"]["model_path"] = str(
            d
            / "models"
            / ("delayed.onnx" if worker_delay or stop_pending else "valid.onnx")
        )
        cfg["application"]["initialization"]["trajectory"].update(
            duration_us=20000, stable_window_us=10000
        )
        config = d / "sdk.json"
        config.write_text(json.dumps(cfg))
        # Test-only loader mode returns actual C++ normalized digests. No Python canonical hashing.
        digest = subprocess.check_output(
            [str(build / "simulation_driver_test"), str(config)], text=True
        ).split()
        bootstrap = dict(
            run_id="12" * 16,
            runtime_id="34" * 16,
            config_digest=digest[0],
            application_digest=digest[1],
            engine_digest="a" * 64,
            physics_dt_us=1000,
            control_dt_us=2000,
            engine_config=dict(q=[0, 0]),
            phase_timeout_us=3000000,
        )
        (d / "bootstrap.json").write_text(json.dumps(bootstrap))
        (d / "bootstrap.json").chmod(0o600)

        class DelayedEngine(FakeEngine):
            def step(self, n):
                if delay:
                    time.sleep(delay)
                return super().step(n)

        class ObservedRunner(Runner):
            application = None
            facts = []
            commands = []
            stop_sent = False
            stop_at = None

            def _advance(self, event):
                f = event.frame
                if f is not None:
                    if f.kind == 3:
                        self.facts.append((f.control_step, f.payload))
                    if f.kind == 4 and f.payload["message"]["kind"] == 5:
                        self.commands.append((f.control_step, f.payload["message"]))
                    if f.kind == 16:
                        self.commands.append((f.control_step, f.payload["command"]))
                return super()._advance(event)

            def _actions(self, actions):
                for a in actions:
                    if (
                        a.kind == "send"
                        and a.peer == 3
                        and a.frame.kind == 15
                        and self.application
                    ):
                        key = {60: b"d", 70: b"i", 120: b"l"}.get(a.frame.control_step)
                        if key:
                            self.application.stdin.write(key)
                            self.application.stdin.flush()
                    if delay and a.kind == "send":
                        time.sleep(delay)
                return super()._actions(actions)

        runner = ObservedRunner(
            load_bootstrap(d / "bootstrap.json"), d / "bridge.sock", DelayedEngine
        )
        thread = threading.Thread(target=runner.run)
        thread.start()
        processes = []
        logs = []
        try:
            for name in ("robot_runtime", "application"):
                log = open(d / (name + ".log"), "w+")
                logs.append(log)
                processes.append(
                    subprocess.Popen(
                        [str(build / name), "--config", str(config)],
                        stdin=subprocess.PIPE,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        cwd=ROOT,
                    )
                )
                if name == "application":
                    runner.application = processes[-1]
            deadline = time.monotonic() + 40
            # Run past the last adoption boundary: the final adoption is
            # recorded at the start of the following step, so stopping exactly
            # at 180 made the compared list depend on interleaving rather than
            # on the property under test.
            while runner.coordinator.step < 210 and time.monotonic() < deadline:
                if (
                    stop_pending
                    and runner.coordinator.step == 120
                    and runner.coordinator.state == "ApplicationPhase"
                    and not runner.stop_sent
                ):
                    # Tick120 arms local inference. Wait for the actual phase to remain pending.
                    if runner.stop_at is None:
                        runner.stop_at = time.monotonic()
                    elif time.monotonic() - runner.stop_at > 0.02:
                        processes[1].stdin.write(b"x")
                        processes[1].stdin.flush()
                        runner.stop_sent = True
                if any(p.poll() is not None for p in processes):
                    break
                time.sleep(0.005)
            for log in logs:
                log.flush()
                log.seek(0)
            output = "\n".join(log.read() for log in logs)
            if stop_pending:
                assert runner.stop_sent, (
                    "did not exercise pending inference",
                    runner.coordinator.state,
                    output,
                )
                processes[1].wait(
                    timeout=max(0.01, 2 - (time.monotonic() - runner.stop_at))
                )
                assert processes[1].poll() is not None, ("stop not responsive", output)
                assert processes[1].returncode == 0, (
                    "graceful leave was not confirmed",
                    output,
                )
                assert runner.coordinator.runtime_only, runner.coordinator.failure
                assert processes[0].poll() is None, output
                assert time.monotonic() - runner.stop_at < 2, (
                    "stop exceeded host bound",
                    output,
                )
                print(
                    "PASS actual application stop while inference pending; LeaveCommitted and runtime retained"
                )
                return None
            assert runner.coordinator.step >= 210, (
                runner.coordinator.failure,
                runner.coordinator.state,
                output,
            )
            assert not runner.coordinator.failure, runner.coordinator.failure
            if rejected_handoff:
                assert "reject=Discontinuity" in output, output
                assert "stop=CommandTimeout" not in output, output
                assert "behavior=Fault" not in output, output
                assert runner.coordinator.release_state == "Fixed", output
                assert runner.coordinator.states[-1]["execution_state"] == 3, output
                proposals = [
                    c for s, c in runner.commands if s >= 120 and c["sequence"] == 1
                ]
                assert len(proposals) == 1, (
                    "rejected takeover was retried",
                    proposals,
                    output,
                )
                held = [(s, t["command"]) for s, t in runner.facts if s >= 130]
                before = next(t["command"] for s, t in runner.facts if s == 110)
                assert held and all(
                    c["epoch"] == before["epoch"]
                    and c["producer_id"] == before["producer_id"]
                    for _, c in held
                ), held
                assert held[-1][1]["sequence"] > held[0][1]["sequence"], held
                print(
                    "PASS actual SDK rejected policy handoff preserves Ready hold without retry/release/watchdog expiry"
                )
                return None
            adoptions = []
            last = None
            for step, torque in runner.facts:
                cmd = torque["command"]
                if cmd["sequence"] and cmd != last:
                    candidates = [(s, c) for s, c in runner.commands if c == cmd]
                    assert candidates, ("invented actual target", step, cmd)
                    assert candidates[-1][0] + 1 == step, (
                        "not next tick",
                        step,
                        candidates[-1],
                    )
                    assert cmd["stamp_us"] == 1 + candidates[-1][0] * 2000
                    assert cmd["source_state_seq"] <= torque["state_seq"] - 1
                    if step > 121:
                        assert cmd["source_state_seq"] == step, (
                            "policy output delayed an extra strategy period",
                            step,
                            cmd,
                        )
                    if step > 120:
                        adoptions.append(
                            (
                                step,
                                cmd["sequence"],
                                cmd["source_state_seq"],
                                cmd["stamp_us"],
                            )
                        )
                last = cmd if cmd["sequence"] else None
            assert len(adoptions) >= 5, ("no 50Hz locomotion", adoptions, output)
            assert all(b[0] - a[0] == 10 for a, b in zip(adoptions, adoptions[1:])), (
                adoptions
            )
            print("PASS actual SDK 50/500Hz next-tick adoption", adoptions)
            return adoptions
        finally:
            runner.request_stop()
            thread.join(3)
            for p in processes:
                if p.poll() is None:
                    p.send_signal(signal.SIGTERM)
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()
            for log in logs:
                log.close()


if __name__ == "__main__":
    build = Path(sys.argv[1]).resolve()
    baseline = run(build)
    delayed = run(build, 0.001, worker_delay=True)
    # Each arm is bounded by a control-step count, so the adoption at the stop
    # boundary may be observed on one side only. The property under test is that
    # injected delays never change WHICH step adopts a command: compare the
    # common prefix entry by entry and require a real sample of adoptions.
    common = min(len(baseline), len(delayed))
    assert common >= 5, (baseline, delayed)
    assert baseline[:common] == delayed[:common], (baseline, delayed)
    run(build, stop_pending=True)
    run(build, rejected_handoff=True)
