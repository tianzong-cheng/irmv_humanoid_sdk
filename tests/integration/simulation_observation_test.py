"""Opt-in GPU proof: GPU feedback reaches a policy observation through the real join.

The chain under test is the production one, not a simulation of it:

  MuJoCo Warp (CUDA) -> local coordinator IPC -> SimulationExchange ->
  SimBackend -> RuntimeDriver -> frozen DDS state topic -> ClientTransport ->
  ApplicationDriver -> RobotClient -> BuildObservation.

`simulation_observation_peer` runs the runtime and application roles with those
production classes; this driver owns the real coordinator and the real engine.
The expected observation is computed here from the GPU's own feedback (joint
values and IMU quaternion) with independent math, so an axis swap or an omitted
conversion in the joined path cannot pass.

The startup GPU forward carries known nonzero body rates on all three axes.
The weld then constrains them during stepping. A second real GPU experiment
changes only startup yaw and verifies the policy's gravity observation is yaw
invariant while the delivered RPY yaw changes.

Domain 225 is the serial reserved simulation domain. This test is opt-in with
the GPU suite, requires CUDA and has no CPU fallback or skipped pass.
"""

import json
import math
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from simulation.main import Runner, load_bootstrap
from simulation.mujoco_warp_engine import MujocoWarpEngine

DOMAIN = 225
COMMAND = [0.35, -0.2, 0.55]
BODY_RATE = [0.2, 0.3, 0.4]
Q = [0.37, -0.52]
TICKS = 6
MODEL = """<mujoco>
  <option timestep=".001"/>
  <worldbody>
    <body name="base" pos="0 0 1.2">
      <freejoint/>
      <geom type="sphere" size=".1" mass="1"/>
      <site name="imu"/>
      <body pos=".2 0 0">
        <joint name="test_left" axis="0 0 1"/>
        <geom type="capsule" size=".03" fromto="0 0 0 0 0 .1" mass=".2"/>
      </body>
      <body pos="-.2 0 0">
        <joint name="test_right" axis="0 1 0"/>
        <geom type="capsule" size=".03" fromto="0 0 0 0 0 .1" mass=".2"/>
      </body>
    </body>
  </worldbody>
  <equality><weld name="support" body1="base"/></equality>
  <actuator>
    <motor name="test_left" joint="test_left" gear="1"/>
    <motor name="test_right" joint="test_right" gear="1"/>
  </actuator>
  <sensor>
    <framequat objtype="site" objname="imu"/>
    <gyro site="imu"/>
    <accelerometer site="imu"/>
  </sensor>
</mujoco>"""


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def gravity(quat):
    w, x, y, z = quat
    return [2 * (w * y - x * z), -2 * (w * x + y * z), 2 * (x * x + y * y) - 1]


def rpy(quat):
    w, x, y, z = quat
    return [
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ]


def close_values(actual, expected, context, tolerance=4e-6):
    assert len(actual) == len(expected), context
    for index, (got, want) in enumerate(zip(actual, expected)):
        assert math.isfinite(got) and abs(got - want) <= tolerance, (
            context,
            index,
            got,
            want,
        )


def run(build, peer, yaw):
    directory = Path(tempfile.mkdtemp(prefix="irmv-obs-"))
    runner = thread = None
    engines = []
    result = {}
    ready = threading.Event()
    try:
        model = directory / "model.xml"
        model.write_text(MODEL)
        cfg = json.loads((ROOT / "config/sdk.json").read_text())
        cfg.update(deployment="simulation", backend="sim")
        cfg["dds"]["domain"] = DOMAIN
        cfg["telemetry"]["directory"] = str(directory)
        cfg["telemetry"]["recorder"]["enabled"] = False
        cfg["application"]["policy"]["model_path"] = None
        cfg["simulation"] = dict(
            socket_directory=str(directory),
            engine="mujoco_warp",
            engine_sha256="a" * 64,
            asset_sha256="b" * 64,
            model_path=str(model),
            physics_dt_us=1000,
            host_startup_timeout_us=30000000,
            host_phase_timeout_us=10000000,
            base_body="base",
            imu_site="imu",
            weld_name="support",
            initial_base_position=[0, 0, 1.2],
            initial_base_quaternion=quaternion(0.45, -0.30, yaw),
            initial_joint_position=dict(zip(("test_left", "test_right"), Q)),
            joint_mapping={
                n: dict(joint=n, actuator=n) for n in ("test_left", "test_right")
            },
        )
        config = directory / "sdk.json"
        config.write_text(json.dumps(cfg))
        contract = json.loads(
            (ROOT / "tests/support/deployment/valid_two_axis.json")
            .read_text()
            .replace('"left"', '"test_left"')
            .replace('"right"', '"test_right"')
        )
        deployment = directory / "deployment.json"
        deployment.write_text(json.dumps(contract))
        bootstrap = json.loads(
            subprocess.check_output(
                [str(build / "simulation_normalize"), str(config)],
                text=True,
                timeout=15,
            )
        )
        for key in ("socket_directory", "asset_digest", "policy_path"):
            bootstrap.pop(key)
        bootstrap.update(run_id=secrets.token_hex(16), runtime_id=secrets.token_hex(16))
        bootstrap["engine_config"]["initial_base_velocity"] = [0, 0, 0, *BODY_RATE]
        path = directory / "bootstrap.json"
        path.write_text(json.dumps(bootstrap))
        path.chmod(0o600)

        class MeasuredEngine(MujocoWarpEngine):
            def __init__(self):
                super().__init__()
                self.samples = {}
                self.closed = False
                engines.append(self)

            def _record(self, value):
                # The runner consumes the coherent Feedback returned by the
                # engine itself: the initial readback and each substep batch.
                # Nothing in the join re-reads state, so probing read_feedback
                # alone would miss every advanced step.
                self.samples[value.physics_step] = value
                return value

            def read_feedback(self):
                return self._record(super().read_feedback())

            def step(self, substeps):
                return self._record(super().step(substeps))

            def close(self):
                super().close()
                self.closed = True

        class ReadyRunner(Runner):
            def _advance(self, event):
                super()._advance(event)
                if event.kind == "feedback":
                    ready.set()

        runner = ReadyRunner(
            load_bootstrap(path), directory / "bridge.sock", MeasuredEngine
        )

        def execute():
            try:
                result["code"] = runner.run()
            except BaseException as exc:
                result["error"] = repr(exc)

        thread = threading.Thread(target=execute, daemon=True)
        thread.start()
        deadline = time.monotonic() + 30
        while not ready.wait(0.01):
            assert not result and time.monotonic() < deadline, (
                result,
                runner.coordinator.failure,
            )
        completed = subprocess.run(
            [str(peer), str(config), str(deployment)],
            capture_output=True,
            text=True,
            timeout=45,
        )
        assert completed.returncode == 0, (completed.stdout, completed.stderr)
        rows = [
            json.loads(s[4:])
            for s in completed.stdout.splitlines()
            if s.startswith("OBS ")
        ]
        assert len(rows) == TICKS, completed.stdout
        samples = engines[0].samples
        for step, row in enumerate(rows):
            assert row["stamp_us"] == 1 + step * 2000, row
            assert row["state_seq"] == step + 1, row
            sample = samples[step * 2]
            close_values(row["q"], sample.q, ("q", step), 1e-9)
            close_values(row["dq"], sample.dq, ("dq", step), 1e-9)
            close_values(row["rpy"], rpy(sample.imu_quaternion), ("rpy", step))
            close_values(row["gyro"], sample.imu_angular_velocity, ("gyro", step), 1e-9)
            # Fixture has reversed position order, canonical velocity order,
            # distinct references and identity clip/scale transformations.
            expected = COMMAND + gravity(sample.imu_quaternion)
            expected += list(sample.imu_angular_velocity)
            expected += [sample.q[1] + 0.2, sample.q[0] - 0.1]
            expected += list(sample.dq) + [0, 0]
            close_values(row["obs"], expected, ("observation", step))
        close_values(rows[0]["q"], Q, "known GPU initial joints")
        close_values(rows[0]["gyro"], BODY_RATE, "known GPU body rate")
        close_values(rows[0]["rpy"], [0.45, -0.30, yaw], "known GPU orientation")
        assert abs(rows[-1]["dq"][0]) > 0.01 and abs(rows[-1]["dq"][1]) > 0.01, rows
        assert abs(rows[-1]["dq"][0] - rows[-1]["dq"][1]) > 0.01, rows
        return rows[0]
    finally:
        if runner:
            runner.request_stop()
        if thread:
            thread.join(10)
            assert not thread.is_alive(), "coordinator did not stop"
        if runner and runner.worker:
            runner.worker.join(10)
            assert not runner.worker.is_alive(), "GPU worker did not stop"
        if engines:
            assert engines[0].closed, "GPU engine not closed"
        shutil.rmtree(directory)


if __name__ == "__main__":
    build, peer = map(lambda p: Path(p).resolve(), sys.argv[1:])
    assert (
        str(DOMAIN)
        in json.loads((ROOT / "tests/integration/dds_domains.json").read_text())[
            "reserved"
        ]
    )
    first = run(build, peer, 0.62)
    yaw_changed = run(build, peer, 1.32)
    close_values(
        first["obs"][3:6], yaw_changed["obs"][3:6], "SDK gravity must be yaw invariant"
    )
    assert abs(yaw_changed["rpy"][2] - first["rpy"][2]) > 0.6
    print(
        "PASS real GPU/DDS observation chain: 2x6 ticks, joint mapping, "
        "roll/pitch/yaw, body gyro XYZ and yaw-invariant gravity"
    )
