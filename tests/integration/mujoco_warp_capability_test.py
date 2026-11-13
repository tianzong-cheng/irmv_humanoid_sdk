"""Real CUDA acceptance gate: unavailable hardware is a failure, never a skip."""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main():
    try:
        from tools.simulation.mujoco_warp_engine import MujocoWarpEngine
    except ImportError as error:
        raise AssertionError(f"required simulation adapter unavailable: {error}") from error
    engine = MujocoWarpEngine()
    try:
        engine.initialize(dict(model_path=str(ROOT / "tests/support/simulation/free_base.xml"),
                               joint_names=["joint"], actuator_names=["torque"],
                               base_body="base", weld_name="support", imu_site="imu"))
        initial = engine.read_feedback()
        for _ in range(100):
            held = engine.step(1)
            assert math.dist(held.base_position, initial.base_position) < 1e-4
            dot = abs(sum(a*b for a, b in zip(held.imu_quaternion, initial.imu_quaternion)))
            assert 2 * math.acos(min(1., dot)) < 1e-4
        assert held.physics_step == 100
        # 100 incremental float32 additions of float32(.001); two ULPs.
        assert abs(held.engine_time_s - 0.10000002384185791) <= 2**-26
        assert max(abs(x) for x in held.imu_angular_velocity) < 1e-4
        assert abs(held.imu_specific_force[2] - 9.81) < .02
        engine.apply_torque([.01])
        driven = engine.step(1)
        # Rotor inertia = 2/5 * 1 kg * (0.05 m)^2 = .001 kg m^2.
        assert 9.8 < (driven.dq[0] - held.dq[0]) / .001 < 10.2
        assert abs(driven.tau[0] - .01) < 1e-6
        engine.apply_torque([0.])
        before = engine.read_feedback()
        engine.release_base()
        after = engine.read_feedback()
        assert before.q == after.q and before.dq == after.dq
        assert before.base_position == after.base_position
        assert before.base_velocity == after.base_velocity
        assert before.imu_quaternion == after.imu_quaternion
        fallen = engine.step(100)
        assert fallen.base_position[2] < before.base_position[2] - .01
        assert max(abs(x) for x in fallen.imu_specific_force) < .02
        assert fallen.physics_step == 201
        assert fallen == engine.read_feedback(), "read must observe completed coherent step"
        # Short arithmetic probes, not elapsed-time rollouts. Verify the pinned
        # Warp time kernel's binary32 addition at long diagnostic checkpoints.
        import numpy as np
        for checkpoint, expected in ((1024., 1024.0009765625),
                                     (2048., 2048.0009765625)):
            engine._data.time.assign(np.array([checkpoint],dtype=np.float32))
            engine._time_diagnostic.expected_s=checkpoint
            actual=engine.step(1)
            assert actual.engine_time_s==expected
        # Inject a missing device clock increment after a real completed step.
        # The adapter must fail before incrementing its host completion count.
        previous_step=engine._physics_step
        checkpoint=engine._time_diagnostic.expected_s
        original_step=engine._mjw.step
        def missing_increment(model,data):
            original_step(model,data)
            engine._wp.synchronize_device(engine._device)
            data.time.assign(np.array([checkpoint],dtype=np.float32))
        engine._mjw.step=missing_increment
        try:
            try:engine.step(1)
            except RuntimeError as exc:
                assert 'exact completed float32 additions' in str(exc)
            else:raise AssertionError('missing GPU increment accepted')
            assert engine._physics_step==previous_step
        finally:engine._mjw.step=original_step
        print('PASS: pinned Warp checkpoint additions and missing-increment rejection')
        print("PASS: CUDA hold/release/torque/completed feedback/IMU/no overflow")
    finally:
        engine.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL: MuJoCo Warp GPU capability: {error}", file=sys.stderr)
        sys.exit(1)
