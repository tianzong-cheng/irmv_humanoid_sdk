"""Real CUDA acceptance gate: unavailable hardware is a failure, never a skip."""

import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

DEVICE_FIELDS = ("time", "qpos", "qvel", "qacc", "qfrc_actuator", "sensordata")


def _device_state(engine):
    data = engine._data
    return {name: getattr(data, name).numpy().copy() for name in DEVICE_FIELDS}


def _manual_steps(engine, count):
    """Reference substeps without the engine's final endpoint forward."""
    with engine._wp.ScopedDevice(engine._device):
        for _ in range(count):
            engine._mjw.step(engine._gpu_model, engine._data)
        engine._wp.synchronize_device(engine._device)


def _assert_same_device_state(observed, reference, label):
    for name in DEVICE_FIELDS:
        assert np.array_equal(observed[name], reference[name]), (
            f"{label}: device {name} differs",
            float(np.max(np.abs(observed[name] - reference[name]))),
        )


def _final_forward_changes(engine):
    before = _device_state(engine)
    with engine._wp.ScopedDevice(engine._device):
        engine._mjw.forward(engine._gpu_model, engine._data)
        engine._wp.synchronize_device(engine._device)
    after = _device_state(engine)
    return not (
        np.array_equal(before["qacc"], after["qacc"])
        and np.array_equal(before["sensordata"], after["sensordata"])
    )


def check_fault_gating(Model, config):
    """A failed substep must not allow later physics or the final forward.

    Isolated engines; references run eager single substeps without the
    engine's endpoint forward. The intermediate case injects overflow at the
    host validation boundary, not real GPU capacity exhaustion.
    """
    cfg = dict(config)
    cfg["initial_base_velocity"] = [0.15, -0.05, 0.0, 0.05, 0.1, 0.2]
    torque = [0.03]

    # (a) clock reversal at the first of three substeps.
    test, reference = Model(), Model()
    try:
        for item in (test, reference):
            item.initialize(cfg)
            item.release_base()
            item.apply_torque(torque)
        dt = test.timestep_s
        start = float(test._data.time.numpy()[0])
        rewind = np.float32(start - 2 * dt)
        test._data.time.assign(np.array([rewind], dtype=np.float32))
        reference._data.time.assign(np.array([rewind], dtype=np.float32))
        test._wp.synchronize_device(test._device)
        reference._wp.synchronize_device(reference._device)
        previous = test._physics_step
        try:
            test.step(3)
        except RuntimeError as exc:
            assert "exact completed float32 additions" in str(exc)
        else:
            raise AssertionError("first-substep clock reversal was not rejected")
        assert test._physics_step == previous, (
            "failed substep advanced completion count"
        )
        _manual_steps(reference, 1)
        observed, expected = _device_state(test), _device_state(reference)
        _assert_same_device_state(observed, expected, "first-substep clock fault")
        advanced = (float(observed["time"][0]) - float(rewind)) / dt
        assert abs(advanced - 1.0) < 1e-3, (
            "later substeps ran after the fault",
            advanced,
        )
        assert _final_forward_changes(reference), (
            "final-forward discriminator is vacuous for this state"
        )
    finally:
        test.close()
        reference.close()

    # (b) overflow surfaced at the host check for the second of three substeps.
    test, reference = Model(), Model()
    try:
        for item in (test, reference):
            item.initialize(cfg)
            item.release_base()
            item.apply_torque(torque)
        dt = test.timestep_s
        start = float(test._data.time.numpy()[0])
        previous = test._physics_step
        original = test._check_counts
        calls = []

        def injected_check(nacon, nefc, ncollision):
            calls.append(1)
            if len(calls) == 2:
                raise RuntimeError("MuJoCo Warp capacity overflow")
            return original(nacon, nefc, ncollision)

        test._check_counts = injected_check
        try:
            try:
                test.step(3)
            except RuntimeError as exc:
                assert "capacity overflow" in str(exc)
            else:
                raise AssertionError("intermediate-substep overflow was not rejected")
        finally:
            test._check_counts = original
        assert test._physics_step - previous == 1, (
            "host completion committed past the failed substep"
        )
        _manual_steps(reference, 2)
        observed, expected = _device_state(test), _device_state(reference)
        _assert_same_device_state(observed, expected, "intermediate-substep fault")
        advanced = (float(observed["time"][0]) - start) / dt
        assert abs(advanced - 2.0) < 1e-3, (
            "later substeps ran after the fault",
            advanced,
        )
        assert _final_forward_changes(reference), (
            "final-forward discriminator is vacuous for this state"
        )
    finally:
        test.close()
        reference.close()
    print(
        "PASS: per-substep fault gating (first clock, intermediate boundary "
        "injection; boundary injection is not real capacity exhaustion)"
    )


def main():
    try:
        from tools.simulation.mujoco_warp_engine import MujocoWarpEngine
    except ImportError as error:
        raise AssertionError(
            f"required simulation adapter unavailable: {error}"
        ) from error
    engine = MujocoWarpEngine()
    config = dict(
        model_path=str(ROOT / "tests/support/simulation/free_base.xml"),
        joint_names=["joint"],
        actuator_names=["torque"],
        base_body="base",
        weld_name="support",
        imu_site="imu",
    )
    try:
        engine.initialize(config)
        initial = engine.read_feedback()
        for _ in range(100):
            held = engine.step(1)
            assert math.dist(held.base_position, initial.base_position) < 1e-4
            dot = abs(
                sum(a * b for a, b in zip(held.imu_quaternion, initial.imu_quaternion))
            )
            assert 2 * math.acos(min(1.0, dot)) < 1e-4
        assert held.physics_step == 100
        # 100 incremental float32 additions of float32(.001); two ULPs.
        assert abs(held.engine_time_s - 0.10000002384185791) <= 2**-26
        assert max(abs(x) for x in held.imu_angular_velocity) < 1e-4
        assert abs(held.imu_specific_force[2] - 9.81) < 0.02
        engine.apply_torque([0.01])
        driven = engine.step(1)
        # Rotor inertia = 2/5 * 1 kg * (0.05 m)^2 = .001 kg m^2.
        assert 9.8 < (driven.dq[0] - held.dq[0]) / 0.001 < 10.2
        assert abs(driven.tau[0] - 0.01) < 1e-6
        # A queued valid H2D must survive a rejected back-to-back apply (finite
        # double that overflows float32), and the latest valid apply must win
        # without rewriting a buffer an earlier transfer may still read.
        engine.apply_torque([0.02])
        try:
            engine.apply_torque([1e300])
        except ValueError:
            pass
        else:
            raise AssertionError("float32-overflow torque accepted")
        queued = engine.step(1)
        assert abs(queued.tau[0] - 0.02) < 1e-6, queued.tau
        engine.apply_torque([0.03])
        engine.apply_torque([0.04])
        latest = engine.step(1)
        assert abs(latest.tau[0] - 0.04) < 1e-6, latest.tau
        engine.apply_torque([0.0])
        before = engine.read_feedback()
        engine.release_base()
        after = engine.read_feedback()
        assert before.q == after.q and before.dq == after.dq
        assert before.base_position == after.base_position
        assert before.base_velocity == after.base_velocity
        assert before.imu_quaternion == after.imu_quaternion
        fallen = engine.step(100)
        assert fallen.base_position[2] < before.base_position[2] - 0.01
        assert max(abs(x) for x in fallen.imu_specific_force) < 0.02
        assert fallen.physics_step == 203
        assert fallen == engine.read_feedback(), (
            "read must observe completed coherent step"
        )
        check_fault_gating(MujocoWarpEngine, config)
        # Short arithmetic probes, not elapsed-time rollouts. Verify the pinned
        # Warp time kernel's binary32 addition at long diagnostic checkpoints.
        for checkpoint, expected in (
            (1024.0, 1024.0009765625),
            (2048.0, 2048.0009765625),
        ):
            engine._data.time.assign(np.array([checkpoint], dtype=np.float32))
            engine._time_diagnostic.expected_s = checkpoint
            actual = engine.step(1)
            assert actual.engine_time_s == expected
        # A missing device clock increment must be rejected before the host
        # completion count moves: rewind by one increment and replay.
        previous_step = engine._physics_step
        checkpoint = engine._time_diagnostic.expected_s
        engine._data.time.assign(
            np.array(
                [checkpoint - round(engine.timestep_s * 1e6) / 1e6], dtype=np.float32
            )
        )
        engine._wp.synchronize_device(engine._device)
        try:
            engine.step(1)
        except RuntimeError as exc:
            assert "exact completed float32 additions" in str(exc)
        else:
            raise AssertionError("missing GPU increment accepted")
        assert engine._physics_step == previous_step
        print("PASS: pinned Warp checkpoint additions and missing-increment rejection")
        print("PASS: CUDA hold/release/torque/completed feedback/IMU/no overflow")
    finally:
        engine.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL: MuJoCo Warp GPU capability: {error}", file=sys.stderr)
        sys.exit(1)
