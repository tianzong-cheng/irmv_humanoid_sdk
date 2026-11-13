"""Validation tests use real compiled models, without stepping a CPU engine."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


class ValidationTest(unittest.TestCase):
    def test_adapter_exists(self):
        try:
            from tools.simulation.mujoco_warp_engine import MujocoWarpEngine
        except ImportError as error:
            self.fail(f"required engine adapter missing: {error}")
        engine = MujocoWarpEngine()
        for action in (engine.read_feedback, engine.release_base, lambda: engine.step(1),
                       lambda: engine.apply_torque([0.])):
            with self.assertRaises(RuntimeError):
                action()
        engine.close()
        engine.close()

    def test_real_warp_buffers_reject_overflow_and_invalid_torque(self):
        # CPU storage only: no CPU physics and no substitute GPU acceptance.
        import mujoco
        import mujoco_warp as mjw
        import warp as wp
        from tools.simulation.mujoco_warp_engine import MujocoWarpEngine
        model = mujoco.MjModel.from_xml_path(str(ROOT / "tests/support/simulation/free_base.xml"))
        engine = MujocoWarpEngine()
        with wp.ScopedDevice("cpu"):
            engine._data = mjw.make_data(model, nworld=1)
        engine._wp, engine._device = wp, wp.get_device("cpu")
        engine._layout = dict(mapping=[(7, 6, 0)])
        engine._ready = True
        try:
            engine._check_overflow()
            for field, limit in [("nacon", engine._data.naconmax),
                                 ("nefc", engine._data.njmax),
                                 ("ncollision", engine._data.naconmax)]:
                array = getattr(engine._data, field)
                values = array.numpy()
                original = values.copy()
                values.flat[0] = limit + 1
                array.assign(values)
                with self.assertRaises(RuntimeError):
                    engine._check_overflow()
                array.assign(original)
            for torque in ([], [float("nan")], [float("inf")], [1, 2]):
                with self.assertRaises(ValueError):
                    engine.apply_torque(torque)
            engine.apply_torque([.01])
            self.assertAlmostEqual(float(engine._data.ctrl.numpy()[0, 0]), .01, places=8)
            engine.timestep_s = .001
            for steps in (0, -1, True, 1.5, 3600001):
                engine._physics_step = 0
                with self.assertRaises(ValueError):
                    engine.step(steps)
        finally:
            engine.close()

    def test_model_contract(self):
        import mujoco
        from tools.simulation.mujoco_warp_engine import validate_model
        xml = (ROOT / "tests/support/simulation/free_base.xml").read_text()
        config = dict(joint_names=["joint"], actuator_names=["torque"],
                      base_body="base", weld_name="support", imu_site="imu")
        validate_model(mujoco.MjModel.from_xml_string(xml), config)
        # These defects would silently alter output-side torque or support/IMU semantics.
        for old, new in [('gear="1"', 'gear="2"'),
                         ('timestep="0.001"', 'timestep="0.0010005"'),
                         ('damping="0"', 'damping="1"'),
                         ('ctrllimited="false"', 'ctrllimited="true" ctrlrange="-1 1"'), ('<motor ', '<position '),
                         ('<freejoint name="root"/>', ''),
                         ('body1="base"', 'body1="rotor"'),
                         ('site="imu"/>\n  </sensor>', 'site="imu" cutoff="2"/>\n  </sensor>')]:
            with self.subTest(defect=new):
                with self.assertRaises(ValueError):
                    validate_model(mujoco.MjModel.from_xml_string(xml.replace(old, new)), config)
        for changes in [dict(joint_names=["missing"]), dict(joint_names=["joint", "joint"]),
                        dict(actuator_names=[]), dict(imu_site="missing")]:
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    validate_model(mujoco.MjModel.from_xml_string(xml), dict(config, **changes))


if __name__ == "__main__":
    unittest.main()
