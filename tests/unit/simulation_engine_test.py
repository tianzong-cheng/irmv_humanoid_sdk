"""Validation tests use real compiled models, without stepping a CPU engine."""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


class ValidationTest(unittest.TestCase):
    def test_adapter_exists(self):
        try:
            from tools.simulation.mujoco_warp_engine import MujocoWarpEngine
        except ImportError as error:
            self.fail(f"required engine adapter missing: {error}")
        engine = MujocoWarpEngine()
        for action in (
            engine.read_feedback,
            engine.release_base,
            lambda: engine.step(1),
            lambda: engine.apply_torque([0.0]),
        ):
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

        model = mujoco.MjModel.from_xml_path(
            str(ROOT / "tests/support/simulation/free_base.xml")
        )
        engine = MujocoWarpEngine()
        with wp.ScopedDevice("cpu"):
            engine._data = mjw.make_data(model, nworld=1)
        engine._wp, engine._device = wp, wp.get_device("cpu")
        engine._layout = dict(mapping=[(7, 6, 0)])
        engine._ready = True
        try:
            engine._check_overflow()
            for field, limit in [
                ("nacon", engine._data.naconmax),
                ("nefc", engine._data.njmax),
                ("ncollision", engine._data.naconmax),
            ]:
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
            engine.apply_torque([0.01])
            self.assertAlmostEqual(
                float(engine._data.ctrl.numpy()[0, 0]), 0.01, places=8
            )
            engine.timestep_s = 0.001
            for steps in (0, -1, True, 1.5, 3600001):
                engine._physics_step = 0
                with self.assertRaises(ValueError):
                    engine.step(steps)
        finally:
            engine.close()

    def test_locomotion_rejection_fails_early_without_gpu(self):
        import runpy

        guard = runpy.run_path(
            str(ROOT / "tests/integration/simulation_mujoco_warp_test.py")
        )["check_locomotion_rejection"]
        ready = "[application] behavior=Ready runtime=Tracking reject=- stop=-"
        rejected = (
            ready
            + "\n[application] behavior=Ready runtime=Tracking reject=Discontinuity stop=-"
        )
        timeout = (
            ready
            + "\n[application] behavior=Damping runtime=Damping reject=- stop=CommandTimeout"
        )
        guard(ready, attempted=True, released=False)
        guard(rejected, attempted=False, released=False)
        for output in (rejected, timeout):
            with self.assertRaisesRegex(AssertionError, "Locomotion protection"):
                guard(output, attempted=True, released=False)

    def test_historical_rejection_before_attempt_is_not_current_failure(self):
        import runpy

        guard = runpy.run_path(
            str(ROOT / "tests/integration/simulation_mujoco_warp_test.py")
        )["check_locomotion_rejection"]
        historical = "reject=Discontinuity\nbehavior=Ready reject=-\n"
        guard(
            historical, attempted=True, released=False, attempt_offset=len(historical)
        )
        with self.assertRaisesRegex(AssertionError, "Locomotion protection"):
            guard(
                historical + "stop=CommandTimeout",
                attempted=True,
                released=False,
                attempt_offset=len(historical),
            )

    def test_model_contract(self):
        import mujoco

        from tools.simulation.mujoco_warp_engine import validate_model

        xml = (ROOT / "tests/support/simulation/free_base.xml").read_text()
        config = dict(
            joint_names=["joint"],
            actuator_names=["torque"],
            base_body="base",
            weld_name="support",
            imu_site="imu",
        )
        validate_model(mujoco.MjModel.from_xml_string(xml), config)
        # These defects would silently alter output-side torque or support/IMU semantics.
        for old, new in [
            ('gear="1"', 'gear="2"'),
            ('timestep="0.001"', 'timestep="0.0010005"'),
            ('timestep="0.001"', 'timestep="0.0001"'),
            ('damping="0"', 'damping="1"'),
            ('<site name="imu"/>', '<site name="imu" quat=".70710678 .70710678 0 0"/>'),
            ('ctrllimited="false"', 'ctrllimited="true" ctrlrange="-1 1"'),
            ("<motor ", "<position "),
            ('<freejoint name="root"/>', ""),
            ('body1="base"', 'body1="rotor"'),
            ('site="imu"/>\n  </sensor>', 'site="imu" cutoff="2"/>\n  </sensor>'),
        ]:
            with self.subTest(defect=new):
                with self.assertRaises(ValueError):
                    validate_model(
                        mujoco.MjModel.from_xml_string(xml.replace(old, new)), config
                    )
        for changes in [
            dict(joint_names=["missing"]),
            dict(joint_names=["joint", "joint"]),
            dict(actuator_names=[]),
            dict(imu_site="missing"),
        ]:
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    validate_model(
                        mujoco.MjModel.from_xml_string(xml), dict(config, **changes)
                    )

    def test_preparation_admits_primitive_feet_not_decoy_mesh(self):
        import hashlib
        import itertools
        import json
        import math
        import struct
        import tempfile
        from unittest.mock import patch

        import mujoco
        import numpy as np

        sys.path.insert(0, str(ROOT / "tools"))
        from prepare_simulation_asset import prepare

        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)
            # A real closed tetrahedron, deliberately far below the collision
            # foot. Visual mesh[-1] must not stand in for a primitive geom.
            vertices = [(0, 0, -5), (1, 0, -5), (0, 1, -5), (0, 0, -4)]
            faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
            mesh = bytes(80) + struct.pack("<I", 4)
            for face in faces:
                values = [v for i in face for v in vertices[i]]
                mesh += struct.pack("<12fH", 0, 0, 0, *values, 0)
            (p / "decoy.stl").write_bytes(mesh)
            inertia = '<inertial><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial>'
            urdf = p / "robot.urdf"
            urdf.write_text(f"""<robot name="support-regression">
              <link name="pelvis">{inertia}<visual><geometry><mesh filename="decoy.stl"/></geometry></visual></link>
              <link name="foot">{inertia}<collision><origin xyz="0 0 -.1"/><geometry><box size=".22 .08 .02"/></geometry></collision></link>
              <joint name="test_left" type="revolute"><parent link="pelvis"/><child link="foot"/><origin xyz=".1 0 -.6"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="10"/></joint>
              <link name="leg">{inertia}<collision><origin rpy=".6 0 0"/><geometry><cylinder radius=".06" length=".18"/></geometry></collision></link>
              <joint name="test_right" type="revolute"><parent link="pelvis"/><child link="leg"/><origin xyz="-.1 0 -.4"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="10"/></joint>
            </robot>""")
            policy = p / "policy.onnx"
            policy.write_bytes(b"metadata boundary fixture")
            contract = dict(
                source=dict(
                    asset_revision=hashlib.sha256(urdf.read_bytes()).hexdigest()
                ),
                action=dict(
                    joints=["test_left", "test_right"],
                    default_q=[0.25, -0.3],
                    kp=[100, 100],
                    kd=[6, 6],
                ),
            )
            profile = p / "profile.json"
            profile.write_text(
                json.dumps(
                    dict(
                        self_collision=False,
                        physics_dt_us=1000,
                        foot_armature=0.01,
                        leg_armature=0.01,
                        joint_damping=0,
                        joint_frictionloss=0,
                        contact_friction=[1, 0.005, 0.0001],
                        ground_clearance_m=0.002,
                    )
                )
            )
            # Only ONNX metadata extraction is substituted. URDF import,
            # mesh conversion, compiled kinematics and preparation are real.
            with patch(
                "prepare_simulation_asset.subprocess.check_output",
                return_value=json.dumps(contract).encode(),
            ):
                config = prepare(
                    urdf,
                    policy,
                    p / "prepared",
                    ROOT / "config/simulation.example.json",
                    profile,
                    sys.executable,
                )
                posed = prepare(
                    urdf,
                    policy,
                    p / "posed",
                    ROOT / "config/simulation.example.json",
                    profile,
                    sys.executable,
                    dict(test_left=0.6, test_right=0.5),
                )
                for index, case in enumerate(
                    [
                        dict(test_left=True, test_right=0),
                        dict(test_left=1.5, test_right=0),
                    ]
                ):
                    target = p / f"rejected-{index}"
                    with self.assertRaises(ValueError):
                        prepare(
                            urdf,
                            policy,
                            target,
                            ROOT / "config/simulation.example.json",
                            profile,
                            sys.executable,
                            case,
                        )
                    self.assertFalse(target.exists())
            cfg = json.loads(config.read_text())
            model = mujoco.MjModel.from_xml_path(cfg["simulation"]["model_path"])
            data = mujoco.MjData(model)
            for name, value in zip(
                contract["action"]["joints"], contract["action"]["default_q"]
            ):
                joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                data.qpos[model.jnt_qposadr[joint]] = value
            mujoco.mj_forward(model, data)  # Kinematics only, no CPU stepping.
            support = []
            for i in range(model.ngeom):
                if model.geom_contype[i] != 1:
                    continue
                self.assertEqual(model.geom_dataid[i], -1)
                rotation = data.geom_xmat[i].reshape(3, 3)
                position = data.geom_xpos[i]
                if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX:
                    corners = np.array(
                        list(itertools.product(*[[-s, s] for s in model.geom_size[i]]))
                    )
                    support.append(min((corners @ rotation.T + position)[:, 2]))
                else:
                    self.assertEqual(model.geom_type[i], mujoco.mjtGeom.mjGEOM_CYLINDER)
                    axis = rotation[:, 2]
                    support.append(
                        position[2]
                        - model.geom_size[i, 1] * abs(axis[2])
                        - model.geom_size[i, 0] * math.hypot(axis[0], axis[1])
                    )
            self.assertEqual(len(support), 2)
            self.assertAlmostEqual(
                cfg["simulation"]["initial_base_position"][2] + min(support),
                0.002,
                places=9,
            )
            manifest = json.loads(
                Path(cfg["simulation"]["model_path"])
                .with_suffix(".manifest.json")
                .read_text()
            )
            self.assertEqual(manifest["collision_support_version"], 1)
            posed_cfg = json.loads(posed.read_text())
            posed_manifest = json.loads(
                Path(posed_cfg["simulation"]["model_path"])
                .with_suffix(".manifest.json")
                .read_text()
            )
            pose = dict(test_left=0.6, test_right=0.5)
            self.assertEqual(
                posed_cfg["application"]["initialization"]["target_position"], pose
            )
            self.assertEqual(posed_cfg["simulation"]["initial_joint_position"], pose)
            self.assertEqual(
                posed_manifest["initial_geometry"]["initial_joint_position"], [0.6, 0.5]
            )
            self.assertEqual(
                posed_manifest["initial_geometry"]["initial_base_position"],
                posed_cfg["simulation"]["initial_base_position"],
            )
            self.assertAlmostEqual(
                posed_manifest["initial_height_m"],
                posed_cfg["simulation"]["initial_base_position"][2],
                places=12,
            )
            self.assertNotAlmostEqual(
                posed_cfg["simulation"]["initial_base_position"][2],
                cfg["simulation"]["initial_base_position"][2],
                places=6,
            )


if __name__ == "__main__":
    unittest.main()
