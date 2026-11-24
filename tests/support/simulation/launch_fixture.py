"""Actual SDK fixture, explicit domain224 and generated test-only ONNX."""

import json
import subprocess
import sys


def prepare(root, build, d):
    cfg = json.loads((root / "config/sdk.json").read_text())
    cfg.update(deployment="simulation", backend="sim")
    cfg["dds"]["domain"] = 224
    cfg["telemetry"]["directory"] = str(d)
    cfg["simulation"] = dict(
        socket_directory=str(d / "run"),
        engine="mujoco_warp",
        engine_sha256="a" * 64,
        model_path=str(root / "tests/support/simulation/free_base.xml"),
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
        (root / "tests/support/deployment/valid_two_axis.json")
        .read_text()
        .replace('"left"', '"test_left"')
        .replace('"right"', '"test_right"')
    )
    for name in ("default_q", "offset", "scale"):
        fixture["action"][name] = [0.0, 0.0]
    for name in ("kp", "kd"):
        fixture["action"][name] = [1.0, 1.0]
    contract = d / "deployment.json"
    contract.write_text(json.dumps(fixture))
    subprocess.run(
        [
            sys.executable,
            str(root / "tools/make_test_model.py"),
            str(d / "models"),
            str(contract),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    cfg["application"]["policy"]["model_path"] = str(d / "models/valid.onnx")
    cfg["application"]["initialization"]["trajectory"].update(
        duration_us=20000, stable_window_us=10000
    )
    config = d / "sdk with spaces.json"
    config.write_text(json.dumps(cfg))
    return config
