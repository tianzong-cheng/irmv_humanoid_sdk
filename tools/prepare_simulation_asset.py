#!/usr/bin/env python3
"""Explicit URDF -> free-base MuJoCo asset preparation; never edits source assets.

Run in the pinned simulation Python, with --metadata-python pointing at an ONNX
metadata reader environment. Output must be a new ignored build directory.
--initial-pose optionally selects a JSON joint-name->angle stance for preparation
and the application initialization target; it defaults to the policy default_q.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from simulation.assets import (
    closure,
    engine_identity,
    sha256,
    binary_stl,
    stl_obj,
    collision_support_z,
)


def initial_stance(names, movable, default_q, initial_pose):
    """Return the prepared stance in policy-joint order.

    The default remains the policy's training default_q. An explicit stance
    must name exactly the controlled joints with finite, in-limit angles.
    """
    if initial_pose is None:
        return list(default_q)
    if not isinstance(initial_pose, dict):
        raise ValueError("initial pose must be a joint-name to angle object")
    missing = set(names) - set(initial_pose)
    extra = set(initial_pose) - set(names)
    if missing or extra:
        raise ValueError(
            f"initial pose joints differ from controlled joints: missing={sorted(missing)}, extra={sorted(extra)}"
        )
    stance = []
    for name in names:
        value = initial_pose[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"initial pose {name} must be a finite number")
        value = float(value)
        limit = movable[name].find("limit")
        if limit is None:
            raise ValueError(f"URDF joint {name} has no limit")
        try:
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        except (TypeError, ValueError):
            raise ValueError(f"URDF joint {name} has invalid limits") from None
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(
                f"initial pose {name}={value} is outside URDF limits [{lower}, {upper}]"
            )
        stance.append(value)
    return stance


def load_initial_pose(value):
    """Accept an inline JSON object or a path to a JSON file."""
    if value is None:
        return None
    if value.lstrip().startswith("{"):
        return json.loads(value)
    return json.loads(Path(value).read_text())


def prepare(
    urdf, policy, output, template, profile, metadata_python, initial_pose=None
):
    import mujoco as mj
    import numpy as np

    urdf, policy, output = (
        Path(urdf).resolve(),
        Path(policy).resolve(),
        Path(output).resolve(),
    )
    contract = json.loads(
        subprocess.check_output(
            [
                metadata_python,
                "-c",
                'import onnx,sys; m=onnx.load(sys.argv[1]); print(next(p.value for p in m.metadata_props if p.key=="irmv.deployment"))',
                str(policy),
            ],
            text=True,
            timeout=30,
        )
    )
    if contract["source"]["asset_revision"] != sha256(urdf):
        raise ValueError("policy source URDF digest mismatch")
    physical = json.loads(Path(profile).read_text())
    if physical["self_collision"] is not False:
        raise ValueError(
            "this profile converter only supports explicit disabled self collision"
        )
    cfg = json.loads(Path(template).read_text())
    names = contract["action"]["joints"]
    tree = ET.parse(urdf)
    root = tree.getroot()
    movable = {
        j.get("name"): j for j in root.findall("joint") if j.get("type") != "fixed"
    }
    if set(movable) != set(names) or any(
        j.get("type") != "revolute" for j in movable.values()
    ):
        raise ValueError("URDF movable joints must exactly match policy")
    q = initial_stance(names, movable, contract["action"]["default_q"], initial_pose)
    output.mkdir(parents=True, exist_ok=False)
    (output / "meshes").mkdir()
    sources = {urdf.name: sha256(urdf)}
    for node in root.iter("mesh"):
        source = (urdf.parent / node.get("filename")).resolve()
        key = source.relative_to(urdf.parent.parent).as_posix()
        sources[key] = sha256(source)
        dest = output / "meshes" / Path(source.name).with_suffix(".obj")
        if dest.exists() and dest.read_text() != stl_obj(source.read_bytes()):
            raise ValueError("mesh basename collision")
        dest.write_text(stl_obj(source.read_bytes()))
        node.set("filename", str(dest))
    ext = ET.SubElement(root, "mujoco")
    ET.SubElement(
        ext,
        "compiler",
        fusestatic="false",
        discardvisual="false",
        balanceinertia="false",
    )
    with tempfile.TemporaryDirectory(dir=output) as tmp:
        intermediate = Path(tmp) / "source.urdf"
        tree.write(intermediate)
        model = mj.MjModel.from_xml_path(str(intermediate))
        intermediate_xml = Path(tmp) / "converted.xml"
        mj.mj_saveLastXML(str(intermediate_xml), model)
        xml = ET.parse(intermediate_xml)
    r = xml.getroot()
    compiler = r.find("compiler")
    compiler.set("meshdir", "meshes")
    compiler.set("autolimits", "true")
    for node in r.findall("./asset/mesh"):
        node.set("file", Path(node.get("file")).name)
    opt = r.find("option")
    if opt is None:
        opt = ET.SubElement(r, "option")
    opt.set("timestep", str(physical["physics_dt_us"] / 1e6))
    opt.set("gravity", "0 0 -9.81")
    opt.set("iterations", "100")
    opt.set("tolerance", "1e-10")
    base = r.find('./worldbody/body[@name="pelvis"]')
    if base is None:
        raise ValueError("expected pelvis root")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(base, "site", name="imu", pos="0 0 0", quat="1 0 0 0")
    for joint in r.findall(".//joint"):
        name = joint.get("name")
        if name in names:
            joint.set(
                "armature",
                str(
                    physical["foot_armature"]
                    if "ankle" in name
                    else physical["leg_armature"]
                ),
            )
            joint.set("damping", str(physical["joint_damping"]))
            joint.set("frictionloss", str(physical["joint_frictionloss"]))
            joint.attrib.pop("actuatorfrcrange", None)
    for geom in r.findall(".//geom"):
        geom.set("friction", " ".join(map(str, physical["contact_friction"])))
        if geom.get("contype", "1") != "0":
            geom.set("contype", "1")
            geom.set("conaffinity", "2")
    ET.SubElement(
        r.find("worldbody"),
        "geom",
        name="ground",
        type="plane",
        size="5 5 .1",
        contype="2",
        conaffinity="1",
        friction=" ".join(map(str, physical["contact_friction"])),
    )
    equality = ET.SubElement(r, "equality")
    ET.SubElement(
        equality,
        "weld",
        name="support",
        body1="pelvis",
        solref=".002 1",
        solimp=".9999 .9999 .001",
    )
    motors = ET.SubElement(r, "actuator")
    for name in names:
        ET.SubElement(
            motors,
            "motor",
            name=name,
            joint=name,
            gear="1",
            ctrllimited="false",
            forcelimited="false",
        )
    sensors = ET.SubElement(r, "sensor")
    ET.SubElement(
        sensors, "framequat", name="orientation", objtype="site", objname="imu"
    )
    ET.SubElement(sensors, "gyro", name="angular_velocity", site="imu")
    ET.SubElement(sensors, "accelerometer", name="specific_force", site="imu")
    path = output / "robot.xml"
    xml.write(path)
    model = mj.MjModel.from_xml_path(str(path))
    data = mj.MjData(model)
    for name, value in zip(names, q):
        data.qpos[
            model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)]
        ] = value
    mj.mj_forward(model, data)  # CPU kinematics only, no CPU stepping.
    # Support is the lowest compiled collision point, not a visual bounding
    # sphere. URDF collision boxes/cylinders/spheres have no mesh data id, so
    # their support must come from the primitive's own size and orientation.
    kinds = {
        mj.mjtGeom.mjGEOM_BOX: "box",
        mj.mjtGeom.mjGEOM_CYLINDER: "cylinder",
        mj.mjtGeom.mjGEOM_SPHERE: "sphere",
        mj.mjtGeom.mjGEOM_MESH: "mesh",
    }
    lowest = None
    for i in range(model.ngeom):
        if model.geom_contype[i] != 1:
            continue
        kind = kinds.get(int(model.geom_type[i]))
        if kind is None:
            raise ValueError("unsupported collision geometry type")
        vertices = ()
        if kind == "mesh":
            mesh = int(model.geom_dataid[i])
            if mesh < 0:
                raise ValueError("mesh collision geom has no mesh data")
            start = model.mesh_vertadr[mesh]
            count = model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start : start + count]
        zmin = collision_support_z(
            kind,
            [float(x) for x in model.geom_size[i]],
            data.geom_xmat[i].reshape(3, 3).tolist(),
            [float(x) for x in data.geom_xpos[i]],
            vertices,
        )
        lowest = zmin if lowest is None else min(lowest, zmin)
    if lowest is None:
        raise ValueError("no collision geometry for initial support")
    height = -lowest + physical["ground_clearance_m"]
    cfg["robot"]["joints"] = [
        dict(
            name=name,
            position_min=float(movable[name].find("limit").get("lower")),
            position_max=float(movable[name].find("limit").get("upper")),
        )
        for name in names
    ]

    def align(mapping):
        first = next(iter(mapping.values()))
        return {name: first for name in names}

    cfg["runtime"]["damping"]["kd"] = align(cfg["runtime"]["damping"]["kd"])
    for field in ("joint_velocity_limits", "torque_limits", "target_velocity_limits"):
        cfg["runtime"]["safety"][field] = align(cfg["runtime"]["safety"][field])
    for field in cfg["runtime"]["continuity"]:
        cfg["runtime"]["continuity"][field] = align(cfg["runtime"]["continuity"][field])
    init = cfg["application"]["initialization"]
    init["target_position"] = dict(zip(names, q))
    init["gains"] = {
        n: dict(kp=kp, kd=kd)
        for n, kp, kd in zip(names, contract["action"]["kp"], contract["action"]["kd"])
    }
    cfg["application"]["policy"]["model_path"] = str(policy)
    cfg["dds"]["domain"] = 225
    cfg["telemetry"]["directory"] = str(output / "logs")
    (output / "logs").mkdir()
    cfg["simulation"].update(
        socket_directory=str(output / "run"),
        model_path=str(path),
        engine_sha256=engine_identity()[0],
        asset_sha256=closure(path)["digest"],
        physics_dt_us=physical["physics_dt_us"],
        initial_base_position=[0, 0, height],
        initial_base_quaternion=[1, 0, 0, 0],
        initial_joint_position=dict(zip(names, q)),
        joint_mapping={n: dict(joint=n, actuator=n) for n in names},
    )
    manifest = dict(
        source_urdf_sha256=sha256(urdf),
        source_dependencies=dict(sorted(sources.items())),
        source_closure_digest=hashlib.sha256(
            json.dumps(sources, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        historical_training_mesh_bytes_attested=False,
        policy_sha256=sha256(policy),
        physical_profile=physical,
        conversion="MuJoCo URDF inertias/fixed links preserved; free pelvis/weld/direct motors/body IMU/flat plane added; STL converted to OBJ preserving binary32 triangle coordinates (no decimation); convex mesh collision; self collision disabled; no implicit PD/clipping",
        tau_est="qfrc_actuator at mapped joint DOF; excludes constraint reactions",
        imu="site wxyz world orientation; gyro body rad/s; acceleration body specific force including upward gravity at rest",
        collision_support_version=1,
        initial_height_m=height,
        initial_geometry=dict(
            initial_base_position=[0, 0, height],
            initial_base_quaternion=[1, 0, 0, 0],
            initial_joint_position=q,
        ),
        prepared_closure=closure(path),
    )
    path.with_suffix(".manifest.json").write_text(json.dumps(manifest, indent=2))
    (output / "sdk.json").write_text(json.dumps(cfg, indent=2))
    return output / "sdk.json"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("urdf", "policy", "output", "template", "profile"):
        p.add_argument("--" + name, required=True)
    p.add_argument("--metadata-python", default="python3")
    p.add_argument(
        "--initial-pose",
        help="optional JSON object (inline or file path) of controlled joint names to initial angles",
    )
    a = p.parse_args()
    print(
        prepare(
            a.urdf,
            a.policy,
            a.output,
            a.template,
            a.profile,
            a.metadata_python,
            load_initial_pose(a.initial_pose),
        )
    )


if __name__ == "__main__":
    main()
