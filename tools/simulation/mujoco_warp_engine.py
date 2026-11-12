"""Single-world CUDA engine; no CPU fallback, implicit PD, or state reset.

MuJoCo Warp 3.9 exposes overflow counters rather than Data.overflow. Check
those before host conversion (get_data_into otherwise truncates overflow).
Feedback is refreshed by GPU forward after integration so IMU and joints
refer to the same endpoint. CPU MuJoCo is used only for model/data storage.
"""
import math
from typing import Sequence

from .engine import Feedback


def validate_model(model, config: dict) -> dict:
    """Validate explicit name mapping and torque/IMU/support semantics on host."""
    import mujoco as mj
    import numpy as np

    def find(kind, name):
        if not isinstance(name, str) or not name:
            raise ValueError("model names must be nonempty strings")
        result = mj.mj_name2id(model, kind, name)
        if result < 0:
            raise ValueError(f"missing model name: {name}")
        return result

    joints = config.get("joint_names", [])
    actuators = config.get("actuator_names", [])
    if (not joints or len(joints) != len(actuators) or len(set(joints)) != len(joints)
            or len(set(actuators)) != len(actuators) or len(actuators) != model.nu):
        raise ValueError("require unique, complete joint/actuator mapping")
    base = find(mj.mjtObj.mjOBJ_BODY, config.get("base_body"))
    root = int(model.body_jntadr[base])
    if (model.body_parentid[base] != 0 or model.body_jntnum[base] != 1
            or root < 0 or model.jnt_type[root] != mj.mjtJoint.mjJNT_FREE):
        raise ValueError("base must have a free joint attached to world")
    weld = find(mj.mjtObj.mjOBJ_EQUALITY, config.get("weld_name"))
    if (model.neq != 1 or model.eq_type[weld] != mj.mjtEq.mjEQ_WELD
            or model.eq_obj1id[weld] != base or model.eq_obj2id[weld] != 0
            or not model.eq_active0[weld]):
        raise ValueError("require one initially active base-to-world weld")
    site = find(mj.mjtObj.mjOBJ_SITE, config.get("imu_site"))
    if model.site_bodyid[site] != base:
        raise ValueError("IMU must be rigidly attached to base")
    mapping = []
    for joint_name, actuator_name in zip(joints, actuators):
        j = find(mj.mjtObj.mjOBJ_JOINT, joint_name)
        a = find(mj.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if (model.jnt_type[j] != mj.mjtJoint.mjJNT_HINGE
                or model.jnt_stiffness[j] != 0 or model.dof_damping[model.jnt_dofadr[j]] != 0
                or model.jnt_actfrclimited[j]
                or model.actuator_trntype[a] != mj.mjtTrn.mjTRN_JOINT
                or model.actuator_trnid[a, 0] != j
                or not np.array_equal(model.actuator_gear[a], [1, 0, 0, 0, 0, 0])
                or model.actuator_dyntype[a] != mj.mjtDyn.mjDYN_NONE
                or model.actuator_gaintype[a] != mj.mjtGain.mjGAIN_FIXED
                or model.actuator_gainprm[a, 0] != 1
                or model.actuator_biastype[a] != mj.mjtBias.mjBIAS_NONE
                or model.actuator_ctrllimited[a] or model.actuator_forcelimited[a]):
            raise ValueError("require direct unit-gear unlimited torque motors without PD")
        mapping.append((int(model.jnt_qposadr[j]), int(model.jnt_dofadr[j]), a))
    sensors = []
    for kind, size in [(mj.mjtSensor.mjSENS_FRAMEQUAT, 4),
                       (mj.mjtSensor.mjSENS_GYRO, 3),
                       (mj.mjtSensor.mjSENS_ACCELEROMETER, 3)]:
        found = [i for i in range(model.nsensor) if model.sensor_type[i] == kind
                 and model.sensor_objtype[i] == mj.mjtObj.mjOBJ_SITE
                 and model.sensor_objid[i] == site]
        if len(found) != 1:
            raise ValueError("require one orientation/gyro/accelerometer at IMU site")
        i = found[0]
        if model.sensor_cutoff[i] != 0 or model.sensor_refid[i] != -1:
            raise ValueError("require unfiltered IMU with world orientation reference")
        sensors.append((int(model.sensor_adr[i]), size))
    dt = float(model.opt.timestep)
    if not math.isfinite(dt) or dt <= 0 or abs(dt * 1e6 - round(dt * 1e6)) > 1e-8:
        raise ValueError("physics timestep must be positive integer microseconds")
    return dict(mapping=mapping, root_q=int(model.jnt_qposadr[root]),
                root_v=int(model.jnt_dofadr[root]), weld=weld, sensors=sensors, timestep=dt)


class MujocoWarpEngine:
    def __init__(self):
        self._ready = False

    def initialize(self, config: dict) -> None:
        if self._ready:
            raise RuntimeError("engine already initialized")
        import mujoco as mj
        import mujoco_warp as mjw
        import warp as wp
        self._mjw, self._wp = mjw, wp
        self._model = mj.MjModel.from_xml_path(str(config["model_path"]))
        self._layout = validate_model(self._model, config)
        self.timestep_s = self._layout["timestep"]
        wp.init()
        devices = wp.get_cuda_devices()
        if not devices:
            raise RuntimeError("CUDA unavailable: GPU acceptance remains open")
        self._device = devices[0]
        self._host = mj.MjData(self._model)
        self._physics_step = 0
        try:
            with wp.ScopedDevice(self._device):
                self._gpu_model = mjw.put_model(self._model)
                self._data = mjw.put_data(self._model, self._host, nworld=1)
                mjw.forward(self._gpu_model, self._data)
            self._ready = True
            self.read_feedback()
        except Exception:
            self.close()
            raise

    def _require_ready(self):
        if not self._ready:
            raise RuntimeError("engine not initialized")

    def _check_overflow(self):
        d = self._data
        for array, capacity in [(d.nacon, d.naconmax), (d.nefc, d.njmax),
                                (d.ncollision, d.naconmax)]:
            if (array.numpy() > capacity).any():
                raise RuntimeError("MuJoCo Warp capacity overflow")

    def read_feedback(self) -> Feedback:
        self._require_ready()
        self._wp.synchronize_device(self._device)
        self._check_overflow()
        self._mjw.get_data_into(self._host, self._model, self._data)
        h, layout = self._host, self._layout
        values = (h.qpos, h.qvel, h.qacc, h.qfrc_actuator, h.sensordata)
        if any(not math.isfinite(float(v)) for array in values for v in array):
            raise RuntimeError("nonfinite engine state")
        # Float32 accumulated engine time is diagnostic only, never logical time.
        # Bound experiments to 100000 steps and allow the worst-case sum error.
        expected = self._physics_step * self.timestep_s
        tolerance = max(1e-7, expected * self._physics_step * 1.2e-7)
        if not math.isfinite(h.time) or abs(h.time - expected) > tolerance:
            raise RuntimeError("engine time inconsistent with completed steps")
        q, v = layout["root_q"], layout["root_v"]
        sensors = [tuple(float(x) for x in h.sensordata[a:a+n]) for a, n in layout["sensors"]]
        return Feedback(self._physics_step, float(h.time),
                        tuple(float(h.qpos[a]) for a, _, _ in layout["mapping"]),
                        tuple(float(h.qvel[b]) for _, b, _ in layout["mapping"]),
                        tuple(float(h.qfrc_actuator[b]) for _, b, _ in layout["mapping"]),
                        *sensors, tuple(float(x) for x in h.qpos[q:q+3]),
                        tuple(float(x) for x in h.qvel[v:v+6]))

    def apply_torque(self, torque: Sequence[float]) -> None:
        self._require_ready()
        if len(torque) != len(self._layout["mapping"]) or any(not math.isfinite(x) for x in torque):
            raise ValueError("torque must be finite and match joint mapping")
        controls = self._data.ctrl.numpy()
        for value, (_, _, actuator) in zip(torque, self._layout["mapping"]):
            controls[0, actuator] = value
        if any(not math.isfinite(float(x)) for x in controls.flat):
            raise ValueError("torque outside engine numeric range")
        with self._wp.ScopedDevice(self._device):
            self._data.ctrl.assign(controls)

    def step(self, substeps: int) -> Feedback:
        self._require_ready()
        if type(substeps) is not int or substeps <= 0 or self._physics_step + substeps > 100000:
            raise ValueError("require positive substeps within 100000-step experiment limit")
        with self._wp.ScopedDevice(self._device):
            for _ in range(substeps):
                self._mjw.step(self._gpu_model, self._data)
                self._wp.synchronize_device(self._device)
                self._check_overflow()
                self._physics_step += 1
            self._mjw.forward(self._gpu_model, self._data)
        return self.read_feedback()

    def release_base(self) -> None:
        self._require_ready()
        active = self._data.eq_active.numpy()
        active[0, self._layout["weld"]] = False
        with self._wp.ScopedDevice(self._device):
            self._data.eq_active.assign(active)
        self._wp.synchronize_device(self._device)

    def close(self) -> None:
        if self._ready:
            self._wp.synchronize_device(self._device)
        self._ready = False
        for name in ("_data", "_gpu_model", "_host", "_model"):
            self.__dict__.pop(name, None)
