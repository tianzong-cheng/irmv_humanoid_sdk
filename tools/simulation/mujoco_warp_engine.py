"""Single-world CUDA engine; no CPU fallback, implicit PD, or state reset.

MuJoCo Warp 3.9 exposes overflow counters rather than Data.overflow. Check
those before host readback (get_data_into otherwise truncates overflow).
Feedback is refreshed by GPU forward after integration so IMU and joints
refer to the same endpoint. CPU MuJoCo is used only for model/data storage.

One captured single-physics-step CUDA graph is replayed per substep with a
compact diagnostic copy and sync after each replay; overflow/time validation
happens before the next substep may run. A second captured tail graph performs
the endpoint forward and compact feedback readback only after every substep
passed, so a failed substep cannot advance the rest of the interval. Fixed
staging buffers and three graphs are allocated at initialization.
"""

import math
from typing import Sequence

from .engine import Feedback, EngineTimeDiagnostic

_KERNELS = {}


def _warp_kernels(wp):
    """Return snapshot/gather kernels once per Warp module; replay never recompiles."""
    cached = _KERNELS.get(wp)
    if cached is not None:
        return cached

    @wp.kernel
    def snapshot(
        time: wp.array(dtype=wp.float32),
        nacon: wp.array(dtype=wp.int32),
        nefc: wp.array(dtype=wp.int32),
        ncollision: wp.array(dtype=wp.int32),
        slot: int,
        time_out: wp.array(dtype=wp.float32),
        int_out: wp.array(dtype=wp.int32),
    ):
        time_out[slot] = time[0]
        int_out[3 * slot + 0] = nacon[0]
        int_out[3 * slot + 1] = nefc[0]
        int_out[3 * slot + 2] = ncollision[0]

    @wp.kernel
    def gather_counts(
        nacon: wp.array(dtype=wp.int32),
        nefc: wp.array(dtype=wp.int32),
        ncollision: wp.array(dtype=wp.int32),
        out: wp.array(dtype=wp.int32),
    ):
        out[0] = nacon[0]
        out[1] = nefc[0]
        out[2] = ncollision[0]

    @wp.kernel
    def gather(
        time: wp.array(dtype=wp.float32),
        qpos: wp.array2d(dtype=wp.float32),
        qvel: wp.array2d(dtype=wp.float32),
        qacc: wp.array2d(dtype=wp.float32),
        qfrc_actuator: wp.array2d(dtype=wp.float32),
        sensordata: wp.array2d(dtype=wp.float32),
        nq: int,
        nv: int,
        ns: int,
        out: wp.array(dtype=wp.float32),
    ):
        tid = wp.tid()
        if tid == 0:
            out[0] = time[0]
        elif tid <= nq:
            out[tid] = qpos[0, tid - 1]
        elif tid <= nq + nv:
            out[tid] = qvel[0, tid - 1 - nq]
        elif tid <= nq + 2 * nv:
            out[tid] = qacc[0, tid - 1 - nq - nv]
        elif tid <= nq + 3 * nv:
            out[tid] = qfrc_actuator[0, tid - 1 - nq - 2 * nv]
        else:
            out[tid] = sensordata[0, tid - 1 - nq - 3 * nv]

    cached = (snapshot, gather_counts, gather)
    _KERNELS[wp] = cached
    return cached


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
    if (
        not joints
        or len(joints) != len(actuators)
        or len(set(joints)) != len(joints)
        or len(set(actuators)) != len(actuators)
        or len(actuators) != model.nu
    ):
        raise ValueError("require unique, complete joint/actuator mapping")
    base = find(mj.mjtObj.mjOBJ_BODY, config.get("base_body"))
    root = int(model.body_jntadr[base])
    if (
        model.body_parentid[base] != 0
        or model.body_jntnum[base] != 1
        or root < 0
        or model.jnt_type[root] != mj.mjtJoint.mjJNT_FREE
    ):
        raise ValueError("base must have a free joint attached to world")
    weld = find(mj.mjtObj.mjOBJ_EQUALITY, config.get("weld_name"))
    if (
        model.neq != 1
        or model.eq_type[weld] != mj.mjtEq.mjEQ_WELD
        or model.eq_obj1id[weld] != base
        or model.eq_obj2id[weld] != 0
        or not model.eq_active0[weld]
    ):
        raise ValueError("require one initially active base-to-world weld")
    site = find(mj.mjtObj.mjOBJ_SITE, config.get("imu_site"))
    if (
        model.site_bodyid[site] != base
        or not np.array_equal(model.site_quat[site], [1, 0, 0, 0])
        or not np.array_equal(model.site_pos[site], [0, 0, 0])
    ):
        raise ValueError("IMU must use base axes and origin")
    mapping = []
    for joint_name, actuator_name in zip(joints, actuators):
        j = find(mj.mjtObj.mjOBJ_JOINT, joint_name)
        a = find(mj.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if (
            model.jnt_type[j] != mj.mjtJoint.mjJNT_HINGE
            or model.jnt_stiffness[j] != 0
            or model.dof_damping[model.jnt_dofadr[j]] != 0
            or model.jnt_actfrclimited[j]
            or model.actuator_trntype[a] != mj.mjtTrn.mjTRN_JOINT
            or model.actuator_trnid[a, 0] != j
            or not np.array_equal(model.actuator_gear[a], [1, 0, 0, 0, 0, 0])
            or model.actuator_dyntype[a] != mj.mjtDyn.mjDYN_NONE
            or model.actuator_gaintype[a] != mj.mjtGain.mjGAIN_FIXED
            or model.actuator_gainprm[a, 0] != 1
            or model.actuator_biastype[a] != mj.mjtBias.mjBIAS_NONE
            or model.actuator_ctrllimited[a]
            or model.actuator_forcelimited[a]
        ):
            raise ValueError(
                "require direct unit-gear unlimited torque motors without PD"
            )
        mapping.append((int(model.jnt_qposadr[j]), int(model.jnt_dofadr[j]), a))
    controlled = {b for _, b, _ in mapping}
    if controlled != set(range(model.nv)) - set(
        range(int(model.jnt_dofadr[root]), int(model.jnt_dofadr[root]) + 6)
    ):
        raise ValueError("joint mapping must cover every non-base DOF")
    sensors = []
    for kind, size in [
        (mj.mjtSensor.mjSENS_FRAMEQUAT, 4),
        (mj.mjtSensor.mjSENS_GYRO, 3),
        (mj.mjtSensor.mjSENS_ACCELEROMETER, 3),
    ]:
        found = [
            i
            for i in range(model.nsensor)
            if model.sensor_type[i] == kind
            and model.sensor_objtype[i] == mj.mjtObj.mjOBJ_SITE
            and model.sensor_objid[i] == site
        ]
        if len(found) != 1:
            raise ValueError("require one orientation/gyro/accelerometer at IMU site")
        i = found[0]
        if model.sensor_cutoff[i] != 0 or model.sensor_refid[i] != -1:
            raise ValueError("require unfiltered IMU with world orientation reference")
        sensors.append((int(model.sensor_adr[i]), size))
    dt = float(model.opt.timestep)
    if not math.isfinite(dt) or dt <= 0 or abs(dt * 1e6 - round(dt * 1e6)) > 1e-8:
        raise ValueError("physics timestep must be positive integer microseconds")
    # Fail unsupported precision ranges before CUDA initialization.
    EngineTimeDiagnostic(round(dt * 1000000))
    return dict(
        mapping=mapping,
        root_q=int(model.jnt_qposadr[root]),
        root_v=int(model.jnt_dofadr[root]),
        weld=weld,
        sensors=sensors,
        timestep=dt,
    )


def LoadConfiguredSpec(config):
    """Load the declared pose before compilation; no CUDA initialization."""
    import mujoco as mj

    spec = mj.MjSpec.from_file(str(config["model_path"]))
    base = spec.body(config["base_body"])
    if base is None:
        raise ValueError("missing base body")
    if "initial_base_position" in config:
        base.pos = config["initial_base_position"]
        base.quat = config["initial_base_quaternion"]
    return spec


def LoadConfiguredModel(config):
    """Compile the authoritative model without display-only resources."""
    return LoadConfiguredSpec(config).compile()


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
        self._model = LoadConfiguredModel(config)
        self._layout = validate_model(self._model, config)
        self.timestep_s = self._layout["timestep"]
        wp.init()
        devices = wp.get_cuda_devices()
        if not devices:
            raise RuntimeError("CUDA unavailable: GPU acceptance remains open")
        self._device = devices[0]
        self._host = mj.MjData(self._model)
        if "initial_joint_position" in config:
            q = config["initial_joint_position"]
            if len(q) != len(self._layout["mapping"]) or any(
                not math.isfinite(x) for x in q
            ):
                raise ValueError("initial joints must be finite and match mapping")
            for value, (address, _, _) in zip(q, self._layout["mapping"]):
                self._host.qpos[address] = value
        if "initial_base_velocity" in config:
            velocity = config["initial_base_velocity"]
            if len(velocity) != 6 or any(not math.isfinite(x) for x in velocity):
                raise ValueError("initial velocity must contain six finite values")
            address = self._layout["root_v"]
            self._host.qvel[address : address + 6] = velocity
        self._physics_step = 0
        self._time_diagnostic = EngineTimeDiagnostic(round(self.timestep_s * 1000000))
        try:
            with wp.ScopedDevice(self._device):
                self._gpu_model = mjw.put_model(self._model)
                self._data = mjw.put_data(self._model, self._host, nworld=1)
                mjw.forward(self._gpu_model, self._data)
            self._setup_readback()
            self._ready = True
            self.read_feedback()
        except Exception:
            self.close()
            raise

    def _setup_readback(self):
        """Allocate fixed staging and capture the step, tail and readback graphs."""
        wp, dev = self._wp, self._device
        self._snapshot, self._gather_counts, self._gather = _warp_kernels(wp)
        nq, nv, ns = self._model.nq, self._model.nv, self._model.nsensordata
        self._sizes = (nq, nv, ns)
        self._compact_len = 1 + nq + 3 * nv + ns
        with wp.ScopedDevice(dev):
            self._compact = wp.zeros((self._compact_len,), dtype=wp.float32, device=dev)
            self._counts = wp.zeros((3,), dtype=wp.int32, device=dev)
            self._step_time = wp.zeros((1,), dtype=wp.float32, device=dev)
            self._step_int = wp.zeros((3,), dtype=wp.int32, device=dev)
        self._host_compact = wp.zeros(
            (self._compact_len,), dtype=wp.float32, device="cpu", pinned=True
        )
        self._host_counts = wp.zeros((3,), dtype=wp.int32, device="cpu", pinned=True)
        self._host_step_time = wp.zeros(
            (1,), dtype=wp.float32, device="cpu", pinned=True
        )
        self._host_step_int = wp.zeros((3,), dtype=wp.int32, device="cpu", pinned=True)
        self._allocate_ctrl_host()
        self._step_graph = self._capture_step()
        self._tail_graph = self._capture_tail()
        self._readback_graph = self._capture_readback()

    def _launch_readback_kernels(self):
        nq, nv, ns = self._sizes
        d = self._data
        self._wp.launch(
            self._gather,
            dim=self._compact_len,
            inputs=[
                d.time,
                d.qpos,
                d.qvel,
                d.qacc,
                d.qfrc_actuator,
                d.sensordata,
                nq,
                nv,
                ns,
                self._compact,
            ],
        )
        self._wp.launch(
            self._gather_counts,
            dim=1,
            inputs=[d.nacon, d.nefc, d.ncollision, self._counts],
        )

    def _capture_step(self):
        """Capture one physics step plus its compact diagnostics (no execution)."""
        wp, d, m = self._wp, self._data, self._gpu_model
        with wp.ScopedDevice(self._device), wp.ScopedCapture() as capture:
            self._mjw.step(m, d)
            wp.launch(
                self._snapshot,
                dim=1,
                inputs=[
                    d.time,
                    d.nacon,
                    d.nefc,
                    d.ncollision,
                    0,
                    self._step_time,
                    self._step_int,
                ],
            )
        return capture.graph

    def _capture_tail(self):
        """Capture the endpoint forward plus compact feedback readback."""
        wp, d, m = self._wp, self._data, self._gpu_model
        with wp.ScopedDevice(self._device), wp.ScopedCapture() as capture:
            self._mjw.forward(m, d)
            self._launch_readback_kernels()
        return capture.graph

    def _capture_readback(self):
        """Capture feedback counters/state without physics or forward."""
        wp = self._wp
        with wp.ScopedDevice(self._device), wp.ScopedCapture() as capture:
            self._launch_readback_kernels()
        return capture.graph

    def _allocate_ctrl_host(self):
        import numpy as np

        shape = tuple(self._data.ctrl.shape)
        host = self._wp.array(
            np.zeros(shape, dtype=np.float32),
            dtype=self._wp.float32,
            device="cpu",
            pinned=True,
        )
        self._ctrl_host = host
        self._ctrl_scratch = np.zeros(shape, dtype=np.float32)
        self._ctrl_pending = False
        return host

    def _require_ready(self):
        if not self._ready:
            raise RuntimeError("engine not initialized")

    def _check_counts(self, nacon, nefc, ncollision):
        d = self._data
        if (
            int(nacon) > int(d.naconmax)
            or int(nefc) > int(d.njmax)
            or int(ncollision) > int(d.naconmax)
        ):
            raise RuntimeError("MuJoCo Warp capacity overflow")

    def _check_overflow(self):
        d = self._data
        self._check_counts(
            max(int(x) for x in d.nacon.numpy().flat),
            max(int(x) for x in d.nefc.numpy().flat),
            max(int(x) for x in d.ncollision.numpy().flat),
        )

    def _readback(self):
        """Launch the readback graph and copy compact state into pinned staging."""
        wp = self._wp
        with wp.ScopedDevice(self._device):
            wp.capture_launch(self._readback_graph)
            wp.copy(self._host_compact, self._compact)
            wp.copy(self._host_counts, self._counts)
            wp.synchronize_device(self._device)
        self._ctrl_pending = False

    def _feedback_from_host(self, host, validate_time):
        import numpy as np

        nq, nv, _ = self._sizes
        qpos = host[1 : 1 + nq]
        qvel = host[1 + nq : 1 + nq + nv]
        qacc = host[1 + nq + nv : 1 + nq + 2 * nv]
        qfrc = host[1 + nq + 2 * nv : 1 + nq + 3 * nv]
        sens = host[1 + nq + 3 * nv :]
        if not np.isfinite(host[1:]).all():
            raise RuntimeError("nonfinite engine state")
        if validate_time:
            self._time_diagnostic.validate(float(host[0]))
        h, layout = self._host, self._layout
        h.qpos[:] = qpos
        h.qvel[:] = qvel
        h.qacc[:] = qacc
        h.qfrc_actuator[:] = qfrc
        h.sensordata[:] = sens
        h.time = float(host[0])
        q, v = layout["root_q"], layout["root_v"]
        sensors = [
            tuple(float(x) for x in sens[a : a + n]) for a, n in layout["sensors"]
        ]
        return Feedback(
            self._physics_step,
            float(h.time),
            tuple(float(qpos[a]) for a, _, _ in layout["mapping"]),
            tuple(float(qvel[b]) for _, b, _ in layout["mapping"]),
            tuple(float(qfrc[b]) for _, b, _ in layout["mapping"]),
            *sensors,
            tuple(float(x) for x in qpos[q : q + 3]),
            tuple(float(x) for x in qvel[v : v + 6]),
            tuple(float(x) for x in qpos[q + 3 : q + 7]),
        )

    def read_feedback(self) -> Feedback:
        self._require_ready()
        self._readback()
        self._check_counts(*self._host_counts.numpy())
        return self._feedback_from_host(self._host_compact.numpy(), validate_time=True)

    def visual_qpos(self):
        """Worker-only immutable pose from the last coherent feedback read."""
        self._require_ready()
        return tuple(float(x) for x in self._host.qpos)

    def apply_torque(self, torque: Sequence[float]) -> None:
        self._require_ready()
        mapping = self._layout["mapping"]
        if len(torque) != len(mapping) or any(not math.isfinite(x) for x in torque):
            raise ValueError("torque must be finite and match joint mapping")
        host = getattr(self, "_ctrl_host", None)
        if host is None:
            host = self._allocate_ctrl_host()
        # Convert and validate the whole candidate first: a rejected value
        # (including finite doubles that overflow float32) must never touch
        # the pinned buffer a queued H2D may still be reading.
        import numpy as np

        with np.errstate(over="ignore"):
            rounded = np.asarray(tuple(torque), dtype=np.float64).astype(np.float32)
        if not np.isfinite(rounded).all():
            raise ValueError("torque outside engine numeric range")
        scratch = self._ctrl_scratch
        for value, (_, _, actuator) in zip(rounded, mapping):
            scratch[0, actuator] = value
        if not np.isfinite(scratch).all():
            raise ValueError("torque outside engine numeric range")
        if self._ctrl_pending:
            # Ownership fence: the previous apply enqueued an async H2D from
            # the pinned buffer. Every engine sync clears the flag, so this
            # only waits when callers apply twice without an intervening step.
            self._wp.synchronize_device(self._device)
            self._ctrl_pending = False
        host.numpy()[:] = scratch
        with self._wp.ScopedDevice(self._device):
            self._wp.copy(self._data.ctrl, host)
        self._ctrl_pending = True

    def step(self, substeps: int) -> Feedback:
        self._require_ready()
        if (
            type(substeps) is not int
            or substeps <= 0
            or (self._physics_step + substeps) * round(self.timestep_s * 1000000)
            > 3600000000
        ):
            raise ValueError(
                "require positive substeps within 3600-logical-second experiment limit"
            )
        wp = self._wp
        for _ in range(substeps):
            # One captured physics step, then host-visible diagnostics before
            # the next step may run. A failed substep cannot advance the rest
            # of the interval or the final endpoint forward.
            with wp.ScopedDevice(self._device):
                wp.capture_launch(self._step_graph)
                wp.copy(self._host_step_time, self._step_time)
                wp.copy(self._host_step_int, self._step_int)
                wp.synchronize_device(self._device)
            self._ctrl_pending = False
            self._check_counts(*self._host_step_int.numpy())
            self._time_diagnostic.advance()
            self._time_diagnostic.validate(float(self._host_step_time.numpy()[0]))
            self._physics_step += 1
        with wp.ScopedDevice(self._device):
            wp.capture_launch(self._tail_graph)
            wp.copy(self._host_compact, self._compact)
            wp.copy(self._host_counts, self._counts)
            wp.synchronize_device(self._device)
        self._ctrl_pending = False
        self._check_counts(*self._host_counts.numpy())
        return self._feedback_from_host(self._host_compact.numpy(), validate_time=False)

    def release_base(self) -> None:
        self._require_ready()
        active = self._data.eq_active.numpy()
        active[0, self._layout["weld"]] = False
        with self._wp.ScopedDevice(self._device):
            self._data.eq_active.assign(active)
        self._wp.synchronize_device(self._device)
        self._ctrl_pending = False

    def close(self) -> None:
        if self._ready:
            self._wp.synchronize_device(self._device)
        self._ready = False
        self._ctrl_pending = False
        for name in (
            "_step_graph",
            "_tail_graph",
            "_readback_graph",
            "_compact",
            "_counts",
            "_step_time",
            "_step_int",
            "_host_compact",
            "_host_counts",
            "_host_step_time",
            "_host_step_int",
            "_ctrl_host",
            "_ctrl_scratch",
            "_sizes",
            "_snapshot",
            "_gather_counts",
            "_gather",
            "_data",
            "_gpu_model",
            "_host",
            "_model",
        ):
            self.__dict__.pop(name, None)
