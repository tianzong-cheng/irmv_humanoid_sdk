"""Optional read-only native display; never a physics or control participant."""

import argparse
import json
import math
import os
import select
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

MAX_PACKET = 16384
MAX_QPOS = 71


def encode(kind, run_id, asset_digest, **payload):
    data = json.dumps(
        dict(version=1, kind=kind, run_id=run_id, asset_digest=asset_digest, **payload),
        allow_nan=False,
    ).encode()
    if len(data) > MAX_PACKET:
        raise ValueError("viewer packet exceeds capacity")
    return data


def decode(data, run_id, asset_digest, nq=None):
    if len(data) > MAX_PACKET:
        raise ValueError("viewer packet exceeds capacity")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("viewer message must be an object")
    if (
        value.get("version") != 1
        or value.get("run_id") != run_id
        or value.get("asset_digest") != asset_digest
    ):
        raise ValueError("viewer identity/version mismatch")
    if value.get("kind") == "pose":
        q = value.get("qpos")
        step = value.get("physics_step")
        if (
            type(step) is not int
            or step < 0
            or not isinstance(q, list)
            or not 0 < len(q) <= MAX_QPOS
            or (nq is not None and len(q) != nq)
            or any(type(x) not in (float, int) or not math.isfinite(x) for x in q)
            or type(value.get("released")) is not bool
        ):
            raise ValueError("invalid viewer pose")
    elif value.get("kind") not in ("initialize", "ready", "closed", "error"):
        raise ValueError("invalid viewer message kind")
    return value


def receive_latest(channel, run_id, asset_digest, nq, last_step):
    """Drain complete display packets; stream EOF retires the orphan viewer."""
    newest = None
    while True:
        try:
            raw = channel.recv(MAX_PACKET + 1)
        except BlockingIOError:
            return newest
        if not raw:
            raise EOFError("viewer owner disconnected")
        pose = decode(raw, run_id, asset_digest, nq)
        if pose["kind"] != "pose":
            raise ValueError("unexpected viewer control packet")
        if pose["physics_step"] > last_step and (
            newest is None or pose["physics_step"] > newest["physics_step"]
        ):
            newest = pose


class ViewerPublisher:
    def __init__(
        self,
        engine_config,
        run_id,
        asset_digest,
        physics_dt_us,
        follow=False,
        no_shadow=False,
    ):
        self.config = engine_config
        self.run_id = run_id.hex() if isinstance(run_id, bytes) else run_id
        self.asset = (
            asset_digest.hex() if isinstance(asset_digest, bytes) else asset_digest
        )
        self.dt = physics_dt_us
        self.follow = follow
        self.no_shadow = no_shadow
        # The launcher hands the viewer one end of its private AF_UNIX
        # SOCK_SEQPACKET socketpair before start().  It is an inherited fd in
        # this process; the launcher closes its own copy after Popen.
        self.input_fd = None
        # Single ASCII keys whose presses/releases the focused window forwards.
        # The launcher fills this from the same configuration the application
        # uses, so both input sources share one binding table.
        self.input_bindings = ()
        self.process = None
        self.socket = None
        self.state = "not_started"
        self.error = ""
        self.dropped = self.superseded = self.sent = 0
        self.last_step = -1
        # Latest-wins display slot. The engine worker only binds an immutable
        # (physics_step, qpos tuple, released) sample under ``slot_lock``; the
        # independent sender serialises the newest one at its own 30 Hz pace.
        # No JSON and no renderer wait can therefore delay a completed step.
        self.pending = None
        self.slot_lock = threading.Lock()
        self.sender = None
        self.stopping = threading.Event()

    def start(self, timeout_s):
        if self.process is not None:
            raise RuntimeError("viewer already started")
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket = parent
        parent.setblocking(False)
        try:
            parent.send(
                encode(
                    "initialize",
                    self.run_id,
                    self.asset,
                    config=self.config,
                    physics_dt_us=self.dt,
                    follow=self.follow,
                    no_shadow=self.no_shadow,
                    input_bindings=list(self.input_bindings),
                )
            )
            environment = dict(os.environ)
            package_root = str(Path(__file__).resolve().parent.parent)
            environment["PYTHONPATH"] = (
                package_root + os.pathsep + environment.get("PYTHONPATH", "")
            )
            command = [
                sys.executable,
                "-m",
                "simulation.viewer",
                "--fd",
                str(child.fileno()),
            ]
            inherited = [child.fileno()]
            if self.input_fd is not None:
                input_fd = self.input_fd
                if type(input_fd) is not int:
                    input_fd = input_fd.fileno()
                if type(input_fd) is not int or input_fd < 0:
                    raise ValueError("viewer input_fd must be a non-negative fd")
                command.extend(["--input-fd", str(input_fd)])
                inherited.append(input_fd)
            self.process = subprocess.Popen(
                command,
                pass_fds=tuple(dict.fromkeys(inherited)),
                env=environment,
                start_new_session=True,
            )
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        self.state = "starting"
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if getattr(self, "cancelled", lambda: False)():
                self.error = "native viewer startup cancelled"
                break
            self.status()
            if self.state == "ready":
                self._start_sender()
                return
            if self.state in ("error", "closed"):
                break
            time.sleep(0.01)
        reason = self.error or "native viewer startup deadline/closure"
        self.close()
        raise RuntimeError(reason)

    def _start_sender(self):
        def send():
            packet = None  # encoded pose awaiting a free display window
            sample = None  # newest published sample, not yet encoded
            next_send = 0.0
            try:
                while not self.stopping.is_set() and self.state == "ready":
                    with self.slot_lock:
                        newest, self.pending = self.pending, None
                    if newest is not None:
                        # A newer completed pose always supersedes an unsent one.
                        if sample is not None or packet is not None:
                            self.superseded += 1
                        sample, packet = newest, None
                    if sample is None and packet is None:
                        self.stopping.wait(0.002)
                        continue
                    remaining = next_send - time.monotonic()
                    if remaining > 0:
                        self.stopping.wait(min(0.002, remaining))
                        continue
                    if packet is None:
                        # Identity, capacity and finite/latest pose checks stay
                        # exactly as before, now off the engine worker.
                        packet = encode(
                            "pose",
                            self.run_id,
                            self.asset,
                            qpos=list(sample[1]),
                            physics_step=sample[0],
                            released=sample[2],
                        )
                        decode(packet, self.run_id, self.asset)
                        sample = None
                    try:
                        self.socket.send(packet)
                        self.sent += 1
                        next_send = time.monotonic() + 1 / 30
                        packet = None
                    except BlockingIOError:
                        select.select([], [self.socket], [], 0.01)
            except Exception as exc:
                if not self.stopping.is_set() and self.state == "ready":
                    self.error, self.state = str(exc), "error"

        self.sender = threading.Thread(
            target=send, name="viewer-display-sender", daemon=True
        )
        self.sender.start()

    def publish(self, qpos, physics_step, released):
        if self.state != "ready" or physics_step <= self.last_step:
            return
        # Worker-side cost is one immutable sample bind. The engine hands over
        # an owned float tuple; any other sequence is copied so a caller cannot
        # mutate a queued display sample. Encoding/validation is the sender's.
        with self.slot_lock:
            if self.pending is not None:
                self.dropped += 1
            self.pending = (
                physics_step,
                qpos if type(qpos) is tuple else tuple(qpos),
                released,
            )
        self.last_step = physics_step

    def status(self):
        if self.socket is not None and self.socket.fileno() >= 0:
            try:
                while True:
                    raw = self.socket.recv(MAX_PACKET + 1)
                    if not raw:
                        if self.state not in ("closed", "error"):
                            self.state = "closing"
                        break
                    msg = decode(raw, self.run_id, self.asset)
                    if msg["kind"] not in ("ready", "closed", "error"):
                        raise ValueError("unexpected viewer status")
                    self.state = msg["kind"]
                    if self.state == "error":
                        self.error = str(msg.get("reason", "native viewer failed"))
            except BlockingIOError:
                pass
            except (OSError, ValueError, TypeError) as exc:
                self.state, self.error = "error", str(exc)
        if self.process is not None and self.process.poll() is not None:
            if self.process.returncode != 0:
                self.state = "error"
                self.error = self.error or "viewer exit " + str(self.process.returncode)
            elif self.state != "error":
                self.state, self.error = "closed", ""
        return dict(
            state=self.state,
            error=self.error,
            sent=self.sent,
            dropped=self.dropped + self.superseded,
            exit_code=None if self.process is None else self.process.returncode,
        )

    def close(self):
        if self.state != "error":
            self.state = "closed"
        self.stopping.set()
        if self.sender is not None:
            self.sender.join(0.2)
        if self.socket is not None and self.socket.fileno() >= 0:
            try:
                self.socket.shutdown(socket.SHUT_WR)
            except OSError:
                pass
        if self.process is not None and self.process.poll() is None:
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
        self.status()
        if self.socket is not None:
            self.socket.close()


def _load_display_model(config):
    """Decorate only the private display spec; never edit physical assets."""
    import mujoco as mj

    from .mujoco_warp_engine import LoadConfiguredSpec

    spec = LoadConfiguredSpec(config)
    headlight = spec.visual.headlight
    headlight.active = 1
    headlight.ambient = [0.3, 0.3, 0.3]
    headlight.diffuse = [0.55, 0.55, 0.55]
    headlight.specular = [0.15, 0.15, 0.15]
    spec.worldbody.add_light(
        name="__irmv_viewer_sun",
        type=mj.mjtLightType.mjLIGHT_DIRECTIONAL,
        pos=[2, -3, 4],
        dir=[-0.4, 0.6, -1],
        castshadow=True,
        ambient=[0.1, 0.1, 0.1],
        diffuse=[0.65, 0.65, 0.65],
        specular=[0.15, 0.15, 0.15],
    )
    if not any(
        texture.type == mj.mjtTexture.mjTEXTURE_SKYBOX for texture in spec.textures
    ):
        spec.add_texture(
            name="__irmv_viewer_sky",
            type=mj.mjtTexture.mjTEXTURE_SKYBOX,
            builtin=mj.mjtBuiltin.mjBUILTIN_GRADIENT,
            width=256,
            height=256,
            rgb1=[0.55, 0.68, 0.82],
            rgb2=[0.9, 0.94, 0.98],
        )
    planes = [geom for geom in spec.geoms if geom.type == mj.mjtGeom.mjGEOM_PLANE]
    if planes:
        texture = spec.add_texture(
            name="__irmv_viewer_grid",
            type=mj.mjtTexture.mjTEXTURE_2D,
            builtin=mj.mjtBuiltin.mjBUILTIN_CHECKER,
            width=256,
            height=256,
            rgb1=[0.65, 0.69, 0.73],
            rgb2=[0.82, 0.85, 0.88],
        )
        material = spec.add_material(
            name="__irmv_viewer_ground",
            texuniform=True,
            texrepeat=[1, 1],
            specular=0.1,
            shininess=0.1,
            reflectance=0,
            rgba=[1, 1, 1, 1],
        )
        material.textures[mj.mjtTextureRole.mjTEXROLE_RGB] = texture.name
        for geom in planes:
            geom.material = material.name
            geom.rgba = [1, 1, 1, 1]
    return spec.compile()


def child_main(fd, input_fd=None):
    channel = socket.socket(fileno=fd)
    channel.settimeout(10)
    raw = channel.recv(MAX_PACKET + 1)
    message = json.loads(raw)
    run_id, asset = message["run_id"], message["asset_digest"]
    window = None
    input_channel = None
    input_stream = None
    try:
        decode(raw, run_id, asset)
        # Rendering resources are compiled only in this child process.
        import mujoco as mj

        from .assets import closure
        from .glfw_viewer import ViewerInput, ViewerWindow

        config = message["config"]
        if closure(config["model_path"])["digest"] != asset:
            raise ValueError("viewer asset closure mismatch")
        model = _load_display_model(config)
        if model.nq > MAX_QPOS:
            raise ValueError("viewer qpos capacity exceeded")
        data = mj.MjData(model)
        for name, q in zip(config["joint_names"], config["initial_joint_position"]):
            index = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[index]] = q
        mj.mj_forward(model, data)  # Kinematics only; never CPU stepping.

        if input_fd is not None and input_fd != fd:
            try:
                input_channel = socket.socket(fileno=input_fd)
            except OSError:
                # A bad optional endpoint must not take the presentation with
                # it; the application simply never sees a viewer press.
                input_channel = None
        bindings = message.get("input_bindings", ())
        if not isinstance(bindings, (list, tuple)):
            bindings = ()
        if input_channel is not None:
            input_stream = ViewerInput(input_channel, bindings)

        window = ViewerWindow(
            model,
            data,
            config,
            follow=message["follow"],
            no_shadow=bool(message.get("no_shadow")),
            input_stream=input_stream,
        )
        window.render()
        channel.send(encode("ready", run_id, asset))
        channel.setblocking(False)
        last = -1
        while window.is_running():
            try:
                newest = receive_latest(channel, run_id, asset, model.nq, last)
            except EOFError:
                return 0
            if newest is not None:
                window.set_pose(newest, message["physics_dt_us"])
                last = newest["physics_step"]
            window.render()
            # swap_buffers already paces against the display; this only keeps
            # a fallback yield so a disabled vsync cannot spin a core.  The
            # short interval also keeps the 30 ms heartbeat cadence.
            time.sleep(0.002)
        channel.send(encode("closed", run_id, asset))
        return 0
    except Exception as exc:
        try:
            channel.send(encode("error", run_id, asset, reason=str(exc)[:512]))
        except OSError:
            pass
        print("simulation viewer: " + str(exc), file=sys.stderr)
        return 1
    finally:
        if input_stream is not None:
            input_stream.close()
        elif input_channel is not None:
            input_channel.close()
        if window is not None:
            window.close()
        channel.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--fd", type=int, required=True)
    parser.add_argument("--input-fd", type=int, default=None)
    arguments = parser.parse_args()
    raise SystemExit(child_main(arguments.fd, arguments.input_fd))
