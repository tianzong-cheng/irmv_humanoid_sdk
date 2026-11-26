"""GLFW presentation window and the viewer-key uplink; never owns physics.

The window is driven on the calling thread so GLFW sees the main thread it was
created on. Rendering never waits for the input channel documented below.
"""

import socket
import time

# The application leases viewer input for 250 ms.  A 30 ms sender interval keeps
# the observed gap below 50 ms even when a frame is rendered between polls.
HEARTBEAT_INTERVAL = 0.03
FRAME_PRESS = b"P"
FRAME_RELEASE = b"R"
FRAME_BLUR = b"B"
FRAME_HEARTBEAT = b"H"

# Transport tests do not need to load the native GLFW library.
GLFW_RELEASE = 0
GLFW_PRESS = 1
MOD_CHORDS = 0x0002 | 0x0004 | 0x0008  # Ctrl, Alt, Super.


class ViewerInput:
    """Routes focused GLFW key events onto the application input channel.

    A frame is ``P``/``R`` plus the configured ASCII key, ``B\\0`` for
    blur or ``H\\0`` for heartbeat, followed by an eight-byte little-endian
    host monotonic timestamp in microseconds. A failed send leaves no backlog.
    """

    def __init__(self, channel, bindings=(), clock=time.monotonic):
        self.channel = channel
        self.bindings = frozenset(
            key
            for key in bindings
            if isinstance(key, str) and len(key) == 1 and 0x20 <= ord(key) <= 0x7E
        )
        self.clock = clock
        self.focused = True
        self.closed = False
        self.ready = False
        self.pending_key = None
        self.last_heartbeat = None
        self.held = {}
        try:
            self.channel.setblocking(False)
        except (AttributeError, OSError):
            pass

    def key_event(self, scancode, action, mods):
        self.pending_key = None
        if self.closed or not self.ready or not self.focused:
            return
        if action == GLFW_RELEASE:
            char = self.held.pop(scancode, None)
            if char is not None and char not in self.held.values():
                self._send(FRAME_RELEASE, char)
        elif action == GLFW_PRESS and not (mods & MOD_CHORDS):
            if scancode not in self.held:
                self.pending_key = scancode

    def text_event(self, codepoint):
        # GLFW delivers the layout-translated character after its key event.
        scancode, self.pending_key = self.pending_key, None
        if scancode is None:
            return
        char = chr(codepoint)
        if char not in self.bindings:
            return
        first = char not in self.held.values()
        self.held[scancode] = char
        if first:
            self._send(FRAME_PRESS, char)

    def focus_event(self, focused):
        focused = bool(focused)
        if self.closed:
            return
        was_focused, self.focused = self.focused, focused
        if was_focused and not focused:
            self.pending_key = None
            self.held.clear()
            if self.ready:
                self._send(FRAME_BLUR)

    def poll(self, now=None):
        """Send a heartbeat on the 30 ms cadence; also called while unfocused."""
        if self.closed:
            return
        if not self.ready:
            self._await_reader()
            if not self.ready:
                return
        now = self.clock() if now is None else now
        if (
            self.last_heartbeat is not None
            and now - self.last_heartbeat < HEARTBEAT_INTERVAL
        ):
            return
        if self._send(FRAME_HEARTBEAT):
            self.last_heartbeat = now

    def _await_reader(self):
        try:
            ack = self.channel.recv(2)
        except BlockingIOError:
            return
        except OSError:
            self.close()
            return
        if ack == b"\x01":
            self.ready = True
        else:
            self.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.held.clear()
        try:
            self.channel.shutdown(socket.SHUT_WR)
        except (AttributeError, OSError):
            pass
        try:
            self.channel.close()
        except (AttributeError, OSError):
            pass

    def _send(self, prefix, char=b"\0"):
        if self.closed:
            return False
        packet = prefix + (char.encode("ascii") if isinstance(char, str) else char)
        packet += int(self.clock() * 1_000_000).to_bytes(8, "little")
        try:
            sent = self.channel.send(packet)
        except OSError:
            self.close()
            return False
        if sent != len(packet):
            self.close()
            return False
        return True


class ViewerWindow:
    """A directly-owned GLFW window rendering with MuJoCo's mjv/mjr calls."""

    def __init__(
        self,
        model,
        data,
        config,
        follow=False,
        no_shadow=False,
        input_stream=None,
    ):
        import glfw
        import mujoco

        self.glfw = glfw
        self.mj = mujoco
        self.model = model
        self.data = data
        self.no_shadow = no_shadow
        self.input_stream = input_stream
        self.window = None
        self.context = None
        self.dragging = None
        self.last_cursor = None
        self.text = ("Waiting for coherent GPU pose", "READ ONLY")
        if not glfw.init():
            raise RuntimeError("GLFW initialization failed")
        try:
            glfw.window_hint(glfw.SAMPLES, 4)
            self.window = glfw.create_window(1280, 800, "IRMV Simulation", None, None)
            if not self.window:
                raise RuntimeError("GLFW window creation failed")
            glfw.make_context_current(self.window)
            glfw.swap_interval(1)
            self.scene = mujoco.MjvScene(model, maxgeom=10000)
            self.option = mujoco.MjvOption()
            self.perturb = mujoco.MjvPerturb()
            self.camera = mujoco.MjvCamera()
            self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)

            base = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, config["base_body"]
            )
            if base < 0:
                raise ValueError("viewer base body not found")
            self.camera.lookat[:] = data.xpos[base]
            self.camera.distance = 2.0
            self.camera.azimuth = 135.0
            self.camera.elevation = -15.0
            if follow:
                self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self.camera.trackbodyid = base
            else:
                self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE

            glfw.set_key_callback(self.window, self._on_key)
            glfw.set_char_callback(self.window, self._on_text)
            self._on_focus(
                self.window, glfw.get_window_attrib(self.window, glfw.FOCUSED)
            )
            glfw.set_window_focus_callback(self.window, self._on_focus)
            glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
            glfw.set_cursor_pos_callback(self.window, self._on_cursor_pos)
            glfw.set_scroll_callback(self.window, self._on_scroll)
        except BaseException:
            self.close()
            raise

    def is_running(self):
        return not self.glfw.window_should_close(self.window)

    def set_pose(self, pose, physics_dt_us):
        self.data.qpos[:] = pose["qpos"]
        self.data.qvel[:] = 0
        self.data.qfrc_applied[:] = 0
        self.data.xfrc_applied[:] = 0
        self.mj.mj_forward(self.model, self.data)
        held = "RELEASED" if pose["released"] else "BASE HELD"
        self.text = (
            f"Simulation {pose['physics_step'] * physics_dt_us / 1e6:.3f}s",
            held,
        )

    def render(self):
        glfw, mujoco = self.glfw, self.mj
        if self.input_stream is not None:
            self.input_stream.poll()
        glfw.poll_events()
        if glfw.window_should_close(self.window):
            return False
        width, height = glfw.get_framebuffer_size(self.window)
        if width <= 0 or height <= 0:
            glfw.swap_buffers(self.window)
            return True
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            self.perturb,
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        if self.no_shadow:
            self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjr_render(viewport, self.scene, self.context)
        mujoco.mjr_overlay(
            mujoco.mjtFontScale.mjFONTSCALE_150,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            viewport,
            self.text[0],
            self.text[1],
            self.context,
        )
        glfw.swap_buffers(self.window)
        if self.input_stream is not None:
            # Polling on both sides of the draw bounds the heartbeat gap by one
            # frame even when the swap waits for vsync.
            self.input_stream.poll()
        return True

    def close(self):
        if self.context is not None:
            try:
                self.context.free()
            except Exception:
                pass
            self.context = None
        if self.window is not None:
            try:
                self.glfw.destroy_window(self.window)
            except Exception:
                pass
            self.window = None
        try:
            self.glfw.terminate()
        except Exception:
            pass

    def _on_key(self, _window, key, scancode, action, mods):
        if key == self.glfw.KEY_ESCAPE and action == self.glfw.PRESS:
            self.camera.type = self.mj.mjtCamera.mjCAMERA_FREE
        if self.input_stream is not None:
            self.input_stream.key_event(scancode, action, mods)

    def _on_text(self, _window, codepoint):
        if self.input_stream is not None:
            self.input_stream.text_event(codepoint)

    def _on_focus(self, _window, focused):
        if self.input_stream is not None:
            self.input_stream.focus_event(focused)

    def _on_mouse_button(self, _window, button, action, _mods):
        glfw = self.glfw
        if action == glfw.PRESS:
            self.dragging = button
            self.last_cursor = glfw.get_cursor_pos(self.window)
        elif action == glfw.RELEASE and self.dragging == button:
            self.dragging = None
            self.last_cursor = None

    def _on_cursor_pos(self, _window, x, y):
        if self.last_cursor is None:
            self.last_cursor = (x, y)
            return
        last_x, last_y = self.last_cursor
        self.last_cursor = (x, y)
        width, height = self.glfw.get_window_size(self.window)
        if width <= 0 or height <= 0:
            return
        dx = (x - last_x) / height
        dy = (y - last_y) / height
        mouse = self.mj.mjtMouse
        glfw = self.glfw
        shift = (
            glfw.get_key(self.window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
            or glfw.get_key(self.window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        )
        if self.dragging == self.glfw.MOUSE_BUTTON_LEFT:
            action = mouse.mjMOUSE_ROTATE_H if shift else mouse.mjMOUSE_ROTATE_V
        elif self.dragging == self.glfw.MOUSE_BUTTON_RIGHT:
            action = mouse.mjMOUSE_MOVE_H if shift else mouse.mjMOUSE_MOVE_V
        elif self.dragging == self.glfw.MOUSE_BUTTON_MIDDLE:
            action = mouse.mjMOUSE_ZOOM
        else:
            return
        self.mj.mjv_moveCamera(self.model, action, dx, dy, self.scene, self.camera)

    def _on_scroll(self, _window, _xoffset, yoffset):
        self.mj.mjv_moveCamera(
            self.model,
            self.mj.mjtMouse.mjMOUSE_ZOOM,
            0.0,
            -yoffset * 0.02,
            self.scene,
            self.camera,
        )
