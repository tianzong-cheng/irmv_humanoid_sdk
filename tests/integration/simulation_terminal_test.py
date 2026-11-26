"""Real control PTY and viewer key channel, real SDK/DDS, fake physics."""

import errno
import fcntl
import json
import os
import pty
import re
import select
import signal
import socket
import sys
import tempfile
import termios
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests/support/simulation"))


def peer(build, config, relay_fd=None):
    from fake_engine import FakeEngine
    from sim_launch import Experiment

    class Engine(FakeEngine):
        def initialize(self, cfg):
            super().initialize(dict(q=[0, 0]))

    relay = None if relay_fd is None else socket.socket(fileno=relay_fd)

    class Viewer:
        def __init__(self, *args, **kwargs):
            self.output = None

        def start(self, _timeout):
            self.output = socket.socket(fileno=os.dup(self.input_fd))
            relay.setblocking(False)

        def publish(self, *_pose):
            pass

        def status(self):
            while self.output is not None:
                try:
                    packet = relay.recv(2)
                except BlockingIOError:
                    break
                if not packet:
                    self.close()
                    break
                stamp = (time.monotonic_ns() // 1000).to_bytes(8, "little")
                self.output.send(packet + stamp)
            return {"state": "ready" if self.output is not None else "closed"}

        def close(self):
            if self.output is not None:
                self.output.close()
                self.output = None

    owner = Experiment(
        config,
        build / "robot_runtime",
        build / "application",
        engine_factory=Engine,
        binding=lambda n: {"fixture_only": True},
        viewer=relay is not None,
        viewer_factory=Viewer,
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, owner.notify_shutdown)
    try:
        owner.start()
        print(
            "TERMINAL_CHILDREN " + json.dumps([p.pid for p in owner.children]),
            flush=True,
        )
        return owner.wait()
    finally:
        owner.close()


def run_case(build, quit_application):
    from launch_fixture import prepare

    with tempfile.TemporaryDirectory(prefix="irmv-tty-") as tmp:
        directory = Path(tmp)
        config = prepare(ROOT, build, directory)
        controls, relay = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        if not quit_application:
            cfg = json.loads(config.read_text())
            cfg["telemetry"]["recorder"]["enabled"] = True
            config.write_text(json.dumps(cfg))
        # Capture restoration baseline before the child can acquire key mode.
        # Give the peer its own controlling terminal and foreground group,
        # matching an interactive shell rather than substituting a key pipe.
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        pid = os.fork()
        if pid == 0:
            try:
                controls.close()
                relay.set_inheritable(not quit_application)
                os.close(master)
                os.setsid()
                fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
                for fd in (0, 1, 2):
                    os.dup2(slave, fd)
                if slave > 2:
                    os.close(slave)
                os.environ["TERM"] = "xterm-256color"
                if quit_application:
                    os.environ["NO_COLOR"] = "1"
                else:
                    os.environ.pop("NO_COLOR", None)
                os.execv(
                    sys.executable,
                    [
                        sys.executable,
                        "-u",
                        str(Path(__file__).resolve()),
                        str(build),
                        "--peer",
                        str(config),
                        *([] if quit_application else [str(relay.fileno())]),
                    ],
                )
            finally:
                os._exit(127)
        os.close(slave)
        relay.close()
        text = ""
        reaped = False
        children = []

        def drain(timeout):
            nonlocal text, children
            if not select.select([master], [], [], timeout)[0]:
                return
            try:
                data = os.read(master, 65536)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    return
                raise
            text += data.decode(errors="replace")
            for line in text.splitlines():
                if line.startswith("TERMINAL_CHILDREN "):
                    children = json.loads(line[len("TERMINAL_CHILDREN ") :])

        def expect(needle, timeout=12):
            deadline = time.monotonic() + timeout
            while needle not in text and time.monotonic() < deadline:
                drain(min(0.1, max(0, deadline - time.monotonic())))
            app = directory / "run/application.log"
            assert needle in text, (
                "missing interactive feedback",
                needle,
                text,
                app.read_text() if app.exists() else "no application log",
            )

        def settle(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                drain(0.02)

        try:
            expect("[application] behavior=Standby")
            # Bytes have no newline: application must own immediate key mode.
            if quit_application:
                os.write(master, b"d")
            else:
                controls.send(b"H\0")
                controls.send(b"Pd")
            expect("[application] behavior=Damping")
            os.write(master, b"i")
            expect("[application] behavior=Ready")
            if not quit_application:
                controls.send(b"H\0")
                os.write(master, b"q")
                settle(0.12)
                controls.send(b"Pw")
                deadline = time.monotonic() + 0.65
                while time.monotonic() < deadline:
                    controls.send(b"H\0")
                    drain(0.03)
                controls.send(b"Rw")
                settle(0.12)
                controls.send(b"H\0")
                controls.send(b"Pa")
                settle(0.12)
                controls.send(b"B\0")
                settle(0.12)
                controls.send(b"H\0")
                controls.send(b"Ps")
                settle(0.45)  # Host lease expiry without a key-up or heartbeat.
                controls.send(b"H\0")
                controls.send(b"Pf")
                settle(0.12)
                controls.close()
                settle(0.12)
            os.write(master, b"l")
            expect("[application] behavior=Locomotion")
            plain = re.sub(r"\x1b\[[0-9;]*m", "", text)
            lines = [
                line
                for line in plain.splitlines()
                if line.startswith("[application] behavior=")
            ]
            for field in ("runtime=", "output=", "session="):
                assert len({line.index(field) for line in lines}) == 1, lines
            assert ("\x1b[" in text) != quit_application, text
            if quit_application:
                os.write(master, b"x")
                expect("exit stop=confirmed state=Damping")
                assert os.waitpid(pid, os.WNOHANG) == (0, 0), (
                    "app x killed launcher/runtime"
                )
                assert len(children) == 2
                os.kill(children[0], 0)
                # The final log precedes main's TerminalGuard destructor.
                # Observe restoration, not an assumed print/exit ordering.
                restore_deadline = time.monotonic() + 2
                while (
                    termios.tcgetattr(master) != before
                    and time.monotonic() < restore_deadline
                ):
                    drain(0.01)
                assert termios.tcgetattr(master) == before, (
                    "terminal not restored after x",
                    before,
                    termios.tcgetattr(master),
                )
            # Real terminal Ctrl-C must reach only the launcher foreground
            # process, which obtains runtime/engine stop proof through IPC.
            os.write(master, b"\x03")
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                drain(0.05)
                result, status = os.waitpid(pid, os.WNOHANG)
                if result:
                    reaped = True
                    assert os.waitstatus_to_exitcode(status) == 0, (
                        text,
                        (directory / "run/application.log").read_text(),
                        (directory / "run/evidence.json").read_text(),
                    )
                    break
            assert reaped, ("launcher ignored terminal Ctrl-C", text)
            # The final Result line can follow the reaped child in the PTY
            # buffer; read it before asserting on the transcript.
            drain_deadline = time.monotonic() + 1
            while time.monotonic() < drain_deadline:
                drain(0.05)
            assert termios.tcgetattr(master) == before, (
                "launcher left terminal attributes changed"
            )
            evidence = json.loads((directory / "run/evidence.json").read_text())
            assert evidence["stop_proven"] and evidence["state"] == "Closed", evidence
            assert evidence["final_child_codes"] == [0, 0], evidence
            recorded = (directory / "run/application.log").read_text()
            assert "\x1b[" not in recorded
            from irmv_console import shutdown_result

            assert shutdown_result(
                "shutdown stopped-unproven: fresh ZeroTorque required", 1
            ) == (
                "FAILED: ZeroTorque was not confirmed at shutdown "
                "(fresh ZeroTorque required)",
                "fault",
            )
            assert shutdown_result("", 0) == (
                "OK: ZeroTorque confirmed; all processes exited",
                "ready",
            )
            assert all(
                state in recorded
                for state in (
                    "behavior=Damping",
                    "behavior=Ready",
                    "behavior=Locomotion",
                )
            )
            if not quit_application:
                record = next(directory.rglob("app_run_*/velocity.jsonl"))
                axes = [
                    (row["forward"], row["lateral"], row["yaw"])
                    for line in record.read_text().splitlines()
                    if (row := json.loads(line)).get("forward") is not None
                ]
                terminal = axes.index((0, 0, 1))
                forward = axes.index((1, 0, 0), terminal + 1)
                released = axes.index((0, 0, 0), forward + 1)
                assert released - forward >= 2, axes
                lateral = axes.index((0, 1, 0), released + 1)
                blurred = axes.index((0, 0, 0), lateral + 1)
                backward = axes.index((-1, 0, 0), blurred + 1)
                expired = axes.index((0, 0, 0), backward + 1)
                right = axes.index((0, -1, 0), expired + 1)
                assert (0, 0, 0) in axes[right + 1 :], axes
            print(
                "PASS controlling PTY keys/status/restore/stop",
                "app-x" if quit_application else "active-Ctrl-C",
            )
        finally:
            controls.close()
            if not reaped:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    result, _ = os.waitpid(pid, os.WNOHANG)
                    if result:
                        reaped = True
                        break
                    drain(0.05)
                if not reaped:
                    os.kill(pid, signal.SIGKILL)
                    os.waitpid(pid, 0)
                for child in children:
                    try:
                        os.kill(child, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
            os.close(master)


if __name__ == "__main__":
    build = Path(sys.argv[1]).resolve()
    if len(sys.argv) > 2 and sys.argv[2] == "--peer":
        raise SystemExit(
            peer(
                build,
                Path(sys.argv[3]),
                int(sys.argv[4]) if len(sys.argv) > 4 else None,
            )
        )
    run_case(build, True)
    run_case(build, False)
