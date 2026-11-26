#!/usr/bin/env python3
"""Independent simulation owner. Ctrl-C closes the whole experiment; app x only leaves it."""

import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import threading
import time
from contextlib import contextmanager, nullcontext
from pathlib import Path

from irmv_console import application_line, notice, shutdown_result
from simulation.assets import verify_binding
from simulation.main import Runner, load_bootstrap, warp_engine


def identities():
    a, b = secrets.token_hex(16), secrets.token_hex(16)
    while a == b:
        b = secrets.token_hex(16)
    return dict(run_id=a, runtime_id=b)


@contextmanager
def startup_log(path):
    """Capture native and Python output before starting the SDK peers."""
    with path.open("w") as log:
        streams = (sys.stdout, sys.stderr)
        saved = [os.dup(stream.fileno()) for stream in streams]
        try:
            for stream in streams:
                stream.flush()
                os.dup2(log.fileno(), stream.fileno())
            yield
        finally:
            for stream, fd in zip(streams, saved):
                try:
                    stream.flush()
                finally:
                    os.dup2(fd, stream.fileno())
                    os.close(fd)


def reserve_directory(path):
    # Never reuse or unlink another run's socket/bootstrap, even after a crash.
    Path(path).mkdir(mode=0o700)


class Experiment:
    def __init__(
        self,
        config,
        runtime,
        application,
        normalizer=None,
        domain=None,
        engine_factory=warp_engine,
        binding=verify_binding,
        runner_type=Runner,
        application_stdin=None,
        viewer=False,
        viewer_follow=False,
        viewer_factory=None,
        viewer_no_shadow=False,
    ):
        self.config = Path(config).resolve()
        self.runtime = Path(runtime).resolve()
        self.application = Path(application).resolve()
        self.normalizer = Path(
            normalizer or self.runtime.with_name("simulation_normalize")
        ).resolve()
        self.domain = domain
        self.engine_factory = engine_factory
        self.binding = binding
        self.runner_type = runner_type
        self.application_stdin = application_stdin
        self.children = []
        self.logs = []
        self.directory_owned = False
        self.result = None
        self.thread = None
        self.runner = None
        self.shutdown_notified = False
        self.evidence = {}
        self.viewer_enabled = viewer
        self.viewer_follow = viewer_follow
        self.viewer_no_shadow = viewer_no_shadow
        self.viewer_factory = viewer_factory
        self.viewer = None
        self.application_output = None

    def notify_shutdown(self, signum=None, frame=None):
        self.shutdown_notified = True

    def _cancel_startup(self):
        # Normal flow only: signal handlers merely assign shutdown_notified.
        if not self.shutdown_notified:
            return False
        self.evidence["startup_cancelled"] = True
        self.evidence["runtime_spawned"] = bool(self.children)
        self.close()
        if self.thread and self.thread.is_alive():
            raise RuntimeError("startup cancellation owner deadline")
        return True

    def _open_application_console(self):
        path = self.directory / "application.log"
        self.application_output = path.open("r", encoding="utf-8", errors="replace")
        notice("Run log", str(path))
        try:
            bindings = json.loads(self.config.read_text())["input"]["bindings"]
        except (OSError, ValueError, KeyError, TypeError):
            bindings = {}
        keys = [
            "{}={}".format("space" if key == " " else key, action)
            for action, key in bindings.items()
            if isinstance(key, str) and len(key) == 1
        ]
        if keys:
            notice("Keys", "  ".join(keys))
        notice(
            "Controls",
            "type without Enter. Press d, i, then l; wait for Ready before l.",
        )
        notice(
            "Stop",
            "Ctrl-C ends the experiment and confirms ZeroTorque; "
            "x only leaves the application.",
        )

    def _mirror_application_output(self):
        if self.application_output is None:
            return
        # Follow the child-owned regular file, never put a logging pipe in the
        # SDK path. Bound each host iteration; application keeps owning stdin.
        text = self.application_output.read(8192)
        if text:
            try:
                for line in text.splitlines(keepends=True):
                    sys.stdout.write(application_line(line, sys.stdout))
                sys.stdout.flush()
            except (BrokenPipeError, BlockingIOError) as exc:
                self.evidence["console_output_error"] = str(exc)
                self.application_output.close()
                self.application_output = None

    def preflight(self):
        normalized = json.loads(
            subprocess.check_output(
                [str(self.normalizer), str(self.config)]
                + (["--domain", str(self.domain)] if self.domain is not None else []),
                text=True,
                timeout=30,
            )
        )
        self.directory = Path(normalized.pop("socket_directory"))
        self.evidence = self.binding(normalized)
        return normalized

    def _start_simulator(self, n, bootstrap, asset_digest):
        ready = threading.Event()

        class ReadyRunner(self.runner_type):
            def _advance(inner, event):
                super()._advance(event)
                if event.kind == "feedback" and inner.coordinator.feedback is not None:
                    ready.set()

        self.runner = ReadyRunner(
            load_bootstrap(bootstrap),
            self.directory / "bridge.sock",
            self.engine_factory,
        )
        if self.viewer_enabled:
            factory = self.viewer_factory
            if factory is None:
                from simulation.viewer import ViewerPublisher

                factory = ViewerPublisher
            self.viewer = factory(
                n["engine_config"],
                n["run_id"],
                asset_digest,
                n["physics_dt_us"],
                follow=self.viewer_follow,
                no_shadow=self.viewer_no_shadow,
            )
            self.viewer.cancelled = lambda: self.shutdown_notified
            self.viewer.start(min(30, n["startup_timeout_us"] / 1e6))
            if self._cancel_startup():
                return False
            self.runner.viewer = self.viewer

        def run():
            self.result = self.runner.run()

        self.thread = threading.Thread(target=run, name="simulation-owner")
        self.thread.start()
        deadline = time.monotonic() + n["startup_timeout_us"] / 1e6
        while True:
            if self._cancel_startup():
                return False
            if ready.wait(0.01):
                return True
            if self.result is not None or time.monotonic() > deadline:
                raise RuntimeError(
                    "engine startup failed: " + self.runner.coordinator.failure
                )

    def start(self, *, capture_startup_log=False):
        if self._cancel_startup():
            return self
        n = self.preflight()
        asset_digest = n["asset_digest"]
        n.pop("asset_digest")
        n.pop("policy_path")
        n.update(identities())
        reserve_directory(self.directory)
        self.directory_owned = True
        bootstrap = self.directory / "bootstrap.json"
        fd = os.open(bootstrap, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as f:
            json.dump(n, f)
        path = self.directory / "simulator.log"
        if capture_startup_log:
            notice("Simulator log", str(path))
        with startup_log(path) if capture_startup_log else nullcontext():
            if not self._start_simulator(n, bootstrap, asset_digest):
                return self
        # Engine coherent initial feedback and listening socket precede both SDK peers.
        self.evidence.update(
            run_id=n["run_id"], runtime_id=n["runtime_id"], engine_ready_before_sdk=True
        )
        for name, binary in [
            ("runtime", self.runtime),
            ("application", self.application),
        ]:
            if self._cancel_startup():
                return self
            log = open(self.directory / (name + ".log"), "w")
            self.logs.append(log)
            if self._cancel_startup():
                return self
            self.children.append(
                subprocess.Popen(
                    [str(binary), "--config", str(self.config)]
                    + (
                        ["--domain", str(self.domain)]
                        if self.domain is not None
                        else []
                    ),
                    stdin=self.application_stdin
                    if name == "application"
                    else subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
            if name == "application" and self.application_stdin is None:
                self._open_application_console()
        return self

    def wait(self):
        if self.evidence.get("startup_cancelled"):
            return 1
        while self.thread.is_alive():
            if self.shutdown_notified:
                self.runner.request_shutdown()
            if (
                self.children[0].poll() is not None
                and self.result is None
                and self.runner.coordinator.state != "Closed"
            ):
                self.runner.request_stop()
            if (
                self.children[1].poll() is not None
                and not self.runner.coordinator.runtime_only
                and self.runner.coordinator.state != "Closed"
            ):
                self.runner.request_stop()
            self.thread.join(0.01)
            self._mirror_application_output()
            if self.viewer is not None:
                status = self.viewer.status()
                previous = self.evidence.get("viewer", {}).get("state")
                self.evidence["viewer"] = status
                if status["state"] != previous:
                    error = ": " + status["error"] if status.get("error") else ""
                    notice("Viewer", status["state"] + error, level="warning")
        codes = [p.wait(timeout=5) for p in self.children]
        self._mirror_application_output()
        c = self.runner.coordinator
        self.evidence.update(
            coordinator_result=self.result,
            child_codes=codes,
            state=c.state,
            failure=c.failure,
            stop_proven=c.stop_proven,
            runtime_only=c.runtime_only,
            release_state=c.release_state,
            release_event_drops=c.release_event_drops,
            release_events=list(c.release_events),
            logical_steps=c.step,
        )
        return 0 if self.result == 0 and codes == [0, 0] and c.stop_proven else 1

    def close(self):
        if self.runner and self.thread and self.thread.is_alive():
            self.runner.request_stop()
            self.thread.join(2)
        for p in self.children:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()
        for p in self.children:
            if p.stdin and not p.stdin.closed:
                p.stdin.close()
        self._mirror_application_output()
        if self.application_output is not None:
            self.application_output.close()
            self.application_output = None
        for log in self.logs:
            log.close()
        if self.viewer is not None:
            self.viewer.close()
            self.evidence["viewer"] = self.viewer.status()
        if self.directory_owned and self.directory.exists():
            if self.runner:
                c = self.runner.coordinator
                self.evidence.update(
                    coordinator_result=self.result,
                    state=c.state,
                    failure=c.failure,
                    stop_proven=c.stop_proven,
                    runtime_only=c.runtime_only,
                    release_state=c.release_state,
                    release_event_drops=c.release_event_drops,
                    release_events=list(c.release_events),
                    logical_steps=c.step,
                )
            self.evidence["final_child_codes"] = [p.returncode for p in self.children]
            (self.directory / "evidence.json").write_text(
                json.dumps(
                    self.evidence,
                    indent=2,
                    default=lambda x: x.hex() if isinstance(x, bytes) else str(x),
                )
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--application", required=True)
    parser.add_argument(
        "--python", required=True, help="separate pinned simulation interpreter"
    )
    parser.add_argument("--normalizer")
    parser.add_argument(
        "--domain",
        type=int,
        help="override dds.domain for this run; the normalized digest and both SDK peers use it",
    )
    parser.add_argument(
        "--viewer", action="store_true", help="open read-only native MuJoCo window"
    )
    parser.add_argument(
        "--camera-follow",
        action="store_true",
        help="start with the camera following the robot base (requires --viewer)",
    )
    parser.add_argument(
        "--no-shadow",
        action="store_true",
        help="disable ground shadows to reduce rendering cost (requires --viewer)",
    )
    parser.add_argument(
        "--simulation-worker", action="store_true", help=argparse.SUPPRESS
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate configuration and asset binding without starting the engine",
    )
    args = parser.parse_args()
    if not args.viewer and (args.camera_follow or args.no_shadow):
        parser.error("--camera-follow and --no-shadow require --viewer")
    if not args.simulation_worker:
        os.execvp(
            args.python,
            [
                args.python,
                str(Path(__file__).resolve()),
                *sys.argv[1:],
                "--simulation-worker",
            ],
        )
    experiment = Experiment(
        args.config,
        args.runtime,
        args.application,
        args.normalizer,
        domain=args.domain,
        viewer=args.viewer,
        viewer_follow=args.camera_follow,
        viewer_no_shadow=args.no_shadow,
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, experiment.notify_shutdown)
    try:
        if args.check_config:
            experiment.preflight()
            return 0
        # CLI runs own the process descriptors during startup; embedded users
        # of Experiment do not.
        experiment.start(capture_startup_log=True)
        if experiment.evidence.get("startup_cancelled"):
            return experiment.wait()
        notice("Simulation", "engine ready; SDK peers running")
        result = experiment.wait()
        text, level = shutdown_result(experiment.evidence.get("failure") or "", result)
        notice("Result", text, level=level, stream=sys.stderr if result else sys.stdout)
        return result
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print("simulation launch failed: " + str(exc), file=sys.stderr)
        return 1
    finally:
        experiment.close()


if __name__ == "__main__":
    raise SystemExit(main())
