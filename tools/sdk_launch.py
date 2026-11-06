#!/usr/bin/env python3
"""Start the runtime and the supervising application as two processes.

The launcher owns only the start-up order:

1. start ``robot_runtime`` with the given configuration and domain;
2. wait until the runtime reports a fresh state (``BootCompleted``);
3. start ``application`` with the same configuration;
4. if the application exits or fails to start, leave the runtime running and
   report its pid. The runtime is never killed here: its ZeroTorque close is the
   ordered shutdown of the lower layer, and only an explicit signal to the
   runtime may end it.

Usage:
  sdk_launch.py --config PATH [--domain N]
                [--locomotion-source none|local|external]
                [--runtime PATH] [--application PATH]
                [--web-telemetry-socket PATH]
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

START_TIMEOUT_S = float(os.environ.get("IRMV_LAUNCH_TIMEOUT_S", "20"))

# One line of the key legend per group. The order is the order an operator uses
# them: the stop, the modes, then the intent. Labels are the behaviour names the
# application prints, so a key can be matched to a status line.
MODE_KEYS = (
    ("emergency_stop", "emergency stop"),
    ("damping", "damping"),
    ("initialize", "initialize"),
    ("start_locomotion", "locomotion"),
    ("zero_torque", "zero torque"),
    ("reset", "reset"),
    ("exit", "exit"),
)
VELOCITY_PAIRS = (
    (("velocity_forward", "forward"), ("velocity_backward", "backward")),
    (("velocity_left", "left"), ("velocity_right", "right")),
    (("velocity_yaw_left", "yaw left"),
     ("velocity_yaw_right", "yaw right")),
)


def find_binary(name: str, explicit: str | None) -> str:
    if explicit:
        return explicit
    environment = os.environ.get("IRMV_BIN_DIR")
    candidates = []
    if environment:
        candidates.append(Path(environment) / name)
    here = Path(__file__).resolve()
    candidates.extend([
        here.parent / name,
        here.parent.parent / "build" / name,
        here.parent.parent / "build-real" / name,
    ])
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    found = shutil_which(name)
    if found:
        return found
    raise SystemExit(f"sdk_launch: cannot find {name}; pass --{name} or set "
                     "IRMV_BIN_DIR")


def shutil_which(name: str) -> str | None:
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(directory) / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def runtime_log_path() -> Path:
    """A diagnostics file the runtime owns for its whole life.

    The runtime must never write into a pipe whose reader is this launcher: a
    launcher exit would close that reader and the next runtime write would end
    the lower layer with SIGPIPE. The file is opened by the child and survives
    the launcher, so diagnostics and readiness remain observable after the
    upper process is gone.
    """
    configured = os.environ.get("IRMV_LAUNCH_LOG_DIR")
    directory = Path(configured) if configured else (Path.cwd() / "logs")
    directory.mkdir(parents=True, exist_ok=True)
    return directory / ("launcher_runtime_%d_%d.log" %
                        (os.getpid(), int(time.time() * 1000)))


def read_log_tail(path: Path, count: int = 20) -> list[str]:
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    return lines[-count:]


def render_key_legend(config_path: str) -> list[str]:
    """Render the configured bindings as the lines printed before the app starts.

    The application owns the keyboard; the launcher only shows the map the
    configuration actually contains, so a rebound key is reported correctly and
    an unbound action is not invented.
    """
    try:
        document = json.loads(Path(config_path).read_text(encoding="utf-8"))
        bindings = document["input"]["bindings"]
    except (OSError, ValueError, KeyError, TypeError):
        return []
    if not isinstance(bindings, dict):
        return []

    def key_of(action: str) -> str | None:
        value = bindings.get(action)
        if not isinstance(value, str) or len(value) != 1:
            return None
        return "space" if value == " " else value

    modes = []
    for action, label in MODE_KEYS:
        key = key_of(action)
        if key is not None:
            modes.append("%s=%s" % (key, label))
    velocities = []
    for (first_action, first_label), (second_action,
                                      second_label) in VELOCITY_PAIRS:
        first = key_of(first_action)
        second = key_of(second_action)
        if first is not None and second is not None:
            velocities.append("%s/%s=%s/%s" %
                              (first, second, first_label, second_label))
        elif first is not None:
            velocities.append("%s=%s" % (first, first_label))
        elif second is not None:
            velocities.append("%s=%s" % (second, second_label))

    lines = []
    if modes:
        lines.append("sdk_launch: keys: " + "  ".join(modes))
    if velocities:
        lines.append("sdk_launch: keys: " + "  ".join(velocities))
    return lines


def wait_ready(log_path: Path, process: subprocess.Popen,
               timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with log_path.open("r", encoding="utf-8", errors="replace") as log:
                for line in log:
                    if "BootCompleted" in line:
                        return True
        except OSError:
            pass
        if process.poll() is not None:
            return False
        time.sleep(0.05)
    return False


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--domain", type=int, default=None)
    parser.add_argument("--locomotion-source", default=None,
                        choices=["none", "local", "external"])
    parser.add_argument("--runtime", default=None)
    parser.add_argument("--application", default=None)
    parser.add_argument(
        "--web-telemetry-socket", default=None,
        help="explicitly enable the workbench live outlet for the two SDK "
             "processes; the launcher never starts the web service")
    parser.add_argument("--stop-all", action="store_true",
                        help="after the application exits, also send the "
                             "runtime SIGTERM (explicit whole-system stop)")
    options = parser.parse_args(argv)

    runtime = find_binary("robot_runtime", options.runtime)
    application = find_binary("application", options.application)

    common = ["--config", options.config]
    if options.domain is not None:
        common += ["--domain", str(options.domain)]
    child_environment = os.environ.copy()
    if options.web_telemetry_socket is not None:
        # Only a user-requested socket is exported, and only to the two SDK
        # processes. The launcher does not start or supervise the web service.
        child_environment["IRMV_WEB_TELEMETRY_SOCKET"] = options.web_telemetry_socket

    # The runtime lives in its own session and process group: a terminal Ctrl+C
    # reaches the launcher and the application, never the lower layer. Its
    # diagnostics go to a file, not a pipe: when this launcher exits, a pipe's
    # reader disappears and the runtime's next write raises SIGPIPE.
    log_path = runtime_log_path()
    with log_path.open("w", encoding="utf-8") as runtime_log:
        runtime_process = subprocess.Popen(
            [runtime, *common], stdin=subprocess.DEVNULL,
            stdout=runtime_log, stderr=subprocess.STDOUT, text=True,
            bufsize=1, start_new_session=True, env=child_environment)
    print("sdk_launch: runtime diagnostics: %s" % log_path)

    if not wait_ready(log_path, runtime_process, START_TIMEOUT_S):
        state = ("exited with %s" % runtime_process.poll()
                 if runtime_process.poll() is not None else "still running")
        print("sdk_launch: the runtime did not become ready (%s)" % state,
              file=sys.stderr)
        for line in read_log_tail(log_path):
            print("runtime: " + line, file=sys.stderr)
        if runtime_process.poll() is None:
            print("sdk_launch: runtime pid %d left running" %
                  runtime_process.pid, file=sys.stderr)
        return 1

    app_arguments = [application, *common]
    if options.locomotion_source is not None:
        app_arguments += ["--locomotion-source", options.locomotion_source]
    for line in render_key_legend(options.config):
        print(line, flush=True)
    try:
        application_process = subprocess.Popen(app_arguments,
                                               env=child_environment)
    except OSError as error:
        print("sdk_launch: the application did not start: %s" % error,
              file=sys.stderr)
        print("sdk_launch: runtime is still running, pid %d" %
              runtime_process.pid, file=sys.stderr)
        return 1

    def forward(signum, _frame):
        # Signals for the upper layer only. The runtime is never stopped by the
        # launcher's own termination path.
        try:
            application_process.send_signal(signum)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)

    try:
        application_status = application_process.wait()
    except KeyboardInterrupt:
        forward(signal.SIGINT, None)
        application_status = application_process.wait()

    runtime_status = runtime_process.poll()
    if runtime_status is None:
        print("sdk_launch: application exited with %d" % application_status)
        print("sdk_launch: runtime is still running, pid %d; send it SIGTERM "
              "for the ordered ZeroTorque stop" % runtime_process.pid)
        if options.stop_all:
            print("sdk_launch: --stop-all requested; sending SIGTERM to the "
                  "runtime")
            runtime_process.send_signal(signal.SIGTERM)
            try:
                runtime_status = runtime_process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print("sdk_launch: the runtime did not stop within 30 s; pid "
                      "%d is still running" % runtime_process.pid,
                      file=sys.stderr)
                return 1
            print("sdk_launch: runtime exited with %d" % runtime_status)
        # The default path leaves the runtime alive; it is a separate decision
        # to stop the machine, and its own ZeroTorque confirmation is the only
        # orderly close.
        return application_status
    print("sdk_launch: application exited with %d" % application_status)
    print("sdk_launch: runtime exited with %d" % runtime_status)
    return application_status


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
