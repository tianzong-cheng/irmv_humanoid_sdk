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
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

START_TIMEOUT_S = float(os.environ.get("IRMV_LAUNCH_TIMEOUT_S", "20"))


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
    parser.add_argument("--stop-all", action="store_true",
                        help="after the application exits, also send the "
                             "runtime SIGTERM (explicit whole-system stop)")
    options = parser.parse_args(argv)

    runtime = find_binary("robot_runtime", options.runtime)
    application = find_binary("application", options.application)

    common = ["--config", options.config]
    if options.domain is not None:
        common += ["--domain", str(options.domain)]

    # The runtime lives in its own session and process group: a terminal Ctrl+C
    # reaches the launcher and the application, never the lower layer. Its
    # diagnostics go to a file, not a pipe: when this launcher exits, a pipe's
    # reader disappears and the runtime's next write raises SIGPIPE.
    log_path = runtime_log_path()
    with log_path.open("w", encoding="utf-8") as runtime_log:
        runtime_process = subprocess.Popen(
            [runtime, *common], stdin=subprocess.DEVNULL,
            stdout=runtime_log, stderr=subprocess.STDOUT, text=True,
            bufsize=1, start_new_session=True)
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
    try:
        application_process = subprocess.Popen(app_arguments)
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
