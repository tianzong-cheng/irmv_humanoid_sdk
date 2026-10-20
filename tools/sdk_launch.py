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
import threading
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


class LineCollector(threading.Thread):
    def __init__(self, stream, name: str):
        super().__init__(daemon=True)
        self.stream = stream
        self.name = name
        self.lines: list[str] = []
        self.ready = threading.Event()

    def run(self) -> None:
        for line in self.stream:
            self.lines.append(line.rstrip("\n"))
            if "BootCompleted" in line:
                self.ready.set()
        self.ready.set()


def wait_ready(collector: LineCollector, process: subprocess.Popen,
               timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if "BootCompleted" in "\n".join(collector.lines):
            return True
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
    options = parser.parse_args(argv)

    runtime = find_binary("robot_runtime", options.runtime)
    application = find_binary("application", options.application)

    common = ["--config", options.config]
    if options.domain is not None:
        common += ["--domain", str(options.domain)]

    runtime_process = subprocess.Popen(
        [runtime, *common], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1)
    runtime_lines = LineCollector(runtime_process.stdout, "runtime")
    runtime_lines.start()

    if not wait_ready(runtime_lines, runtime_process, START_TIMEOUT_S):
        state = ("exited with %s" % runtime_process.poll()
                 if runtime_process.poll() is not None else "still running")
        print("sdk_launch: the runtime did not become ready (%s)" % state,
              file=sys.stderr)
        for line in runtime_lines.lines[-20:]:
            print("runtime: " + line, file=sys.stderr)
        if runtime_process.poll() is None:
            print("sdk_launch: runtime pid %d left running" %
                  runtime_process.pid, file=sys.stderr)
        return 1

    app_arguments = [application, *common]
    if options.locomotion_source is not None:
        app_arguments += ["--locomotion-source", options.locomotion_source]
    application_process = subprocess.Popen(app_arguments)

    def forward(signum, _frame):
        try:
            application_process.send_signal(signum)
        except ProcessLookupError:
            pass
        try:
            runtime_process.send_signal(signum)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)

    try:
        application_status = application_process.wait()
    except KeyboardInterrupt:
        forward(signal.SIGINT, None)
        application_status = application_process.wait()

    print("sdk_launch: application exited with %d" % application_status)
    if runtime_process.poll() is None:
        print("sdk_launch: runtime is still running, pid %d; send it SIGTERM "
              "for the ordered ZeroTorque stop" % runtime_process.pid)
        # The launcher never kills the runtime on the application's exit.
        return application_status
    print("sdk_launch: runtime exited with %d" % runtime_process.poll())
    return application_status


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
