#!/usr/bin/env python3
"""Process level contract of the standalone fake runtime (robot_runtime).

Usage: runtime_cli_test.py <path to robot_runtime> <source directory>

The transport is always started, so every launch gets its own DDS domain below
the 230 bound the configuration enforces. Every wait is bounded; on timeout the
process is killed and the captured output is reported.
"""

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

# 20..209, derived from the process id so parallel test runs stay apart.
DOMAIN = 20 + (os.getpid() % 60)
# The configuration on disk carries a different domain; --domain must override
# it, which the startup line proves.
CONFIG_DOMAIN = 7

START_WAIT_S = 15.0
FAULT_WAIT_S = 10.0
EXIT_WAIT_S = 5.0
STAY_ALIVE_WAIT_S = 2.0
ZERO_TORQUE_STATUS_LINES = 4

FAILURES = []


def check(condition, message):
    if condition:
        return True
    FAILURES.append(message)
    print("FAIL: " + message)
    return False


class Runtime:
    """One robot_runtime process with a bounded, non-blocking output reader."""

    def __init__(self, arguments, cwd):
        self.command = [str(BINARY), *arguments]
        self.workspace = cwd
        self.process = subprocess.Popen(
            self.command,
            cwd=str(cwd),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.lines = []
        self._condition = threading.Condition()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.process.stdout:
            with self._condition:
                self.lines.append(line.rstrip("\n"))
                self._condition.notify_all()
        with self._condition:
            self._condition.notify_all()

    def _drain(self):
        """Joins the reader once the process ended, so no line is in flight."""
        if self.alive():
            return
        self._reader.join(EXIT_WAIT_S)

    def output(self):
        self._drain()
        with self._condition:
            return "\n".join(self.lines)

    def alive(self):
        return self.process.poll() is None

    def wait_for(self, predicate, timeout, minimum=1):
        """Waits for `minimum` matching lines and returns them."""
        deadline = time.monotonic() + timeout
        while True:
            with self._condition:
                found = [line for line in self.lines if predicate(line)]
            if len(found) >= minimum:
                return found
            if not self.alive():
                # A finished process may still have output in flight. Drain it
                # once and look again, so a run that ended early fails here
                # instead of waiting out the whole timeout.
                self._drain()
                with self._condition:
                    return [line for line in self.lines if predicate(line)]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return found
            with self._condition:
                self._condition.wait(min(remaining, 0.5))

    def wait_exit(self, timeout):
        """Returns the exit code, or None when the process is still running."""
        try:
            return self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            return None

    def terminate(self):
        if self.alive():
            self.process.send_signal(signal.SIGTERM)

    def kill(self):
        if self.alive():
            self.process.kill()
        self.process.wait(timeout=EXIT_WAIT_S)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.kill()


def failure_report(name, runtime, detail):
    output = runtime.output()
    if len(output) > 4000:
        output = "...\n" + output[-4000:]
    return "%s: %s\ncommand: %s\noutput:\n%s" % (
        name,
        detail,
        " ".join(runtime.command),
        output,
    )


def build_workspace(source_dir):
    workspace = Path(tempfile.mkdtemp(prefix="runtime_cli_"))
    config = json.loads((source_dir / "config/sdk.json").read_text("utf-8"))
    config["telemetry"]["directory"] = str(workspace / "logs")
    config["dds"]["domain"] = CONFIG_DOMAIN
    (workspace / "config.json").write_text(json.dumps(config), "utf-8")
    real = json.loads(json.dumps(config))
    real["backend"] = "real"
    (workspace / "real.json").write_text(json.dumps(real), "utf-8")
    (workspace / "malformed.json").write_text(
        '{"schema_version": 1, "backend": "fake"', "utf-8"
    )
    shutil.copy(
        source_dir / "tests/support/fake_scenario.json", workspace / "normal.json"
    )
    (workspace / "fault.json").write_text(
        json.dumps({"entries": [{"elapsed_us": 0, "feedback": {
            "joints": [{"dq": 15.0}, {}]}}]}),
        "utf-8",
    )
    (workspace / "write_fail.json").write_text(
        json.dumps({"entries": [{"elapsed_us": 0,
                                 "write_result": {"accepted": False}}]}),
        "utf-8",
    )
    (workspace / "scenario_broken.json").write_text("{not json", "utf-8")
    return workspace


def base_arguments(workspace):
    return ["--config", str(workspace / "config.json"), "--domain", str(DOMAIN)]


def test_startup_rejections(workspace):
    cases = [
        ("missing config", workspace / "absent.json", "cannot open configuration"),
        ("malformed config", workspace / "malformed.json", "invalid JSON"),
        ("real backend", workspace / "real.json", "only the fake backend"),
    ]
    for name, config_path, expected in cases:
        with Runtime(["--config", str(config_path), "--domain", str(DOMAIN)],
                     workspace) as runtime:
            code = runtime.wait_exit(EXIT_WAIT_S)
            check(code is not None and code != 0,
                  failure_report(name, runtime, "expected a non-zero exit, got %s" % code))
            output = runtime.output()
            check(expected in output,
                  failure_report(name, runtime, "message %r is missing" % expected))
            check("[runtime]" not in output,
                  failure_report(name, runtime, "the runtime started anyway"))

    with Runtime(["--config", str(workspace / "real.json"), "--domain", str(DOMAIN),
                  "--fake-scenario", str(workspace / "normal.json")],
                 workspace) as runtime:
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code is not None and code != 0,
              failure_report("real backend with scenario", runtime,
                             "expected a non-zero exit, got %s" % code))
        output = runtime.output()
        check("backend" in output and "fake" in output,
              failure_report("real backend with scenario", runtime,
                             "the message does not name the backend"))

    with Runtime(["--config", str(workspace / "config.json"), "--domain", str(DOMAIN),
                  "--fake-scenario", str(workspace / "scenario_broken.json")],
                 workspace) as runtime:
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code is not None and code != 0,
              failure_report("broken scenario", runtime,
                             "expected a non-zero exit, got %s" % code))
        check("scenario" in runtime.output(),
              failure_report("broken scenario", runtime,
                             "the message does not name the scenario"))

    with Runtime(["--config", str(workspace / "config.json"), "--frobnicate"],
                 workspace) as runtime:
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code is not None and code != 0,
              failure_report("unknown argument", runtime,
                             "expected a non-zero exit, got %s" % code))
        check("usage: robot_runtime" in runtime.output(),
              failure_report("unknown argument", runtime, "no usage line"))

    with Runtime(["--config", str(workspace / "config.json"), "--domain", "1000"],
                 workspace) as runtime:
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code is not None and code != 0,
              failure_report("domain out of range", runtime,
                             "expected a non-zero exit, got %s" % code))
        # The usage line of every argument error contains the word "domain", so
        # the check names the rejection itself.
        check("--domain must be between 0 and 230" in runtime.output(),
              failure_report("domain out of range", runtime,
                             "the message does not name the accepted range"))


def test_normal_start_and_shutdown(workspace):
    with Runtime(base_arguments(workspace) +
                 ["--fake-scenario", str(workspace / "normal.json")],
                 workspace) as runtime:
        started = runtime.wait_for(
            lambda line: "[runtime] runtime started" in line
            and 'domain="%d"' % DOMAIN in line, START_WAIT_S)
        check(started, failure_report("normal start", runtime,
                                      "no startup line with the overridden domain"))
        if started:
            check('state="Boot"' in started[0],
                  failure_report("normal start", runtime,
                                 "the startup line does not report Boot"))
        completed = runtime.wait_for(
            lambda line: "BootCompleted" in line and 'from="Boot"' in line
            and 'to="ZeroTorque"' in line, START_WAIT_S)
        check(completed, failure_report("normal start", runtime,
                                        "Boot did not complete to ZeroTorque"))
        status = runtime.wait_for(
            lambda line: "[runtime] status" in line and 'state="ZeroTorque"' in line,
            START_WAIT_S)
        check(status, failure_report("normal start", runtime,
                                     "no status line in ZeroTorque"))
        runtime.terminate()
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code == 0, failure_report("normal shutdown", runtime,
                                        "expected exit 0, got %s" % code))
        output = runtime.output()
        check("[runtime] shutdown" in output and 'state="ZeroTorque"' in output,
              failure_report("normal shutdown", runtime, "no clean shutdown line"))
        check("still required" not in output,
              failure_report("normal shutdown", runtime,
                             "a ZeroTorque confirmation was reported as missing"))
        check("clean shutdown" in output,
              failure_report("normal shutdown", runtime,
                             "the process did not report the clean stop"))


def test_fault_latch_keeps_running(workspace):
    with Runtime(base_arguments(workspace) +
                 ["--fake-scenario", str(workspace / "fault.json")],
                 workspace) as runtime:
        latched = runtime.wait_for(
            lambda line: "FaultLatched" in line
            and 'fault="JointVelocityExceeded"' in line, FAULT_WAIT_S)
        check(latched, failure_report("overspeed fault", runtime,
                                      "the velocity fault was not logged"))
        if latched:
            check(latched[0].startswith("ERROR "),
                  failure_report("overspeed fault", runtime,
                                 "the fault was not logged at ERROR level"))
            check('from="Boot"' in latched[0] and 'to="Fault"' in latched[0]
                  and 'action="' in latched[0],
                  failure_report("overspeed fault", runtime,
                                 "the fault line lacks its transition or action"))
        fault_status = runtime.wait_for(
            lambda line: "[runtime] status" in line and 'state="Fault"' in line,
            FAULT_WAIT_S)
        check(fault_status, failure_report("overspeed fault", runtime,
                                           "the runtime did not stay in Fault"))
        runtime.terminate()
        code = runtime.wait_exit(STAY_ALIVE_WAIT_S)
        check(code is None,
              failure_report("fault shutdown", runtime,
                             "the process exited with %s instead of staying up" % code))
        output = runtime.output()
        check("Reset is required" in output,
              failure_report("fault shutdown", runtime, "no Reset warning"))
        check("[runtime] shutdown" not in output,
              failure_report("fault shutdown", runtime,
                             "a clean shutdown was reported under a latched fault"))
        runtime.kill()
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code is not None and code < 0,
              failure_report("fault shutdown", runtime,
                             "hard kill reported exit %s" % code))


def test_write_failure_keeps_running(workspace):
    with Runtime(base_arguments(workspace) +
                 ["--fake-scenario", str(workspace / "write_fail.json")],
                 workspace) as runtime:
        latched = runtime.wait_for(
            lambda line: "FaultLatched" in line and 'fault="WriteFailed"' in line,
            FAULT_WAIT_S)
        check(latched, failure_report("write failure", runtime,
                                      "the refused write was not logged"))
        status = runtime.wait_for(
            lambda line: "[runtime] status" in line and 'state="Fault"' in line,
            FAULT_WAIT_S)
        check(status, failure_report("write failure", runtime,
                                     "the runtime did not stay in Fault"))
        check(runtime.alive(), failure_report("write failure", runtime,
                                              "the runtime exited"))
        runtime.kill()


def test_unsupervised_zerotorque(workspace):
    with Runtime(base_arguments(workspace) +
                 ["--fake-scenario", str(workspace / "normal.json")],
                 workspace) as runtime:
        lines = runtime.wait_for(
            lambda line: "[runtime] status" in line and 'state="ZeroTorque"' in line,
            START_WAIT_S, minimum=ZERO_TORQUE_STATUS_LINES)
        check(len(lines) >= ZERO_TORQUE_STATUS_LINES,
              failure_report("unsupervised runtime", runtime,
                             "only %d ZeroTorque status lines" % len(lines)))
        check(runtime.alive(), failure_report("unsupervised runtime", runtime,
                                              "the runtime exited"))
        runtime.terminate()
        code = runtime.wait_exit(EXIT_WAIT_S)
        check(code == 0, failure_report("unsupervised runtime", runtime,
                                        "expected exit 0, got %s" % code))


def main():
    global BINARY
    global SOURCE_DIR
    if len(sys.argv) != 3:
        print("usage: runtime_cli_test.py <robot_runtime> <source directory>")
        return 2
    BINARY = Path(sys.argv[1]).resolve()
    SOURCE_DIR = Path(sys.argv[2]).resolve()
    if not BINARY.is_file():
        print("FAIL: robot_runtime binary %s does not exist" % BINARY)
        return 1

    print("runtime_cli_test: binary=%s source=%s domain=%d"
          % (BINARY, SOURCE_DIR, DOMAIN))
    workspace = build_workspace(SOURCE_DIR)
    try:
        test_startup_rejections(workspace)
        test_normal_start_and_shutdown(workspace)
        test_fault_latch_keeps_running(workspace)
        test_write_failure_keeps_running(workspace)
        test_unsupervised_zerotorque(workspace)
    finally:
        shutil.rmtree(workspace, ignore_errors=True)

    if FAILURES:
        print("%d check(s) failed" % len(FAILURES))
        return 1
    print("runtime_cli_test: all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
