#!/usr/bin/env python3
"""Process level contract of the standalone fake application (application).

Usage: application_cli_test.py <application> <robot_runtime> <source directory>

The application is the supervising upper process: it owns the terminal, drives
the frozen protocol through ClientTransport/RobotClient/BehaviorFsm, and never
starts, stops or signals the runtime. Every case that needs a lower layer
therefore starts a real robot_runtime on its own DDS domain and drives the
application through a PTY, so the bytes the test writes are the bytes
InputRouter reads - there is no second event-injection path.

Every wait is bounded; on timeout the process is killed and its captured output
is reported. Nothing here asserts anything about motor execution: only the host
side (exit codes, exit lines, status lines and the terminal) is observed.
"""

import json
import os
import pty
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import termios
import threading
import time
from pathlib import Path

# 1..19, derived from the process id: a band of its own, and one that is below
# the band the runtime CLI test and application_client_test share
# (20 + pid % 190, that is 20..209), so no two entries of a parallel `ctest -j`
# run can land on the same DDS domain. Each case walks inside this band.
DOMAIN_BAND = 19
DOMAIN = 1 + (os.getpid() % DOMAIN_BAND)
# The configuration on disk carries a different domain; --domain must override
# it for both processes, which is also what makes their config digests agree.
CONFIG_DOMAIN = 9

START_WAIT_S = 15.0
# The ordered exit is bounded by supervision.request_deadline_us (1 s) plus the
# loop period and the stop of the transport.
EXIT_WAIT_S = 6.0
EOF_WAIT_S = 6.0
STOP_BOUND_S = 4.0
# Status lines the runtime has to emit after the application is gone. The
# runtime logs one per second, so several of them cannot have been produced
# before the exit even though a single line could still have been in flight.
STATUS_LINES_AFTER_EXIT = 2

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
        self.command = [str(RUNTIME_BINARY), *arguments]
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
        if self.alive():
            return
        self._reader.join(EXIT_WAIT_S)

    def output(self):
        self._drain()
        with self._condition:
            return "\n".join(self.lines)

    def alive(self):
        return self.process.poll() is None

    def line_count(self):
        with self._condition:
            return len(self.lines)

    def wait_for(self, predicate, timeout, minimum=1, start=0):
        deadline = time.monotonic() + timeout
        while True:
            with self._condition:
                found = [line for line in self.lines[start:]
                         if predicate(line)]
            if len(found) >= minimum:
                return found
            if not self.alive():
                self._drain()
                with self._condition:
                    return [line for line in self.lines[start:]
                            if predicate(line)]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return found
            with self._condition:
                self._condition.wait(min(remaining, 0.5))

    def wait_exit(self, timeout):
        try:
            return self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            return None

    def kill(self):
        if self.alive():
            self.process.kill()
        try:
            self.process.wait(timeout=EXIT_WAIT_S)
        except subprocess.TimeoutExpired:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.kill()


class Application:
    """One application process, driven through a real PTY by default."""

    def __init__(self, arguments, cwd, terminal=True):
        self.command = [str(APPLICATION_BINARY), *arguments]
        self.terminal = terminal
        self.master = None
        self.slave = None
        self.initial_attributes = None
        self.lines = []
        self._buffer = ""
        self._condition = threading.Condition()
        if terminal:
            self.master, self.slave = pty.openpty()
            # The termios of a PTY belongs to the terminal, not to a file
            # descriptor, so this snapshot describes exactly the attributes the
            # application starts from - and the ones it has to give back.
            self.initial_attributes = termios.tcgetattr(self.slave)
            self.process = subprocess.Popen(
                self.command,
                cwd=str(cwd),
                stdin=self.slave,
                stdout=self.slave,
                stderr=self.slave,
            )
        else:
            self.process = subprocess.Popen(
                self.command,
                cwd=str(cwd),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        if not self.terminal:
            for line in self.process.stdout:
                self._append(line)
            with self._condition:
                self._condition.notify_all()
            return
        master = self.master
        while True:
            try:
                ready, _, _ = select.select([master], [], [], 0.1)
            except (OSError, ValueError):
                return
            if not ready:
                if self.process.poll() is not None:
                    return
                continue
            try:
                chunk = os.read(master, 4096)
            except OSError:
                return
            if not chunk:
                return
            self._append(chunk.decode("utf-8", "replace"))

    def _append(self, text):
        with self._condition:
            self._buffer += text
            while "\n" in self._buffer:
                line, self._buffer = self._buffer.split("\n", 1)
                self.lines.append(line.rstrip("\r"))
            self._condition.notify_all()

    def _drain(self):
        if self.alive():
            return
        self._reader.join(EXIT_WAIT_S)

    def output(self):
        self._drain()
        with self._condition:
            return "\n".join(self.lines)

    def status_lines(self):
        return [line for line in self.output().split("\n")
                if line.startswith("[application] ")]

    def send_keys(self, keys):
        if isinstance(keys, str):
            keys = keys.encode()
        os.write(self.master, keys)

    def close_terminal(self):
        """Hangs the PTY up: the application sees EOF on its next read."""
        for name in ("master", "slave"):
            handle = getattr(self, name)
            if handle is not None:
                try:
                    os.close(handle)
                except OSError:
                    pass
                setattr(self, name, None)

    def terminal_attributes(self):
        return termios.tcgetattr(self.slave)

    def alive(self):
        return self.process.poll() is None

    def line_count(self):
        with self._condition:
            return len(self.lines)

    def wait_for(self, predicate, timeout, minimum=1, start=0):
        deadline = time.monotonic() + timeout
        while True:
            with self._condition:
                found = [line for line in self.lines[start:]
                         if predicate(line)]
            if len(found) >= minimum:
                return found
            if not self.alive():
                self._drain()
                with self._condition:
                    return [line for line in self.lines[start:]
                            if predicate(line)]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return found
            with self._condition:
                self._condition.wait(min(remaining, 0.5))

    def wait_exit(self, timeout):
        try:
            return self.process.wait(timeout)
        except subprocess.TimeoutExpired:
            return None

    def kill(self):
        if self.alive():
            self.process.kill()
        try:
            self.process.wait(timeout=EXIT_WAIT_S)
        except subprocess.TimeoutExpired:
            pass
        self.close_terminal()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.kill()


def failure_report(name, process, detail):
    output = process.output()
    if len(output) > 4000:
        output = "...\n" + output[-4000:]
    return "%s: %s\ncommand: %s\noutput:\n%s" % (
        name, detail, " ".join(process.command), output)


def build_workspace(source_dir):
    workspace = Path(tempfile.mkdtemp(prefix="application_cli_"))
    config = json.loads((source_dir / "config/sdk.json").read_text("utf-8"))
    config["telemetry"]["directory"] = str(workspace / "logs")
    config["dds"]["domain"] = CONFIG_DOMAIN
    (workspace / "config.json").write_text(json.dumps(config), "utf-8")
    (workspace / "malformed.json").write_text(
        '{"schema_version": 1, "backend": "fake"', "utf-8")
    shutil.copy(source_dir / "tests/support/fake_scenario.json",
                workspace / "normal.json")
    # A refused torque write: the runtime latches WriteFailed on Boot, so the
    # application meets a lower layer that is already in Fault.
    (workspace / "write_refused.json").write_text(
        json.dumps({"entries": [{"elapsed_us": 0,
                                 "write_result": {"accepted": False}}]}),
        "utf-8")
    return workspace


def case_domain(case):
    """The domain of one case: an offset inside this process's own band,
    wrapping at its top so the offsets can never leave it."""
    return 1 + ((DOMAIN + case - 1) % DOMAIN_BAND)


# `case` gives every case its own domain inside the band: a leftover process of
# an earlier case then cannot share a session with the runtime of a later one.
def runtime_arguments(workspace, case, scenario=None):
    arguments = ["--config", str(workspace / "config.json"),
                 "--domain", str(case_domain(case))]
    if scenario is not None:
        arguments += ["--fake-scenario", str(scenario)]
    return arguments


def application_arguments(workspace, case):
    return ["--config", str(workspace / "config.json"),
            "--domain", str(case_domain(case))]


def test_startup_rejections(workspace):
    cases = [
        ("missing config", ["--config", str(workspace / "absent.json")], 1,
         "cannot open configuration"),
        ("malformed config", ["--config", str(workspace / "malformed.json")],
         1, "invalid JSON"),
        ("unknown argument", ["--config", str(workspace / "config.json"),
                              "--frobnicate"], 2, "usage: application"),
        ("domain out of range", ["--config", str(workspace / "config.json"),
                                 "--domain", "1000"], 2,
         "--domain must be between 0 and 230"),
        # A number too large even for the parser's own type is still a range
        # problem, not a syntax one: stoull throws, and the message has to say
        # what the range is.
        ("domain too large", ["--config", str(workspace / "config.json"),
                              "--domain", "999999999999999999999999"], 2,
         "--domain must be between 0 and 230"),
        ("domain not an integer", ["--config", str(workspace / "config.json"),
                                   "--domain", "abc"], 2,
         "--domain must be an integer"),
    ]
    for name, arguments, expected_code, expected in cases:
        with Application(arguments, workspace, terminal=False) as app:
            code = app.wait_exit(EXIT_WAIT_S)
            check(code == expected_code,
                  failure_report(name, app,
                                 "expected exit %d, got %s"
                                 % (expected_code, code)))
            output = app.output()
            check(expected in output,
                  failure_report(name, app,
                                 "message %r is missing" % expected))
            # Nothing has touched DDS: a rejected invocation exits before the
            # transport exists, so no endpoint or participant error can appear.
            for forbidden in ("endpoint", "participant", "transport",
                              "[application] behavior="):
                check(forbidden not in output,
                      failure_report(name, app,
                                     "the application started anyway (%r)"
                                     % forbidden))


def test_normal_exit_by_key(workspace):
    with Runtime(runtime_arguments(workspace, 1, workspace / "normal.json"),
                 workspace) as runtime:
        started = runtime.wait_for(
            lambda line: "[runtime] runtime started" in line, START_WAIT_S)
        check(started, failure_report("normal exit", runtime,
                                      "the runtime did not start"))

        with Application(application_arguments(workspace, 1), workspace) as app:
            standby = app.wait_for(
                lambda line: "[application] behavior=Standby" in line,
                START_WAIT_S)
            check(standby, failure_report("normal exit", app,
                                          "the application never reached "
                                          "Standby"))
            if standby:
                check("session=owned" in standby[0],
                      failure_report("normal exit", app,
                                     "Standby without an owned session"))
                check("runtime=ZeroTorque" in standby[0],
                      failure_report("normal exit", app,
                                     "Standby without a lower ZeroTorque"))
            app.send_keys("x")
            exit_line = app.wait_for(
                lambda line: line.startswith("application: exit "),
                EXIT_WAIT_S)
            check(exit_line, failure_report("normal exit", app,
                                            "no exit line"))
            if exit_line:
                check("stop=confirmed" in exit_line[0],
                      failure_report("normal exit", app,
                                     "the exit was not confirmed: %s"
                                     % exit_line[0]))
                check("state=Damping" in exit_line[0],
                      failure_report("normal exit", app,
                                     "the confirmed exit does not name the "
                                     "lower state: %s" % exit_line[0]))
            code = app.wait_exit(EXIT_WAIT_S)
            check(code == 0, failure_report("normal exit", app,
                                            "expected exit 0, got %s" % code))

        # Every line from here on is evidence about the lower layer after the
        # upper process is gone: the boundary is the line count taken after the
        # application's exit, so only status lines *emitted* after it are
        # accepted - a few of them, because one line could still have been in
        # flight, while several at the runtime's own status cadence cannot have
        # been produced before the exit.
        runtime_before = runtime.line_count()
        check(runtime.alive(), failure_report("runtime survival", runtime,
                                              "the runtime exited with the "
                                              "application"))
        # The lower layer is an independent process: it keeps damping on its own
        # configuration after the upper one is gone (its supervisor lease simply
        # expires). A clean shutdown of it is a separate operation and is
        # deliberately not asserted here.
        kept = runtime.wait_for(
            lambda line: "[runtime] status" in line and 'state="Damping"' in line,
            5.0, minimum=STATUS_LINES_AFTER_EXIT, start=runtime_before)
        check(kept, failure_report("runtime survival", runtime,
                                   "the runtime stopped controlling after the "
                                   "application exited"))
        runtime.kill()


def test_signal_exit(workspace):
    with Runtime(runtime_arguments(workspace, 2, workspace / "normal.json"),
                 workspace) as runtime:
        runtime.wait_for(lambda line: "[runtime] runtime started" in line,
                         START_WAIT_S)
        with Application(application_arguments(workspace, 2), workspace) as app:
            standby = app.wait_for(
                lambda line: "[application] behavior=Standby" in line,
                START_WAIT_S)
            check(standby, failure_report("signal exit", app,
                                          "the application never reached "
                                          "Standby"))
            app.process.send_signal(signal.SIGINT)
            exit_line = app.wait_for(
                lambda line: line.startswith("application: exit "),
                EXIT_WAIT_S)
            check(exit_line, failure_report("signal exit", app,
                                            "Ctrl+C did not reach the ordered "
                                            "exit"))
            if exit_line:
                check("stop=confirmed" in exit_line[0],
                      failure_report("signal exit", app,
                                     "the signalled exit was not confirmed: %s"
                                     % exit_line[0]))
            code = app.wait_exit(EXIT_WAIT_S)
            check(code == 0, failure_report("signal exit", app,
                                            "expected exit 0, got %s" % code))
        check(runtime.alive(), failure_report("signal exit", runtime,
                                              "the runtime exited with the "
                                              "application"))


def test_latched_runtime_fault(workspace):
    with Runtime(runtime_arguments(workspace, 3, workspace / "write_refused.json"),
                 workspace) as runtime:
        latched = runtime.wait_for(
            lambda line: "FaultLatched" in line and 'fault="WriteFailed"' in line,
            START_WAIT_S)
        check(latched, failure_report("latched fault", runtime,
                                      "the runtime did not latch the refused "
                                      "write"))
        with Application(application_arguments(workspace, 3), workspace) as app:
            faults = app.wait_for(
                lambda line: "[application] behavior=Fault" in line
                and "runtime=Fault" in line, START_WAIT_S)
            check(faults, failure_report("latched fault", app,
                                         "the application did not report the "
                                         "latched lower fault"))
            app.send_keys("x")
            exit_line = app.wait_for(
                lambda line: line.startswith("application: exit "),
                EXIT_WAIT_S)
            check(exit_line, failure_report("latched fault", app,
                                            "no exit line"))
            if exit_line:
                check("stop=unknown" in exit_line[0],
                      failure_report("latched fault", app,
                                     "a refused exit was not reported as "
                                     "unknown: %s" % exit_line[0]))
                check("stop=confirmed" not in exit_line[0],
                      failure_report("latched fault", app,
                                     "the exit claimed a confirmed stop under "
                                     "a latched fault: %s" % exit_line[0]))
                check("lower=Fault" in exit_line[0],
                      failure_report("latched fault", app,
                                     "the exit does not name the lower fault: "
                                     "%s" % exit_line[0]))
            code = app.wait_exit(EXIT_WAIT_S)
            check(code == 3, failure_report("latched fault", app,
                                            "expected exit 3, got %s" % code))
        check(runtime.alive(), failure_report("latched fault", runtime,
                                              "the runtime exited with the "
                                              "application"))
        # Only status lines emitted after the application's exit count, and more
        # than one of them: the latch survived the supervision, it is not a line
        # that was already in the pipe.
        fault_status_before = runtime.line_count()
        still_faulted = runtime.wait_for(
            lambda line: "[runtime] status" in line and 'state="Fault"' in line,
            5.0, minimum=STATUS_LINES_AFTER_EXIT, start=fault_status_before)
        check(still_faulted, failure_report("latched fault", runtime,
                                            "the application cleared the "
                                            "latched fault of the runtime"))
        runtime.kill()


def test_unconfirmable_stop(workspace):
    with Runtime(runtime_arguments(workspace, 4, workspace / "normal.json"),
                 workspace) as runtime:
        runtime.wait_for(lambda line: "[runtime] runtime started" in line,
                         START_WAIT_S)
        with Application(application_arguments(workspace, 4), workspace) as app:
            standby = app.wait_for(
                lambda line: "[application] behavior=Standby" in line,
                START_WAIT_S)
            check(standby, failure_report("unknown stop", app,
                                          "the application never reached "
                                          "Standby"))
            app.send_keys("d")
            damping = app.wait_for(
                lambda line: "[application] behavior=Damping" in line,
                START_WAIT_S)
            check(damping, failure_report("unknown stop", app,
                                          "the application never confirmed "
                                          "Damping"))
            # The lower layer disappears without a chance to answer: the exit
            # can no longer be confirmed and must say so, bounded.
            runtime.kill()
            started_at = time.monotonic()
            app.send_keys("x")
            exit_line = app.wait_for(
                lambda line: line.startswith("application: exit "),
                EOF_WAIT_S)
            check(exit_line, failure_report("unknown stop", app,
                                            "no exit line"))
            if exit_line:
                check("stop=unknown" in exit_line[0],
                      failure_report("unknown stop", app,
                                     "a stop that could not be confirmed was "
                                     "not reported as unknown: %s"
                                     % exit_line[0]))
                check("confirmed" not in exit_line[0],
                      failure_report("unknown stop", app,
                                     "an unconfirmed stop was reported as "
                                     "confirmed: %s" % exit_line[0]))
                check("lower=Unknown" in exit_line[0],
                      failure_report("unknown stop", app,
                                     "the exit does not report an unknown "
                                     "lower state: %s" % exit_line[0]))
            code = app.wait_exit(EOF_WAIT_S)
            elapsed = time.monotonic() - started_at
            check(code == 3, failure_report("unknown stop", app,
                                            "expected exit 3, got %s" % code))
            check(elapsed < STOP_BOUND_S,
                  failure_report("unknown stop", app,
                                 "the exit took %.1f s, more than the bounded "
                                 "%.1f s" % (elapsed, STOP_BOUND_S)))


def test_terminal_attributes(workspace):
    with Runtime(runtime_arguments(workspace, 5, workspace / "normal.json"),
                 workspace) as runtime:
        runtime.wait_for(lambda line: "[runtime] runtime started" in line,
                         START_WAIT_S)

        # A normal exit.
        app = Application(application_arguments(workspace, 5), workspace)
        try:
            opened = app.wait_for(
                lambda line: "[application] behavior=" in line, START_WAIT_S)
            check(opened, failure_report("terminal attributes", app,
                                         "no status line"))
            raw = app.terminal_attributes()
            check(not raw[3] & termios.ICANON,
                  failure_report("terminal attributes", app,
                                 "the application left ICANON set"))
            check(not raw[3] & termios.ECHO,
                  failure_report("terminal attributes", app,
                                 "the application left ECHO set"))
            # The exit is only confirmable against a live session, so the
            # application is given the ordered exit it can actually complete.
            app.wait_for(lambda line: "[application] behavior=Standby" in line,
                         START_WAIT_S)
            app.send_keys("x")
            app.wait_for(lambda line: line.startswith("application: exit "),
                         EXIT_WAIT_S)
            check(app.wait_exit(EXIT_WAIT_S) == 0,
                  failure_report("terminal attributes", app,
                                 "the application did not exit 0"))
            restored = app.terminal_attributes()
            check(restored == app.initial_attributes,
                  failure_report("terminal attributes", app,
                                 "the attributes were not restored on a normal "
                                 "exit: %r != %r"
                                 % (restored, app.initial_attributes)))
        finally:
            app.kill()

        # A handled signal: the same restore on an abnormal exit path.
        app = Application(application_arguments(workspace, 5), workspace)
        try:
            app.wait_for(lambda line: "[application] behavior=Standby" in line,
                         START_WAIT_S)
            app.process.send_signal(signal.SIGINT)
            app.wait_for(lambda line: line.startswith("application: exit "),
                         EXIT_WAIT_S)
            check(app.wait_exit(EXIT_WAIT_S) == 0,
                  failure_report("terminal attributes", app,
                                 "the signalled exit did not return 0"))
            restored = app.terminal_attributes()
            check(restored == app.initial_attributes,
                  failure_report("terminal attributes", app,
                                 "the attributes were not restored after "
                                 "SIGINT: %r != %r"
                                 % (restored, app.initial_attributes)))
        finally:
            app.kill()


def test_input_eof(workspace):
    with Runtime(runtime_arguments(workspace, 6, workspace / "normal.json"),
                 workspace) as runtime:
        runtime.wait_for(lambda line: "[runtime] runtime started" in line,
                         START_WAIT_S)
        app = Application(application_arguments(workspace, 6), workspace)
        try:
            standby = app.wait_for(
                lambda line: "[application] behavior=Standby" in line,
                START_WAIT_S)
            check(standby, failure_report("input loss", app,
                                          "the application never reached "
                                          "Standby"))
            started_at = time.monotonic()
            app.close_terminal()
            code = app.wait_exit(EOF_WAIT_S)
            elapsed = time.monotonic() - started_at
            check(code == 0, failure_report("input loss", app,
                                            "expected exit 0 after EOF, got %s"
                                            % code))
            check(elapsed < STOP_BOUND_S,
                  failure_report("input loss", app,
                                 "the exit took %.1f s, more than the bounded "
                                 "%.1f s" % (elapsed, STOP_BOUND_S)))
        finally:
            app.kill()
        check(runtime.alive(), failure_report("input loss", runtime,
                                              "the runtime exited with the "
                                              "application"))


def main():
    global APPLICATION_BINARY
    global RUNTIME_BINARY
    global SOURCE_DIR
    if len(sys.argv) != 4:
        print("usage: application_cli_test.py <application> <robot_runtime> "
              "<source directory>")
        return 2
    APPLICATION_BINARY = Path(sys.argv[1]).resolve()
    RUNTIME_BINARY = Path(sys.argv[2]).resolve()
    SOURCE_DIR = Path(sys.argv[3]).resolve()
    if not APPLICATION_BINARY.is_file():
        print("FAIL: application binary %s does not exist"
              % APPLICATION_BINARY)
        return 1
    if not RUNTIME_BINARY.is_file():
        print("FAIL: robot_runtime binary %s does not exist" % RUNTIME_BINARY)
        return 1

    print("application_cli_test: application=%s runtime=%s source=%s domain=%d"
          % (APPLICATION_BINARY, RUNTIME_BINARY, SOURCE_DIR, DOMAIN))
    workspace = build_workspace(SOURCE_DIR)
    try:
        test_startup_rejections(workspace)
        test_normal_exit_by_key(workspace)
        test_signal_exit(workspace)
        test_latched_runtime_fault(workspace)
        test_unconfirmable_stop(workspace)
        test_terminal_attributes(workspace)
        test_input_eof(workspace)
    finally:
        shutil.rmtree(workspace, ignore_errors=True)

    if FAILURES:
        print("%d check(s) failed" % len(FAILURES))
        return 1
    print("application_cli_test: all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
