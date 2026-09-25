#!/usr/bin/env python3
"""Cross-process phase acceptance of the real application against the real runtime.

Usage: application_flow_test.py <application> <robot_runtime> <source directory>

The scenario matrix drives the application through the harness, one process
tree per scenario. This entry is the orchestration the single-scenario format
cannot express: cases in which the two real processes are interrupted while the
other one keeps running, and the phase invariant that no run ever enters
`Locomotion`. Every case gets its own DDS domain through the harness's
`choose_domain` (the lock file and the RTPS port probe), its own runtime, its own
configuration and its own PTY, and every process it starts is killed on both the
success and the failure path.

Cases, each printing one `case <name>: ...` line:

  restart             the runtime is replaced while the application supervises
                      it: the upper reports Fault and never returns to a
                      tracking behaviour on the new instance by itself
  state_link          the runtime is SIGSTOPped past the state window and
                      SIGCONTinued: the upper reports Unknown inside the window,
                      latches its own Fault, and the runtime keeps running
  app_paused          the application is SIGSTOPped while it holds Ready: the
                      runtime falls back on its own command timeout
  app_killed          the application is SIGKILLed while it holds Ready: the
                      runtime expires the supervisor lease on its own
  normal_exit         a normal application exit with a runtime left damping and
                      unsupervised: run through the scenario harness itself
                      (application_normal), one of the shipped scenario files
  locomotion          the phase invariant: no run in this acceptance ever shows
                      behavior=Locomotion. Recorded as the third-phase boundary,
                      not as a pass: the enumerator exists and the entry guard
                      refuses, but this phase ships no locomotion target
                      generator and this acceptance does not accept one

Nothing here asserts anything about motor execution, and no case reads the
application's behaviour out of the runtime's process or the other way round:
each fact is read from the process that owns it.

Only the standard library is used. `tools/run_integration.py` is reused for the
domain claim and, for the `normal_exit` case, invoked as the subprocess it is.
"""

import importlib.util
import json
import os
import pty
import re
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

# Waits are bounded; the bounds are properties of the phase's configuration
# (command_us 100 ms, source_state_us 150 ms, supervisor_us 300 ms,
# request_deadline_us 1 s) plus scheduling slack on the test host.
START_WAIT_S = 15.0
BEHAVIOR_WAIT_S = 10.0
FALLBACK_WAIT_S = 5.0
EXIT_WAIT_S = 10.0
STOP_BOUND_S = 5.0
KILL_WAIT_S = 10.0
# The application must report an unknown lower state inside
# application.supervision.state_timeout_us plus slack.
STATE_WINDOW_MS = 150.0
STATE_SLACK_MS = 450.0

FAILURES = []
# Every case ever built, so a case whose builder raised half-way is still
# cleaned up: its processes are killed by the driver's final sweep.
CASES = []


def check(condition, message):
    """Records one assertion; a failing assertion fails the entry at the end."""
    if condition:
        return True
    FAILURES.append(message)
    print("FAIL: " + message)
    return False


def pattern(text):
    """A line predicate matching a substring, the way the harness matches."""
    return lambda line: text in line


class Process:
    """One child process with a bounded, non-blocking output reader."""

    def __init__(self, binary, arguments, cwd, terminal=False):
        self.command = [str(binary), *arguments]
        self.terminal = terminal
        self.master = None
        self.lines = []
        self.stamps = []
        self.pid = None
        self._condition = threading.Condition()
        if terminal:
            # The application owns a terminal: it reads the keyboard from it and
            # prints its status to it. The slave is closed here right after the
            # spawn, so the master reports end-of-file the moment the process
            # leaves; on Linux the master still reports the slave's termios.
            master, slave = pty.openpty()
            self.process = subprocess.Popen(
                self.command, cwd=str(cwd), stdin=slave, stdout=slave,
                stderr=slave)
            os.close(slave)
            self.master = os.fdopen(master, "rb")
        else:
            self.process = subprocess.Popen(
                self.command, cwd=str(cwd), stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.master = self.process.stdout
        self.pid = self.process.pid
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        descriptor = self.master
        while True:
            try:
                if self.terminal:
                    ready, _, _ = select.select([descriptor], [], [], 0.1)
                    if not ready:
                        if self.process.poll() is not None:
                            return
                        continue
                chunk = os.read(descriptor.fileno(), 4096) if self.terminal \
                    else descriptor.readline()
            except (OSError, ValueError):
                return
            if not chunk:
                return
            for line in chunk.decode("utf-8", "replace").splitlines(True):
                with self._condition:
                    self.lines.append(line.rstrip("\r\n"))
                    self.stamps.append(time.monotonic())
                    self._condition.notify_all()

    def _drain(self):
        if not self.alive():
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

    def stamped(self, start=0):
        """The `(arrival, line)` pairs from `start` on."""
        with self._condition:
            return list(zip(self.stamps[start:], self.lines[start:]))

    def wait_lines(self, predicate, timeout, minimum=1, start=0):
        """Bounded wait for `minimum` lines matching `predicate` from `start`."""
        deadline = time.monotonic() + timeout
        while True:
            with self._condition:
                found = [line for line in self.lines[start:] if predicate(line)]
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
            code = self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None
        self._drain()
        return code

    def signal(self, name):
        if not self.alive():
            return False
        self.process.send_signal(getattr(signal, name))
        return True

    def send_keys(self, keys):
        if not self.alive():
            return False
        os.write(self.master.fileno(), keys.encode("utf-8")
                 if isinstance(keys, str) else keys)
        return True

    def exit_line(self):
        found = re.search(r"^application: exit [^\n]*", self.output(), re.M)
        return found.group(0) if found is not None else None

    def kill(self):
        if self.process.poll() is None:
            try:
                self.process.kill()
            except OSError:
                pass
        try:
            self.process.wait(timeout=KILL_WAIT_S)
        except subprocess.TimeoutExpired:
            pass
        self._drain()
        if self.terminal and self.master is not None:
            try:
                self.master.close()
            except OSError:
                pass
            self.master = None


class Case:
    """One case: its domain, its workspace and everything it started.

    Nothing survives a case: `finish()` kills whatever it started, whatever
    happened before, and the survivor list of the summary is built from exactly
    those process ids.
    """

    def __init__(self, root, name):
        self.name = name
        self.work = Path(root) / name
        self.work.mkdir(parents=True, exist_ok=True)
        CASES.append(self)
        self.domain = 0
        self.lock = None
        self.processes = []
        self.applications = []
        self.notes = []

    def claim_domain(self):
        """Claims one DDS domain for this case through the harness's claim rule.

        The declared domain differs per case, so the summary shows one domain
        per case; the claim (exclusive lock file plus the RTPS port probe) is
        what makes the isolation real even when two cases, or a case and a
        scenario entry, run at the same time.

        The lock has to be claimed where the other entries claim theirs - the
        shared `integration` directory, one level above this entry's own
        `application_flow` root - because a lock in a private namespace excludes
        nobody: this entry's declared band (190..229) is the same band the
        scenario files declare, so without a shared lock two ctest entries can
        still pick the same domain and share a DDS participant.
        """
        declared = 190 + ((os.getpid() * 7 + sum(self.name.encode())) % 40)
        self.domain, self.lock = RUN_INTEGRATION.choose_domain(
            declared, self.work, lock_dir=self.work.parent.parent)
        return self.domain

    def configuration(self, fake_scenario="tests/support/fake_static.json"):
        config = json.loads((SOURCE_DIR / "config/sdk.json").read_text("utf-8"))
        config["dds"]["domain"] = self.domain
        config["telemetry"]["directory"] = str(self.work / "logs")
        (self.work / "config.json").write_text(json.dumps(config, indent=2),
                                               "utf-8")
        scenario = self.work / "fake_scenario.json"
        shutil.copyfile(SOURCE_DIR / fake_scenario, scenario)
        return self.work / "config.json", scenario

    def runtime_arguments(self, config, scenario, trace=False, run=1):
        arguments = ["--config", str(config), "--domain", str(self.domain),
                     "--fake-scenario", str(scenario)]
        if trace:
            arguments += ["--output-trace", str(self.work / ("trace%d.jsonl"
                                                             % run))]
        return arguments

    def application_arguments(self, config):
        return ["--config", str(config), "--domain", str(self.domain)]

    def start_runtime(self, arguments):
        runtime = Process(RUNTIME_BINARY, arguments, self.work)
        self.processes.append(runtime)
        return runtime

    def start_application(self, arguments):
        application = Process(APPLICATION_BINARY, arguments, self.work,
                              terminal=True)
        self.processes.append(application)
        self.applications.append(application)
        return application

    def write_logs(self):
        """The evidence of this case: every process's merged output."""
        for index, process in enumerate(self.processes, start=1):
            name = "application" if process.terminal else "runtime"
            (self.work / ("%s_run%d.log" % (name, index))).write_text(
                process.output(), "utf-8")

    def finish(self):
        for process in reversed(self.processes):
            process.kill()
        self.write_logs()
        if self.lock is not None:
            RUN_INTEGRATION.release_domain(self.lock)
            self.lock = None

    def survivors(self):
        return [process.pid for process in self.processes if process.alive()]


def wait_status(process, behaviour, timeout=BEHAVIOR_WAIT_S, start=0):
    """Waits for one status line of the application reporting `behaviour`."""
    return process.wait_lines(lambda line: line.startswith("[application] ")
                              and ("behavior=%s" % behaviour) in line,
                              timeout, start=start)


def press(process, keys, behaviour, timeout=BEHAVIOR_WAIT_S):
    """Sends one key and waits for the behaviour it must produce."""
    marker = process.line_count()
    process.send_keys(keys)
    found = wait_status(process, behaviour, timeout, start=marker)
    check(bool(found), "key %r did not produce behavior=%s: %s"
          % (keys, behaviour, process.output()[-600:]))
    return found


def reach_ready(case):
    """Drives an application to the Ready hold over a fresh runtime."""
    config, scenario = case.configuration()
    runtime = case.start_runtime(case.runtime_arguments(config, scenario,
                                                        trace=True))
    check(bool(runtime.wait_lines(pattern('state="ZeroTorque"'), START_WAIT_S)),
          "the runtime did not reach ZeroTorque: %s" % runtime.output()[-600:])
    application = case.start_application(case.application_arguments(config))
    check(bool(wait_status(application, "Standby", START_WAIT_S)),
          "the application never reached Standby: %s"
          % application.output()[-600:])
    press(application, "d", "Damping")
    press(application, "i", "Ready")
    return runtime, application


def latency_ms(process, predicate, reference):
    """Arrival of the first matching line at or after `reference`."""
    for stamp, line in process.stamped():
        if stamp >= reference and predicate(line):
            return (stamp - reference) * 1000.0
    return None


# ---------------------------------------------------------------------------
# Cases
# ---------------------------------------------------------------------------

def case_restart(root):
    """The runtime is replaced under a running application."""
    case = Case(root, "restart")
    case.claim_domain()
    config, scenario = case.configuration()
    first = case.start_runtime(case.runtime_arguments(config, scenario))
    check(bool(first.wait_lines(pattern('state="ZeroTorque"'), START_WAIT_S)),
          "the first runtime did not reach ZeroTorque")
    application = case.start_application(case.application_arguments(config))
    check(bool(wait_status(application, "Standby", START_WAIT_S)),
          "the application never reached Standby: %s"
          % application.output()[-600:])

    identity = re.search(r'runtime_id="([0-9a-f]+)"', first.output())
    first.signal("SIGTERM")
    code = first.wait_exit(STOP_BOUND_S)
    check(code == 0, "the runtime in ZeroTorque did not stop cleanly (exit %s)"
          % code)

    marker = application.line_count()
    second = case.start_runtime(case.runtime_arguments(config, scenario))
    check(bool(second.wait_lines(pattern('state="ZeroTorque"'), START_WAIT_S)),
          "the second runtime did not reach ZeroTorque")
    restarted = re.search(r'runtime_id="([0-9a-f]+)"', second.output())
    check(identity is not None and restarted is not None
          and identity.group(1) != restarted.group(1),
          "the restarted runtime reports the same instance identity")

    faults = wait_status(application, "Fault", FALLBACK_WAIT_S, start=marker)
    check(bool(faults), "the application did not report Fault after the runtime "
          "was replaced: %s" % application.output()[-600:])
    rebound = application.wait_lines(
        lambda line: "behavior=Fault" in line and "runtime=ZeroTorque" in line
        and "session=owned" in line, FALLBACK_WAIT_S, start=marker)
    check(bool(rebound), "the application did not bind the new instance while "
          "staying latched in Fault: %s" % application.output()[-600:])
    forbidden = [line for line in application.stamped(marker)
                 if any(("behavior=%s" % state) in line
                        for state in ("Standby", "Damping", "Initializing",
                                      "Ready", "Locomotion"))]
    check(not forbidden, "the application returned to a behaviour of its own "
          "after the runtime was replaced: %s" % forbidden[:2])
    case.notes.append("instance %s -> %s, %d Fault line(s) after the restart"
                      % (identity.group(1)[:8] if identity else "?",
                         restarted.group(1)[:8] if restarted else "?",
                         len(faults)))
    return case


def case_state_link(root):
    """A one-way interruption of the state link: SIGSTOP, then SIGCONT."""
    case = Case(root, "state_link")
    case.claim_domain()
    config, scenario = case.configuration()
    runtime = case.start_runtime(case.runtime_arguments(config, scenario))
    runtime.wait_lines(pattern('state="ZeroTorque"'), START_WAIT_S)
    application = case.start_application(case.application_arguments(config))
    wait_status(application, "Standby", START_WAIT_S)

    marker = application.line_count()
    runtime_marker = runtime.line_count()
    stopped = time.monotonic()
    check(runtime.signal("SIGSTOP"), "the runtime could not be stopped")
    unknown = application.wait_lines(
        lambda line: "runtime=Unknown" in line, FALLBACK_WAIT_S, start=marker)
    check(bool(unknown), "the application never reported an unknown lower "
          "state: %s" % application.output()[-600:])
    latency = latency_ms(application, lambda line: "runtime=Unknown" in line,
                         stopped)
    check(latency is not None and latency <= STATE_WINDOW_MS + STATE_SLACK_MS,
          "the unknown lower state was reported after %s ms" % latency)
    check(latency is not None and latency >= STATE_WINDOW_MS * 0.5,
          "the unknown lower state came after only %s ms, before the state "
          "window could have lapsed" % latency)
    check(bool(wait_status(application, "Fault", FALLBACK_WAIT_S, start=marker)),
          "the application did not latch its own Fault while the link was "
          "silent: %s" % application.output()[-600:])
    check(runtime.alive(), "the runtime died while it was stopped")

    time.sleep(1.2)
    check(runtime.signal("SIGCONT"), "the runtime could not be continued")
    kept = runtime.wait_lines(pattern('state="'), 4.0, minimum=2,
                              start=runtime_marker)
    check(len(kept) >= 2, "the runtime stopped controlling after SIGCONT")
    owned = application.wait_lines(pattern("session=owned"), FALLBACK_WAIT_S,
                                   start=marker)
    check(bool(owned), "the application did not bind the resumed runtime again: "
          "%s" % application.output()[-600:])
    after = [line for line in application.stamped(marker)
             if any(("behavior=%s" % state) in line
                    for state in ("Standby", "Damping", "Initializing", "Ready",
                                  "Locomotion"))]
    check(not after, "the latched Fault did not survive the resumption: %s"
          % after[:2])

    check(application.send_keys("x"), "the exit key could not be sent")
    code = application.wait_exit(EXIT_WAIT_S)
    line = application.exit_line()
    check(code == 0 and line is not None and "stop=confirmed" in line,
          "the exit after the resumed link was not confirmed (exit %s, %r)"
          % (code, line))
    # The runtime is left damping with a session whose supervisor is gone: it
    # deliberately refuses to leave on its own, which is what is asserted here.
    kept_damping = runtime.wait_lines(pattern('state="Damping"'), 5.0,
                                     minimum=2, start=runtime.line_count())
    check(len(kept_damping) >= 2,
          "the runtime stopped logging while it was left damping")
    check(runtime.signal("SIGTERM"), "the runtime could not be signalled")
    refused = runtime.wait_lines(pattern("no request path is available"), 3.0)
    check(bool(refused), "the runtime left a damping state without a "
          "supervision session, which it must not: %s"
          % runtime.output()[-400:])
    check(runtime.alive(), "the runtime exited although it had no supervisor")
    case.notes.append("unknown after %.0f ms, resume kept %d status line(s)"
                      % (latency if latency is not None else -1, len(kept)))
    return case


def case_app_paused(root):
    """The application is SIGSTOPped while it holds Ready."""
    case = Case(root, "app_paused")
    case.claim_domain()
    runtime, application = reach_ready(case)
    marker = runtime.line_count()
    stopped = time.monotonic()
    check(application.signal("SIGSTOP"), "the application could not be stopped")
    fallback = runtime.wait_lines(pattern("CommandTimeout"), FALLBACK_WAIT_S,
                                  start=marker)
    check(bool(fallback), "the runtime did not fall back on its own command "
          "timeout while the application was paused: %s"
          % runtime.output()[-600:])
    latency = latency_ms(runtime, pattern("CommandTimeout"), stopped)
    check(latency is not None and 50.0 <= latency <= 600.0,
          "the command timeout came after %s ms" % latency)
    check('stop="CommandTimeout"' in fallback[0],
          "the runtime left tracking for another reason: %s" % fallback[0])
    check('from="Tracking"' in fallback[0],
          "the runtime was not tracking when the timeout hit: %s" % fallback[0])
    check(runtime.alive(), "the runtime died while the application was paused")

    resume_marker = application.line_count()
    check(application.signal("SIGCONT"), "the application could not be "
          "continued")
    # Only lines printed after the resumption count: the same status was on the
    # terminal before the pause, and accepting it would prove nothing.
    settled = application.wait_lines(
        lambda line: "runtime=Damping" in line
        and ("behavior=Damping" in line or "behavior=Fault" in line),
        FALLBACK_WAIT_S, start=resume_marker)
    check(bool(settled), "the application did not show the lower layer's own "
          "fallback after it was resumed: %s" % application.output()[-600:])
    # The fallback is not just a behaviour change: the lower layer's own stop
    # reason has to reach the display, or an operator cannot tell its command
    # timeout from any other stop.
    check(bool(settled) and "stop=CommandTimeout" in settled[0],
          "the application did not surface the lower layer's stop reason: %s"
          % (settled[0] if settled else application.output()[-600:]))
    again = [line for line in application.stamped(resume_marker)
             if any(("behavior=%s" % state) in line
                    for state in ("Initializing", "Ready", "Locomotion"))]
    check(not again, "the application resumed the tracking behaviour the runtime "
          "had already dropped: %s" % again[:2])
    case.notes.append("runtime fell back %.0f ms after the SIGSTOP, %r"
                      % (latency if latency is not None else -1,
                         settled[0][:60] if settled else ""))
    return case


def case_app_killed(root):
    """The application is SIGKILLed while it holds Ready."""
    case = Case(root, "app_killed")
    case.claim_domain()
    runtime, application = reach_ready(case)
    marker = runtime.line_count()
    killed = time.monotonic()
    check(application.signal("SIGKILL"), "the application could not be killed")
    code = application.wait_exit(KILL_WAIT_S)
    check(code == -signal.SIGKILL,
          "the killed application reported exit %s" % code)

    fallback = runtime.wait_lines(pattern("CommandTimeout"), FALLBACK_WAIT_S,
                                  start=marker)
    check(bool(fallback), "the runtime did not fall back on its own command "
          "timeout after the application was killed: %s"
          % runtime.output()[-600:])
    expired = runtime.wait_lines(pattern("SessionExpired"), FALLBACK_WAIT_S,
                                 start=marker)
    check(bool(expired), "the runtime did not expire the supervisor lease after "
          "the application was killed: %s" % runtime.output()[-600:])
    lease_ms = latency_ms(runtime, pattern("SessionExpired"), killed)
    check(lease_ms is not None and 200.0 <= lease_ms <= 1500.0,
          "the lease expired after %s ms (supervisor_us is 300 ms)" % lease_ms)
    check(runtime.alive(), "the runtime died with the application")
    kept = runtime.wait_lines(pattern('state="Damping"'), 5.0, minimum=2,
                              start=runtime.line_count())
    check(len(kept) >= 2, "the runtime stopped damping after the application "
          "was killed")

    # The runtime's own per-tick record is the evidence for the revocation: the
    # session of every tick after the expiry is the all-zero "none" id.
    trace = case.work / "trace1.jsonl"
    ticks = []
    for line in trace.read_text("utf-8", "replace").splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue  # the writer was killed mid-line; the trace is evidence
        if record.get("kind") == "tick":
            ticks.append(record)
    revoked = [tick for tick in ticks
               if tick["session_id"] == "0" * 32 and tick["state"] == "Damping"]
    check(len(revoked) >= 10, "the runtime's own trace holds only %d damping "
          "tick(s) without a session out of %d" % (len(revoked), len(ticks)))

    check(runtime.signal("SIGTERM"), "the runtime could not be signalled")
    refused = runtime.wait_lines(pattern("no request path is available"), 3.0)
    check(bool(refused), "the runtime left a damping state without a "
          "supervision session, which it must not")
    check(runtime.alive(), "the runtime exited although it had no supervisor")
    case.notes.append("fallback and lease expiry in the runtime's own log, %d "
                      "damping tick(s) without a session" % len(revoked))
    return case


def case_normal_exit(root):
    """A normal application exit with the runtime surviving, through the harness.

    The scenario file carries the steps and the checks; this case only runs it
    and reports its summary line, so the single-scenario wording and the
    cross-process entry agree instead of being maintained twice.
    """
    case = Case(root, "normal_exit")
    client = RUNTIME_BINARY.parent / "runtime_client"
    scenario = SOURCE_DIR / "tests/integration/scenarios/application_normal.json"
    if not client.is_file():
        check(False, "the harness reuse case needs the test client %s" % client)
        return case
    command = [
        sys.executable, str(SOURCE_DIR / "tools/run_integration.py"),
        "--scenario", str(scenario),
        "--runtime", str(RUNTIME_BINARY),
        "--client", str(client),
        "--application", str(APPLICATION_BINARY),
        "--source", str(SOURCE_DIR),
        "--work-dir", str(case.work / "harness"),
    ]
    result = subprocess.run(command, cwd=str(SOURCE_DIR), text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            timeout=300)
    summary = [line.strip() for line in result.stdout.splitlines()
               if line.strip().startswith("scenario application_normal:")]
    passed = any(line.endswith("PASSED") for line in summary)
    check(result.returncode == 0 and passed,
          "the harness scenario application_normal did not pass (exit %s):\n%s"
          % (result.returncode, result.stdout[-1500:]))
    # The harness claims the domain and the evidence directory exclusively and
    # releases both; a leftover lock would mean it did not finish its cleanup.
    leftovers = sorted(path.name for path in (case.work / "harness").glob(".*")
                       if path.name.startswith((".claim-", ".domain-")))
    check(not leftovers, "the harness left its locks behind: %s" % leftovers)
    domain = case.work / "harness" / "application_normal" / "domain.txt"
    check(domain.is_file(), "the harness recorded no domain for the scenario")
    if domain.is_file():
        case.notes.append("harness domain %s" % domain.read_text().strip())
    if summary:
        case.notes.append(summary[0].split(", ", 1)[0])
    return case


def case_locomotion(root):
    """The phase invariant: no run of this acceptance enters Locomotion."""
    case = Case(root, "locomotion")
    logs = sorted(Path(root).glob("*/application*.log")) + \
        sorted(Path(root).glob("*/harness/*/application*.log"))
    offenders = []
    inspected = 0
    for path in logs:
        text = path.read_text("utf-8", "replace")
        inspected += 1
        if "behavior=Locomotion" in text:
            offenders.append(str(path))
    check(not offenders, "behavior=Locomotion was observed in %s" % offenders)
    case.notes.append("BOUNDARY not accepted: behavior=Locomotion never "
                      "appeared in %d application log(s); the enumerator exists "
                      "and its entry guard refuses, but this phase ships no "
                      "locomotion target generator" % inspected)
    return case


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    global APPLICATION_BINARY
    global RUNTIME_BINARY
    global SOURCE_DIR
    global RUN_INTEGRATION
    if len(sys.argv) != 4:
        print("usage: application_flow_test.py <application> <robot_runtime> "
              "<source directory>")
        return 2
    APPLICATION_BINARY = Path(sys.argv[1]).resolve()
    RUNTIME_BINARY = Path(sys.argv[2]).resolve()
    SOURCE_DIR = Path(sys.argv[3]).resolve()
    for name, binary in (("application", APPLICATION_BINARY),
                         ("robot_runtime", RUNTIME_BINARY)):
        if not binary.is_file():
            print("FAIL: the %s binary %s does not exist" % (name, binary))
            return 1

    spec = importlib.util.spec_from_file_location(
        "run_integration", SOURCE_DIR / "tools/run_integration.py")
    RUN_INTEGRATION = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(RUN_INTEGRATION)

    print("application_flow_test: application=%s runtime=%s source=%s"
          % (APPLICATION_BINARY, RUNTIME_BINARY, SOURCE_DIR))
    # The evidence lives next to the scenario matrix's evidence and is kept: the
    # logs of the cases are what a reader inspects afterwards.
    root = RUNTIME_BINARY.parent / "integration" / "application_flow"
    shutil.rmtree(root, ignore_errors=True)
    root.mkdir(parents=True, exist_ok=True)
    print("application_flow_test: evidence in %s" % root)
    try:
        for builder in (case_restart, case_state_link, case_app_paused,
                        case_app_killed, case_normal_exit, case_locomotion):
            try:
                builder(root)
            except Exception as error:  # noqa: BLE001 - reported, not hidden
                check(False, "case %s raised %r" % (builder.__name__, error))
            # Each finished case has its evidence on disk before the next one
            # starts, and the invariant scan reads exactly those logs.
            for case in CASES:
                case.finish()
    finally:
        # Whatever a case did before it failed, nothing of it survives: this is
        # the one place every started process is killed.
        for case in CASES:
            case.finish()

    print()
    for case in CASES:
        survivors = case.survivors()
        if survivors:
            check(False, "case %s left process(es) %s behind"
                  % (case.name, survivors))
        print("case %-12s domain %3d %s" % (
            case.name, case.domain,
            "; ".join(case.notes) if case.notes else "-"))
    if FAILURES:
        print("%d check(s) failed" % len(FAILURES))
        return 1
    print("application_flow_test: all cases passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
