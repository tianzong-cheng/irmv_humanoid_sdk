#!/usr/bin/env python3
"""Scenario harness of the runtime failure matrix (Task 10).

Every file in tests/integration/scenarios/ is one scenario: an isolated DDS
domain, the runtime invocation (configuration, fake scenario, output trace), the
client invocations with their steps (start, wait, SIGSTOP/SIGCONT, SIGKILL,
SIGTERM, restart) and the checks that must hold. This script runs exactly one of
them and returns non-zero unless every check passed, so CTest can register each
scenario as an independent entry:

  run_integration.py --scenario FILE --runtime BIN --client BIN --source DIR \\
                     --work-dir DIR [--application BIN]

A scenario may also drive the real upper process instead of a test client: with
`--application BIN`, `scenario["application"]` describes that invocation and the
steps `start_application`, `stop_application`, `kill_application`,
`restart_application`, `wait_application_log` and `application_keys` drive it
through a real PTY. The application and the runtime of one scenario always share
the generated `config.json` and the chosen domain, so their configuration
digests agree; the application is never simulated by `runtime_client`.

`--all` runs every scenario of the source tree in one process and prints one
summary line per scenario; it is meant for manual runs, never for CTest.

Evidence: the runtime log, the application log, every client's JSON-lines
observations and stderr and the per-tick output trace are collected in the work
directory, the selected domain is recorded in domain.txt, and every check prints
what it measured. A step that did not run is never reported as a pass: the
scenario fails at the first step that does not complete.

Only the standard library is used.
"""

import argparse
import array
import errno
import fcntl
import json
import os
import pty
import re
import shutil
import signal
import socket
import subprocess
import sys
import termios
import threading
import time
from pathlib import Path

# F_GETPIPE_SZ is not exported by every Python build.
kFGetPipeSize = getattr(fcntl, "F_GETPIPE_SZ", 1032)

# The client and the runtime both stamp observations with the host monotonic
# clock, which is the clock Python exposes as time.monotonic on Linux: arrival
# stamps of different processes are therefore directly comparable.
def now_us():
    return time.monotonic_ns() // 1000


class Failure(Exception):
    """A step or a check that did not hold."""


class Processes:
    """Every process the scenario started, so nothing survives a failure."""

    def __init__(self):
        self.runtime = None
        self.application = None
        self.clients = {}

    def shutdown(self):
        for client in list(self.clients.values()):
            client.kill()
        if self.application is not None:
            self.application.kill()
        if self.runtime is not None:
            self.runtime.kill()


class StreamDrain(threading.Thread):
    """Reads one pipe into a byte sink without blocking the child."""

    def __init__(self, stream, sink, stamps=None):
        super().__init__(daemon=True)
        self.stream = stream
        self.sink = sink
        # An optional second sink taking the arrival instant of every line, for
        # a caller that has to measure when a process printed what.
        self.stamps = stamps

    def run(self):
        try:
            for line in iter(self.stream.readline, b""):
                self.sink.append(line)
                if self.stamps is not None:
                    self.stamps.append(now_us())
        except (OSError, ValueError):
            pass


class Runtime:
    """One robot_runtime process.

    stdout is merged into stderr, which is where the structured log goes, and
    both are left undrained until `drain()` is called: the slow-logger scenario
    needs the logger's consumer to block on a full pipe.
    """

    def __init__(self, binary, work_dir, run_index):
        self.binary = Path(binary)
        self.work_dir = Path(work_dir)
        self.run_index = run_index
        self.log_path = self.work_dir / ("runtime_run%d.log" % run_index)
        self.trace_path = self.work_dir / ("trace_run%d.jsonl" % run_index)
        self.process = None
        self.lines = []
        self.drainer = None
        self.arguments = []
        self.started_us = 0

    def start(self, arguments):
        self.arguments = list(arguments)
        self.lines = []
        self.process = subprocess.Popen(
            [str(self.binary)] + self.arguments,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            cwd=str(self.work_dir),
        )
        self.started_us = now_us()
        return self

    def drain(self):
        if self.drainer is None and self.process is not None:
            self.drainer = StreamDrain(self.process.stdout, self.lines)
            self.drainer.start()

    def pipe_occupancy(self):
        """Bytes waiting in the stdout/stderr pipe and its capacity.

        Read without consuming: FIONREAD reports what a read would return right
        now, so a scenario can prove that a blocked consumer really left the
        pipe full before it measures anything else.
        """
        descriptor = self.process.stdout.fileno()
        pending = array.array("i", [0])
        fcntl.ioctl(descriptor, termios.FIONREAD, pending, True)
        try:
            capacity = fcntl.fcntl(descriptor, kFGetPipeSize)
        except (OSError, ValueError):
            capacity = 65536
        return pending[0], capacity

    def text(self):
        return b"".join(self.lines).decode("utf-8", "replace")

    def wait_for(self, pattern, timeout_s, minimum=1, start=0):
        """Waits for `minimum` log lines while draining, returns the matching line.

        `start` is the index to begin scanning at, so a scenario can require
        lines that were printed *after* something it did rather than accepting
        one that was already in the pipe.
        """
        self.drain()
        deadline = time.monotonic() + timeout_s
        expression = re.compile(pattern)
        seen = start
        found = 0
        while True:
            while seen < len(self.lines):
                line = self.lines[seen].decode("utf-8", "replace")
                seen += 1
                if expression.search(line):
                    found += 1
                    if found >= minimum:
                        return line
            if self.process.poll() is not None and seen >= len(self.lines):
                raise Failure(
                    "the runtime exited with %s before matching %r %d time(s)"
                    % (self.process.returncode, pattern, minimum)
                )
            if time.monotonic() >= deadline:
                raise Failure("the runtime matched %r only %d time(s) within "
                              "%.1f s" % (pattern, found, timeout_s))
            time.sleep(0.02)

    def signal(self, name):
        if self.process is None or self.process.poll() is not None:
            raise Failure("the runtime is not running, cannot send %s" % name)
        self.process.send_signal(getattr(signal, name))

    def wait_exit(self, timeout_s, drain=True):
        """Waits for the exit; returns the code or None on timeout.

        A scenario that measures the exit itself while the log consumer is
        blocked passes drain=False, so the pipe stays full until the process
        has left; the log is read afterwards.
        """
        if drain:
            self.drain()
        try:
            return self.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return None

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def kill(self):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            self.process.kill()
        except OSError:
            pass
        self.process.wait(timeout=10)
        self.drain()

    def write_log(self):
        # Drain first: a scenario that never waited on a log line would otherwise
        # leave its evidence file empty even though the process produced output.
        self.drain()
        if self.process is not None:
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        self.log_path.write_text(self.text())

    def clean_stop(self):
        """Parsed `clean shutdown:` line, or None when the run did not print it."""
        found = re.search(r"robot_runtime: clean shutdown: ([^\n]*)\n", self.text())
        if found is None:
            return None
        values = {}
        for token in found.group(1).split():
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            try:
                values[key] = int(value)
            except ValueError:
                values[key] = value
        return values


class Application:
    """One `application` process, driven through a real PTY.

    The application owns a terminal: it puts it into raw mode, reads the
    keyboard from it and prints its status line to stdout. The harness
    therefore starts it the way an operator does, on a pty pair, and writes the
    bytes of `application_keys` to the master - the bytes the application reads
    are exactly the bytes an operator's keyboard would produce.

    stdout and stderr are merged into that terminal (one stream, like the
    runtime's merged log): the status lines, the exit line and a startup
    rejection all appear in the same sink. The slave is closed in the parent
    right after the spawn, so the master reports end-of-file the moment the
    application leaves; on Linux the master reports the slave's termios
    (`termios_attrs()`), so closing it loses nothing.

    Every wait is bounded, and the reader is a daemon thread like the runtime's
    log consumer: a scenario that never matched a line is failed by the
    harness instead of stalling it.
    """

    def __init__(self, binary, work_dir, run_index):
        self.binary = Path(binary)
        self.work_dir = Path(work_dir)
        self.run_index = run_index
        self.log_path = self.work_dir / ("application_run%d.log" % run_index)
        self.process = None
        self.stream = None
        self.lines = []
        self.stamps = []
        self.drainer = None
        self.arguments = []
        self.started_us = 0
        # Index the next wait starts scanning at: the terminal keeps the whole
        # session, and a pattern the application printed before the keys of this
        # step is not evidence that it reacted to them.
        self.watermark = 0
        self.initial_attributes = None
        # The terminal the application gave back, read the moment it left. It
        # is only meaningful on an exit the application took itself; a killed
        # one never runs its restore.
        self.final_attributes = None

    def start(self, arguments):
        self.arguments = list(arguments)
        self.lines = []
        self.stamps = []
        master, slave = pty.openpty()
        # The termios of a pty belongs to the terminal, not to a descriptor: the
        # snapshot taken here is exactly the state the application starts from
        # and has to give back.
        self.initial_attributes = termios.tcgetattr(slave)
        self.process = subprocess.Popen(
            [str(self.binary)] + self.arguments,
            stdin=slave,
            stdout=slave,
            stderr=slave,
            cwd=str(self.work_dir),
        )
        os.close(slave)
        self.stream = os.fdopen(master, "rb")
        self.started_us = now_us()
        # The terminal buffer is small: an unread master would stall the
        # application instead of the test, so the reader runs from the first
        # instant.
        self.drain()
        return self

    def drain(self):
        if self.drainer is None and self.process is not None:
            self.drainer = StreamDrain(self.stream, self.lines, self.stamps)
            self.drainer.start()

    def finish(self):
        """Waits for the reader to see the end of the terminal."""
        if self.drainer is not None:
            self.drainer.join(2.0)

    def text(self):
        return b"".join(self.lines).decode("utf-8", "replace")

    def line_stamps(self):
        """One `(arrival_us, line)` per line the application printed."""
        return [(stamp, line.decode("utf-8", "replace").rstrip("\r\n"))
                for stamp, line in zip(self.stamps, self.lines)]

    def wait_for(self, pattern, timeout_s, start=None):
        """Waits for a printed line while draining, and returns the matching one.

        `start` is the line index to begin scanning at; the default is the
        position of the last key this harness wrote, so a scenario waits for a
        line the application produced *after* its input instead of accepting one
        that was already on the terminal.
        """
        self.drain()
        deadline = time.monotonic() + timeout_s
        expression = re.compile(pattern)
        seen = self.watermark if start is None else start
        while True:
            while seen < len(self.lines):
                line = self.lines[seen].decode("utf-8", "replace")
                seen += 1
                if expression.search(line):
                    return line
            if self.process.poll() is not None:
                self.finish()
                while seen < len(self.lines):
                    line = self.lines[seen].decode("utf-8", "replace")
                    seen += 1
                    if expression.search(line):
                        return line
                raise Failure("the application exited with %s before matching %r"
                              % (self.process.returncode, pattern))
            if time.monotonic() >= deadline:
                raise Failure("the application did not match %r within %.1f s"
                              % (pattern, timeout_s))
            time.sleep(0.02)

    def signal(self, name):
        if self.process is None or self.process.poll() is not None:
            raise Failure("the application is not running, cannot send %s" % name)
        self.process.send_signal(getattr(signal, name))

    def send_keys(self, keys):
        """Writes the bytes of `keys` to the terminal, exactly as a keyboard."""
        if self.process is None or self.process.poll() is not None:
            raise Failure("the application is not running, keys %r were not sent"
                          % keys)
        if isinstance(keys, str):
            keys = keys.encode("utf-8")
        os.write(self.stream.fileno(), keys)
        # Everything printed before this instant is the state the keys are about
        # to change, not the reaction to them.
        self.watermark = len(self.lines)

    def termios_attrs(self):
        """The attributes of the terminal the application was given, or None.

        On Linux the master reports the slave's termios (one terminal, one set
        of attributes), so the snapshot is taken from the master while the
        application runs and compared against the one taken before the spawn.
        """
        if self.stream is None:
            return None
        return termios.tcgetattr(self.stream.fileno())

    def wait_exit(self, timeout_s):
        self.drain()
        try:
            code = self.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return None
        # The application restores the terminal before it returns from main, so
        # the attributes read here are the ones it handed back.
        try:
            self.final_attributes = self.termios_attrs()
        except (OSError, ValueError, termios.error):
            self.final_attributes = None
        self.finish()
        return code

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def exit_line(self):
        """The one `application: exit ...` line, or None when it printed none."""
        found = re.search(r"^application: exit [^\r\n]*", self.text(),
                          re.MULTILINE)
        return found.group(0) if found is not None else None

    def kill(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            try:
                self.process.kill()
            except OSError:
                pass
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        self.finish()
        if self.stream is not None:
            try:
                self.stream.close()
            except OSError:
                pass
            self.stream = None

    def write_log(self):
        # Drain first for the same reason the runtime log does: a scenario that
        # never waited on a line would otherwise leave its evidence empty.
        self.drain()
        if self.process is not None and self.process.poll() is None:
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        self.finish()
        self.log_path.write_text(self.text())


class Client:
    """One runtime_client process with an unbounded JSON-lines reader."""

    def __init__(self, binary, name, work_dir):
        self.binary = Path(binary)
        self.name = name
        self.work_dir = Path(work_dir)
        self.stdout_path = self.work_dir / ("client_%s.jsonl" % name)
        self.stderr_path = self.work_dir / ("client_%s.err" % name)
        self.process = None
        self.out_lines = []
        self.err_lines = []
        self.out_drain = None
        self.err_drain = None
        self.command = []
        self.started_us = 0

    def start(self, arguments):
        self.command = list(arguments)
        self.process = subprocess.Popen(
            [str(self.binary)] + self.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=str(self.work_dir),
        )
        self.started_us = now_us()
        self.out_drain = StreamDrain(self.process.stdout, self.out_lines)
        self.err_drain = StreamDrain(self.process.stderr, self.err_lines)
        self.out_drain.start()
        self.err_drain.start()
        return self

    def signal(self, name):
        if self.process is None or self.process.poll() is not None:
            raise Failure("client %s is not running, cannot send %s"
                          % (self.name, name))
        self.process.send_signal(getattr(signal, name))
        return now_us()

    def wait_exit(self, timeout_s):
        if self.process.poll() is None:
            try:
                return self.process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                return None
        return self.process.returncode

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def kill(self):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            self.process.kill()
        except OSError:
            pass
        self.process.wait(timeout=10)

    def records(self):
        """Every JSON line the client printed, in arrival order."""
        parsed = []
        for line in list(self.out_lines):
            text = line.decode("utf-8", "replace").strip()
            if not text:
                continue
            try:
                parsed.append(json.loads(text))
            except json.JSONDecodeError:
                continue
        return parsed

    def summary(self):
        for record in reversed(self.records()):
            if record.get("kind") == "client_summary":
                return record
        return None

    def stderr_text(self):
        return b"".join(self.err_lines).decode("utf-8", "replace")

    def write_logs(self):
        self.stdout_path.write_bytes(b"".join(self.out_lines))
        self.stderr_path.write_bytes(b"".join(self.err_lines))

    def check(self, name):
        summary = self.summary()
        if summary is None:
            raise Failure("client %s printed no summary" % self.name)
        return bool(summary.get("checks", {}).get(name, False))

    def measurement(self, name, default=None):
        summary = self.summary()
        if summary is None:
            return default
        return summary.get("measurements", {}).get(name, default)

    def states(self):
        return [r for r in self.records() if r.get("kind") == "state"]

    def results(self):
        return [r for r in self.records()
                if r.get("kind") in ("result", "result_suppressed")]


class Context:
    """Everything a step or a check may look at, and the evidence collected."""

    def __init__(self, scenario, source, work_dir):
        self.scenario = scenario
        self.source = Path(source)
        self.work_dir = Path(work_dir)
        self.processes = Processes()
        self.domain = 0
        self.steps = []
        self.signals = []
        self.runs = []
        self.applications = []
        self.traces = []
        self.domain_lock = None
        self.measurements = []
        self.report = []

    def client(self, name):
        client = self.processes.clients.get(name)
        if client is None:
            raise Failure("no client named %s was started" % name)
        return client

    def runtime(self):
        if self.processes.runtime is None:
            raise Failure("the runtime was never started")
        return self.processes.runtime

    def application(self):
        if self.processes.application is None:
            raise Failure("the application was never started")
        return self.processes.application

    def step_time(self, action):
        """When the last step using `action` started, or None when there is none."""
        for entry in reversed(self.steps):
            if entry["action"] == action:
                return entry["t_us"]
        return None

    def signal_time(self, client_name, signal_name):
        for entry in reversed(self.signals):
            if entry["client"] == client_name and entry["signal"] == signal_name:
                return entry["t_us"]
        raise Failure("no %s was sent to %s" % (signal_name, client_name))

    def write_evidence(self):
        self.processes.runtime and self.processes.runtime.write_log()
        for application in self.applications:
            application.write_log()
        for client in self.processes.clients.values():
            client.write_logs()
        (self.work_dir / "domain.txt").write_text(
            "%d\n" % self.domain, encoding="utf-8")


# ---------------------------------------------------------------------------
# Scenario setup
# ---------------------------------------------------------------------------

def set_dotted(mapping, dotted, value):
    parts = dotted.split(".")
    target = mapping
    for part in parts[:-1]:
        target = target.setdefault(part, {})
    target[parts[-1]] = value


def rtps_ports(domain):
    base = 7400 + 250 * domain
    return [base, base + 1, base + 10, base + 11]


def domain_is_free(domain):
    """True when every RTPS port of the domain can be bound right now."""
    sockets = []
    try:
        for port in rtps_ports(domain):
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.bind(("0.0.0.0", port))
            sockets.append(probe)
    except OSError:
        return False
    finally:
        for probe in sockets:
            probe.close()
    return True


def owner_is_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError as error:
        return error.errno == errno.EPERM  # alive, just not ours to signal


def release_domain(lock_path):
    if lock_path is not None:
        try:
            lock_path.unlink()
        except OSError:
            pass


def lock_owner(lock):
    """The pid recorded in a lock file, or None when it cannot be read."""
    try:
        return int(lock.read_text().strip())
    except (OSError, ValueError):
        return None


def lock_is_stale(lock):
    """Whether a lock file may be reclaimed.

    A live claimant creates the file and writes its pid in the next syscall, so
    an unreadable file is re-read for a moment and must additionally be older
    than a second before it counts as abandoned: otherwise a second claimant
    reading inside that window would mistake a live owner for a dead one and
    both would claim the same domain or evidence directory.
    """
    for _ in range(10):
        owner = lock_owner(lock)
        if owner is not None:
            return not owner_is_alive(owner)
        time.sleep(0.005)
    try:
        return time.time() - lock.stat().st_mtime > 1.0
    except OSError:
        return False


def choose_domain(declared, work_dir, lock_dir=None):
    """The declared domain, claimed so that no two scenarios can share one.

    Below 230 because the default RTPS port mapping overflows above it. Free
    means every RTPS port of the domain can be bound right now, and it is
    claimed with an exclusive lock file next to the work directories, so two
    scenario entries running at the same time (ctest -j) never select the same
    domain even when the declared one is already taken. The lock records the
    process id, so a lock left behind by a killed run does not block a domain
    for ever.

    `lock_dir` is the directory the exclusive lock is claimed in, and it
    defaults to the parent of `work_dir` — one directory per entry, which is
    what makes concurrent entries exclude each other. A caller whose own work
    directory is deeper than one level must pass the shared parent explicitly,
    or its locks would live in a namespace no other entry looks at.
    """
    shared = (Path(lock_dir).resolve() if lock_dir is not None
              else Path(work_dir).resolve().parent)
    for offset in range(0, 40):
        candidate = 190 + ((declared - 190 + offset) % 40)
        if candidate >= 230:
            continue
        if not domain_is_free(candidate):
            continue
        lock = shared / (".domain-%d.lock" % candidate)
        # A candidate whose lock belongs to a live scenario is taken; a lock left
        # behind by a killed run is reclaimed, so a dead run cannot consume a
        # domain for ever. Two reclaimers race on the same file: both may remove
        # it, and the exclusive create then decides which one owns the domain.
        for _ in range(3):
            try:
                handle = os.open(str(lock), os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                                 0o644)
            except FileExistsError:
                if not lock_is_stale(lock):
                    break  # a live scenario holds this domain
                try:
                    lock.unlink()
                except OSError:
                    pass
                continue
            os.write(handle, str(os.getpid()).encode())
            os.close(handle)
            (Path(work_dir) / "domain.txt").write_text("%d\n" % candidate,
                                                       encoding="utf-8")
            return candidate, lock
    raise Failure("no free DDS domain below 230 was found for %d" % declared)


def config_document(ctx, settings):
    """The configuration document of one process, before it is written out.

    Both processes of a scenario are handed this same document: the application
    drives the runtime through the generated `config.json`, so its
    configuration digest has to be the one the runtime published.
    """
    base = ctx.source / settings.get("config", "config/sdk.json")
    config = json.loads(base.read_text())
    for dotted, value in settings.get("config_overrides", {}).items():
        set_dotted(config, dotted, value)
    config["dds"]["domain"] = ctx.domain
    config["telemetry"]["directory"] = str(ctx.work_dir / "logs")
    return config


def build_runtime_arguments(ctx, run_index):
    settings = ctx.scenario.get("runtime", {})
    config = config_document(ctx, settings)
    config_path = ctx.work_dir / "config.json"
    config_path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

    arguments = ["--config", str(config_path), "--domain", str(ctx.domain)]
    fake = settings.get("fake_scenario")
    if fake:
        source_path = ctx.source / fake
        copied = ctx.work_dir / "fake_scenario.json"
        shutil.copyfile(source_path, copied)
        arguments += ["--fake-scenario", str(copied)]
    trace = settings.get("output_trace")
    if trace:
        name = trace % run_index if "%" in trace else trace
        arguments += ["--output-trace", str(ctx.work_dir / name)]
    return arguments


def build_application_arguments(ctx):
    """The arguments of the application of this scenario.

    `scenario["application"]` carries the same `config`, `config_overrides` and
    `domain` keys as `scenario["runtime"]`, and is resolved the same way - but
    the application has to run on the configuration the runtime was given, or
    the two would publish different digests and never bind a session. A
    scenario whose application section describes another configuration is
    refused here instead of failing later as an unexplained handshake.
    """
    runtime_settings = ctx.scenario.get("runtime", {})
    settings = ctx.scenario.get("application", {})
    expected = config_document(ctx, runtime_settings)
    declared = config_document(ctx, settings)
    if declared != expected:
        raise Failure(
            "the application section of %s describes a configuration the "
            "runtime does not use; one scenario gives both processes the same "
            "configuration (and therefore the same config_id)"
            % ctx.scenario["name"])
    declared_domain = settings.get("domain", ctx.scenario["domain"])
    if declared_domain != ctx.scenario["domain"]:
        raise Failure(
            "the application section declares domain %s but the scenario runs "
            "on %s; the two processes of one scenario share one domain"
            % (declared_domain, ctx.scenario["domain"]))
    config_path = ctx.work_dir / "config.json"
    if not config_path.exists():
        raise Failure("the application needs the configuration the runtime "
                      "generated; start the runtime first")
    # The invocation itself: `args` carries what the scenario wants the
    # application started with - `--locomotion-source external` being the case
    # that exists - and nothing the harness derives, so a scenario can never
    # contradict the configuration the runtime was given.
    extra = settings.get("args", [])
    if not isinstance(extra, list) or not all(
        isinstance(argument, str) for argument in extra
    ):
        raise Failure("the application args of %s must be a list of strings"
                      % ctx.scenario["name"])
    return ["--config", str(config_path), "--domain", str(ctx.domain)] + extra


def start_runtime(ctx):
    run_index = len(ctx.runs) + 1
    runtime = Runtime(ctx.runtime_binary, ctx.work_dir, run_index)
    arguments = build_runtime_arguments(ctx, run_index)
    runtime.start(arguments)
    ctx.processes.runtime = runtime
    ctx.runs.append(runtime)
    if "--output-trace" in arguments:
        ctx.traces.append(runtime.trace_path)
    return runtime


def start_application(ctx):
    """Starts the next application run of this scenario.

    One scenario runs one application at a time: a second one on the same
    runtime would be a second supervisor on the same session, which is not a
    state a scenario may create by accident.
    """
    if getattr(ctx, "application_binary", None) is None:
        raise Failure("this scenario starts the application, but no application "
                      "binary was given to the harness (--application)")
    running = ctx.processes.application
    if running is not None and running.alive():
        raise Failure("an application is already running, run %d; stop it "
                      "first" % running.run_index)
    arguments = build_application_arguments(ctx)
    application = Application(ctx.application_binary, ctx.work_dir,
                              len(ctx.applications) + 1)
    application.start(arguments)
    ctx.processes.application = application
    ctx.applications.append(application)
    return application


def client_arguments(ctx, step):
    arguments = [
        "--mode", step["mode"],
        "--scenario", step["scenario"],
        "--domain", str(ctx.domain),
        "--config", str(ctx.work_dir / "config.json"),
    ]
    arguments += [str(value) for value in step.get("args", [])]
    return arguments


# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------

def step_start_runtime(ctx, step):
    runtime = start_runtime(ctx)
    return "started %s" % " ".join(runtime.arguments)


def step_restart_runtime(ctx, step):
    previous = ctx.runtime()
    previous.drain()
    previous.signal("SIGTERM")
    code = previous.wait_exit(step.get("timeout_s", 10.0))
    if code is None:
        previous.kill()
        raise Failure("the runtime did not exit on SIGTERM; a restart cannot "
                      "reuse its domain")
    runtime = start_runtime(ctx)
    return "restarted the runtime (previous exit %s), run %d" % (
        code, len(ctx.runs))


def step_kill_runtime(ctx, step):
    runtime = ctx.runtime()
    runtime.drain()
    runtime.signal(step.get("signal", "SIGKILL"))
    code = runtime.wait_exit(step.get("timeout_s", 5.0))
    if code is None:
        runtime.kill()
        raise Failure("the runtime ignored %s" % step.get("signal", "SIGKILL"))
    return "runtime exited with %s" % code


def step_stop_runtime(ctx, step):
    runtime = ctx.runtime()
    # Orderly shutdown: the caller has already confirmed ZeroTorque, so the
    # runtime only has to leave. The pipe is normally drained first, so a full
    # pipe cannot delay the final report; a scenario that measures exactly that
    # (the bounded shutdown diagnostic) sets "drain": false and keeps the
    # consumer blocked until the process has left.
    drain_first = step.get("drain", True)
    if drain_first:
        runtime.drain()
    else:
        # The scenario asserts that a blocked consumer cannot delay the
        # shutdown, so it has to be blocked for real: the pipe must hold as much
        # as it can before the signal, otherwise the step would prove nothing.
        occupancy, capacity = runtime.pipe_occupancy()
        if capacity - occupancy > capacity // 8:
            raise Failure("the log pipe of the runtime is not under back "
                          "pressure (%d of %d bytes free), so the blocked "
                          "consumer is not being measured"
                          % (capacity - occupancy, capacity))
    runtime.signal(step.get("signal", "SIGTERM"))
    timeout_s = step.get("timeout_s", 10.0)
    code = runtime.wait_exit(timeout_s, drain=drain_first)
    if not drain_first:
        # The exit is what was measured; the log is read afterwards.
        runtime.drain()
    if not step.get("expect_exit", True):
        # A scenario may end in a state the shutdown policy deliberately refuses
        # to leave on its own (Damping with a revoked session). That the process
        # ignores the signal is then the assertion, and it is killed afterwards.
        if code is not None:
            raise Failure("the runtime left a state it must not leave without a "
                          "supervisor (exit %s)" % code)
        # The runtime renders the state quoted, in the status and shutdown
        # lines alike; the unquoted form exists only in the clean-shutdown line,
        # which a refused stop never prints.
        reported = re.findall(r'state="(\w+)"', runtime.text())
        state = reported[-1] if reported else "unknown"
        runtime.kill()
        return ("the runtime stayed in %s and was killed" % state)
    if code is None:
        runtime.kill()
        raise Failure("the runtime did not exit within %.1f s of %s"
                      % (timeout_s, step.get("signal", "SIGTERM")))
    expected_code = step.get("expect_code")
    if expected_code is not None and code != expected_code:
        raise Failure("the runtime exited with %s instead of %s"
                      % (code, expected_code))
    clean = runtime.clean_stop()
    if step.get("expect_clean", True):
        if code != 0 or clean is None:
            raise Failure("the runtime did not stop cleanly: exit %s, clean "
                          "line %s" % (code, clean))
    return "runtime exit=%s clean=%s" % (code, clean)


def step_drain_runtime_logs(ctx, step):
    ctx.runtime().drain()
    time.sleep(step.get("ms", 100) / 1000.0)
    return "runtime log drained (%d lines so far)" % len(ctx.runtime().lines)


def step_start_application(ctx, step):
    application = start_application(ctx)
    return "started %s" % " ".join(application.arguments)


def step_stop_application(ctx, step):
    """Leaves the application through one of its own exit paths.

    `keys` (optional) is written to the terminal first - the ordered exit of the
    exit key - and `signal` (optional) is delivered after it; without either,
    the step only waits for an exit the scenario caused elsewhere. The exit line
    is required: a process that died without one did not take the bounded path
    this step claims, and `expect_code` pins the code when the scenario knows it.
    """
    application = ctx.application()
    keys = step.get("keys")
    if keys:
        application.send_keys(keys)
    signal_name = step.get("signal")
    if signal_name:
        application.signal(signal_name)
    timeout_s = step.get("timeout_s", 10.0)
    code = application.wait_exit(timeout_s)
    if code is None:
        application.kill()
        raise Failure("the application did not leave within %.1f s%s"
                      % (timeout_s,
                         " of %s" % signal_name if signal_name else ""))
    line = application.exit_line()
    if line is None:
        raise Failure("the application exited with %s without an exit line; "
                      "the bounded exit never ran" % code)
    expected = step.get("expect_code")
    if expected is not None and code != expected:
        raise Failure("the application exited with %s instead of %s (line %r)"
                      % (code, expected, line))
    return "application exit=%s line=%r" % (code, line)


def step_kill_application(ctx, step):
    application = ctx.application()
    name = step.get("signal", "SIGKILL")
    application.signal(name)
    code = application.wait_exit(step.get("timeout_s", 5.0))
    if code is None:
        application.kill()
        raise Failure("the application ignored %s" % name)
    return "application exited with %s after %s" % (code, name)


def step_restart_application(ctx, step):
    previous = ctx.application()
    keys = step.get("keys", "x")
    if keys:
        previous.send_keys(keys)
    code = previous.wait_exit(step.get("timeout_s", 10.0))
    if code is None:
        previous.kill()
        raise Failure("the application did not leave its ordered exit, so a "
                      "restart would leave one runtime two supervision sessions")
    expected = step.get("expect_code")
    if expected is not None and code != expected:
        raise Failure("the previous application exited with %s instead of %s "
                      "(line %r)" % (code, expected, previous.exit_line()))
    application = start_application(ctx)
    return ("restarted the application (previous exit %s, line %r), run %d"
            % (code, previous.exit_line(), len(ctx.applications)))


def step_wait_application_log(ctx, step):
    line = ctx.application().wait_for(step["pattern"], step.get("timeout_s", 10.0))
    return "matched %r" % line.strip()


def step_application_keys(ctx, step):
    keys = step["keys"]
    ctx.application().send_keys(keys)
    return "sent keys %r" % keys


def step_wait_log(ctx, step):
    runtime = ctx.runtime()
    count = step.get("count", 1)
    # `after_now` scans from the end of the log as it is when the step starts,
    # so the step proves lines this scenario caused instead of accepting one
    # that was already in the pipe.
    start = len(runtime.lines) if step.get("after_now") else 0
    line = runtime.wait_for(step["pattern"], step.get("timeout_s", 10.0),
                            minimum=count, start=start)
    return "matched %d line(s) matching %r, last %r" % (count, step["pattern"],
                                                        line.strip())


def step_client(ctx, step):
    name = step["name"]
    if name in ctx.processes.clients:
        raise Failure("client %s was already started" % name)
    client = Client(ctx.client_binary, name, ctx.work_dir)
    client.start(client_arguments(ctx, step))
    ctx.processes.clients[name] = client
    if step.get("background"):
        return "started %s in the background" % name
    return wait_for_client(ctx, client, step)


def wait_for_client(ctx, client, step):
    timeout_s = step.get("timeout_s", 30.0)
    code = client.wait_exit(timeout_s)
    if code is None:
        client.kill()
        raise Failure("client %s did not finish within %.1f s"
                      % (client.name, timeout_s))
    allowed = step.get("allow_exit", [0])
    if code not in allowed:
        raise Failure("client %s exited with %s (allowed %s); stderr: %s"
                      % (client.name, code, allowed,
                         client.stderr_text().strip()[-800:]))
    summary = client.summary()
    if code == 0 and summary is None:
        raise Failure("client %s exited 0 without a summary" % client.name)
    return "%s exit=%s checks=%s" % (
        client.name, code, None if summary is None
        else json.dumps(summary.get("checks", {})))


def step_wait_client(ctx, step):
    client = ctx.client(step["name"])
    return wait_for_client(ctx, client, step)


def step_signal_client(ctx, step):
    client = ctx.client(step["name"])
    name = step["signal"]
    if not client.alive():
        # Pausing a process that already ended would be a step that did not run;
        # stopping one that ended by itself is already satisfied.
        if name in ("SIGTERM", "SIGKILL"):
            return "%s had already exited with %s" % (client.name,
                                                      client.process.returncode)
        raise Failure("client %s had already exited, %s was not delivered"
                      % (client.name, name))
    sent_us = client.signal(name)
    ctx.signals.append({"client": client.name, "signal": name, "t_us": sent_us})
    return "%s received %s at %d" % (client.name, name, sent_us)


def step_sleep(ctx, step):
    milliseconds = step.get("ms", 100)
    time.sleep(milliseconds / 1000.0)
    return "slept %d ms" % milliseconds


STEP_HANDLERS = {
    "start_runtime": step_start_runtime,
    "restart_runtime": step_restart_runtime,
    "kill_runtime": step_kill_runtime,
    "stop_runtime": step_stop_runtime,
    "drain_runtime_logs": step_drain_runtime_logs,
    "wait_log": step_wait_log,
    "start_application": step_start_application,
    "stop_application": step_stop_application,
    "kill_application": step_kill_application,
    "restart_application": step_restart_application,
    "wait_application_log": step_wait_application_log,
    "application_keys": step_application_keys,
    "client": step_client,
    "wait_client": step_wait_client,
    "signal_client": step_signal_client,
    "sleep_ms": step_sleep,
}


def run_steps(ctx):
    for index, step in enumerate(ctx.scenario["steps"], start=1):
        action = step.get("action")
        handler = STEP_HANDLERS.get(action)
        if handler is None:
            raise Failure("step %d uses the unknown action %r" % (index, action))
        started = now_us()
        try:
            detail = handler(ctx, step)
        except Failure as failure:
            ctx.steps.append({"index": index, "action": action, "ok": False,
                              "detail": str(failure), "t_us": started})
            ctx.report.append("    step %2d %-20s FAILED: %s"
                              % (index, action, failure))
            raise Failure("step %d (%s) failed: %s" % (index, action, failure))
        ctx.steps.append({"index": index, "action": action, "ok": True,
                          "detail": detail, "t_us": started})
        line = "    step %2d %-20s %s" % (index, action, detail)
        ctx.report.append(line)
        print(line)


# ---------------------------------------------------------------------------
# Traces
# ---------------------------------------------------------------------------

def load_trace(ctx, name):
    """The trace of one run, reduced to its effective records.

    The trace contract documents that a refused write makes one tick carry two
    records and that the last of them is the write that took effect. The
    consumer implements exactly that rule: records of one tick arrive adjacent
    (a single writer renders them in drain order), the last one replaces the
    earlier ones, and the number of ticks that carried more than one write is
    reported as evidence instead of being mistaken for a gap.
    """
    path = ctx.work_dir / name
    if not path.exists():
        raise Failure("the output trace %s does not exist" % name)
    records = []
    ticks = []
    duplicates = 0
    end = None
    lines = 0
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        lines += 1
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            raise Failure("the output trace %s has an unparseable line" % name)
        if record.get("kind") == "tick":
            records.append(record)
            if ticks and ticks[-1]["tick"] == record["tick"]:
                ticks[-1] = record
                duplicates += 1
            else:
                ticks.append(record)
        elif record.get("kind") == "end":
            end = record
    if not ticks:
        raise Failure("the output trace %s has no tick record" % name)
    return {"path": path, "ticks": ticks, "records": records,
            "duplicates": duplicates, "end": end, "lines": lines}


def tracked_ticks(trace):
    return [t for t in trace["ticks"]
            if t["action"] == "Tracking" and t["target_present"]]


def clamp(value, limit):
    return max(-limit, min(limit, value))


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_trace_complete(ctx, params):
    trace = load_trace(ctx, params["file"])
    ticks = trace["ticks"]
    numbers = [t["tick"] for t in ticks]
    contiguous = numbers == list(range(numbers[0], numbers[0] + len(numbers)))
    end = trace["end"]
    duplicates = trace["duplicates"]
    records = trace["records"]
    minimum = params.get("min_ticks", 1)
    problems = []
    if not contiguous:
        gaps = [b - a for a, b in zip(numbers, numbers[1:]) if b != a + 1]
        problems.append("%d tick gaps in the sequence (e.g. %s)"
                        % (len(gaps), gaps[:3]))
    if end is None:
        if not params.get("allow_missing_end"):
            problems.append(
                "the trace has no end record: the writer never finished")
    elif end.get("dropped") or end.get("write_failures"):
        problems.append("the writer dropped %s records and failed %s writes"
                        % (end.get("dropped"), end.get("write_failures")))
    if len(ticks) < minimum:
        problems.append("only %d ticks, expected at least %d"
                        % (len(ticks), minimum))
    # A scenario may restart the runtime: the counters of this trace belong to
    # the run that wrote it, not to whichever run happens to be last.
    run_index = None
    found = re.search(r"trace_run(\d+)\.jsonl$", str(trace["path"]))
    if found is not None:
        run_index = int(found.group(1))
    if run_index is not None and 1 <= run_index <= len(ctx.runs):
        clean = ctx.runs[run_index - 1].clean_stop()
    else:
        clean = ctx.runtime().clean_stop()
    if end is not None and clean is not None and "trace_written" in clean:
        # The runtime reports its counters after joining the writer; the end
        # record is written by that same join. Disagreeing numbers would mean
        # the report read a value the writer still owned.
        if (clean.get("trace_written") != end.get("recorded")
                or clean.get("trace_dropped") != end.get("dropped")
                or clean.get("trace_failures") != end.get("write_failures")):
            problems.append(
                "the shutdown line reports trace_written=%s dropped=%s "
                "failures=%s but the end record says recorded=%s dropped=%s "
                "write_failures=%s"
                % (clean.get("trace_written"), clean.get("trace_dropped"),
                   clean.get("trace_failures"), end.get("recorded"),
                   end.get("dropped"), end.get("write_failures")))
    detail = ("%d records, %d ticks %d..%d contiguous=%s, %d ticks carried a "
              "refused write and were reduced to their effective record, end=%s"
              % (len(records), len(ticks), numbers[0], numbers[-1], contiguous,
                 duplicates, end))
    return (not problems, detail + ("" if not problems else "; " + "; ".join(problems)),
            [("trace_ticks", len(ticks)),
             ("trace_duplicate_ticks", duplicates)])


def check_refused_write(ctx, params):
    """A refused write, the zero-torque fallback it forces and the fault latch.

    One tick writes once, or twice when a refused write is followed by the safe
    fallback; the consumer reduces such a tick to its last record. While the
    write path is down the fallback is refused as well, so the promise is the
    attempt: the second record of a refused tick commands zero torque.
    """
    trace = load_trace(ctx, params["file"])
    groups = []
    for record in trace["records"]:
        if groups and groups[-1][-1]["tick"] == record["tick"]:
            groups[-1].append(record)
        else:
            groups.append([record])
    refused_ticks = 0
    fallback_ticks = 0
    attempted_target = 0
    for group in groups:
        if len(group) > 2:
            raise Failure("tick %d carries %d records: a tick writes once, or "
                          "twice when a refusal is followed by the fallback"
                          % (group[0]["tick"], len(group)))
        if any(not record["write_accepted"] for record in group):
            refused_ticks += 1
        if len(group) == 2:
            last = group[-1]
            torque = last["torque"][:last["joint_count"]]
            if any(value != 0.0 for value in torque):
                raise Failure("the second record of tick %d is not the "
                              "zero-torque fallback: %s" % (last["tick"], torque))
            fallback_ticks += 1
        first = group[0]
        if any(abs(value) > 1e-12
               for value in first["torque"][:first["joint_count"]]):
            attempted_target += 1
    if trace["duplicates"] != fallback_ticks:
        raise Failure("the consumer reduced %d records but %d ticks carry two "
                      "records" % (trace["duplicates"], fallback_ticks))
    faults = [tick for tick in trace["ticks"]
              if tick["state"] == "Fault" and tick["fault_id"] != 0]
    minimum = params.get("min_refusals", 1)
    detail = ("%d tick(s) refused a write, %d of them carried the zero-torque "
              "fallback as their effective record, %d commanded a non-zero "
              "target first, %d tick(s) reported a latched fault id"
              % (refused_ticks, fallback_ticks, attempted_target, len(faults)))
    ok = (refused_ticks >= minimum and fallback_ticks >= 1
          and attempted_target >= 1 and bool(faults))
    return ok, detail, [("refused_write_ticks", refused_ticks),
                        ("refused_write_fallbacks", fallback_ticks),
                        ("fault_ticks", len(faults))]


def check_output_formula(ctx, params):
    """Every tracked tick recomputed from the frame it really held."""
    trace = load_trace(ctx, params["file"])
    limits = params["limits"]
    tolerance = params.get("tolerance", 1e-9)
    checked = 0
    mismatches = []
    for tick in tracked_ticks(trace):
        for joint in range(tick["joint_count"]):
            limit = limits[joint]
            raw = (tick["target_kp"][joint] * (tick["target_q"][joint]
                                               - tick["measured_q"][joint])
                   + tick["target_kd"][joint] * (tick["target_dq"][joint]
                                                 - tick["measured_dq"][joint])
                   + tick["target_tau_ff"][joint])
            expected = clamp(raw, limit)
            checked += 1
            if abs(expected - tick["torque"][joint]) > tolerance:
                mismatches.append((tick["tick"], joint, expected,
                                   tick["torque"][joint]))
    detail = ("%d joint values over %d tracked ticks recomputed exactly, "
              "%d mismatches%s"
              % (checked, len(tracked_ticks(trace)), len(mismatches),
                 "" if not mismatches else " e.g. %s" % (mismatches[:2],)))
    return (not mismatches, detail, [("formula_joint_values", checked)])


def check_command_recomputed(ctx, params):
    trace = load_trace(ctx, params["file"])
    groups = {}
    for tick in tracked_ticks(trace):
        key = (tick["epoch"], tick["target"]["command_seq"])
        groups.setdefault(key, []).append(tick)
    state_dependent = 0
    varying = 0
    stale = []
    for key, ticks in groups.items():
        # Only a frame whose gains read the measured state can require a
        # different torque for a different measurement.
        joint = next((index for index in range(ticks[0]["joint_count"])
                      if ticks[0]["target_kp"][index] != 0.0
                      or ticks[0]["target_kd"][index] != 0.0), None)
        if joint is None:
            continue
        state_dependent += 1
        # Only the measured quantities the frame really reads: a frame with kd=0
        # does not look at the measured velocity at all.
        def reading(tick, index=joint):
            values = []
            if tick["target_kp"][index] != 0.0:
                values.append(round(tick["measured_q"][index], 9))
            if tick["target_kd"][index] != 0.0:
                values.append(round(tick["measured_dq"][index], 9))
            return tuple(values)
        measurements = {reading(t) for t in ticks}
        if len(measurements) < 2:
            continue
        varying += 1
        torques = {round(t["torque"][joint], 9) for t in ticks}
        if len(torques) < 2:
            stale.append(key)
    minimum = params.get("min_command_seqs", 1)
    ok = varying >= minimum and not stale
    detail = ("%d of %d state-reading command sequences saw more than one "
              "measurement and recomputed their torque; %d kept the torque "
              "despite a different measurement%s"
              % (varying, state_dependent, len(stale),
                 "" if not stale else " %s" % stale[:3]))
    return ok, detail, [("command_seqs_recomputed", varying),
                        ("command_seqs_state_reading", state_dependent)]


def check_torque_clamp(ctx, params):
    trace = load_trace(ctx, params["file"])
    limit = params["limit"]
    bound = 0
    for tick in tracked_ticks(trace):
        for joint in range(tick["joint_count"]):
            raw = (tick["target_kp"][joint] * (tick["target_q"][joint]
                                               - tick["measured_q"][joint])
                   + tick["target_kd"][joint] * (tick["target_dq"][joint]
                                                 - tick["measured_dq"][joint])
                   + tick["target_tau_ff"][joint])
            if abs(raw) > limit and abs(abs(tick["torque"][joint]) - limit) < 1e-9:
                bound += 1
    minimum = params.get("min_values", 1)
    detail = "%d joint values were held at the %g Nm limit" % (bound, limit)
    return bound >= minimum, detail, [("clamped_joint_values", bound)]


def check_torque_step(ctx, params):
    trace = load_trace(ctx, params["file"])
    ticks = trace["ticks"]
    largest = 0.0
    where = None
    for previous, current in zip(ticks, ticks[1:]):
        if current["tick"] != previous["tick"] + 1:
            continue
        for joint in range(min(previous["joint_count"], current["joint_count"])):
            step = abs(current["torque"][joint] - previous["torque"][joint])
            if step > largest:
                largest = step
                where = (previous["tick"], joint,
                         previous["torque"][joint], current["torque"][joint])
    minimum = params.get("min_step_nm", 1.0)
    detail = ("largest torque change between neighbouring ticks is %.6g Nm at %s"
              % (largest, where))
    return largest >= minimum, detail, [("max_torque_step_nm", largest)]


def check_target_expiry(ctx, params):
    trace = load_trace(ctx, params["file"])
    command_us = params["command_us"]
    tracked = tracked_ticks(trace)
    if not tracked:
        raise Failure("no tracked tick holds a target")
    epochs = sorted({t["epoch"] for t in tracked})
    index = params.get("epoch_index", 0)
    if index >= len(epochs):
        raise Failure("the trace holds only %d tracking epochs" % len(epochs))
    epoch = epochs[index]
    ticks = trace["ticks"]
    last_index = max(position for position, tick in enumerate(ticks)
                     if tick["epoch"] == epoch and tick["target_present"])
    last = ticks[last_index]
    following = ticks[last_index + 1] if last_index + 1 < len(ticks) else None
    if following is None:
        raise Failure("the trace ends before the target expired")
    held_us = last["stamp_us"] - last["target"]["rx_us"]
    expired_us = following["stamp_us"] - last["target"]["rx_us"]
    tolerance_us = params.get("tolerance_us", 0)
    problems = []
    if held_us > command_us + tolerance_us:
        problems.append("the last tick that still held the target was already "
                        "%.0f us past the deadline" % held_us)
    if not (expired_us > command_us - tolerance_us):
        problems.append("the next tick was only %.0f us past the receive stamp, "
                        "so the target was not held to its deadline" % expired_us)
    if following["target_present"]:
        problems.append("the next tick still reported a target")
    if following["action"] == "Tracking":
        problems.append("the next tick was still tracking")
    detail = ("target received at %d, last tick with the target %d us later, "
              "next tick %d us later in state %s/%s"
              % (last["target"]["rx_us"], held_us, expired_us,
                 following["state"], following["action"]))
    return not problems, detail + ("" if not problems else "; " + "; ".join(problems)), [
        ("held_us", held_us), ("expired_us", expired_us)]


def check_deadlines(ctx, params):
    trace = load_trace(ctx, params["file"])
    ticks = trace["ticks"]
    largest = 0
    for previous, current in zip(ticks, ticks[1:]):
        if current["tick"] == previous["tick"] + 1:
            largest = max(largest, current["stamp_us"] - previous["stamp_us"])
    maximum = params.get("max_gap_us", 10000)
    span_us = ticks[-1]["stamp_us"] - ticks[0]["stamp_us"]
    rate = (len(ticks) - 1) * 1e6 / span_us if span_us > 0 else 0.0
    minimum_rate = params.get("min_rate_hz", 400.0)
    detail = ("largest tick gap %d us, %.1f Hz over %d ticks"
              % (largest, rate, len(ticks)))
    ok = largest <= maximum and rate >= minimum_rate
    return ok, detail, [("max_tick_gap_us", largest), ("tick_rate_hz", rate)]


def check_fallback(ctx, params):
    client = ctx.client(params["client"])
    base = params["base"]
    if base["kind"] == "signal":
        base_us = ctx.signal_time(base["client"], base["signal"])
    elif base["kind"] == "client_start":
        base_us = ctx.client(base["client"]).started_us
    else:
        base_us = 0
    states = [s for s in client.states()
              if s["t_us"] >= base_us and s["stop_reason"] == params["reason"]]
    if not states:
        raise Failure("no state with stop_reason %s was observed after %d"
                      % (params["reason"], base_us))
    first = states[0]
    latency_ms = (first["t_us"] - base_us) / 1000.0
    problems = []
    if first["execution_state"] != params["state"]:
        problems.append("the fallback state is %s" % first["execution_state"])
    if params.get("session_required"):
        if first["session_id"] == "0" * 32:
            problems.append("the fallback revoked the session")
    if params.get("session_revoked"):
        if first["session_id"] != "0" * 32:
            problems.append("the fallback kept the session %s" % first["session_id"])
    if latency_ms < params.get("min_ms", 0.0):
        problems.append("the fallback came after only %.1f ms" % latency_ms)
    if latency_ms > params.get("max_ms", 1e9):
        problems.append("the fallback came after %.1f ms" % latency_ms)
    detail = ("%s after %.1f ms (state %s, action %s, session %s, epoch %s)"
              % (params["reason"], latency_ms, first["execution_state"],
                 first["output_action"], first["session_id"][:8], first["epoch"]))
    return not problems, detail + ("" if not problems else "; " + "; ".join(problems)), [
        ("fallback_latency_ms", latency_ms)]


def check_tracking_frozen(ctx, params):
    client = ctx.client(params["client"])
    base = params["base"]
    if base["kind"] == "signal":
        base_us = ctx.signal_time(base["client"], base["signal"])
    elif base["kind"] == "after_reason":
        # The fallback needs a few control periods to be noticed: everything
        # published before the state that reports it may still be tracking.
        reported = [s for s in client.states()
                    if s["stop_reason"] == base["reason"]]
        if not reported:
            raise Failure("no state reported the stop reason %s" % base["reason"])
        base_us = reported[0]["t_us"]
    else:
        base_us = ctx.client(base["client"]).started_us
    states = [s for s in client.states() if s["t_us"] >= base_us]
    if len(states) < params.get("min_states", 5):
        raise Failure("only %d states were observed after the event"
                      % len(states))
    tracking = [s for s in states if s["execution_state"] == "Tracking"]
    sequences = {s["last_applied_command_seq"] for s in states}
    detail = ("%d states after the event, %d still tracking, "
              "last_applied_command_seq values %s"
              % (len(states), len(tracking), sorted(sequences)))
    ok = not tracking and len(sequences) <= params.get("max_sequences", 1)
    return ok, detail, [("states_after_event", len(states))]


def check_replay_rejected(ctx, params):
    client = ctx.client(params["client"])
    allowed = set(params["reasons"])
    answers = [r for r in client.records() if r.get("kind") == "replay_answer"]
    if not answers:
        raise Failure("the replay produced no answer at all")
    scope = params.get("scope", "session")
    problems = []
    scoped_applied = []
    bootstrap_applied = []
    unexpected = []
    for answer in answers:
        names_scope = (answer.get("sent_session_id", "0" * 32) != "0" * 32
                       or answer.get("sent_epoch", 0) != 0)
        if answer["status"] == "Applied":
            (scoped_applied if names_scope else bootstrap_applied).append(answer)
        elif answer["reason"] not in allowed:
            unexpected.append(answer)
    if scope == "session" and scoped_applied:
        problems.append("%d replayed packets that named a session or an epoch "
                        "were applied, which would re-authorize motion"
                        % len(scoped_applied))
    if scope == "all" and (scoped_applied or bootstrap_applied):
        problems.append("%d replayed packets were applied"
                        % (len(scoped_applied) + len(bootstrap_applied)))
    if unexpected:
        problems.append("unexpected rejection reasons %s"
                        % sorted({a["reason"] for a in unexpected}))
    counted = client.measurement("replay_session_scoped_applied", None)
    if counted:
        problems.append("the client counted %s applied session-scoped replays"
                        % counted)
    reasons = {}
    for answer in answers:
        reasons[answer["reason"]] = reasons.get(answer["reason"], 0) + 1
    detail = ("%d replayed requests answered, %d applied session-scoped, "
              "%d bootstrap packets answered with a session for the same "
              "supervisor (by design), reasons %s"
              % (len(answers), len(scoped_applied), len(bootstrap_applied),
                 reasons))
    return not problems, detail + ("" if not problems else "; " + "; ".join(problems)), [
        ("replayed_answers", len(answers)),
        ("replayed_session_scoped_applied", len(scoped_applied))]


def check_flood_drops(ctx, params):
    client = ctx.client(params["client"])
    sent = client.measurement("flood_sent", None)
    answered = client.measurement("flood_answered", None)
    if sent is None or answered is None:
        raise Failure("the flood client reported no counts")
    clean = ctx.runtime().clean_stop() or {}
    request_dropped = clean.get("request_dropped")
    transport_dropped = clean.get("transport_dropped")
    if request_dropped is None or transport_dropped is None:
        raise Failure("the runtime's clean-stop line did not report its inbound "
                      "and transport drop counters, so the flood cannot be "
                      "attributed")
    # The runtime must own the losses it caused: unanswered requests have to be
    # visible in its own counters, not only in the client's arithmetic.
    runtime_observed = request_dropped + transport_dropped
    detail = ("the flood sent %s requests and the runtime answered %s of them; "
              "the runtime's own counters report request_dropped=%s of its "
              "inbound queue plus transport_dropped=%s samples, so %s of the "
              "unanswered requests are not attributed"
              % (sent, answered, request_dropped, transport_dropped,
                 (sent - answered) - runtime_observed))
    return (answered < sent and runtime_observed >= 1,
            detail,
            [("flood_sent", sent), ("flood_answered", answered),
             ("runtime_request_dropped", request_dropped),
             ("runtime_transport_dropped", transport_dropped),
             ("flood_unattributed", (sent - answered) - runtime_observed)])


def check_client_check(ctx, params):
    client = ctx.client(params["client"])
    summary = client.summary()
    if summary is None:
        raise Failure("client %s printed no summary" % params["client"])
    checks = summary.get("checks", {})
    names = params["names"] if isinstance(params["names"], list) else [params["names"]]
    missing = [name for name in names if not checks.get(name, False)]
    detail = "%s: %s" % (params["client"], json.dumps(checks))
    return not missing, detail, []


def check_client_measurement(ctx, params):
    client = ctx.client(params["client"])
    value = client.measurement(params["name"], None)
    if value is None:
        raise Failure("client %s reported no measurement %s"
                      % (params["client"], params["name"]))
    problems = []
    if "min" in params and not value >= params["min"]:
        problems.append("%s < %s" % (value, params["min"]))
    if "max" in params and not value <= params["max"]:
        problems.append("%s > %s" % (value, params["max"]))
    detail = "%s = %s" % (params["name"], value)
    return not problems, detail + ("" if not problems else " (%s)"
                                   % "; ".join(problems)), [
        (params["name"], value)]


def check_confirmation_orders(ctx, params):
    client = ctx.client(params["client"])
    orders = [r.get("order") for r in client.records()
              if r.get("kind") == "confirmed"]
    required = set(params["orders"])
    detail = "%d confirmations, orders %s" % (len(orders), orders)
    return required.issubset(set(orders)), detail, [("confirmations", len(orders))]


def check_targets_served(ctx, params):
    trace = load_trace(ctx, params["file"])
    frames = {(t["epoch"], t["target"]["command_seq"])
              for t in tracked_ticks(trace)}
    minimum = params.get("min_command_seqs", 2)
    detail = "%d distinct target frames were commanded" % len(frames)
    return len(frames) >= minimum, detail, [("distinct_target_frames", len(frames))]


def check_replay_no_authorization(ctx, params):
    client = ctx.client(params["client"])
    tracking = [s for s in client.states() if s["execution_state"] == "Tracking"]
    sessions = {s["session_id"] for s in client.states()}
    detail = ("%d states during the replay, %d in Tracking, sessions %s"
              % (len(client.states()), len(tracking),
                 sorted(value[:8] for value in sessions)))
    ok = not tracking and client.measurement("replay_tracking_states", 0) == 0
    return ok, detail, [("replay_tracking_states", len(tracking))]


def check_instance_changed(ctx, params):
    record = ctx.client(params["record_client"])
    replay = ctx.client(params["replay_client"])

    def instances(client):
        return {s["runtime_id"] for s in client.states()}

    before = instances(record)
    after = instances(replay)
    if len(before) != 1 or len(after) != 1:
        raise Failure("expected one runtime instance per run, saw %s and %s"
                      % (sorted(before), sorted(after)))
    sent = {r["sent_runtime_id"] for r in replay.records()
            if r.get("kind") == "replay"
            and r.get("packet") == "request"
            and "sent_runtime_id" in r}
    old = next(iter(before))
    new = next(iter(after))
    detail = ("recorded instance %s, live instance %s%s"
              % (old[:12], new[:12],
                 "" if not sent else ", replayed packets named %s"
                 % sorted(value[:12] for value in sent)))
    return old != new and sent == {old}, detail, []


def check_logger_dropped(ctx, params):
    clean = ctx.runtime().clean_stop()
    if clean is None:
        raise Failure("the runtime printed no clean shutdown line")
    dropped = clean.get("dropped", 0)
    undrained = clean.get("undrained", 0)
    detail = ("the logger dropped %d events and left %d undrained while the "
              "consumer was blocked" % (dropped, undrained))
    return dropped >= params.get("min", 1), detail, [("log_dropped", dropped),
                                                     ("log_undrained", undrained)]


def check_overruns(ctx, params):
    clean = ctx.runtime().clean_stop()
    if clean is None:
        raise Failure("the runtime printed no clean shutdown line")
    worst = clean.get("worst_overrun_us", 0)
    overruns = clean.get("overruns", 0)
    detail = ("the control loop counted %d overruns, worst %d us, over %d ticks"
              % (overruns, worst, clean.get("ticks", 0)))
    ok = (worst <= params.get("max_worst_overrun_us", 5000)
          and overruns <= params.get("max_overruns", 10 ** 9))
    return ok, detail, [("overruns", overruns), ("worst_overrun_us", worst)]


def check_log_sequence(ctx, params):
    """An ordered list of lines the collected log really produced, and lines it must not.

    The check reads the evidence file a human reads (`application_run1.log`,
    `runtime_run1.log`, ...). `sequence` is an ordered subsequence: every
    pattern has to match a line that comes after the line of the pattern before
    it, so one repeated line cannot satisfy two entries.
    """
    path = ctx.work_dir / params["file"]
    if not path.exists():
        raise Failure("the log %s does not exist" % params["file"])
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    sequence = params.get("sequence", [])
    cursor = 0
    matched = []
    missing = None
    for pattern in sequence:
        found = None
        while cursor < len(lines):
            if pattern in lines[cursor]:
                found = cursor
                cursor += 1
                break
            cursor += 1
        if found is None:
            missing = pattern
            break
        matched.append(found)
    forbidden = []
    for pattern in params.get("forbid", []):
        for index, line in enumerate(lines):
            if pattern in line:
                forbidden.append((pattern, index, line.strip()[:160]))
                break
    problems = []
    if missing is not None:
        problems.append("no line matches %r after line %d" % (missing, cursor))
    if forbidden:
        problems.append("%s" % forbidden[:2])
    detail = ("%d line(s); %d of %d ordered pattern(s) matched%s%s"
              % (len(lines), len(matched), len(sequence),
                 "" if not matched else " at lines %s" % matched,
                 "" if not problems else "; " + "; ".join(problems)))
    return not problems, detail, [("log_lines", len(lines)),
                                  ("log_matched", len(matched))]


def check_application_reaction(ctx, params):
    """How long the application took to print a line after one of its own steps.

    `after_step` names the step whose start is the reference instant - the
    harness and the application read the same host monotonic clock - and
    `pattern` the line that reports the reaction. Only lines that arrived after
    that instant count, so the bound is the reaction to the event and not a line
    that was already in the terminal. `min_ms` is what makes an upper bound
    meaningful: an instant answer would be a line that did not wait for the
    deadline this check is about.
    """
    run = params.get("run", 1)
    if run < 1 or run > len(ctx.applications):
        raise Failure("the scenario started %d application run(s), not %d"
                      % (len(ctx.applications), run))
    application = ctx.applications[run - 1]
    base_us = ctx.step_time(params["after_step"])
    if base_us is None:
        raise Failure("the scenario has no %s step" % params["after_step"])
    expression = re.compile(params["pattern"])
    found = [(stamp, line) for stamp, line in application.line_stamps()
             if stamp >= base_us and expression.search(line)]
    if not found:
        raise Failure("the application printed no line matching %r after the "
                      "%s step" % (params["pattern"], params["after_step"]))
    latency_ms = (found[0][0] - base_us) / 1000.0
    problems = []
    if latency_ms < params.get("min_ms", 0.0):
        problems.append("it arrived after only %.1f ms" % latency_ms)
    if latency_ms > params.get("max_ms", 1e9):
        problems.append("it arrived after %.1f ms" % latency_ms)
    detail = ("%r arrived %.1f ms after %s: %r"
              % (params["pattern"], latency_ms, params["after_step"],
                 found[0][1].strip()[:160]))
    return not problems, detail + ("" if not problems else "; " + "; ".join(problems)), [
        ("application_latency_ms", latency_ms)]


def check_command_advancing(ctx, params):
    """The upper layer really kept publishing new targets while it held.

    The trace is the runtime's own per-tick record and
    `last_applied_command_seq` is the sequence of the newest frame the command
    gate applied. A hold that let one frame age out shows a plateau and a
    fallback; an advancing set of sequences over a window longer than every
    deadline the runtime has is the evidence that the upper process kept
    generating instead of relying on one frame holding for ever.
    """
    trace = load_trace(ctx, params["file"])
    tracked = tracked_ticks(trace)
    if not tracked:
        raise Failure("no tick of %s tracked a target" % params["file"])
    sequences = [tick["last_applied_command_seq"] for tick in tracked]
    distinct = sorted(set(sequences))
    backwards = [(before, after) for before, after in zip(sequences, sequences[1:])
                 if after < before]
    span_us = tracked[-1]["stamp_us"] - tracked[0]["stamp_us"]
    problems = []
    if len(distinct) < params.get("min_command_seqs", 2):
        problems.append("only %d distinct command sequence(s)" % len(distinct))
    if span_us < params.get("min_span_us", 0):
        problems.append("the tracked window was only %.0f us" % span_us)
    if backwards:
        problems.append("the applied sequence went backwards at %s"
                        % backwards[:2])
    detail = ("%d tracked tick(s) over %.0f us applied %d distinct command "
              "sequence(s) %d..%d, %d backwards step(s)"
              % (len(tracked), span_us, len(distinct), distinct[0],
                 distinct[-1], len(backwards)))
    return not problems, detail + ("" if not problems else "; " + "; ".join(problems)), [
        ("distinct_command_seqs", len(distinct)),
        ("tracked_window_us", span_us)]


def check_application_terminal(ctx, params):
    """The application handed the terminal back the way it found it."""
    run = params.get("run", 1)
    if run < 1 or run > len(ctx.applications):
        raise Failure("the scenario started %d application run(s), not %d"
                      % (len(ctx.applications), run))
    application = ctx.applications[run - 1]
    if application.final_attributes is None:
        raise Failure("the terminal of run %d was never read after the exit"
                      % run)
    restored = application.final_attributes
    initial = application.initial_attributes
    detail = ("the terminal attributes %s the ones the run started from"
              % ("equal" if restored == initial else
                 "differ from (%r != %r)" % (restored, initial)))
    return restored == initial, detail, []


CHECKS = {
    "trace_complete": check_trace_complete,
    "output_formula": check_output_formula,
    "refused_write": check_refused_write,
    "command_recomputed": check_command_recomputed,
    "torque_clamp": check_torque_clamp,
    "torque_step": check_torque_step,
    "target_expiry": check_target_expiry,
    "deadlines": check_deadlines,
    "fallback": check_fallback,
    "tracking_frozen": check_tracking_frozen,
    "replay_rejected": check_replay_rejected,
    "replay_no_authorization": check_replay_no_authorization,
    "instance_changed": check_instance_changed,
    "logger_dropped": check_logger_dropped,
    "overruns": check_overruns,
    "confirmation_orders": check_confirmation_orders,
    "flood_drops": check_flood_drops,
    "targets_served": check_targets_served,
    "client_check": check_client_check,
    "client_measurement": check_client_measurement,
    "log_sequence": check_log_sequence,
    "application_reaction": check_application_reaction,
    "command_advancing": check_command_advancing,
    "application_terminal": check_application_terminal,
}


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def claim_work_dir(work):
    """Exclusive claim of one scenario's evidence directory.

    Two invocations sharing a build directory would wipe each other's evidence;
    the claim turns that into a clear failure before anything is deleted. The
    file lives next to the scenario directory so it survives the wipe of a
    previous run.
    """
    claim = work.parent / (".claim-%s" % work.name)
    if claim.exists():
        if not lock_is_stale(claim):
            owner = lock_owner(claim)
            raise Failure("%s is in use by another run (pid %s): two concurrent "
                          "invocations must not share one work directory"
                          % (work, owner if owner is not None else "unknown"))
        try:
            claim.unlink()
        except OSError:
            pass
    handle = os.open(str(claim), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    os.write(handle, str(os.getpid()).encode())
    os.close(handle)
    return claim


def release_work_dir(claim):
    try:
        claim.unlink()
    except OSError:
        pass


def run_scenario(runtime_binary, client_binary, source, scenario_path, work_dir,
                 application_binary=None):
    scenario = json.loads(Path(scenario_path).read_text())
    name = scenario["name"]
    actions = {step.get("action") for step in scenario.get("steps", [])}
    if actions & {"start_application", "restart_application"} and not (
            application_binary and Path(application_binary).is_file()):
        # Refused before anything is claimed or wiped: a scenario that drives
        # the real application never silently runs without it.
        raise Failure("scenario %s starts the application, but no application "
                      "binary was given (--application)" % name)
    work = Path(work_dir) / name
    # The claim file sits next to the scenario directory, so the directory that
    # holds it must exist before anything is claimed or wiped.
    work.parent.mkdir(parents=True, exist_ok=True)
    claim = claim_work_dir(work)
    shutil.rmtree(work, ignore_errors=True)
    work.mkdir(parents=True, exist_ok=True)
    ctx = Context(scenario, source, work)
    ctx.runtime_binary = Path(runtime_binary).resolve()
    ctx.client_binary = Path(client_binary).resolve()
    ctx.application_binary = (Path(application_binary).resolve()
                              if application_binary else None)
    ctx.domain, ctx.domain_lock = choose_domain(scenario["domain"], work)
    print("  scenario %s: domain %d, %s"
          % (name, ctx.domain, scenario.get("description", "")))
    failures = []
    steps_done = False
    try:
        run_steps(ctx)
        steps_done = True
    except Failure as failure:
        failures.append(str(failure))
    finally:
        ctx.write_evidence()
        try:
            ctx.processes.shutdown()
        finally:
            ctx.write_evidence()
            release_domain(ctx.domain_lock)
            release_work_dir(claim)

    if steps_done:
        for check in scenario.get("checks", []):
            handler = CHECKS.get(check["check"])
            parameters = check.get("params", {})
            if handler is None:
                ok, detail, measurements = False, "unknown check", []
            else:
                try:
                    ok, detail, measurements = handler(ctx, parameters)
                except Failure as failure:
                    ok, detail, measurements = False, str(failure), []
                except Exception as error:  # noqa: BLE001 - reported, not hidden
                    ok, detail, measurements = False, "check error: %r" % error, []
            line = ("    check %-24s %s  %s"
                    % (check["check"], "PASS" if ok else "FAIL", detail))
            ctx.report.append(line)
            print(line)
            for key, value in measurements:
                ctx.measurements.append("%s.%s=%s" % (check["check"], key, value))
            if not ok:
                failures.append("check %s: %s" % (check["check"], detail))
    else:
        print("    %d check(s) not evaluated: a step did not complete"
              % len(scenario.get("checks", [])))

    summary = ["  scenario %s: domain %d, %s"
               % (name, ctx.domain, scenario.get("description", ""))
               ] + ctx.report + [
        "    domain=%d runtime_arguments=%s" % (
            ctx.domain, " ".join(ctx.runs[0].arguments) if ctx.runs else "-")]
    if ctx.applications:
        summary.append("    application_arguments=%s"
                       % " ".join(ctx.applications[0].arguments))
    if ctx.measurements:
        summary.append("    measurements: %s" % ", ".join(ctx.measurements))
    for failure in failures:
        summary.append("  failure: %s" % failure)
    summary.append("  scenario %s: %s" % (name, "FAILED" if failures else "PASSED"))
    (work / "report.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
    for line in summary[1:]:
        print(line)
    return failures


def scenario_files(source):
    directory = Path(source) / "tests" / "integration" / "scenarios"
    return sorted(directory.glob("*.json"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", help="one scenario JSON file")
    parser.add_argument("--all", action="store_true",
                        help="run every scenario of the source tree")
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--client", required=True)
    parser.add_argument("--application",
                        help="the real upper process; required by a scenario "
                             "that drives the application")
    parser.add_argument("--source", required=True)
    parser.add_argument("--work-dir", required=True)
    arguments = parser.parse_args()
    if not arguments.scenario and not arguments.all:
        parser.error("either --scenario or --all is required")

    if arguments.all:
        results = []
        for path in scenario_files(arguments.source):
            failures = run_scenario(arguments.runtime, arguments.client,
                                    arguments.source, path, arguments.work_dir,
                                    arguments.application)
            results.append((path.stem, failures))
            # Participants of the previous scenario need a moment to release
            # their ports before the next domain is claimed.
            time.sleep(0.5)
        print()
        for name, failures in results:
            print("%-20s %s" % (name, "FAILED" if failures else "PASSED"))
            for failure in failures:
                print("    %s" % failure)
        return 1 if any(failures for _, failures in results) else 0

    failures = run_scenario(arguments.runtime, arguments.client, arguments.source,
                            arguments.scenario, arguments.work_dir,
                            arguments.application)
    if failures:
        for failure in failures:
            print("  failure: %s" % failure)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
