#!/usr/bin/env python3
"""The state-switching stability run: the real application and the real runtime
cycling through their behaviours for a configured wall-clock time, with the
memory, thread and descriptor counts of both processes sampled as they go.

    application_stability_test.py <application> <robot_runtime> <source-root>
                                  [--duration-s SECONDS] [--sample-s SECONDS]

Each round walks the documented path - `d` (Damping), `i` (Initialize) to
`Ready`, `z` (ZeroTorque) back to `Standby` - and waits for the behaviour it
asked for before asking for the next, so a round that does not complete is a
failure and not a silent drift. The counts are read from `/proc`, which is a
statement about these two processes and never about a device: the deployment is
the shipped Fake configuration on the fake backend.

The default duration is the thirty minutes of the phase-3 acceptance run; the
CTest entry uses a short one, and the long run is the evidence.
"""

from __future__ import annotations

import argparse
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

KEYS = {
    "damping": b"d",
    "initialize": b"i",
    "locomotion": b"l",
    "zero_torque": b"z",
    "exit": b"x",
}

# The behaviours each key asks for, in the order a round walks them. The
# locomotion round is the same walk with a policy context in the middle, which
# is what a long run has to include: it is the only state where a source
# publishes a stream of its own, and the state the acceptance statistics are
# about.
ROUND = (
    ("damping", r"behavior=Damping runtime=Damping "),
    ("initialize", r"behavior=Ready runtime=Tracking "),
    ("zero_torque", r"behavior=Standby runtime=ZeroTorque "),
)
ROUND_LOCOMOTION = (
    ("damping", r"behavior=Damping runtime=Damping "),
    ("initialize", r"behavior=Ready runtime=Tracking "),
    ("locomotion", r"behavior=Locomotion runtime=Tracking "),
    ("damping", r"behavior=Damping runtime=Damping "),
    ("zero_torque", r"behavior=Standby runtime=ZeroTorque "),
)


def wait_for(path: Path, pattern: str, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    text = ""
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="replace")
            if re.search(pattern, text):
                return text
        time.sleep(0.02)
    return text


def sample_stats(pid: int) -> dict:
    """The kernel's own numbers for one process: RSS, threads and descriptors."""

    stats = {"rss_kb": 0, "threads": 0, "fds": 0}
    try:
        status = Path(f"/proc/{pid}/status").read_text(encoding="utf-8")
        for line in status.splitlines():
            if line.startswith("VmRSS:"):
                stats["rss_kb"] = int(line.split()[1])
            elif line.startswith("Threads:"):
                stats["threads"] = int(line.split()[1])
        stats["fds"] = len(list(Path(f"/proc/{pid}/fd").iterdir()))
    except (OSError, ValueError):
        pass
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application")
    parser.add_argument("robot_runtime")
    parser.add_argument("source_root", type=Path)
    parser.add_argument("--duration-s", type=float, default=1800.0)
    parser.add_argument("--sample-s", type=float, default=30.0)
    parser.add_argument("--domain", type=int, default=230)
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="the configuration both processes read (default: the shipped one); "
        "a diagnostic copy is how a timing window is made wide enough to "
        "reproduce in seconds",
    )
    parser.add_argument("--work-dir", type=Path, default=None)
    parser.add_argument(
        "--fake-scenario",
        type=Path,
        default=None,
        help="the scenario the runtime replays (default: the shipped one); a "
        "generated deployment carries its own beside its configuration",
    )
    parser.add_argument(
        "--locomotion",
        action="store_true",
        help="walk a policy context in every round, which needs a configuration "
        "whose policy.model_path names a model this deployment can load",
    )
    parser.add_argument(
        "--trace",
        action="store_true",
        help="record the runtime's per-tick output trace in the work directory",
    )
    args = parser.parse_args()

    source_root = args.source_root.resolve()
    # The children run in their own working directory, so every binary and file
    # they are handed has to be an absolute path: a relative one would be
    # resolved against that directory and not against the caller's.
    application_binary = str(Path(args.application).resolve())
    config_path = (
        args.config.resolve()
        if args.config is not None
        else source_root / "config" / "sdk.json"
    )
    runtime_binary = str(Path(args.robot_runtime).resolve())
    work = args.work_dir
    if work is None:
        import tempfile

        work = Path(tempfile.mkdtemp(prefix="irmv-stability-"))
    work.mkdir(parents=True, exist_ok=True)
    application_log = work / "application.log"
    runtime_log = work / "runtime.log"
    failures: list[str] = []

    def fail(note: str) -> None:
        failures.append(note)
        print(f"application_stability_test: {note}", file=sys.stderr, flush=True)

    runtime_out = runtime_log.open("w", encoding="utf-8")
    # The resource samples as a file beside the logs, so the curves can be
    # recomputed from the run's own directory instead of only from the driver's
    # summary: one line per sample, named by the column it holds.
    resource_log = (work / "resources.tsv").open("w", encoding="utf-8")
    resource_log.write("sample\tprocess\trss_kb\tthreads\tfds\n")
    resource_log.flush()
    application_out = application_log.open("w", encoding="utf-8")
    runtime_arguments = [
        runtime_binary,
        "--config",
        str(config_path),
        "--domain",
        str(args.domain),
        "--fake-scenario",
        str(
            args.fake_scenario.resolve()
            if args.fake_scenario
            else source_root / "tests" / "support" / "fake_static.json"
        ),
    ]
    if args.trace:
        runtime_arguments += ["--output-trace", str(work / "trace_run1.jsonl")]
    runtime_process = subprocess.Popen(
        runtime_arguments,
        stdout=runtime_out,
        stderr=subprocess.STDOUT,
        cwd=work,
    )
    application_process = None
    cycles = 0
    resyncs = 0
    command_timeouts = 0
    source_timeouts = 0
    stats_application: list[dict] = []
    stats_runtime: list[dict] = []
    started = time.monotonic()
    try:
        text = wait_for(runtime_log, r'state="ZeroTorque"', 20)
        if 'state="ZeroTorque"' not in text:
            fail(f"the runtime never reached ZeroTorque: {text[-600:]}")
            return 1

        application_process = subprocess.Popen(
            [
                application_binary,
                "--config",
                str(config_path),
                "--domain",
                str(args.domain),
            ],
            stdin=subprocess.PIPE,
            stdout=application_out,
            stderr=subprocess.STDOUT,
            cwd=work,
        )

        def key(name: str) -> None:
            assert application_process is not None
            assert application_process.stdin is not None
            application_process.stdin.write(KEYS[name])
            application_process.stdin.flush()

        def occurrences(pattern: str) -> int:
            return len(re.findall(pattern, application_log.read_text(
                encoding="utf-8", errors="replace")))

        def expect(note: str, pattern: str, timeout_s: float = 25) -> None:
            text = wait_for(application_log, pattern, timeout_s)
            if not re.search(pattern, text):
                fail(f"{note}: {pattern!r} never appeared; tail: {text[-600:]}")

        def wait_one_more(pattern: str, before: int, timeout_s: float) -> bool:
            # Like expect_one_more, but a timeout is not a failure: the retry
            # loop is what decides whether a round really did not complete.
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if occurrences(pattern) > before:
                    return True
                time.sleep(0.02)
            return False

        def expect_one_more(note: str, pattern: str, before: int,
                            timeout_s: float = 20) -> bool:
            # A round has to produce a *transition*, not a line that was already
            # in the log from an earlier round: the count has to grow. Without
            # this the run would report hundreds of thousands of cycles that
            # never happened, because every pattern of a documented round is in
            # the log after the first one.
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if occurrences(pattern) > before:
                    return True
                time.sleep(0.02)
            fail(
                f"{note}: no new {pattern!r} within {timeout_s:.0f} s; "
                f"tail: {application_log.read_text(encoding='utf-8', errors='replace')[-600:]}"
            )
            return False

        expect("the handover", r"behavior=Standby runtime=ZeroTorque ")

        next_sample = started
        round_keys = ROUND_LOCOMOTION if args.locomotion else ROUND
        while time.monotonic() - started < args.duration_s and not failures:
            for name, pattern in round_keys:
                # An operator who sees no transition asks again, and that is what
                # this does: a key that produced nothing (a stalled request, a
                # request the runtime refused, a transition that raced the
                # runtime's own deadline) is repeated instead of ending the run.
                # The keys are idempotent by their own rules - a repeated stop is
                # already on the wire, a repeated Initialize is a request the
                # client already holds - so asking again costs nothing and is
                # how a real operator recovers a round.
                before = occurrences(pattern)
                key(name)
                if not wait_one_more(pattern, before, 10):
                    # The transition did not happen: ask again, the way an
                    # operator does. The keys are idempotent by their own rules
                    # (a repeated stop is already on the wire, a repeated
                    # Initialize is a request the client already holds), and a
                    # round that a runtime deadline interrupted is recovered by
                    # repeating the key of the state the round wanted.
                    for attempt in range(2):
                        resyncs += 1
                        before = occurrences(pattern)
                        key(name)
                        if wait_one_more(pattern, before, 10):
                            break
                    else:
                        fail(
                            f"the {name} of cycle {cycles + 1}: no new "
                            f"{pattern!r} after three keys; tail: "
                            + application_log.read_text(
                                encoding="utf-8", errors="replace")[-600:]
                        )
            cycles += 1
            # One line per round, so an acceptance record carries the state the
            # round reached and when, not only the totals at the end.
            print(
                f"application_stability_test: round={cycles} "
                f"t={time.monotonic() - started:.1f}s "
                f"state={pattern.split()[0]} key={name}",
                flush=True,
            )
            now = time.monotonic()
            if now >= next_sample:
                stats_application.append(sample_stats(application_process.pid))
                stats_runtime.append(sample_stats(runtime_process.pid))
                for label, stats in (("application", stats_application[-1]),
                                     ("runtime", stats_runtime[-1])):
                    resource_log.write(
                        f"{len(stats_application)}\t{label}\t"
                        f"{stats['rss_kb']}\t{stats['threads']}\t"
                        f"{stats['fds']}\n")
                resource_log.flush()
                next_sample = now + args.sample_s

        stats_application.append(sample_stats(application_process.pid))
        stats_runtime.append(sample_stats(runtime_process.pid))
        for label, stats in (("application", stats_application[-1]),
                             ("runtime", stats_runtime[-1])):
            resource_log.write(
                f"{len(stats_application)}\t{label}\t"
                f"{stats['rss_kb']}\t{stats['threads']}\t{stats['fds']}\n")
        resource_log.flush()

        text = application_log.read_text(encoding="utf-8", errors="replace")
        for pattern in (
            r"behavior=Fault",
            r"cause=LocalPolicyFailed",
        ):
            if re.search(pattern, text):
                fail(f"the run reported {pattern}")
        # Every supervision timeout the runtime logged, with the sequence it
        # last accepted and the age it expired at: that pair is the measurement
        # the acceptance is read from when a run ends on the lease.
        runtime_text = (work / "runtime.log").read_text(
            encoding="utf-8", errors="replace")
        for line in runtime_text.splitlines():
            if "SupervisorTimeout" in line:
                print(f"application_stability_test: {line.strip()}", flush=True)

        # The runtime's own target deadlines are counted, not failed on: a
        # `CommandTimeout` is the runtime's legal fallback when the upper layer
        # stops publishing its target - which is exactly what a pending stop
        # does by design - and the run reports how often the two raced so a
        # reader can tell a healthy cycle from a racy one.
        command_timeouts = len(re.findall(r"stop=CommandTimeout", text))
        source_timeouts = len(re.findall(r"stop=SourceStateTimeout", text))

        key("exit")
        expect("the ordered exit", r"application: exit stop=confirmed", 25)
        code = application_process.wait(timeout=15)
        if code != 0:
            fail(f"the application exited with {code}")
    finally:
        if application_process is not None and application_process.poll() is None:
            application_process.send_signal(signal.SIGTERM)
            try:
                application_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                application_process.kill()
                application_process.wait(timeout=10)
        if runtime_process.poll() is None:
            runtime_process.send_signal(signal.SIGTERM)
            try:
                runtime_process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                runtime_process.kill()
                runtime_process.wait(timeout=10)
        resource_log.close()
        runtime_out.close()
        application_out.close()

    def summarise(label: str, samples: list[dict]) -> str:
        rss = [sample["rss_kb"] for sample in samples]
        threads = [sample["threads"] for sample in samples]
        fds = [sample["fds"] for sample in samples]
        return (
            f"{label}: rss_kb first={rss[0]} max={max(rss)} last={rss[-1]}; "
            f"threads first={threads[0]} max={max(threads)} last={threads[-1]}; "
            f"fds first={fds[0]} max={max(fds)} last={fds[-1]}"
        )

    elapsed = time.monotonic() - started
    print(
        f"application_stability_test: {cycles} cycle(s) in {elapsed:.1f} s, "
        f"{resyncs} re-asked key(s)"
    )
    print(
        f"application_stability_test: runtime-deadline stops: "
        f"command={command_timeouts} source_state={source_timeouts}"
    )
    print(summarise("application", stats_application))
    print(summarise("runtime", stats_runtime))
    if failures:
        return 1
    states = (
        "Standby <-> Damping <-> Ready <-> Locomotion"
        if args.locomotion
        else "Standby <-> Damping <-> Ready"
    )
    print(
        f"application_stability_test: {states} for "
        f"{cycles} round(s), no fault, memory/thread/descriptor counts above"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
