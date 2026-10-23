#!/usr/bin/env python3
"""tests/integration/application_record_test.py <application> <robot_runtime> <source-root>
                                                    [--keep-work DIR]

The application-driven recorded run (§11.1, Task 9): a real runtime records a
session the real application supervised, and the run is read back as the
evidence of what the *machine* did.

    Standby -> Damping -> Initializing -> Ready -> (hold) -> Damping -> exit

The recorder is enabled in a copy of the shipped configuration, so both
processes keep the same configuration digest and the recorded `meta.json` names
the deployment that ran. What the run has to show, and what this test asserts,
is the coherence the recorder exists for: every row of the authorized window
carries the command that was in force in the same tick - its identity, its
epoch and its sequence - alongside the state the runtime published for it, the
feedback it used and the output it produced. The authorization context is also
in the run's own `sessions.json`, so a consumer can join a row's epoch to the
context it was authorized under.

Nothing here is a machine limit: the deployment is the shipped Fake one, whose
amplitudes and gains are demonstration values.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

DOMAIN = 151
RECORD_HOLD_S = 2.0
KEYS = {"damping": b"d", "initialize": b"i", "exit": b"x"}


def wait_for(path: Path, pattern: str, timeout_s: float) -> str:
    deadline = time.monotonic() + timeout_s
    text = ""
    while time.monotonic() < deadline:
        if path.exists():
            text = path.read_text(encoding="utf-8", errors="replace")
            if re.search(pattern, text):
                return text
        time.sleep(0.05)
    return text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application")
    parser.add_argument("robot_runtime")
    parser.add_argument("source_root", type=Path)
    parser.add_argument("--keep-work", type=Path, default=None)
    args = parser.parse_args()

    source_root = args.source_root.resolve()
    application_binary = str(Path(args.application).resolve())
    runtime_binary = str(Path(args.robot_runtime).resolve())
    read_record = Path(runtime_binary).parent / "read_record"
    if not read_record.exists():
        print(f"application_record_test: no read_record next to {runtime_binary}")
        return 1

    failures: list[str] = []

    def fail(note: str) -> None:
        failures.append(note)
        print(f"application_record_test: {note}", file=sys.stderr, flush=True)

    work = (
        args.keep_work
        if args.keep_work is not None
        else Path(tempfile.mkdtemp(prefix="irmv-record-"))
    )
    work.mkdir(parents=True, exist_ok=True)
    try:
        return run(source_root, application_binary, runtime_binary, read_record,
                   work, failures, fail)
    finally:
        if args.keep_work is None:
            shutil.rmtree(work, ignore_errors=True)


def run(source_root, application_binary, runtime_binary, read_record, work,
        failures, fail) -> int:
    # The recorder's own directory, and a configuration that turns it on. It is
    # a copy of the shipped document with one section changed, so the two
    # processes of this run still agree on every other parameter - and on the
    # digest they publish.
    runs_directory = work / "runs"
    runs_directory.mkdir(parents=True, exist_ok=True)
    document = json.loads((source_root / "config" / "sdk.json").read_text())
    document["telemetry"]["recorder"] = {
        "enabled": True,
        "sample_hz": 100,
        "queue_capacity": 256,
        "directory": str(runs_directory),
    }
    config_path = work / "sdk.json"
    config_path.write_text(json.dumps(document, indent=2) + "\n")

    application_log = work / "application.log"
    runtime_log = work / "runtime.log"
    application_out = application_log.open("w", encoding="utf-8")
    runtime_out = runtime_log.open("w", encoding="utf-8")
    runtime_process = subprocess.Popen(
        [
            runtime_binary,
            "--config",
            str(config_path),
            "--domain",
            str(DOMAIN),
            "--fake-scenario",
            str(source_root / "tests" / "support" / "fake_imu.json"),
        ],
        stdout=runtime_out,
        stderr=subprocess.STDOUT,
        cwd=work,
    )
    application_process = None
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
                str(DOMAIN),
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

        def expect(note: str, pattern: str, timeout_s: float = 25) -> str:
            text = application_log.read_text(encoding="utf-8", errors="replace")
            if not re.search(pattern, text):
                text = wait_for(application_log, pattern, timeout_s)
            if not re.search(pattern, text):
                fail(f"{note}: {pattern!r} never appeared; tail: {text[-600:]}")
            return text

        expect("the handover", r"behavior=Standby runtime=ZeroTorque ")
        key("damping")
        expect("the damping stop", r"behavior=Damping runtime=Damping ")
        key("initialize")
        expect("the initialization", r"behavior=Ready runtime=Tracking ")
        # Hold the authorized window long enough to be more than a handful of
        # recorded rows.
        time.sleep(RECORD_HOLD_S)
        key("damping")
        expect("the damping stop", r"behavior=Damping runtime=Damping ")
        key("exit")
        expect("the ordered exit", r"application: exit stop=confirmed", 25)
        application_process.wait(timeout=15)
        if application_process.returncode != 0:
            fail(f"the application exited with {application_process.returncode}")
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
        application_out.close()
        runtime_out.close()

    if failures:
        return 1

    # Both recorders write into the configured base directory. The directory is
    # identified by the schema its own metadata declares, never by a name
    # prefix: the runtime's `run_*` and the application's `app_run_*` are two
    # independent streams, and a test must not assume how many of either exists
    # before it has looked.
    runtime_runs = []
    application_runs = []
    for path in sorted(runs_directory.iterdir()):
        if not path.is_dir() or not (path / "meta.json").is_file():
            continue
        try:
            meta = json.loads((path / "meta.json").read_text())
        except (OSError, ValueError):
            continue
        if not isinstance(meta, dict):
            continue
        if meta.get("schema") == "irmv.application_velocity":
            application_runs.append((path, meta))
        elif "schema_version" in meta:
            runtime_runs.append((path, meta))
    if len(runtime_runs) != 1:
        fail(
            "the runtime recorder created "
            f"{len(runtime_runs)} run director(ies), expected one"
        )
        return 1
    if len(application_runs) != 1:
        fail(
            "the application recorder created "
            f"{len(application_runs)} run director(ies), expected one"
        )
        return 1
    run_directory, runtime_meta = runtime_runs[0]
    app_directory, app_meta = application_runs[0]

    summary = subprocess.run(
        [str(read_record), "--run", str(run_directory), "--summary-only"],
        capture_output=True, text=True, check=False,
    )
    if summary.returncode != 0:
        fail(f"read_record failed: {summary.stderr.strip()}")
        return 1
    summary_json = json.loads(summary.stdout.splitlines()[-1])
    if summary_json["schema_version"] != 4:
        fail(f"the run declares schema {summary_json['schema_version']}, expected 4")
    if summary_json["corrupt"]:
        fail(f"the intact run reports a corrupt chunk: {summary_json.get('error', '')}")
    if summary_json["rows_in_range"] <= 0:
        fail("the run recorded no rows")
    if summary_json["sample_hz"] != 100 or summary_json["control_hz"] != 500:
        fail(
            "the run declares another rate than the configured one: "
            f"{summary_json['sample_hz']} Hz on {summary_json['control_hz']} Hz"
        )

    # The rows themselves: the authorized window has to be coherent, and the
    # identity of the command in force has to be the one the application sent.
    rows = subprocess.run(
        [str(read_record), "--run", str(run_directory)],
        capture_output=True, text=True, check=False,
    )
    if rows.returncode != 0:
        fail(f"read_record (rows) failed: {rows.stderr.strip()}")
        return 1
    lines = [line for line in rows.stdout.splitlines() if line.strip()]
    records = [json.loads(line) for line in lines[:-1]]
    samples = [
        row for row in records if row.get("record_type", "sample") == "sample"
    ]
    fault_rows = [row for row in records if row.get("record_type") == "fault"]
    if len(samples) != summary_json["rows_in_range"]:
        fail(
            f"the reader printed {len(samples)} sample row(s) but its summary "
            f"reports {summary_json['rows_in_range']}"
        )
    # The fault stream is separate and empty for this healthy run; its
    # completion fact still has to be published, not inferred from the absence
    # of rows.
    fault_summary = summary_json.get("faults")
    if not isinstance(fault_summary, dict):
        fail("the runtime summary carries no fault statistics")
        fault_summary = {}
    if fault_summary.get("complete") is not True:
        fail(f"the fault stream is not complete: {fault_summary}")
    if fault_rows or int(fault_summary.get("rows_in_range", -1)) != 0:
        fail(
            "the healthy run recorded fault occurrences: "
            f"{fault_rows} / {fault_summary}"
        )
    tracking = [row for row in samples if row["execution_state"] == 3]
    if not tracking:
        fail("no row of the run shows the runtime tracking")
    else:
        producers = {row["producer_id"] for row in tracking}
        if len(producers) != 1 or "" in producers:
            fail(f"the tracking rows do not name one producer: {sorted(producers)}")
        without_command = [
            row for row in tracking if not row["command_present"]
        ]
        if without_command:
            fail(
                f"{len(without_command)} tracking row(s) carry no command in force"
            )
        epochs = {row["epoch"] for row in tracking}
        if 0 in epochs:
            fail("a tracking row carries no authorization epoch")
        sequences = [row["command_seq"] for row in tracking]
        if any(later < earlier for earlier, later in zip(sequences, sequences[1:])):
            fail("the recorded command sequence goes backwards")
        # The snapshot is one tick: the feedback and the output of a row belong
        # to the state that row names, and the output the runtime produced is
        # recorded next to the target it was asked for. The shipped Fake
        # scenario holds its measured position at the initialization target, so
        # the target itself is constant here by construction - what this run
        # proves is that the row carries both sides at all.
        for row in tracking:
            if row["feedback_stamp_us"] == 0 or row["feedback_seq"] == 0:
                fail("a tracking row carries no feedback snapshot")
                break
            if not row["feedback_valid"]:
                fail("a tracking row reports the feedback it used as invalid")
                break
            if not row["output_valid"]:
                fail("a tracking row carries no output of its own tick")
                break
            if not row["measured_q"] or not row["output_torque"]:
                fail("a tracking row carries no measured feedback or output")
                break
            if not row["send_accepted"]:
                fail("a tracking row reports its output as refused by the host")
                break
            # The feedback fixture is not the all-zero snapshot: the recorded
            # torque estimate and the per-joint validity are the fake backend's
            # own values, and the IMU snapshot the configuration collects is
            # marked valid with a real receive stamp.
            if any(
                abs(float(value) - expected) > 1e-9
                for value, expected in zip(
                    row["measured_tau_est"], (0.25, -0.15)
                )
            ):
                fail(
                    "a tracking row does not carry the fixture's tau_est: "
                    f"{row['measured_tau_est']}"
                )
                break
            if not all(bool(value) for value in row["joint_valid"]):
                fail(
                    "a tracking row marks a configured joint's feedback invalid"
                )
                break
            if not row["imu_valid"]:
                fail("a tracking row does not mark the fixture's IMU valid")
                break
            if int(row["imu_rx_stamp_us"]) <= 0:
                fail("a tracking row carries no IMU receive stamp")
                break
            expected_imu = {
                "imu_orientation_rpy": (0.1, -0.2, 0.3),
                "imu_angular_velocity": (0.4, -0.5, 0.6),
                "imu_linear_acceleration": (0.7, -0.8, 9.81),
            }
            for vector, expected in expected_imu.items():
                actual = row[vector]
                if len(actual) != 3 or any(
                    not math.isfinite(float(actual[index]))
                    or abs(float(actual[index]) - expected[index]) > 1e-12
                    for index in range(3)
                ):
                    fail(
                        f"a tracking row does not carry the fixture's {vector}: "
                        f"{actual}"
                    )
                    break

    # The run's own session table: the authorization context the rows belong to,
    # with the runtime's explicit `unknown` where the model digest would be.
    sessions_path = run_directory / "sessions.json"
    if not sessions_path.exists():
        fail("the run has no sessions.json")
    else:
        sessions = json.loads(sessions_path.read_text())
        if not isinstance(sessions, list) or not sessions:
            fail("sessions.json holds no session event")
        else:
            if not any(int(entry.get("epoch", 0)) != 0 for entry in sessions):
                fail("no session event names an authorization epoch")
            if not any(
                entry.get("model_sha256") == "unknown" for entry in sessions
            ):
                fail(
                    "the runtime invented a model digest instead of writing "
                    "`unknown`"
                )
            row_epochs = {row["epoch"] for row in tracking}
            session_epochs = {int(entry.get("epoch", 0)) for entry in sessions}
            if not row_epochs & session_epochs:
                fail(
                    "no session event names an epoch the rows were recorded "
                    f"under: rows={sorted(row_epochs)} sessions="
                    f"{sorted(session_epochs)}"
                )

    # The application's independent run: same boot and clock, same sampling
    # rate, the same configuration digests, and the model state the configured
    # local source actually had. A velocity stream from another run would be
    # worthless evidence, so every one of these is checked against both
    # processes' metadata rather than against the test's assumptions.
    if runtime_meta.get("schema_version") != 4:
        fail(f"the runtime meta declares schema {runtime_meta.get('schema_version')}")
    if app_meta.get("schema") != "irmv.application_velocity" or int(
        app_meta.get("version", 0)
    ) != 1:
        fail(
            "the application meta is not the velocity schema: "
            f"{app_meta.get('schema')} v{app_meta.get('version')}"
        )
    runtime_clock = runtime_meta.get("clock") or {}
    application_clock = app_meta.get("clock") or {}
    for label, clock in (
        ("runtime", runtime_clock),
        ("application", application_clock),
    ):
        if clock.get("clock") != "linux_clock_monotonic":
            fail(f"the {label} meta declares clock {clock.get('clock')!r}")
        if clock.get("available") is not True:
            fail(f"the {label} meta has no available clock identity")
        if not isinstance(clock.get("boot_id"), str) or not clock.get("boot_id"):
            fail(f"the {label} meta has no boot id")
    if runtime_clock.get("boot_id") != application_clock.get("boot_id"):
        fail(
            "the two runs name different boots: "
            f"{runtime_clock.get('boot_id')} != "
            f"{application_clock.get('boot_id')}"
        )
    runtime_sha = runtime_meta.get("sdk_git_sha")
    application_sha = app_meta.get("sdk_git_sha")
    if not runtime_sha or runtime_sha != application_sha:
        fail(
            "the two runs do not name the same SDK revision: "
            f"{runtime_sha} != {application_sha}"
        )
    if runtime_meta.get("sample_hz") != app_meta.get("sample_hz"):
        fail(
            "the two recorders sampled at different rates: "
            f"{runtime_meta.get('sample_hz')} != {app_meta.get('sample_hz')}"
        )
    if runtime_meta.get("sample_hz") != summary_json.get("sample_hz"):
        fail(
            "the runtime meta and the runtime reader disagree on sample_hz: "
            f"{runtime_meta.get('sample_hz')} != {summary_json.get('sample_hz')}"
        )
    for key in ("config_id", "layout_id"):
        if app_meta.get(key) != runtime_meta.get(key):
            fail(
                f"the application meta names another {key}: "
                f"{app_meta.get(key)} != {runtime_meta.get(key)}"
            )
    if not app_meta.get("application_config_id"):
        fail("the application meta carries no application_config_id")

    # The model the application ran: the shipped configuration has no local
    # model, so the metadata must say `unloaded` and invent neither a name nor
    # a digest. A configured model would have to name its file.
    configured_model = (
        document.get("application", {}).get("policy", {}).get("model_path")
    )
    model_meta = app_meta.get("model") or {}
    if configured_model is None:
        # A configuration without a model runs no local source at all
        # (`none`), and a local source that could not load one says
        # `unloaded`. Both are explicit absences; neither may invent a name or
        # a digest.
        if model_meta.get("status") not in {"none", "unloaded"}:
            fail(
                "the application invented a model state without a configured "
                f"model: {model_meta}"
            )
        if model_meta.get("name") is not None:
            fail(f"the application invented a model name: {model_meta.get('name')}")
    else:
        if model_meta.get("status") != "loaded":
            fail(f"the application did not report its loaded model: {model_meta}")
        if model_meta.get("name") != Path(str(configured_model)).name:
            fail(
                "the application names another model file: "
                f"{model_meta.get('name')}"
            )

    app_read = subprocess.run(
        [str(read_record), "--run", str(app_directory)],
        capture_output=True,
        text=True,
        check=False,
    )
    if app_read.returncode != 0:
        fail(f"read_record (application) failed: {app_read.stderr.strip()}")
    else:
        app_lines = [line for line in app_read.stdout.splitlines() if line.strip()]
        app_records = [json.loads(line) for line in app_lines[:-1]]
        app_summary_json = json.loads(app_lines[-1])
        velocity = [
            row for row in app_records if row.get("record_type") == "velocity"
        ]
        policy_rows = [
            row
            for row in app_records
            if row.get("record_type") == "policy_session"
        ]
        if app_summary_json.get("record_kind") != "application":
            fail(
                "read_record did not dispatch the application run: "
                f"{app_summary_json.get('record_kind')}"
            )
        if app_summary_json.get("complete") is not True:
            fail(f"the application run is not complete: {app_summary_json}")
        if app_summary_json.get("truncated"):
            fail("the application run reads truncated")
        if int(app_summary_json.get("sample_hz", 0)) != 100:
            fail(
                "the application run declares another sample rate: "
                f"{app_summary_json.get('sample_hz')}"
            )
        if int(app_summary_json.get("joint_count", 0)) != int(
            summary_json["joint_count"]
        ):
            fail("the application run declares another joint count")
        if int(app_summary_json.get("rows_in_range", -1)) != len(velocity):
            fail(
                f"the application reader printed {len(velocity)} velocity row(s), "
                f"its summary reports {app_summary_json.get('rows_in_range')}"
            )
        if int(app_summary_json.get("policy_sessions", -1)) != len(policy_rows):
            fail(
                f"the application reader printed {len(policy_rows)} policy "
                f"session(s), its summary reports "
                f"{app_summary_json.get('policy_sessions')}"
            )

        # Real sample times and cadence: the stream is the configured 100 Hz,
        # not the runtime's 500 Hz control rate.
        stamps = [int(row["stamp_us"]) for row in velocity]
        if len(stamps) < 10:
            fail(f"the application recorded only {len(stamps)} velocity row(s)")
        else:
            if any(later <= earlier for earlier, later in zip(stamps, stamps[1:])):
                fail("the application velocity stamps are not strictly increasing")
            differences = [
                later - earlier for earlier, later in zip(stamps, stamps[1:])
            ]
            middle = sorted(differences)[len(differences) // 2]
            if not 8000 <= middle <= 12000:
                fail(
                    f"the application's median sample interval is {middle} us, "
                    "expected about 10000"
                )
            if max(differences) > 50000:
                fail(
                    "the application left a cadence gap of "
                    f"{max(differences)} us in a 100 Hz stream"
                )

        # Identity: a known row names the runtime/session the runtime record
        # authorized. Rows from before discovery are not a mismatch; rows that
        # name a foreign identity are.
        runtime_identities = {
            (entry.get("runtime_id"), entry.get("session_id"))
            for entry in summary_json.get("sessions", [])
            if entry.get("runtime_id")
            and entry.get("session_id")
            and entry.get("runtime_id") != "0" * 32
            and entry.get("session_id") != "0" * 32
        }
        known = [row for row in velocity if row.get("identity_known")]
        if not known:
            fail("the application recorded no velocity row with a known identity")
        else:
            foreign = [
                row
                for row in known
                if (row.get("runtime_id"), row.get("session_id"))
                not in runtime_identities
            ]
            if foreign:
                fail(
                    f"{len(foreign)} velocity row(s) name an identity the "
                    "runtime record never authorized"
                )
            if not any(int(row.get("latest_state_seq", 0)) > 0 for row in known):
                fail("no known velocity row carries the runtime's state sequence")

        # Phase: the application starts after the runtime and stamps its own
        # deadlines, so its window lies inside the runtime's but is not the same
        # rows. Matching is by monotonic stamp and must overlap.
        runtime_first = summary_json.get("first_stamp_us")
        runtime_last = summary_json.get("last_stamp_us")
        if known and runtime_first is not None and runtime_last is not None:
            known_stamps = [int(row["stamp_us"]) for row in known]
            if known_stamps[0] < runtime_first:
                fail(
                    "the application stream starts before the runtime run: "
                    f"{known_stamps[0]} < {runtime_first}"
                )
            if known_stamps[0] > runtime_last or known_stamps[-1] < runtime_first:
                fail("the application and runtime windows do not overlap")
            if known_stamps[-1] > runtime_last + 100000:
                fail(
                    "the application stream extends past the runtime run: "
                    f"{known_stamps[-1]} > {runtime_last}"
                )

        # A window of the application run selects the same rows by stamp; the
        # filter is real time, not a row index.
        if len(known) >= 3:
            window_start = int(known[len(known) // 3]["stamp_us"])
            window_end = int(known[2 * len(known) // 3]["stamp_us"])
            windowed = subprocess.run(
                [
                    str(read_record),
                    "--run",
                    str(app_directory),
                    "--from-us",
                    str(window_start),
                    "--to-us",
                    str(window_end),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if windowed.returncode != 0:
                fail(f"the windowed application read failed: {windowed.stderr.strip()}")
            else:
                window_lines = [
                    line for line in windowed.stdout.splitlines() if line.strip()
                ]
                window_rows = [json.loads(line) for line in window_lines[:-1]]
                window_summary = json.loads(window_lines[-1])
                selected = [
                    int(row["stamp_us"])
                    for row in window_rows
                    if row.get("record_type") == "velocity"
                ]
                expected = [
                    stamp for stamp in stamps if window_start <= stamp < window_end
                ]
                if selected != expected:
                    fail(
                        "the application window selected "
                        f"{len(selected)} row(s), expected {len(expected)}"
                    )
                if int(window_summary.get("rows_in_range", -1)) != len(expected):
                    fail(
                        "the application window summary reports "
                        f"{window_summary.get('rows_in_range')} row(s), expected "
                        f"{len(expected)}"
                    )

    # The same values through the exporter: the scenario's IMU vectors reach
    # the recorded writer, the reader and the NumPy archive unchanged. This is
    # the end-to-end proof that the nonzero fixture is data, not a flag.
    archive_path = work / "runtime.npz"
    converted = subprocess.run(
        [
            sys.executable,
            str(source_root / "tools" / "record_to_npz.py"),
            str(run_directory),
            str(archive_path),
            "--reader",
            str(read_record),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if converted.returncode != 0:
        fail(f"the runtime export failed: {converted.stderr.strip()}")
    else:
        archived = np.load(archive_path, allow_pickle=False)
        expected_imu = {
            "imu_orientation_rpy": (0.1, -0.2, 0.3),
            "imu_angular_velocity": (0.4, -0.5, 0.6),
            "imu_linear_acceleration": (0.7, -0.8, 9.81),
        }
        rows_archived = int(archived["stamp_us"].shape[0])
        if rows_archived != int(summary_json["rows_in_range"]):
            fail(
                f"the archive carries {rows_archived} row(s), expected "
                f"{summary_json['rows_in_range']}"
            )
        for vector, expected in expected_imu.items():
            column = archived[vector]
            if column.shape != (rows_archived, 3):
                fail(f"the archive's {vector} has shape {column.shape}")
            elif not np.allclose(
                column, np.array(expected, dtype=np.float64), atol=1e-12, rtol=0
            ):
                fail(
                    f"the archive's {vector} is not the fixture's value: "
                    f"{column[:1].tolist()}"
                )

    if failures:
        return 1
    print(
        f"application_record_test: {summary_json['rows_in_range']} row(s), "
        f"{summary_json['sample_hz']} Hz on {summary_json['control_hz']} Hz, "
        f"{len(tracking)} tracking row(s), schema "
        f"{summary_json['schema_version']}, sessions "
        f"{len(json.loads(sessions_path.read_text())) if sessions_path.exists() else 0}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
