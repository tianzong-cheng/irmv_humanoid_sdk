"""Actual SDK + deterministic FakeEngine recording, serial reserved domain226.

This proves recording and protocol provenance, not MuJoCo/GPU or policy safety.
"""

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
MODULE_ROOT = (
    Path(sys.argv[1]).resolve()
    if len(sys.argv) > 1 and (Path(sys.argv[1]) / "simulation").is_dir()
    else ROOT / "tools"
)
sys.path.insert(0, str(MODULE_ROOT))
# Exporter is source-owned in this fixture; installed simulation imports stay first.
sys.path.append(str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests/support/simulation"))
from fake_engine import FakeEngine
from launch_fixture import prepare
from sim_launch import Experiment
from simulation.main import Runner
from simulation.record import read


def run(build):
    assert (
        "226"
        in json.loads((ROOT / "tests/integration/dds_domains.json").read_text())[
            "reserved"
        ]
    )

    class Engine(FakeEngine):
        def initialize(self, c):
            super().initialize(dict(q=[0, 0]))

    class Keys(Runner):
        def _actions(self, actions):
            for a in actions:
                if a.kind == "send" and a.peer == 3 and a.frame.kind == 15:
                    key = {60: b"d", 70: b"i", 120: b"l", 150: b"x"}.get(
                        a.frame.control_step
                    )
                    if key:
                        owner.children[1].stdin.write(key)
                        owner.children[1].stdin.flush()
            return super()._actions(actions)

    with tempfile.TemporaryDirectory(prefix="simulation record spaces ") as temp:
        d = Path(temp)
        config = prepare(ROOT, build, d)
        cfg = json.loads(config.read_text())
        cfg["dds"]["domain"] = 226
        cfg["telemetry"]["recorder"]["enabled"] = True
        cfg["telemetry"]["recorder"]["directory"] = str(d)
        config.write_text(json.dumps(cfg))
        owner = Experiment(
            config,
            build / "robot_runtime",
            build / "application",
            engine_factory=Engine,
            binding=lambda n: {"fixture_only": True},
            runner_type=Keys,
            application_stdin=subprocess.PIPE,
        )
        try:
            owner.start()
            end = time.monotonic() + 25
            while owner.runner.coordinator.step < 220 and time.monotonic() < end:
                assert owner.result is None, owner.runner.coordinator.failure
                time.sleep(0.002)
            assert owner.runner.coordinator.step >= 220
            assert owner.runner.coordinator.runtime_only
            owner.notify_shutdown()
            assert owner.wait() == 0, owner.runner.coordinator.failure
        finally:
            owner.close()
        events, summary = read(d / "run/simulation.jsonl")
        assert summary["evidence_complete"], summary
        feedback = [e for e in events if e["kind"] == "event_feedback"]
        assert len(feedback) == 1 + owner.runner.coordinator.step
        for e in feedback:
            measured = e["feedback"]
            assert measured["base_position"] == [0.0, 0.0, 0.0]
            assert measured["base_velocity"] == [0.0] * 6
            assert measured["imu_quaternion"] == [1.0, 0.0, 0.0, 0.0]
            assert measured["imu_angular_velocity"] == [0.0, 0.0, 0.0]
            assert measured["base_quaternion"] == [1.0, 0.0, 0.0, 0.0]
            assert measured["physics_step"] == e["completed_physics_step"]
            assert e["logical_us"] == 1 + measured["physics_step"] * 1000

        accepted = [e for e in events if e["kind"] == "write_accepted"]
        applied = [e for e in events if e["kind"] == "AppliedToStep"]
        assert accepted and applied
        by_write = {e["frame"]["payload"]["write_id"]: e for e in accepted}
        for e in applied:
            payload = e["frame"]["payload"]
            a = by_write[payload["write_id"]]
            assert a["sequence"] < e["sequence"]
            assert payload["physics_step_end"] > payload["physics_step_begin"]
        # A frozen stop has a fresh FSM state sequence and write, but no
        # fabricated AppliedToStep: closing zero output does not advance physics.
        stop = [e for e in accepted if e["phase"] == 4]
        assert len(stop) == 1
        assert stop[0]["frame"]["payload"]["write_id"] not in {
            e["frame"]["payload"]["write_id"] for e in applied
        }
        assert any(
            e["host_stop_duration_us"] >= 0
            for e in events
            if e["kind"] == "event_stopped"
        )
        arms = [e for e in events if e.get("frame", {}).get("name") == "ReleaseArm"]
        assert arms, "no actual consumed first-command lifecycle proof"
        arm = arms[0]["frame"]["payload"]
        first = arm["first_command"]
        first_accept = [
            e for e in accepted if e["frame"]["payload"]["command"] == first
        ]
        assert first_accept
        assert any(
            e["frame"]["payload"]["write_id"]
            == first_accept[0]["frame"]["payload"]["write_id"]
            for e in applied
        )
        # Move the complete experiment group; metadata references remain relative.
        moved = d / "moved experiment"
        moved.mkdir()
        for path in list(d.iterdir()):
            if path != moved:
                shutil.move(str(path), moved / path.name)
        runs = []
        for path in moved.glob("*/meta.json"):
            meta = json.loads(path.read_text())
            runs.append((path.parent, meta))
        runtime = next(p for p, m in runs if m.get("schema_version") == 4)
        application = next(
            p for p, m in runs if m.get("schema") == "irmv.application_velocity"
        )

        def output(path):
            lines = subprocess.check_output(
                [str(build / "read_record"), "--run", str(path)], text=True
            ).splitlines()
            return [json.loads(line) for line in lines]

        runtime_rows = output(runtime)
        app_rows = output(application)
        for rows in (runtime_rows, app_rows):
            assert rows[-1]["clock"]["clock_domain"] == "simulation"
            assert rows[-1]["simulation_evidence"]["evidence_complete"]
            assert rows[-1]["simulation_evidence"]["clock"] == summary["clock"]
        samples = [r for r in runtime_rows if r.get("record_type") == "sample"]
        if not samples:
            samples = [r for r in runtime_rows if "stamp_us" in r and "state_seq" in r]
        stamps = [r["stamp_us"] for r in samples]
        assert stamps and min(stamps) >= 1 and max(stamps) < 10000000
        assert len(stamps) == len(set(stamps)), (
            "frozen stop duplicated a scheduled sample"
        )
        archive = moved / "result.npz"
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/record_to_npz.py"),
                str(runtime),
                str(archive),
                "--reader",
                str(build / "read_record"),
                "--application-record",
                str(application),
            ],
            check=True,
        )
        import numpy as np

        with np.load(archive, allow_pickle=False) as data:
            assert data["simulation_evidence_complete"]
            assert data["association_ok"]
            assert str(data["clock_domain"]) == "simulation"
            assert len(data["simulation_host_operation_duration_us"]) == len(events)
            assert (data["logical_stamp_us"] == data["stamp_us"]).all()

        def web(query):
            result = subprocess.run(
                [str(build / "web_record_query"), "--records-root", str(moved)],
                input=json.dumps(query) + "\n",
                text=True,
                capture_output=True,
                check=True,
            )
            return json.loads(result.stdout)

        catalog = web(dict(op="describe"))
        runtime_id = next(
            r["run_id"] for r in catalog["runs"] if r["name"] == runtime.name
        )
        app_id = next(
            r["run_id"] for r in catalog["runs"] if r["name"] == application.name
        )
        context = web(dict(op="context", run_id=runtime_id, app_run_id=app_id))[
            "context"
        ]
        assert context["association"]["ok"]
        assert context["simulation_evidence"]["evidence_complete"]
        assert context["simulation_evidence"]["measured_feedback_complete"]
        assert context["simulation_evidence"]["final"]["stop_proven"]
        assert context["clock"]["clock_run_id"] == summary["clock"]["clock_run_id"]
        # Foreign simulation run on the same host must be refused by both readers.
        meta_path = application / "meta.json"
        meta = json.loads(meta_path.read_text())
        meta["clock"]["clock_run_id"] = "ef" * 16
        meta_path.write_text(json.dumps(meta))
        assert not output(application)[-1]["simulation_evidence"]["evidence_complete"]
        assert not web(dict(op="context", run_id=runtime_id, app_run_id=app_id))[
            "context"
        ]["association"]["ok"]
        import record_to_npz

        assert not record_to_npz.associate(
            runtime_rows[-1],
            output(application)[-1],
            [r for r in app_rows if r.get("record_type") == "velocity"],
        )["ok"]
        # Required 60 logical seconds at 500 Hz, eight evidence events per
        # control step. No total-file cap may reject a normal recording horizon.
        long_path = moved / "run/long-horizon.jsonl"
        header = json.loads(
            (moved / "run/simulation.jsonl").read_text().splitlines()[0]
        )
        count = 0
        with long_path.open("w") as f:
            f.write(json.dumps(header) + "\n")
            for step in range(30001):
                for phase in (1, 1, 1, 2, 2, 2, 3, 3):
                    count += 1
                    f.write(
                        json.dumps(
                            dict(
                                record_type="event",
                                sequence=count,
                                kind="horizon_test_event",
                                logical_us=1 + step * 2000,
                                control_step=step,
                                phase=phase,
                            )
                        )
                        + "\n"
                    )
            f.write(
                json.dumps(
                    dict(
                        record_type="footer",
                        offered=count,
                        written=count,
                        dropped=0,
                        write_failures=0,
                    )
                )
                + "\n"
            )
        streamed = 0

        def consume(row):
            nonlocal streamed
            streamed += 1

        _, long_summary = read(long_path, visitor=consume)
        assert streamed == 240008 and long_summary["evidence_complete"]
        assert not long_summary["final"].get("stop_proven", False)
        try:
            read(long_path)
        except ValueError as exc:
            assert "streaming visitor" in str(exc)
        else:
            raise AssertionError(
                "bounded convenience materialization silently exceeded its cap"
            )
        runtime_meta = json.loads((runtime / "meta.json").read_text())
        runtime_meta["simulation"]["sidecar"] = "../run/long-horizon.jsonl"
        (runtime / "meta.json").write_text(json.dumps(runtime_meta))
        text = subprocess.check_output(
            [str(build / "read_record"), "--run", str(runtime), "--summary-only"],
            text=True,
        )
        long_cpp = json.loads(text)["simulation_evidence"]
        assert long_cpp["evidence_complete"] and long_cpp["rows"] == 240008
        assert not long_cpp.get("final", {}).get("stop_proven", False)
        long_web = web(dict(op="context", run_id=runtime_id))["context"][
            "simulation_evidence"
        ]
        assert long_web["evidence_complete"] and long_web["rows"] == 240008
        print(
            "PASS 60s/500Hz/8-events-per-step: 240008 events streamed by Python/C++/web; materialization limit explicit; no inferred experiment success"
        )
        print(
            "PASS actual SDK/FakeEngine: distinct acceptance/application, frozen stop, movable evidence, logical NPZ and foreign-run refusal"
        )


if __name__ == "__main__":
    run(Path(sys.argv[1]).resolve())
