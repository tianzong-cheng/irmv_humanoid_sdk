"""Launcher preparation contracts; no optional simulator imports or DDS peers."""

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))


class LauncherTest(unittest.TestCase):
    def test_launcher_available(self):
        self.assertTrue(
            (ROOT / "tools/sim_launch.py").is_file(),
            "Task 9 independent launcher is missing",
        )

    def test_exclusive_run_directory(self):
        from sim_launch import reserve_directory

        with tempfile.TemporaryDirectory(
            prefix="launcher paths with spaces "
        ) as parent:
            path = Path(parent) / "private run"
            reserve_directory(path)
            self.assertEqual(path.stat().st_mode & 0o777, 0o700)
            sentinel = path / "bootstrap.json"
            sentinel.write_text("owned by previous run")
            with self.assertRaises(FileExistsError):
                reserve_directory(path)
            self.assertEqual(sentinel.read_text(), "owned by previous run")

    def test_fresh_independent_ids(self):
        from sim_launch import identities

        a, b = identities(), identities()
        self.assertNotEqual(a, b)
        for pair in (a, b):
            self.assertNotEqual(pair["run_id"], pair["runtime_id"])
            self.assertTrue(
                all(
                    len(bytes.fromhex(value)) == 16 and int(value, 16)
                    for value in pair.values()
                )
            )


class OwnershipTest(unittest.TestCase):
    def fixture(self, directory, engine_factory=None, domain=None):
        import json
        from sim_launch import Experiment
        from unittest.mock import patch

        sys.path.insert(0, str(ROOT / "tests/support/simulation"))
        from fake_engine import FakeEngine

        d = Path(directory)
        normalized = dict(
            socket_directory=str(d / "run"),
            asset_digest="b" * 64,
            policy_path="unused",
            config_digest="a" * 64,
            application_digest="c" * 64,
            engine_digest="d" * 64,
            physics_dt_us=1000,
            control_dt_us=2000,
            startup_timeout_us=2000000,
            phase_timeout_us=1000000,
            engine_config=dict(q=[0, 0]),
        )
        owner = Experiment(
            d / "config with spaces.json",
            d / "runtime with spaces",
            d / "app with spaces",
            domain=domain,
            engine_factory=engine_factory or FakeEngine,
            binding=lambda n: {},
        )
        return owner, patch(
            "sim_launch.subprocess.check_output", return_value=json.dumps(normalized)
        )

    def test_cancel_before_readiness_starts_no_children(self):
        from unittest.mock import patch
        import threading

        entered = threading.Event()
        release = threading.Event()

        class WaitingEngine:
            def initialize(self, c):
                entered.set()
                release.wait(3)
                raise ValueError("cancelled initialization")

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d, WaitingEngine)

            def cancel():
                self.assertTrue(entered.wait(2))
                owner.notify_shutdown()

            notifier = threading.Thread(target=cancel)
            with normalize, patch("sim_launch.subprocess.Popen") as spawn:
                notifier.start()
                try:
                    self.assertIs(owner.start(), owner)
                    self.assertTrue(owner.evidence["startup_cancelled"])
                    self.assertFalse(owner.runner.coordinator.stop_proven)
                    self.assertFalse(owner.evidence["runtime_spawned"])
                    self.assertFalse(owner.thread.is_alive())
                    spawn.assert_not_called()
                finally:
                    release.set()
                    notifier.join(2)
                    owner.close()
                    if owner.runner.worker:
                        owner.runner.worker.join(2)

    def test_cancel_between_spawns_never_starts_application(self):
        from unittest.mock import patch, Mock

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d)
            child = Mock()
            child.poll.return_value = 0
            child.returncode = 0

            def spawn(*a, **kw):
                owner.notify_shutdown()
                return child

            with (
                normalize,
                patch("sim_launch.subprocess.Popen", side_effect=spawn) as launch,
            ):
                try:
                    owner.start()
                    self.assertEqual(launch.call_count, 1)
                    self.assertEqual(len(owner.children), 1)
                    self.assertTrue(owner.evidence["runtime_spawned"])
                    self.assertTrue(owner.evidence["startup_cancelled"])
                    self.assertFalse(owner.thread.is_alive())
                    self.assertFalse(owner.runner.coordinator.stop_proven)
                finally:
                    owner.close()

    def test_cancel_during_log_open_is_checked_at_spawn(self):
        from unittest.mock import patch
        import builtins

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d)

            def opened(path, *args, **kwargs):
                value = builtins.open(path, *args, **kwargs)
                if str(path).endswith("runtime.log"):
                    owner.notify_shutdown()
                return value

            with (
                normalize,
                patch("sim_launch.open", side_effect=opened),
                patch("sim_launch.subprocess.Popen") as spawn,
            ):
                try:
                    owner.start()
                    spawn.assert_not_called()
                    self.assertTrue(owner.evidence["startup_cancelled"])
                finally:
                    owner.close()

    def test_engine_failure_starts_no_children(self):
        from unittest.mock import patch

        class BadEngine:
            def initialize(self, c):
                raise ValueError("startup sentinel")

            def close(self):
                pass

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d, BadEngine)
            with normalize, patch("sim_launch.subprocess.Popen") as spawn:
                try:
                    with self.assertRaisesRegex(RuntimeError, "startup sentinel"):
                        owner.start()
                    spawn.assert_not_called()
                finally:
                    owner.close()
            import json

            evidence = json.loads((Path(d) / "run/evidence.json").read_text())
            self.assertEqual(evidence["state"], "Failed")
            self.assertIn("startup sentinel", evidence["failure"])
            self.assertFalse(evidence["stop_proven"])

    def test_engine_ready_precedes_children_and_socket_collision(self):
        from unittest.mock import patch, Mock
        import socket

        with tempfile.TemporaryDirectory(prefix="launcher paths with spaces ") as d:
            owner, normalize = self.fixture(d)

            def spawn(argv, **kwargs):
                self.assertTrue(
                    kwargs.get("start_new_session"),
                    "terminal signals must not hit SDK children",
                )
                self.assertIsNotNone(owner.runner.coordinator.feedback)
                self.assertTrue((Path(d) / "run/bridge.sock").exists())
                self.assertEqual(
                    argv[1:], ["--config", str(Path(d) / "config with spaces.json")]
                )
                p = Mock()
                p.poll.return_value = 0
                p.returncode = 0
                return p

            with normalize, patch("sim_launch.subprocess.Popen", side_effect=spawn):
                try:
                    owner.start()
                    self.assertEqual(len(owner.children), 2)
                    other, _ = self.fixture(d)
                    sentinel = Path(d) / "run/evidence.json"
                    sentinel.write_text("previous evidence")
                    with self.assertRaises(FileExistsError):
                        other.start()
                    other.close()
                    self.assertEqual(sentinel.read_text(), "previous evidence")
                finally:
                    owner.close()
            # Runner bind collision never deletes the existing socket.
            from simulation.main import Runner

            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            sock.bind(str(Path(d) / "run/bridge.sock"))
            inode = (Path(d) / "run/bridge.sock").stat().st_ino
            collision = Runner(owner.runner.coordinator.b, Path(d) / "run/bridge.sock")
            self.assertEqual(collision.run(), 1)
            self.assertEqual((Path(d) / "run/bridge.sock").stat().st_ino, inode)
            sock.close()

    def test_domain_override_reaches_normalizer_and_both_peers(self):
        from unittest.mock import patch, Mock

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d, domain=17)
            spawns = []
            child = Mock()
            child.poll.return_value = 0
            child.returncode = 0

            def spawn(argv, **kwargs):
                spawns.append(list(argv))
                return child

            with (
                normalize as check_output,
                patch("sim_launch.subprocess.Popen", side_effect=spawn),
            ):
                try:
                    owner.start()
                finally:
                    owner.close()
            normalized = check_output.call_args.args[0]
            self.assertEqual(normalized[1], str(Path(d) / "config with spaces.json"))
            self.assertEqual(normalized[2:], ["--domain", "17"])
            expected = [
                "--config",
                str(Path(d) / "config with spaces.json"),
                "--domain",
                "17",
            ]
            self.assertEqual([argv[1:] for argv in spawns], [expected, expected])

    def test_partial_spawn_cleanup_only_owned_children(self):
        from unittest.mock import patch, Mock

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d)
            child = Mock()
            child.poll.return_value = None
            with (
                normalize,
                patch(
                    "sim_launch.subprocess.Popen",
                    side_effect=[child, OSError("app spawn")],
                ),
            ):
                try:
                    with self.assertRaisesRegex(OSError, "app spawn"):
                        owner.start()
                finally:
                    owner.close()
            child.terminate.assert_called_once()
            child.wait.assert_called_once()
            self.assertEqual(len(owner.children), 1)

    def test_runner_and_both_child_codes_are_required(self):
        from unittest.mock import Mock
        from sim_launch import Experiment

        for runner_code, codes, proven in [
            (1, [0, 0], True),
            (0, [0, 3], True),
            (0, [1, 0], True),
            (0, [0, 0], False),
            (0, [0, 0], True),
        ]:
            owner = Experiment("config", "runtime", "app")
            owner.thread = Mock()
            owner.thread.is_alive.return_value = False
            owner.result = runner_code
            owner.runner = Mock()
            c = owner.runner.coordinator
            c.state = "Closed"
            c.failure = ""
            c.stop_proven = proven
            c.runtime_only = True
            c.release_state = "Released"
            c.release_event_drops = 0
            c.release_events = []
            c.step = 10
            owner.children = [Mock(), Mock()]
            for child, code in zip(owner.children, codes):
                child.wait.return_value = code
            self.assertEqual(
                owner.wait(),
                0 if runner_code == 0 and codes == [0, 0] and proven else 1,
            )

    def test_viewer_start_failure_never_spawns_sdk(self):
        from unittest.mock import patch

        class FailedViewer:
            def __init__(self, *args, **kwargs):
                self.closed = False

            def start(self, timeout):
                raise RuntimeError("native window unavailable")

            def close(self):
                self.closed = True

            def status(self):
                return dict(state="error", error="native window unavailable")

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d)
            owner.viewer_enabled = True
            owner.viewer_factory = FailedViewer
            with normalize, patch("sim_launch.subprocess.Popen") as spawn:
                try:
                    with self.assertRaisesRegex(
                        RuntimeError, "native window unavailable"
                    ):
                        owner.start()
                    spawn.assert_not_called()
                finally:
                    owner.close()
            self.assertTrue(owner.viewer.closed)
            self.assertFalse(owner.runner.coordinator.stop_proven)

    def test_closed_viewer_does_not_cancel_ready_experiment(self):
        from unittest.mock import patch, Mock

        class ClosedViewer:
            def __init__(self, *args, **kwargs):
                self.closed = False

            def start(self, timeout):
                pass

            def publish(self, *args):
                pass

            def close(self):
                self.closed = True

            def status(self):
                return dict(state="closed")

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d)
            owner.viewer_enabled = True
            owner.viewer_factory = ClosedViewer
            child = Mock()
            child.poll.return_value = 0
            child.returncode = 0
            with normalize, patch("sim_launch.subprocess.Popen", return_value=child):
                try:
                    owner.start()
                    self.assertEqual(len(owner.children), 2)
                    self.assertFalse(owner.runner.stop.is_set())
                    self.assertIsNotNone(owner.runner.coordinator.feedback)
                    self.assertEqual(owner.viewer.status()["state"], "closed")
                finally:
                    owner.close()
            self.assertTrue(owner.viewer.closed)

    def test_shadowless_flag_opens_viewer_with_the_mode_set(self):
        from unittest.mock import patch, Mock

        seen = {}

        class RecordingViewer:
            def __init__(self, *args, **kwargs):
                seen.update(kwargs)
                self.closed = False

            def start(self, timeout):
                pass

            def publish(self, *args):
                pass

            def close(self):
                self.closed = True

            def status(self):
                return dict(state="closed")

        with tempfile.TemporaryDirectory() as d:
            owner, normalize = self.fixture(d)
            # Only the shadow option is given: it must still open a viewer, and
            # it must tell the publisher to clear the shadow pass.
            owner.viewer_enabled = True
            owner.viewer_no_shadow = True
            owner.viewer_factory = RecordingViewer
            child = Mock()
            child.poll.return_value = 0
            child.returncode = 0
            with normalize, patch("sim_launch.subprocess.Popen", return_value=child):
                try:
                    owner.start()
                    self.assertIs(seen.get("no_shadow"), True)
                    self.assertIs(seen.get("follow"), False)
                finally:
                    owner.close()
            self.assertTrue(owner.viewer.closed)

    def test_owner_closes_active_application(self):
        self.active_close("healthy")

    def test_owner_cancels_pending_inference(self):
        self.active_close("pending")

    def test_owner_preserves_preexisting_application_fault(self):
        self.active_close("fault")

    def test_alternate_grid_production_handoff(self):
        self.active_close("alternate")

    def test_owner_missing_application_proof_fails(self):
        self.active_close("missing")

    def test_owner_wrong_application_proof_fails(self):
        self.active_close("wrong")

    def test_owner_unsolicited_application_proof_fails(self):
        self.active_close("unsolicited")

    def test_owner_wrong_run_application_proof_fails(self):
        self.active_close("wrong_run")

    def test_owner_application_disconnect_fails(self):
        self.active_close("disconnect")

    def active_close(self, mode):
        import json, subprocess, time
        from sim_launch import Experiment
        from simulation.main import Runner

        sys.path.insert(0, str(ROOT / "tests/support/simulation"))
        from fake_engine import FakeEngine
        from launch_fixture import prepare

        engines = []

        class Engine(FakeEngine):
            def initialize(self, c):
                super().initialize(dict(q=[0, 0]))
                self.closes = 0
                engines.append(self)

            def close(self):
                self.closes += 1
                super().close()

        class Keys(Runner):
            def _actions(self, actions):
                for action in actions:
                    if (
                        action.kind == "send"
                        and action.peer == 3
                        and action.frame.kind == 15
                    ):
                        key = {60: b"d", 70: b"i", 120: b"l"}.get(
                            action.frame.control_step
                        )
                        if key and mode != "fault":
                            owner.children[1].stdin.write(key)
                            owner.children[1].stdin.flush()
                        elif mode == "fault" and action.frame.control_step in (60, 70):
                            owner.children[1].stdin.write(
                                {60: b"d", 70: b"i"}[action.frame.control_step]
                            )
                            owner.children[1].stdin.flush()
                from dataclasses import replace

                filtered = []
                for action in actions:
                    if (
                        mode == "unsolicited"
                        and action.kind == "send"
                        and action.peer == 3
                        and action.frame.kind == 17
                    ):
                        filtered.append(
                            replace(
                                action,
                                frame=replace(
                                    action.frame,
                                    kind=19,
                                    payload=dict(write_id=1, state_seq=1),
                                ),
                            )
                        )
                        continue
                    if (
                        action.kind == "send"
                        and action.peer == 3
                        and action.frame.kind == 19
                    ):
                        if mode == "missing":
                            continue
                        if mode == "wrong":
                            action = replace(
                                action, frame=replace(action.frame, phase=3)
                            )
                        if mode == "wrong_run":
                            action = replace(
                                action, frame=replace(action.frame, run_id=b"X" * 16)
                            )
                    filtered.append(action)
                return super()._actions(filtered)

        with tempfile.TemporaryDirectory(prefix="fixwave active SDK ") as directory:
            d = Path(directory)
            config = prepare(ROOT, BUILD, d)
            cfg = json.loads(config.read_text())
            cfg["simulation"]["host_phase_timeout_us"] = 1000000
            if mode == "alternate":
                cfg["runtime"]["control_hz"] = 1000
            if mode == "fault":
                cfg["application"]["initialization"]["target_position"]["test_left"] = (
                    0.04
                )
                cfg["application"]["initialization"]["trajectory"]["duration_us"] = (
                    20000
                )
                cfg["application"]["initialization"]["trajectory"]["deadline_us"] = (
                    100000
                )
            if mode == "pending":
                import importlib.util

                spec = importlib.util.spec_from_file_location(
                    "generator", ROOT / "tools/make_test_model.py"
                )
                generator = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(generator)
                fixture = json.loads((d / "deployment.json").read_text())
                generator.save(
                    d / "models",
                    "delayed.onnx",
                    generator.make_model(
                        generator.long_running_graph(
                            fixture["model"]["input"]["dimension"], 2, iterations=200000
                        ),
                        fixture,
                    ),
                )
                cfg["application"]["policy"]["model_path"] = str(
                    d / "models/delayed.onnx"
                )
            config.write_text(json.dumps(cfg))
            owner = Experiment(
                config,
                BUILD / "robot_runtime",
                BUILD / "application",
                engine_factory=Engine,
                binding=lambda n: {"fixture_only": True},
                runner_type=Keys,
                application_stdin=subprocess.PIPE,
            )
            try:
                owner.start()
                end = time.monotonic() + 30
                while time.monotonic() < end:
                    output = (d / "run/application.log").read_text()
                    if mode == "pending":
                        ready = "simulation inference pending" in output
                    elif mode == "fault":
                        ready = "cause=InitializationFailed" in output
                    else:
                        ready = owner.runner.coordinator.step >= 150
                    if ready:
                        if mode == "pending":
                            self.assertEqual(owner.runner.coordinator.step, 120)
                            self.assertEqual(
                                owner.runner.coordinator.state, "ApplicationPhase"
                            )
                        break
                    self.assertIsNone(
                        owner.result, (owner.runner.coordinator.failure, output)
                    )
                    time.sleep(0.002)
                self.assertTrue(ready, (mode, output))
                self.assertFalse(owner.runner.coordinator.runtime_only)
                self.assertIsNone(owner.children[1].poll())
                if mode == "disconnect":
                    owner.children[1].kill()
                owner.notify_shutdown()
                outcome = owner.wait()
                print(
                    "owner-close",
                    mode,
                    "result",
                    outcome,
                    "children",
                    owner.evidence["child_codes"],
                    "state",
                    owner.runner.coordinator.state,
                    "stop_proven",
                    owner.runner.coordinator.stop_proven,
                    "engine_closes",
                    engines[0].closes,
                )
                self.assertEqual(
                    outcome,
                    1
                    if mode
                    in (
                        "fault",
                        "missing",
                        "wrong",
                        "wrong_run",
                        "unsolicited",
                        "disconnect",
                    )
                    else 0,
                    (
                        owner.runner.coordinator.failure,
                        (d / "run/application.log").read_text(),
                    ),
                )
                if mode in (
                    "missing",
                    "wrong",
                    "wrong_run",
                    "unsolicited",
                    "disconnect",
                ):
                    self.assertNotEqual(owner.evidence["child_codes"][1], 0)
                    return
                self.assertTrue(owner.runner.coordinator.stop_proven)
                self.assertEqual(
                    owner.evidence["child_codes"], [0, 3] if mode == "fault" else [0, 0]
                )
                self.assertEqual(engines[0].closes, 1)
                self.assertTrue(engines[0].closed)
                self.assertEqual(engines[0].tau, (0.0, 0.0))
                self.assertTrue(owner.runner.coordinator.torque["accepted"])
                self.assertEqual(
                    owner.runner.coordinator.states[-1]["execution_state"], 1
                )
            finally:
                owner.close()

    def test_application_quit_retains_runtime_then_owner_closes(self):
        import json, subprocess, time
        from sim_launch import Experiment
        from simulation.main import Runner

        sys.path.insert(0, str(ROOT / "tests/support/simulation"))
        from fake_engine import FakeEngine
        from launch_fixture import prepare

        build = BUILD
        engines = []

        class Engine(FakeEngine):
            def initialize(self, c):
                super().initialize(dict(q=[0, 0]))
                self.closes = 0
                engines.append(self)

            def close(self):
                self.closes += 1
                super().close()

        class Keys(Runner):
            def _actions(self, actions):
                for action in actions:
                    if (
                        action.kind == "send"
                        and action.peer == 3
                        and action.frame.kind == 15
                    ):
                        key = {60: b"d", 70: b"i", 120: b"l", 150: b"x"}.get(
                            action.frame.control_step
                        )
                        if key:
                            owner.children[1].stdin.write(key)
                            owner.children[1].stdin.flush()
                return super()._actions(actions)

        with tempfile.TemporaryDirectory(prefix="t9 sdk spaces ") as directory:
            d = Path(directory)
            config = prepare(ROOT, build, d)
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
                end = time.monotonic() + 30
                while owner.runner.coordinator.step < 450 and time.monotonic() < end:
                    self.assertIsNone(owner.result, owner.runner.coordinator.failure)
                    time.sleep(0.002)
                c = owner.runner.coordinator
                self.assertGreaterEqual(c.step, 450)
                self.assertEqual(owner.children[1].wait(timeout=3), 0)
                self.assertIsNone(owner.children[0].poll())
                self.assertTrue(c.runtime_only)
                self.assertEqual(c.states[-1]["execution_state"], 2)
                self.assertEqual(c.release_state, "Released")
                owner.notify_shutdown()
                self.assertEqual(owner.wait(), 0, c.failure)
                self.assertTrue(c.stop_proven)
                self.assertEqual(engines[0].closes, 1)
            finally:
                owner.close()
            evidence = json.loads((d / "run/evidence.json").read_text())
            self.assertEqual(evidence["child_codes"], [0, 0])
            self.assertTrue(evidence["engine_ready_before_sdk"])


if __name__ == "__main__":
    BUILD = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else ROOT / "build"
    unittest.main()
