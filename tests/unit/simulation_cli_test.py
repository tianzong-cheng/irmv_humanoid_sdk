"""Real coordinator CLI signal installation; bounded isolated subprocesses."""

import os
import signal
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests/support/simulation"))


def child(signum, directory):
    import simulation.main as cli
    from fake_engine import FakeEngine
    from simulation.coordinator import Bootstrap

    class GateHeldRunner(cli.Runner):
        def __init__(self, bootstrap, path):
            super().__init__(bootstrap, path, FakeEngine)

        def run(self):
            result = super().run()
            assert self.shutdown.is_set(), "normal flow never consumed notification"
            print("normal flow consumed shutdown notification", flush=True)
            return result

        def _listen(self):
            super()._listen()
            # main() has already installed the real production handlers. Kernel
            # signals run on this main thread while it owns the ordinary lock.
            with self.engine_gate:
                os.kill(os.getpid(), signum)
                os.kill(os.getpid(), signum)
                print("handlers returned while gate held", flush=True)

    cli.Runner = GateHeldRunner
    cli.load_bootstrap = lambda _: Bootstrap(
        b"r" * 16,
        b"t" * 16,
        b"c" * 32,
        b"a" * 32,
        b"e" * 32,
        1000,
        2000,
        {},
        startup_timeout_us=1000000,
        phase_timeout_us=200000,
    )
    sys.argv = [
        "simulation",
        "--bootstrap",
        "test-only",
        "--socket",
        str(Path(directory) / "bridge.sock"),
    ]
    return cli.main()


class CliSignalTest(unittest.TestCase):
    def test_real_signals_return_under_main_thread_gate_and_fail_bounded(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            with (
                self.subTest(signal=signum),
                tempfile.TemporaryDirectory() as directory,
            ):
                try:
                    result = subprocess.run(
                        [
                            sys.executable,
                            str(Path(__file__).resolve()),
                            "--child",
                            str(int(signum)),
                            directory,
                        ],
                        capture_output=True,
                        text=True,
                        timeout=2,
                    )
                except subprocess.TimeoutExpired as exc:
                    self.fail(
                        f"signal {signum} deadlocked while main held engine_gate: {exc}"
                    )
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("handlers returned while gate held", result.stdout)
                self.assertIn(
                    "normal flow consumed shutdown notification", result.stdout
                )
                self.assertIn("simulation failed:", result.stdout)
                self.assertIn("stopped-unproven", result.stdout)
                self.assertFalse((Path(directory) / "bridge.sock").exists())
                print(
                    f"PASS real signal {signum.name}: handlers returned, finite failure, socket removed"
                )


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--child":
        raise SystemExit(child(int(sys.argv[2]), sys.argv[3]))
    unittest.main()
