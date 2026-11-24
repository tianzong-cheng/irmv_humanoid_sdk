"""Deterministic interleaving at the key-write/log-drainer boundary."""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from run_integration import Application


class KeyWatermarkTest(unittest.TestCase):
    def test_fast_reaction_during_write_remains_visible(self):
        application = Application("/unused", "/unused", 1)

        class Running:
            def poll(self):
                return None

        class Terminal:
            def fileno(self):
                return 9

        application.process = Running()
        application.stream = Terminal()
        application.lines = [b"old pending=ZeroTorque\n"]
        application.drain = lambda: None

        def write(fd, data):
            self.assertEqual((fd, data), (9, b"z"))
            # The app/drainer may run before the writer regains the GIL.
            application.lines.append(b"new pending=ZeroTorque\n")
            return 1

        with patch("run_integration.os.write", side_effect=write):
            application.send_keys("z")
        self.assertEqual(application.watermark, 1)
        self.assertEqual(
            application.wait_for("pending=ZeroTorque", 0), "new pending=ZeroTorque\n"
        )


if __name__ == "__main__":
    unittest.main()
