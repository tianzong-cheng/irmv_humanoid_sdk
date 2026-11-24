"""Display transport tests: standard library only, no renderer or GPU imports."""

from pathlib import Path
import json
import socket
import sys
import select
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from simulation.viewer import (
    ViewerPublisher,
    encode,
    decode,
    receive_latest,
    MAX_PACKET,
)


class ViewerTest(unittest.TestCase):
    def test_view_mode_travels_in_the_initialize_handshake(self):
        # The child decides the shadow pass from the handshake, so the flag must
        # survive the wire round trip and default to "keep the shadow".
        for no_shadow in (False, True):
            packet = encode(
                "initialize",
                "run",
                "asset",
                config={},
                physics_dt_us=1000,
                follow=False,
                no_shadow=no_shadow,
            )
            self.assertIs(decode(packet, "run", "asset")["no_shadow"], no_shadow)
        publisher = ViewerPublisher({}, "run", "asset", 1000)
        self.assertFalse(publisher.no_shadow)
        self.assertTrue(
            ViewerPublisher({}, "run", "asset", 1000, no_shadow=True).no_shadow
        )

    def test_foreign_and_malformed_pose_never_admitted(self):
        good = encode(
            "pose", "run", "asset", qpos=[0.2, -0.4], physics_step=4, released=True
        )
        self.assertEqual(decode(good, "run", "asset", 2)["qpos"], [0.2, -0.4])
        for run, asset, nq in [
            ("other", "asset", 2),
            ("run", "other", 2),
            ("run", "asset", 3),
        ]:
            with self.assertRaises(ValueError):
                decode(good, run, asset, nq)
        value = json.loads(good)
        for change in [
            dict(qpos=[float("nan"), 0]),
            dict(qpos=[0] * 72),
            dict(physics_step=-1),
            dict(physics_step=True),
            dict(released=1),
        ]:
            with self.assertRaises(ValueError):
                decode(json.dumps(dict(value, **change)).encode(), "run", "asset", 2)
        with self.assertRaises(ValueError):
            decode(good[:-1], "run", "asset")
        with self.assertRaises(ValueError):
            decode(b" " * (MAX_PACKET + 1), "run", "asset")

    def test_latest_pose_arrives_after_saturation_without_more_physics(self):
        publisher = ViewerPublisher({}, "run", "asset", 1000)
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        parent.setblocking(False)
        child.setblocking(False)
        parent.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2048)
        stale = encode(
            "pose", "run", "asset", qpos=[0, 0], physics_step=0, released=False
        )
        while True:
            try:
                parent.send(stale)
            except BlockingIOError:
                break
        publisher.socket = parent
        publisher.state = "ready"
        try:
            for step in range(1000):
                publisher.publish([step, -0.4], step, step == 999)
            self.assertGreater(publisher.dropped, 0)
            publisher._start_sender()
            # No more publication or physics from this point. The final
            # completed pose must still replace the saturated stale backlog.
            deadline = time.monotonic() + 2
            latest = None
            while time.monotonic() < deadline:
                select.select([child], [], [], max(0, deadline - time.monotonic()))
                value = receive_latest(child, "run", "asset", 2, -1)
                if value is not None:
                    latest = value
                if latest and latest["physics_step"] == 999:
                    break
            self.assertIsNotNone(latest)
            self.assertEqual(latest["qpos"], [999, -0.4])
            self.assertTrue(latest["released"])
        finally:
            publisher.close()
            child.close()

    def test_owner_eof_retires_display_without_waiting_for_physics(self):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        child.setblocking(False)
        try:
            parent.close()
            self.assertTrue(select.select([child], [], [], 1)[0])
            with self.assertRaises(EOFError):
                receive_latest(child, "run", "asset", 2, 99)
        finally:
            child.close()

    def test_newest_packet_wins_over_reordered_backlog(self):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        child.setblocking(False)
        try:
            for step in (2, 7, 4):
                parent.send(
                    encode(
                        "pose",
                        "run",
                        "asset",
                        qpos=[step, 0],
                        physics_step=step,
                        released=False,
                    )
                )
            self.assertEqual(
                receive_latest(child, "run", "asset", 2, 1)["physics_step"], 7
            )
        finally:
            parent.close()
            child.close()

    def test_publish_binds_sample_without_serialising_on_worker(self):
        import simulation.viewer as module

        publisher = ViewerPublisher({}, "run", "asset", 1000)
        publisher.state = "ready"
        calls = []
        original = module.encode
        module.encode = lambda *a, **k: calls.append(a) or original(*a, **k)
        try:
            publisher.publish((0.1, 0.2), 3, False)
            # Worker path: only the immutable slot is bound, no JSON work.
            self.assertEqual(calls, [])
            self.assertEqual(publisher.pending, (3, (0.1, 0.2), False))
            self.assertEqual(publisher.dropped, 0)
            publisher.publish((0.3, 0.4), 4, True)
            self.assertEqual(calls, [])
            self.assertEqual(publisher.pending, (4, (0.3, 0.4), True))
            self.assertEqual(publisher.dropped, 1)
        finally:
            module.encode = original
            publisher.close()

    def test_display_failure_retires_publication(self):
        publisher = ViewerPublisher({}, "run", "asset", 1000)
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        parent.setblocking(False)
        publisher.socket = parent
        publisher.state = "ready"
        try:
            child.send(encode("error", "run", "asset", reason="render failure"))
            self.assertEqual(publisher.status()["state"], "error")
            publisher.publish([0.2], 2, False)
            self.assertFalse(publisher.pending)
            self.assertEqual(publisher.sent, 0)
        finally:
            publisher.close()
            child.close()

    def test_confirmed_clean_close_clears_transport_close_race(self):
        import subprocess

        process = subprocess.Popen([sys.executable, "-c", "pass"])
        self.assertEqual(process.wait(timeout=2), 0)
        publisher = ViewerPublisher({}, "run", "asset", 1000)
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        parent.setblocking(False)
        publisher.socket = parent
        publisher.process = process
        publisher.state = "error"
        publisher.error = "Broken pipe"
        try:
            child.send(encode("closed", "run", "asset"))
            status = publisher.status()
            self.assertEqual(status["state"], "closed")
            self.assertEqual(status["error"], "")
            self.assertEqual(status["exit_code"], 0)
        finally:
            publisher.close()
            child.close()

    def test_eof_before_failed_process_exit_is_not_clean_closure(self):
        import os, subprocess

        reader, writer = os.pipe()
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import os,sys;os.read(int(sys.argv[1]),1);sys.exit(7)",
                str(reader),
            ],
            pass_fds=(reader,),
        )
        os.close(reader)
        publisher = ViewerPublisher({}, "run", "asset", 1000)
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        parent.setblocking(False)
        publisher.socket = parent
        publisher.process = process
        publisher.state = "ready"
        child.close()
        try:
            self.assertIsNone(process.poll())
            self.assertNotEqual(publisher.status()["state"], "closed")
            os.write(writer, b"x")
            self.assertEqual(process.wait(timeout=2), 7)
            status = publisher.status()
            self.assertEqual(status["state"], "error")
            self.assertIn("7", status["error"])
        finally:
            os.close(writer)
            publisher.close()


if __name__ == "__main__":
    unittest.main()
