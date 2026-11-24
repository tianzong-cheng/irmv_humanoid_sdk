"""Local coordinator runner. Invoke: python3 -m tools.simulation.main --help.

No SDK processes are launched here. The normalized bootstrap is produced before
process startup, with independent fresh run/runtime IDs. Configuration canonical
hashing belongs to the C++ loader, not this module.
"""

import argparse
import importlib.metadata
import json
import os
import platform
import queue
import selectors
import signal
import socket
import stat
import struct
import threading
import time
from collections import deque
from dataclasses import fields
from pathlib import Path

from . import protocol as p
from .coordinator import Bootstrap, Event, SimulationCoordinator
from .engine import Feedback
from .record import Writer

# Feedback schema is frozen: every field is a scalar or an immutable tuple, so a
# field-name dict is an equivalent detached evidence snapshot. asdict() instead
# rebuilds every nested tuple element by element, costing ~20 us per control
# step on the coordinator thread for identical recorded bytes.
FEEDBACK_FIELDS = tuple(field.name for field in fields(Feedback))


def feedback_snapshot(feedback):
    return {name: getattr(feedback, name) for name in FEEDBACK_FIELDS}


def host_us():
    return time.monotonic_ns() // 1000


# Frame kinds used on the coordinator hot path; resolved once, not per event.
TORQUE_KIND = p.KINDS.index("Torque") + 1
RELEASE_CANCEL_KIND = p.KINDS.index("ReleaseCancel") + 1
EVIDENCE_ACTION_KINDS = frozenset(
    p.KINDS.index(name) + 1
    for name in ("AppliedToStep", "ShutdownCommitted", "LeaveCommitted")
)


def _fd_identity(fd):
    # Identity of the open file description behind an fd number, for deciding
    # whether a saved wakeup fd still names the same object after it was
    # temporarily displaced. None when the number is not open.
    try:
        info = os.fstat(fd)
    except OSError:
        return None
    return (info.st_dev, info.st_ino, stat.S_IFMT(info.st_mode))


def _restore_wakeup_fd(previous, identity):
    # Restore the process-global wakeup fd only if the saved number still
    # identifies the same open object. A number closed and reused by someone
    # else must be disarmed, never made a signal destination.
    if previous >= 0 and _fd_identity(previous) != identity:
        previous = -1
    try:
        signal.set_wakeup_fd(previous)
    except (ValueError, OSError):
        try:
            signal.set_wakeup_fd(-1)
        except (ValueError, OSError):
            pass


def load_bootstrap(path):
    data = json.loads(Path(path).read_text())
    for name in (
        "run_id",
        "runtime_id",
        "config_digest",
        "application_digest",
        "engine_digest",
    ):
        data[name] = bytes.fromhex(data[name])
    b = Bootstrap(**data)
    SimulationCoordinator(b)  # Validate before binding sockets or touching an engine.
    return b


def warp_engine():
    # Selected only by main; fake tests never import optional dependencies.
    from .mujoco_warp_engine import MujocoWarpEngine

    return MujocoWarpEngine()


class Runner:
    def __init__(self, bootstrap, socket_path, engine_factory=warp_engine):
        self.coordinator = SimulationCoordinator(bootstrap)
        self.path = Path(socket_path).absolute()
        self.engine_factory = engine_factory
        self.stop = threading.Event()
        self.shutdown = threading.Event()
        self._shutdown_notified = False
        self.engine_gate = threading.Lock()
        self.release_cancel = threading.Event()
        self.step_started = False
        # One engine completion may arrive while the application handshake for
        # the same step is still running. It is held here, not in the
        # coordinator, so its recorded identity stays the step it completed.
        self.held_feedback = None
        self.jobs = queue.Queue(maxsize=32)
        self.results = queue.Queue(maxsize=32)
        self.worker = None
        self.viewer = None
        self.selector = selectors.DefaultSelector()
        self.wake_read, self.wake_write = socket.socketpair()
        self.wake_read.setblocking(False)
        self.wake_write.setblocking(False)
        # Serialises the only cross-thread writer (the engine worker) with the
        # handoff close, so a send can never run against a recycled descriptor.
        self.wake_lock = threading.Lock()
        # Signal wake ownership: the C handler writes the signal number to the
        # socketpair while the Python handler only assigns the flag.
        self._wakeup_owned = False
        self._wakeup_previous = -1
        self._wakeup_identity = None
        self.selector.register(self.wake_read, selectors.EVENT_READ)
        self.connections = {}
        self.peers = {}
        self.listener = None
        self.directory = None
        self.socket_inode = None
        b = bootstrap
        versions = {}
        for name in ("mujoco", "mujoco-warp", "warp-lang"):
            try:
                versions[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                versions[name] = "unavailable"
        self.record = Writer(
            self.path.parent / "simulation.jsonl",
            dict(
                clock="simulation",
                clock_domain="simulation",
                available=True,
                clock_run_id=b.run_id.hex(),
                logical_origin_us=1,
                physics_dt_us=b.physics_dt_us,
                control_dt_us=b.control_dt_us,
            ),
            provenance=dict(
                runtime_id=b.runtime_id.hex(),
                config_digest=b.config_digest.hex(),
                application_digest=b.application_digest.hex(),
                engine_digest=b.engine_digest.hex(),
                engine_config=b.engine_config,
                engine_factory=engine_factory.__module__
                + "."
                + engine_factory.__qualname__,
                python=platform.python_version(),
                dependencies=versions,
            ),
        )

    def request_stop(self):
        self.stop.set()
        self._wake()

    def _notify_shutdown(self, signum, frame):
        # Signal context: assignment only, even if this thread owns engine_gate.
        # The selector's wake comes from signal.set_wakeup_fd, whose C handler
        # writes the signal number to the nonblocking socketpair.
        self._shutdown_notified = True

    def request_shutdown(self):
        # Independent owner capability. Application never signals runtime.
        with self.engine_gate:
            self.shutdown.set()
        self._wake()

    def _wake(self):
        with self.wake_lock:
            try:
                self.wake_write.send(b"\0")
            except (BlockingIOError, OSError):
                pass  # Already readable, or retired.

    def _post_result(self, event):
        self.results.put_nowait(event)
        self._wake()

    def _engine_worker(self):
        engine = None
        visual_released = False
        try:
            while not self.stop.is_set():
                try:
                    action = self.jobs.get(timeout=0.01)
                except queue.Empty:
                    continue
                released = False
                operation_start = host_us()
                release_duration = 0
                if action.kind == "initialize":
                    engine = self.engine_factory()
                    engine.initialize(self.coordinator.b.engine_config)
                    if (
                        abs(
                            engine.timestep_s * 1000000
                            - self.coordinator.b.physics_dt_us
                        )
                        > 1e-6
                    ):
                        raise ValueError("engine timestep differs from bootstrap")
                    feedback = engine.read_feedback()
                elif action.kind == "step":
                    with self.engine_gate:
                        cancelled = (
                            self._shutdown_notified
                            or self.shutdown.is_set()
                            or self.stop.is_set()
                        )
                        release = action.release and not self.release_cancel.is_set()
                        if not cancelled:
                            self.step_started = True
                    if cancelled:
                        self._post_result(Event("cancelled"))
                        continue
                    # Passing the gate commits this worker operation. Cancellation
                    # cannot undo an already executing GPU call or release.
                    if release:
                        release_start = host_us()
                        engine.release_base()
                        release_duration = host_us() - release_start
                        released = True
                    engine.apply_torque(action.torque)
                    feedback = engine.step(action.substeps)
                elif action.kind == "stop_output":
                    engine.apply_torque(action.torque)
                    # Completion includes resource close, not just zero-output submission.
                    engine.close()
                    engine = None
                    self._post_result(
                        Event(
                            "stopped",
                            host_operation_start_us=operation_start,
                            host_operation_duration_us=host_us() - operation_start,
                        )
                    )
                    continue
                else:
                    raise ValueError("invalid engine action")
                visual_released = visual_released or released
                if self.viewer is not None:
                    # One latest display slot; the independent sender paces UI.
                    try:
                        self.viewer.publish(
                            engine.visual_qpos(), feedback.physics_step, visual_released
                        )
                    except Exception as exc:
                        # A display failure cannot become a physics failure.
                        self.viewer.state = "error"
                        self.viewer.error = str(exc)
                if not self.stop.is_set():
                    self._post_result(
                        Event(
                            "feedback",
                            feedback=feedback,
                            released=released,
                            host_operation_start_us=operation_start,
                            host_operation_duration_us=host_us() - operation_start,
                            host_release_duration_us=release_duration,
                        )
                    )
        except Exception as exc:
            if not self.stop.is_set():
                try:
                    self._post_result(Event("engine_error", reason=str(exc)))
                except queue.Full:
                    self.request_stop()
        finally:
            # Only this worker ever touches the engine, including close. If GPU
            # code is stuck this daemon can outlive run(); Python cannot kill it.
            if engine is not None:
                try:
                    engine.close()
                except Exception:
                    pass

    def _listen(self):
        self.directory = os.open(
            self.path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
        )
        info = os.fstat(self.directory)
        if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) != 0o700:
            raise ValueError("socket parent must be same-user private0700")
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.listener.setblocking(False)
        # Pin the validated parent through the descriptor (no unlink on bind).
        self.listener.bind(f"/proc/self/fd/{self.directory}/{self.path.name}")
        self.socket_inode = os.stat(
            self.path.name, dir_fd=self.directory, follow_symlinks=False
        ).st_ino
        self.listener.listen(2)
        self.selector.register(self.listener, selectors.EVENT_READ)

    def _interest(self, conn):
        # Interest is polled through a cached mask: an empty output deque is
        # the steady state, so a flush must not issue a redundant epoll_ctl per
        # sent frame. Entries built by tests without a mask just resync once.
        entry = self.connections[conn]
        desired = selectors.EVENT_READ | (selectors.EVENT_WRITE if entry["out"] else 0)
        if entry.get("mask") == desired:
            return
        self.selector.modify(conn, desired)
        entry["mask"] = desired

    def _retired_application(self, conn):
        return self.coordinator.runtime_only and self.connections[conn]["peer"] == 3

    def _receive_packet(self, conn):
        try:
            packet, _, flags, _ = conn.recvmsg(p.MAX_PACKET_BYTES)
            return packet, flags
        except ConnectionResetError:
            if not self._retired_application(conn):
                raise
            # Membership was already revoked by a validated LeaveReady. Linux
            # reports reset rather than EOF when a killed peer had unread data.
            return b"", 0

    def _disconnect(self, conn):
        peer = self.connections[conn]["peer"]
        self.selector.unregister(conn)
        del self.connections[conn]
        self.peers.pop(peer, None)
        conn.close()
        self._advance(Event("disconnect", peer, reason="peer disconnected"))

    def _flush(self, conn):
        outgoing = self.connections[conn]["out"]
        while outgoing:
            try:
                n = conn.send(outgoing[0])
            except BlockingIOError:
                break
            except (BrokenPipeError, ConnectionResetError):
                if not self._retired_application(conn):
                    raise
                self._disconnect(conn)
                return
            if n != len(outgoing[0]):
                raise ValueError("partial seqpacket send")
            outgoing.popleft()
        self._interest(conn)

    def _actions(self, actions):
        for a in actions:
            if a.kind in ("initialize", "step", "stop_output"):
                self.jobs.put_nowait(a)
            elif a.kind == "send":
                conn = self.peers.get(a.peer)
                if conn is None:
                    raise ValueError("missing action peer")
                outgoing = self.connections[conn]["out"]
                if len(outgoing) >= 32:
                    raise ValueError("socket output queue capacity")
                outgoing.append(p.encode(a.frame))
                self._flush(conn)
            elif a.kind == "cancel":
                self.stop.set()
            else:
                raise ValueError("unknown action")

    def _evidence(self, kind, frame=None, **facts):
        c = self.coordinator
        row = dict(
            kind=kind,
            control_step=c.step,
            phase=c.phase,
            logical_us=1 + c.step * c.b.control_dt_us,
            coordinator_state=c.state,
            host_observed_us=host_us(),
            **facts,
        )
        if frame:
            row.update(
                control_step=frame.control_step,
                phase=frame.phase,
                logical_us=frame.logical_us,
            )
            row["frame"] = dict(name=p.KINDS[frame.kind - 1], payload=frame.payload)
        # The row is built here from an immutable feedback snapshot and this
        # event's frame payload; it is handed over by ownership so the single
        # file worker serialises it, and nothing in the coordinator thread
        # touches it again.
        self.record.offer_owned(row)

    def _feedback_ready(self):
        # The coordinator consumes a completion immediately only when the
        # handshakes are already closed (PhysicsAdvance/runtime-only), for the
        # initial handshake, or when a shutdown has abandoned them after the
        # interval was committed. Otherwise an overlapped completion stays held.
        # With no outstanding action the event is delivered so the coordinator
        # rejects a stale/duplicate completion instead of hiding it.
        c = self.coordinator
        if c.state in ("Failed", "Closed"):
            return False
        if c.physics_in_flight and c.pending_feedback is None:
            return c.shutdown_pending or c.state in ("Handshake", "PhysicsAdvance")
        return True

    def _deliver_result(self, event):
        if event.kind == "feedback" and not self._feedback_ready():
            if self.held_feedback is not None:
                # One outstanding engine action can only complete once.
                self._advance(
                    Event(
                        "engine_error",
                        reason="duplicate engine completion before handshake",
                    )
                )
                return
            self.held_feedback = event
            return
        self._advance(event)

    def _advance(self, event):
        if event.kind in ("feedback", "cancelled"):
            with self.engine_gate:
                self.step_started = False
                self.release_cancel.clear()
        if event.frame and event.frame.kind == RELEASE_CANCEL_KIND:
            with self.engine_gate:
                started = self.step_started
                if event.frame.control_step == self.coordinator.step and not started:
                    self.release_cancel.set()
            if started:
                from dataclasses import replace

                event = replace(event, reason="operation_started")
        old_torque = self.coordinator.torque
        actions = self.coordinator.advance(event, host_us())
        c = self.coordinator
        if event.kind != "poll":
            self._evidence(
                "event_" + event.kind,
                event.frame,
                peer=event.peer,
                reason=event.reason,
                released=event.released,
                host_operation_start_us=event.host_operation_start_us,
                host_operation_duration_us=event.host_operation_duration_us,
                host_release_duration_us=event.host_release_duration_us,
                host_stop_duration_us=event.host_operation_duration_us
                if event.kind == "stopped"
                else 0,
                engine_time_s=event.feedback.engine_time_s if event.feedback else None,
                completed_physics_step=event.feedback.physics_step
                if event.feedback
                else None,
                feedback=feedback_snapshot(event.feedback) if event.feedback else None,
                feedback_validated=event.feedback is not None
                and (
                    event.feedback is c.feedback or event.feedback is c.pending_feedback
                ),
                physics_in_flight=c.physics_in_flight,
                stop_proven=c.stop_proven,
                release_state=c.release_state,
                failure=c.failure,
            )
        if (
            event.frame
            and event.frame.kind == TORQUE_KIND
            and c.torque is not None
            and c.torque is not old_torque
        ):
            self._evidence("write_accepted", event.frame)
        for action in actions:
            if action.frame and action.frame.kind in EVIDENCE_ACTION_KINDS:
                self._evidence(p.KINDS[action.frame.kind - 1], action.frame)
        self._actions(actions)
        if self.held_feedback is not None and self._feedback_ready():
            # Handshake closure or shutdown opened the gate. Deliver after this
            # event's rows so replay sees the same completion ordering.
            held = self.held_feedback
            self.held_feedback = None
            self._advance(held)

    def run(self):
        # The selector needs a descriptor wake for an assignment-only Python
        # signal handler. Register the C-level wakeup only on the thread that
        # may own it, and restore it before the socketpair is closed.
        if threading.current_thread() is threading.main_thread():
            try:
                previous = signal.set_wakeup_fd(self.wake_write.fileno())
                if previous == self.wake_write.fileno():
                    previous = -1
                identity = _fd_identity(previous) if previous >= 0 else None
                self._wakeup_previous = previous if identity is not None else -1
                self._wakeup_identity = identity
                self._wakeup_owned = True
            except (ValueError, OSError):
                self._wakeup_owned = False
        try:
            self.record.start()
            self._listen()
            self.worker = threading.Thread(
                target=self._engine_worker, daemon=True, name="simulation-engine"
            )
            self.worker.start()
            self._advance(Event("start"))
            while self.coordinator.state not in ("Failed", "Closed"):
                if self._shutdown_notified and not self.shutdown.is_set():
                    self.request_shutdown()
                if self.shutdown.is_set() and not self.coordinator.shutdown_pending:
                    self._advance(
                        Event("stop", reason="independent experiment shutdown")
                    )
                if self.stop.is_set():
                    self._actions(
                        self.coordinator.fail("forced stop; stopped-unproven")
                    )
                    break
                for _ in range(32):
                    # Single consumer: emptiness is advisory only, and the
                    # producer's wake keeps a later result from being parked.
                    if self.results.empty():
                        break
                    self._deliver_result(self.results.get_nowait())
                    if self.coordinator.state in ("Failed", "Closed"):
                        break
                if self.coordinator.state in ("Failed", "Closed"):
                    break
                # A poll only lets the coordinator observe its own host
                # deadline; every event already checks it. Skip the call while
                # the deadline is in the future, so an idle selector wait is
                # not preceded by a redundant state-machine step.
                now = host_us()
                deadline = self.coordinator.deadline
                if deadline is None or now >= deadline:
                    self._advance(Event("poll"))
                    if self.coordinator.state in ("Failed", "Closed"):
                        break
                    now = host_us()
                    deadline = self.coordinator.deadline
                # Every other readiness source (result, stop, shutdown, socket)
                # wakes the selector; only the coordinator's own host deadline
                # needs a timeout. If this process could not own the C-level
                # signal wake (run off the main thread), keep a short slice so
                # an assignment-only handler is still observed.
                wait = max(0, (deadline - now) / 1000000)
                if not self._wakeup_owned:
                    wait = min(wait, 0.01)
                for key, mask in self.selector.select(wait):
                    conn = key.fileobj
                    if conn is self.wake_read:
                        try:
                            self.wake_read.recv(4096)
                        except BlockingIOError:
                            pass
                        continue
                    if conn is self.listener:
                        client, _ = self.listener.accept()
                        client.setblocking(False)
                        credentials = client.getsockopt(
                            socket.SOL_SOCKET, socket.SO_PEERCRED, 12
                        )
                        _, uid, _ = struct.unpack("3i", credentials)
                        if uid != os.getuid() or len(self.connections) >= 2:
                            client.close()
                            raise ValueError("peer uid or connection capacity")
                        self.connections[client] = {
                            "peer": 0,
                            "out": deque(),
                            "mask": selectors.EVENT_READ,
                        }
                        self.selector.register(client, selectors.EVENT_READ)
                    else:
                        if mask & selectors.EVENT_WRITE:
                            self._flush(conn)
                        if conn not in self.connections:
                            continue
                        if mask & selectors.EVENT_READ:
                            packet, flags = self._receive_packet(conn)
                            if not packet:
                                self._disconnect(conn)
                                continue
                            if flags & socket.MSG_TRUNC:
                                raise ValueError("oversized packet")
                            f = p.decode(packet)
                            entry = self.connections[conn]
                            if not entry["peer"]:
                                if f.kind != 1 or f.payload["role"] not in (2, 3):
                                    raise ValueError(
                                        "first frame must identify SDK role"
                                    )
                                role = f.payload["role"]
                                if role in self.peers:
                                    raise ValueError("duplicate peer role")
                                entry["peer"] = role
                                self.peers[role] = conn
                            # Hand over the exact received packet: the
                            # coordinator reuses it for duplicate identity and
                            # its own detached decode instead of re-encoding.
                            self._advance(Event("frame", entry["peer"], f, wire=packet))
                    if self.coordinator.state in ("Failed", "Closed"):
                        break
            if self.coordinator.state == "Closed":
                # Keep both authenticated endpoints alive until their owners
                # consume the proof and close. Never use a fixed delay as proof.
                end = host_us() + self.coordinator.b.phase_timeout_us
                required = {2} if self.coordinator.runtime_only else {2, 3}
                while required and host_us() < end:
                    remaining = (end - host_us()) / 1000000
                    if remaining <= 0:
                        break
                    for key, mask in self.selector.select(remaining):
                        conn = key.fileobj
                        if conn is self.wake_read:
                            try:
                                self.wake_read.recv(4096)
                            except BlockingIOError:
                                pass
                            continue
                        if conn is self.listener:
                            continue
                        if mask & selectors.EVENT_WRITE:
                            self._flush(conn)
                        if conn not in self.connections:
                            continue
                        if mask & selectors.EVENT_READ:
                            packet, _ = self._receive_packet(conn)
                            if packet:
                                f = p.decode(packet)
                                # Normal-phase output already in flight when
                                # cancellation arrived is never a stop proof.
                                if (
                                    f.run_id != self.coordinator.b.run_id
                                    or f.control_step > self.coordinator.step
                                    or f.phase >= 4
                                ):
                                    raise ValueError(
                                        "unexpected output after shutdown commit"
                                    )
                                continue
                            peer = self.connections[conn]["peer"]
                            if self.connections[conn]["out"]:
                                raise ValueError(
                                    "peer disconnected before proof delivery"
                                )
                            self.selector.unregister(conn)
                            del self.connections[conn]
                            self.peers.pop(peer, None)
                            conn.close()
                            required.discard(peer)
                if required:
                    self.coordinator.fail("stopped-unproven: SDK close deadline")
            return 0 if self.coordinator.state == "Closed" else 1
        except (OSError, ValueError, queue.Full) as exc:
            actions = self.coordinator.fail(str(exc))
            try:
                self._actions(actions)
            except (OSError, ValueError, queue.Full):
                pass
            return 1
        finally:
            if self._wakeup_owned:
                _restore_wakeup_fd(self._wakeup_previous, self._wakeup_identity)
                self._wakeup_owned = False
            self._evidence(
                "runner_final",
                stop_proven=self.coordinator.stop_proven,
                failure=self.coordinator.failure,
                # An unresolved overlap means the failed experiment
                # may have advanced one physics step; never hidden.
                physics_in_flight=self.coordinator.physics_in_flight,
            )
            self.record.close()
            self.stop.set()
            # Bounded logical cancellation, never a physical-stop assertion.
            if self.worker is not None:
                self.worker.join(0.05)
            for conn in self.connections:
                conn.close()
            if self.listener is not None:
                self.listener.close()
            self.selector.close()
            self.wake_read.close()
            # The worker may outlive the bounded join; close its wake writer
            # under the same lock so no send can touch a reused descriptor.
            with self.wake_lock:
                self.wake_write.close()
            if self.directory is not None:
                try:
                    info = os.stat(
                        self.path.name, dir_fd=self.directory, follow_symlinks=False
                    )
                    if info.st_ino == self.socket_inode and stat.S_ISSOCK(info.st_mode):
                        os.unlink(self.path.name, dir_fd=self.directory)
                except FileNotFoundError:
                    pass
                finally:
                    os.close(self.directory)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bootstrap", required=True, help="normalized JSON from independent launcher"
    )
    parser.add_argument(
        "--socket", required=True, help="path inside existing private0700 directory"
    )
    args = parser.parse_args()
    runner = Runner(load_bootstrap(args.bootstrap), args.socket)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, runner._notify_shutdown)
    result = runner.run()
    if result:
        print("simulation failed: " + runner.coordinator.failure)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
