"""Engine-independent lockstep state machine. No clocks, sockets or engine imports.

Only advance mutates state. Each event is consumed atomically; actions are ordered.
The runner owns host time and executes all actions outside this object.
"""

import hashlib
import math
from collections import deque
from dataclasses import dataclass, fields

from . import protocol as p


@dataclass(frozen=True)
class Bootstrap:
    run_id: bytes
    runtime_id: bytes
    config_digest: bytes
    application_digest: bytes
    engine_digest: bytes
    physics_dt_us: int
    control_dt_us: int
    engine_config: dict
    startup_timeout_us: int = 120000000
    phase_timeout_us: int = 30000000
    max_duration_us: int = 3600000000


@dataclass(frozen=True)
class Event:
    kind: str
    peer: int = 0
    frame: p.Frame | None = None
    feedback: object = None
    reason: str = ""
    released: bool = False
    host_operation_start_us: int = 0
    host_operation_duration_us: int = 0
    host_release_duration_us: int = 0
    # Exact received packet for a frame event. When present it is the framing
    # and duplicate-detection authority and supplies the decoded payload; the
    # frame must carry the same identity. Callers that build frames locally
    # leave it None and the coordinator encodes the frame to validate it.
    wire: bytes | None = None


@dataclass(frozen=True)
class Action:
    kind: str
    peer: int = 0
    frame: p.Frame | None = None
    torque: tuple = ()
    substeps: int = 0
    release: bool = False
    reason: str = ""


class SimulationCoordinator:
    def __init__(self, bootstrap: Bootstrap):
        self.b = bootstrap
        if (
            type(bootstrap.physics_dt_us) is not int
            or type(bootstrap.control_dt_us) is not int
            or not 0 < bootstrap.physics_dt_us <= bootstrap.control_dt_us < 2**64
            or bootstrap.control_dt_us % bootstrap.physics_dt_us
            or not 0 < bootstrap.max_duration_us <= 3600000000
            or bootstrap.control_dt_us > bootstrap.max_duration_us
            or not 0 < bootstrap.phase_timeout_us < 2**63
            or not 0 < bootstrap.startup_timeout_us < 2**63
        ):
            raise ValueError("invalid bootstrap time bounds")
        for name, n in (
            ("run_id", 16),
            ("runtime_id", 16),
            ("config_digest", 32),
            ("application_digest", 32),
            ("engine_digest", 32),
        ):
            value = getattr(bootstrap, name)
            if not isinstance(value, bytes) or len(value) != n or not any(value):
                raise ValueError("invalid bootstrap " + name)
        self.hello = {
            name: getattr(bootstrap, name)
            for name in (
                "runtime_id",
                "config_digest",
                "application_digest",
                "engine_digest",
                "physics_dt_us",
                "control_dt_us",
            )
        }
        self.hello["role"] = 1
        self.legacy_shutdown = (
            False  # Offline v1 replay only; live peers require wire v2.
        )
        self.state = "Handshake"
        self.step = 0
        self.phase = 0
        self.deadline = None
        self.host = -1
        self.started = False
        self.peers = set()
        self.feedback = None
        self.torque = None
        self.last_write_id = 0
        self.inventory = []
        self.first_commands = []
        self.seal = None
        self.permitted = False
        self.recent = deque(maxlen=32)  # Completed phase caches, not just 32 frames.
        self.seen = set()
        self.failure = ""
        # Metadata evidence, independent of replay/dedup phase cache rotation.
        self.states = deque(maxlen=8192)  # Runtime's maximum source-state window.
        self.candidates = deque(maxlen=32)  # <=128 targets per completed app phase.
        self.runtime_only = False
        self.shutdown_pending = False
        self.stop_proven = False
        self.release_state = "Fixed"
        self.arm = None
        self.release_events = deque(maxlen=64)
        self.release_event_drops = 0
        self.requests = deque(maxlen=64)
        self.proofs = deque(maxlen=64)
        self.consumed = ()
        self.highest_state_seq = 0
        self.current_target = None  # Pinned (command, actual source state) pair.
        # Overlapped step: [t,t+dt) may already be executing while the phase
        # handshakes for step t finish. Completion and the next tick are held
        # until both proofs and this feedback exist.
        self.physics_in_flight = False
        self.pending_feedback = None
        self.pending_released = False
        self.tick_validated = False
        # Phase-4 stop identity may freeze below the committed engine horizon
        # when the application handshake for that step was abandoned.
        self.stop_step = None

    def frame(self, name, payload, phase=None, step=None):
        step = self.step if step is None else step
        return p.Frame(
            p.KINDS.index(name) + 1,
            payload,
            self.b.run_id,
            step,
            self.phase if phase is None else phase,
            1 + step * self.b.control_dt_us,
            self.b.physics_dt_us,
            self.b.control_dt_us,
        )

    def send(self, peer, name, payload, phase=None, step=None):
        return Action("send", peer, self.frame(name, payload, phase, step))

    def fail(self, reason):
        self.state = "Failed"
        self.failure = reason
        self.stop_proven = False
        self.arm = None
        if self.release_state == "Armed":
            self.release_state = "Fixed"
        return tuple(
            self.send(peer, "Abort", dict(code=1, reason=reason.encode()[:256]))
            for peer in sorted(self.peers)
        ) + (Action("cancel", reason=reason),)

    def enter(self, state, phase):
        if self.seen:
            self.recent.append(self.seen)
        self.seen = set()
        self.state = state
        self.phase = phase
        self.deadline = self.host + self.b.phase_timeout_us
        self.inventory = []
        self.first_commands = []
        self.seal = None
        self.permitted = False
        self.tick_validated = False
        self.stop_step = None

    def feedback_payload(self, feedback=None):
        # Wire payload for the next tick/stop. It is encoded immediately and
        # never retained, so the immutable dataclass field tuples are shared
        # instead of copied by asdict; base_quaternion is recording-only and
        # is not part of the IPC schema.
        feedback = self.feedback if feedback is None else feedback
        payload = {
            field.name: getattr(feedback, field.name)
            for field in fields(feedback)
            if field.name != "base_quaternion"
        }
        payload.update(valid=True, imu_valid=True)
        return payload

    def tick(self):
        self.enter("RuntimeTick", 1)
        self.torque = None
        payload = self.feedback_payload()
        return (
            self.send(2, "Feedback", payload),
            self.send(2, "TickPermit", dict(role=2, stage=1)),
        )

    def check_feedback(self, feedback, expected):
        if feedback.physics_step != expected:
            raise ValueError("unexpected completed physics step")
        for field in fields(feedback):
            if field.name == "physics_step":
                continue
            value = getattr(feedback, field.name)
            if not all(
                math.isfinite(x)
                for x in (value if isinstance(value, tuple) else (value,))
            ):
                raise ValueError("nonfinite feedback")
        if not 0 < len(feedback.q) <= 64 or not len(feedback.q) == len(
            feedback.dq
        ) == len(feedback.tau):
            raise ValueError("feedback joints")
        if any(
            len(getattr(feedback, k)) != n
            for k, n in (
                ("imu_quaternion", 4),
                ("imu_angular_velocity", 3),
                ("imu_specific_force", 3),
                ("base_position", 3),
                ("base_velocity", 6),
            )
        ):
            raise ValueError("feedback shape")
        if feedback.base_quaternion and (
            len(feedback.base_quaternion) != 4
            or abs(sum(x * x for x in feedback.base_quaternion) - 1) > 1e-6
        ):
            raise ValueError("invalid measured base quaternion")
        if expected == 0 and feedback.engine_time_s != 0:
            raise ValueError("initial engine time")
        if (
            self.feedback is not None
            and feedback.engine_time_s <= self.feedback.engine_time_s
        ):
            raise ValueError("engine time did not advance")
        if abs(sum(x * x for x in feedback.imu_quaternion) - 1) > 1e-6:
            raise ValueError("invalid IMU quaternion")
        if self.feedback is not None:
            count = expected - self.feedback.physics_step
            dt = self.b.physics_dt_us / 1000000
            upper = self.feedback.engine_time_s + count * dt
            ulp = 2.0 ** max(-149, math.frexp(upper)[1] - 24)
            # Local handoff bound: each float32 addition rounds by <= half ULP.
            # This is only a sanity diagnostic, never execution evidence.
            # The adapter must verify device completion at every physical substep.
            tolerance = (count + 2) * ulp + count * dt * 2**-23
            if abs(feedback.engine_time_s - upper) > tolerance:
                raise ValueError("engine time inconsistent with physics handoff")
        # EngineAdapter validates incremental float32 accumulation at each substep.
        # Integer physics_step is authoritative, engine time is diagnostic only.

    def source_state(self, command):
        for state in reversed(self.states):
            if state["sequence"] == command["source_state_seq"]:
                return state
        if (
            self.current_target
            and self.current_target[1]["sequence"] == command["source_state_seq"]
        ):
            return self.current_target[1]
        return None

    def commit_ingress(self):
        # A request's nested first command is not a separate DDS sample.
        requests = [m for m in self.inventory if m["kind"] == 1 and m["operation"] == 3]
        if len(requests) != len(self.first_commands):
            raise ValueError("Tracking request requires exact first-command proof")
        targets = [(m, self.source_state(m)) for m in self.inventory if m["kind"] == 5]
        for proof in self.first_commands:
            request, command = proof["request"], proof["command"]
            if request not in requests:
                raise ValueError(
                    "first-command proof is not in sealed request inventory"
                )
            if (
                request["runtime_id"] != command["runtime_id"]
                or request["session_id"] != command["session_id"]
                or request["producer_id"] != command["producer_id"]
                or request["proposed_epoch"] != command["epoch"]
            ):
                raise ValueError("nested first-command context mismatch")
            source = self.source_state(command)
            if source is None:
                raise ValueError("nested first-command source state was never observed")
            targets.append((command, source))
        self.candidates.append(tuple(targets))
        self.requests.extend(m for m in self.inventory if m["kind"] == 1)
        self.proofs.extend(self.first_commands)
        if any(m["kind"] == 1 and m["operation"] in (1, 2, 4) for m in self.inventory):
            self.cancel_arm()

    def bind_tick_target(self, state):
        command = self.torque["command"]
        if state["output_action"] != 2:
            # Exact local sentinel, not Identify(default JointCommand): no DDS
            # sample/content digest exists for a tick with no active output.
            empty = {
                name: (
                    bytes(16)
                    if field == "Id"
                    else bytes(32)
                    if field == "Digest"
                    else 0
                )
                for field, name in p.SCHEMA["MessageIdentity"]
            }
            empty.update(kind=5, runtime_id=self.b.runtime_id)
            if command != empty:
                raise ValueError(
                    "non-Tracking output must name exact no-active-command metadata"
                )
            self.current_target = None
            # Revoked targets cannot be resurrected from an old evidence cache.
            self.candidates.clear()
            return
        if state["execution_state"] != 3 or command["kind"] != 5:
            raise ValueError("Tracking output requires an actual Command target")
        evidence = [self.current_target] if self.current_target else []
        evidence.extend(target for phase in self.candidates for target in phase)
        target = next((target for target in evidence if target[0] == command), None)
        if target is None or target[1] is None:
            raise ValueError(
                "tick target has no completed ingress/source-state provenance"
            )
        if (
            any(
                state[k] != command[k]
                for k in ("runtime_id", "session_id", "producer_id", "epoch")
            )
            or state["source_state_seq"] != command["sequence"]
        ):
            raise ValueError("tick state and actual current_target metadata disagree")
        self.current_target = target

    def cancel_arm(self):
        if self.release_state == "Armed":
            self.arm = None
            self.release_state = "Fixed"

    def stop_tick(self, stop_step=None, feedback=None):
        # stop_step freezes a phase-4 stop below the completed engine horizon.
        # The feedback payload must then be the exact snapshot the peers last
        # saw for that control step: the runtime accepts a repeated frozen
        # sample, never a relabelled measurement.
        if self.feedback is None or 2 not in self.peers:
            return self.fail("shutdown stopped-unproven: runtime/feedback unavailable")
        step = self.step if stop_step is None else stop_step
        self.cancel_arm()
        self.enter("StopTick", 4)
        self.stop_step = stop_step
        self.torque = None
        payload = self.feedback_payload(feedback)
        upper = (
            ()
            if self.runtime_only or self.legacy_shutdown
            else (self.send(3, "ShutdownPermit", dict(role=3), step=step),)
        )
        return upper + (
            self.send(2, "ShutdownPermit", dict(role=2), step=step),
            self.send(2, "Feedback", payload, step=step),
        )

    def engine_action(self):
        # Exactly one immutable step action per control step. The release flag
        # is decided at issue time; an arm proven after this point can only
        # authorize the next interval, never rewrite this already-issued one.
        next_us = (self.step + 1) * self.b.control_dt_us
        if next_us > self.b.max_duration_us or next_us >= 2**64 - 1:
            raise ValueError("logical duration or overflow")
        if self.physics_in_flight:
            raise ValueError("physics already in flight")
        release = self.arm is not None and self.release_state == "Armed"
        if release:
            self.release_state = "Releasing"
        self.physics_in_flight = True
        self.pending_feedback = None
        self.pending_released = False
        return Action(
            "step",
            torque=tuple(self.torque["tau"]),
            substeps=self.b.control_dt_us // self.b.physics_dt_us,
            release=release,
        )

    def start_physics(self):
        # Non-overlapped starts (runtime-only and leave transitions) already
        # hold every proof for this step, so the engine handoff defines the
        # waiting state directly.
        action = self.engine_action()
        self.enter("PhysicsAdvance", 3)
        return (action,)

    def begin_overlap(self):
        # [t,t+dt) starts as soon as the runtime torque is accepted and the
        # runtime phase-1 inventory/seal is verified. The application phase then
        # runs concurrently; complete_step() is the only way to emit the next
        # tick, and it requires both phase completions plus this feedback.
        if self.physics_in_flight or self.torque is None:
            return ()
        if not self.tick_validated:
            self.validate_tick()
        return (self.engine_action(),)

    def complete_step(self, handshake_complete=True):
        # Shared completion of an overlapped in-flight step. A shutdown that
        # abandoned the application handshake must not move the peers' logical
        # clock past the step they were consuming: the phase-4 stop then freezes
        # at this step with the exact feedback snapshot they already hold.
        if not self.physics_in_flight or self.pending_feedback is None:
            raise ValueError("missing in-flight physics completion")
        freeze = self.shutdown_pending and not handshake_complete
        stop_step = self.step if freeze else None
        stop_feedback = self.feedback if freeze else None
        expected = (self.step + 1) * (self.b.control_dt_us // self.b.physics_dt_us)
        applied = self.send(
            2,
            "AppliedToStep",
            dict(
                write_id=self.torque["write_id"],
                physics_step_begin=self.step
                * (self.b.control_dt_us // self.b.physics_dt_us),
                physics_step_end=expected,
            ),
            phase=3,
        )
        self.feedback = self.pending_feedback
        released = self.pending_released
        self.pending_feedback = None
        self.pending_released = False
        self.physics_in_flight = False
        self.step += 1
        if released:
            # Releasing is the overlapped-normal path: the arm was present when
            # the action was issued. Armed means a valid arm proof arrived after
            # that issue but the engine actually released; the completion is
            # still bound to that proven authorization (legacy evidence replay
            # and a worker that reported a real release). No arm means the
            # release claim has no proof.
            if self.release_state not in ("Releasing", "Armed"):
                raise ValueError("unsolicited release")
            self.release_state = "Released"
            if len(self.release_events) == 64:
                self.release_event_drops += 1
            self.release_events.append(("Released", self.step, self.arm))
            self.arm = None
        elif self.release_state == "Releasing":
            raise ValueError("missing worker release completion")
        return (applied,) + (
            self.stop_tick(stop_step, stop_feedback)
            if self.shutdown_pending
            else self.tick()
        )

    def validate_tick(self, step=None):
        step = self.step if step is None else step
        states = [m for m in self.inventory if m["kind"] == 3]
        if (
            self.torque is None
            or len(states) != 1
            or states[0]["sequence"] != self.torque["state_seq"]
            or states[0]["stamp_us"] != 1 + step * self.b.control_dt_us
        ):
            raise ValueError("tick state does not match torque")
        state = states[0]
        if state["sequence"] <= self.highest_state_seq:
            raise ValueError("tick state sequence did not advance")
        self.highest_state_seq = state["sequence"]
        self.bind_tick_target(state)
        self.states.append(state)
        self.tick_validated = True
        return state

    def arm_release(self, v):
        request, command, result, state = (
            v[k]
            for k in (
                "tracking_request",
                "first_command",
                "applied_result",
                "tracking_state",
            )
        )
        if (
            dict(request=request, command=command) not in self.proofs
            or request not in self.requests
            or request["operation"] != 3
            or result not in self.consumed
            or state not in self.consumed
            or state != self.states[-1]
            or self.torque["command"] != command
            or self.torque["state_seq"] != state["sequence"]
            or result["kind"] != 2
            or result["status"] != 0
            or result["execution_state"] != 3
            or result["output_action"] != 2
            or state["execution_state"] != 3
            or state["output_action"] != 2
            or result["request_id"] != request["request_id"]
            or result["supervisor_id"] != request["supervisor_id"]
            or result["stamp_us"] != state["stamp_us"]
            or any(
                result[k] != command[k] or state[k] != command[k]
                for k in ("runtime_id", "session_id", "producer_id", "epoch")
            )
        ):
            raise ValueError(
                "ReleaseArm lacks exact consumed first-policy confirmation"
            )
        if self.release_state == "Released":
            return
        if self.release_state == "Releasing":
            raise ValueError("release already in flight")
        self.arm = v
        self.release_state = "Armed"
        if len(self.release_events) == 64:
            self.release_event_drops += 1
        self.release_events.append(("Armed", self.step, v))

    def leave(self, v):
        request, result, state = (
            v[k] for k in ("stop_request", "stop_result", "stopped_state")
        )
        fault = state["execution_state"] == 4 and state["output_action"] in (0, 1)
        damping = state["execution_state"] == 2 and state["output_action"] == 1

        def absent(kind):
            m = {
                name: (
                    bytes(16)
                    if field == "Id"
                    else bytes(32)
                    if field == "Digest"
                    else 0
                )
                for field, name in p.SCHEMA["MessageIdentity"]
            }
            m.update(kind=kind, runtime_id=self.b.runtime_id)
            return m

        stopped = (
            state in self.consumed
            and state == self.states[-1]
            and state["producer_id"] == bytes(16)
            and self.torque is not None
            and self.torque["state_seq"] == state["sequence"]
        )
        fault_proof = fault and request == absent(1) and result == absent(2)
        damping_proof = (
            damping
            and request in self.requests
            and request["operation"] == 1
            and result in self.consumed
            and result["status"] == 0
            and result["request_id"] == request["request_id"]
            and all(
                result[k] == request[k]
                for k in ("runtime_id", "supervisor_id", "session_id")
            )
            and result["execution_state"] == 2
            and result["output_action"] == 1
        )
        if not stopped or not (fault_proof or damping_proof):
            raise ValueError("LeaveReady lacks actual stopped output/consumption proof")
        self.cancel_arm()
        self.runtime_only = True
        committed = self.send(
            3,
            "LeaveCommitted",
            dict(
                proof=v,
                authorization_revoked=True,
                session_closed=state["session_id"] == bytes(16),
                heartbeat_resolved=True,
            ),
        )
        permit = self.send(2, "RuntimeOnlyPermit", dict(role=2))
        if self.physics_in_flight:
            # The interval for this step was already committed; never issue a
            # second step action. Join it (or close it) without the app phase.
            if self.pending_feedback is not None:
                return (committed, permit) + self.complete_step()
            self.enter("PhysicsAdvance", 3)
            return (committed, permit)
        return (committed, permit) + self.start_physics()

    def advance(self, event: Event, host_now_us: int) -> tuple[Action, ...]:
        if self.state in ("Failed", "Closed"):
            return ()
        if type(host_now_us) is not int or host_now_us < self.host:
            return self.fail("host clock reversed")
        self.host = host_now_us
        if self.deadline is not None and self.host >= self.deadline:
            return self.fail("host phase deadline")
        try:
            # Actions are caller-owned, including every nested payload
            # dictionary. _advance satisfies that by construction: retained
            # protocol dictionaries are shallow-copied before a frame is
            # echoed, and every other payload is freshly built from immutable
            # scalars, so a per-event deep copy is unnecessary.
            return self._advance(event)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as exc:
            return self.fail(str(exc))

    def _advance(self, e):
        if e.kind == "start":
            if self.started:
                raise ValueError("duplicate start")
            self.started = True
            self.deadline = self.host + self.b.startup_timeout_us
            return (Action("initialize"),)
        if not self.started:
            raise ValueError("not started")
        if e.kind == "poll":
            return ()
        if e.kind == "disconnect":
            self.peers.discard(e.peer)
            if e.peer == 3 and self.runtime_only:
                return ()
            return self.fail(
                "stopped-unproven: " + (e.reason or "required peer disconnected")
            )
        if e.kind == "engine_error":
            return self.fail(e.reason)
        if e.kind == "stop":
            if self.shutdown_pending:
                return ()
            self.shutdown_pending = True
            self.cancel_arm()
            if self.physics_in_flight:
                # Never pretend a committed interval did not happen: consume a
                # completed feedback now, otherwise wait for the outstanding
                # engine action before starting the stop phase. The phase-4
                # identity stays at this step unless both application phase
                # handshakes were already complete.
                return (
                    self.complete_step(
                        handshake_complete=self.state == "PhysicsAdvance"
                    )
                    if self.pending_feedback is not None
                    else ()
                )
            return self.stop_tick()
        if e.kind == "cancelled":
            if self.release_state == "Releasing":
                self.release_state = "Fixed"
                self.arm = None
            if not self.shutdown_pending:
                raise ValueError("unsolicited cancellation")
            # The worker refused the queued interval before executing it; no
            # physics step happened, so logical time does not advance.
            self.physics_in_flight = False
            self.pending_feedback = None
            self.pending_released = False
            return self.stop_tick()
        if e.kind == "stopped":
            if self.state != "StopEngine":
                raise ValueError("unsolicited engine stop completion")
            self.stop_proven = True
            self.state = "Closed"
            proof = dict(
                write_id=self.torque["write_id"], state_seq=self.torque["state_seq"]
            )
            peers = (2,) if self.runtime_only or self.legacy_shutdown else (2, 3)
            return tuple(
                self.send(
                    peer,
                    "ShutdownCommitted",
                    proof,
                    step=self.stop_step if self.stop_step is not None else self.step,
                )
                for peer in peers
            )

        if e.kind == "feedback":
            if self.state == "Handshake":
                if self.feedback is not None:
                    raise ValueError("duplicate initial feedback")
                self.check_feedback(e.feedback, 0)
                self.feedback = e.feedback
                return self.tick() if self.peers == {2, 3} else ()
            if not self.physics_in_flight:
                raise ValueError("unsolicited feedback")
            if self.pending_feedback is not None:
                raise ValueError("duplicate engine feedback")
            expected = (self.step + 1) * (self.b.control_dt_us // self.b.physics_dt_us)
            self.check_feedback(e.feedback, expected)
            self.pending_feedback = e.feedback
            self.pending_released = e.released
            # An overlapped step may complete before the application handshake.
            # Hold its feedback (and the next tick) until both phase handshakes
            # are complete. A shutdown abandons the remaining handshake but
            # still records the committed interval.
            if self.state == "PhysicsAdvance" or self.shutdown_pending:
                return self.complete_step(
                    handshake_complete=self.state == "PhysicsAdvance"
                )
            if self.state not in ("RuntimeTick", "ApplicationPhase", "IngressWait"):
                raise ValueError("unsolicited feedback")
            return ()
        if e.kind != "frame" or e.peer not in (2, 3):
            raise ValueError("event or role")
        f = e.frame
        wire = e.wire
        if wire is None:
            wire = p.encode(f)
            detached = None
        else:
            if (
                f is None
                or type(wire) is not bytes
                or not 0 < len(wire) <= p.MAX_PACKET_BYTES
            ):
                raise ValueError("frame wire")
            # The supplied packet is authoritative. Decode and validate it
            # before duplicate detection or any early return can discard it,
            # then require the accompanying frame to carry the same identity.
            detached = p.decode(wire)
            if (
                detached.kind,
                detached.run_id,
                detached.control_step,
                detached.phase,
                detached.logical_us,
                detached.physics_dt_us,
                detached.control_dt_us,
            ) != (
                f.kind,
                f.run_id,
                f.control_step,
                f.phase,
                f.logical_us,
                f.physics_dt_us,
                f.control_dt_us,
            ):
                raise ValueError("frame wire identity")
            f = detached
        if (
            f.run_id != self.b.run_id
            or f.physics_dt_us != self.b.physics_dt_us
            or f.control_dt_us != self.b.control_dt_us
        ):
            raise ValueError("run or dt")
        key = (e.peer, hashlib.sha256(wire).digest())
        if key in self.seen or any(key in phase for phase in self.recent):
            return ()
        if f.kind == p.KINDS.index("ReleaseCancel") + 1:
            if (
                e.peer != 3
                or f.payload["role"] != 3
                or f.phase != 2
                or f.control_step > self.step
            ):
                raise ValueError("invalid release cancellation")
            if f.control_step == self.step:
                self.cancel_arm()
                if (
                    self.release_state == "Releasing"
                    and e.reason != "operation_started"
                ):
                    self.release_state = "Fixed"
                    self.arm = None
            return ()
        if self.shutdown_pending and f.control_step <= self.step and f.phase < 4:
            return ()  # Cancelled normal phase; never used as stop proof.
        if self.runtime_only and e.peer == 3:
            raise ValueError("application membership revoked")
        phase_step = (
            self.stop_step
            if self.phase == 4 and self.stop_step is not None
            else self.step
        )
        if (
            f.control_step != phase_step
            or f.logical_us != 1 + phase_step * self.b.control_dt_us
            or f.phase != self.phase
        ):
            raise ValueError("wrong phase")
        if len(self.seen) >= (134 if self.phase == 2 else 70):
            raise ValueError("phase frame capacity")
        self.seen.add(key)
        if detached is None:
            # Detach caller-owned dictionaries before retaining protocol facts.
            detached = p.decode(wire)
        f = detached
        v = f.payload
        name = p.KINDS[f.kind - 1]
        actions = ()
        if name == "Hello":
            if (
                self.state != "Handshake"
                or e.peer in self.peers
                or v["role"] != e.peer
                or not p.matches_hello(v, self.hello)
            ):
                raise ValueError("hello identity")
            self.peers.add(e.peer)
            actions = (self.send(e.peer, "Hello", dict(self.hello)),)
            if self.peers == {2, 3} and self.feedback is not None:
                actions += self.tick()
        elif e.peer not in self.peers:
            raise ValueError("peer not ready")
        elif name == "Abort":
            raise ValueError("peer aborted")
        elif name == "Torque":
            if (
                self.state not in ("RuntimeTick", "StopTick")
                or e.peer != 2
                or self.torque is not None
                or not v["accepted"]
            ):
                raise ValueError("unexpected torque")
            if (
                len(v["tau"]) != len(self.feedback.q)
                or v["write_id"] <= self.last_write_id
                or v["command"]["runtime_id"] != self.b.runtime_id
            ):
                raise ValueError("torque identity or shape")
            self.torque = v
            self.last_write_id = v["write_id"]
            # The runtime seal may already be verified; torque then closes the
            # safety gate and the interval starts before the app phase.
            if (
                self.state == "RuntimeTick"
                and self.phase == 1
                and self.seal is not None
            ):
                actions = self.begin_overlap()
        elif name in ("ReleaseArm", "LeaveReady"):
            if e.peer != 3 or self.phase != 2 or self.state != "ApplicationPhase":
                raise ValueError("lifecycle proof outside consumed application phase")
            if name == "ReleaseArm":
                self.arm_release(v)
            else:
                return self.leave(v)
        elif name == "FirstCommandProof":
            if (
                e.peer != 3
                or self.phase != 2
                or self.state != "ApplicationPhase"
                or self.seal is not None
            ):
                raise ValueError("first-command proof outside application inventory")
            if len(self.first_commands) == 64 or any(
                proof["request"] == v["request"] for proof in self.first_commands
            ):
                raise ValueError("conflicting or excess first-command proof")
            self.first_commands.append(v)
        elif name in ("Manifest", "PhaseSeal", "InventoryReady", "PhaseAck"):
            if self.state not in (
                "RuntimeTick",
                "ApplicationPhase",
                "IngressWait",
                "StopTick",
            ):
                raise ValueError("inventory outside phase")
            sender = 2 if self.phase in (1, 4) else 3
            consumer = 5 - sender
            if name == "Manifest":
                if e.peer != sender or v["sender"] != sender or self.seal is not None:
                    raise ValueError("manifest role or sealed")
                m = v["message"]
                if m["runtime_id"] != self.b.runtime_id or m["kind"] not in (
                    (2, 3) if sender == 2 else (1, 4, 5)
                ):
                    raise ValueError("manifest identity or kind")
                projection = {k: x for k, x in m.items() if k != "content_digest"}
                if any(
                    {k: x for k, x in old.items() if k != "content_digest"}
                    == projection
                    for old in self.inventory
                ):
                    raise ValueError("conflicting manifest")
                if len(self.inventory) == 64:
                    raise ValueError("manifest capacity")
                # The retained inventory value and the echoed frame must not
                # alias. MessageIdentity values are immutable scalars/bytes,
                # so a shallow copy detaches while the action frame stays
                # caller-owned.
                self.inventory.append(dict(m))
                actions = (
                    ()
                    if self.runtime_only or self.phase == 4
                    else (Action("send", consumer, f),)
                )
            elif name == "PhaseSeal":
                digest = hashlib.sha256(
                    b"".join(
                        p._pack("Manifest", dict(sender=sender, message=m))
                        for m in self.inventory
                    )
                ).digest()
                if (
                    e.peer != sender
                    or v
                    != dict(
                        sender=sender,
                        manifest_count=len(self.inventory),
                        manifest_digest=digest,
                    )
                    or self.seal is not None
                ):
                    raise ValueError("seal mismatch")
                # The echoed frame stays caller-owned: the retained seal is a
                # shallow copy of immutable scalars and digest bytes.
                self.seal = dict(v)
                if self.phase == 2:
                    self.state = "IngressWait"
                if self.phase == 4 or (self.runtime_only and self.phase == 1):
                    state = self.validate_tick(
                        self.stop_step if self.phase == 4 else None
                    )
                    if self.phase == 4:
                        if (
                            state["execution_state"] != 1
                            or state["output_action"] != 0
                            or any(self.torque["tau"])
                        ):
                            raise ValueError(
                                "shutdown stopped-unproven: fresh ZeroTorque required"
                            )
                        self.state = "StopEngine"
                        actions = (
                            Action("stop_output", torque=tuple(self.torque["tau"])),
                        )
                    else:
                        if state["execution_state"] == 3:
                            raise ValueError("RuntimeOnly forbids Tracking")
                        actions = self.start_physics()
                else:
                    actions = (Action("send", consumer, f),)
                    if self.phase == 1:
                        # Verified runtime phase-1 inventory: the application
                        # may now install it while [t,t+dt) already executes.
                        actions += self.begin_overlap()
            else:
                if (
                    e.peer != consumer
                    or v["consumer"] != consumer
                    or self.seal is None
                    or v["seal"] != self.seal
                ):
                    raise ValueError("consumer seal mismatch")
                if name == "InventoryReady":
                    if self.permitted:
                        raise ValueError("conflicting readiness")
                    self.permitted = True
                    actions = (
                        self.send(sender, "PublishPermit", dict(seal=dict(self.seal))),
                    )
                    if self.phase == 1:
                        self.state = "ApplicationPhase"
                        actions += (self.send(3, "TickPermit", dict(role=3, stage=2)),)
                else:
                    if not self.permitted or v["disposition"] != (
                        2 if sender == 2 else 1
                    ):
                        raise ValueError(
                            "completion before install or wrong disposition"
                        )
                    if self.phase == 1:
                        if self.torque is None:
                            raise ValueError("missing tick torque")
                        # Normal path: begin_overlap already validated the tick
                        # and committed [t,t+dt) at the runtime seal. This is
                        # the defensive start if that seal/torque order was
                        # interrupted before the engine action was issued.
                        actions = self.begin_overlap()
                        self.consumed = tuple(self.inventory)
                        self.enter("ApplicationPhase", 2)
                    else:
                        self.commit_ingress()
                        if not self.physics_in_flight:
                            actions = self.start_physics()
                        elif self.pending_feedback is not None:
                            # Both handshakes complete and the engine already
                            # reported t+dt: only now may the next tick start.
                            actions = self.complete_step()
                        else:
                            self.enter("PhysicsAdvance", 3)
                            actions = ()
        else:
            raise ValueError("unsupported control frame")
        return actions
