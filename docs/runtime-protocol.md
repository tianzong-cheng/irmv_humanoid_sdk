# Runtime protocol (version 2)

The wire contract between `robot_runtime` and a supervision client. Everything
in this document is frozen for the phase: the topics, the QoS, the field layout,
the identity rules and the timing semantics are the ones the implementation and
its real-DDS tests enforce. The IDL is `protocol/execution.idl` (module
`irmv::protocol::wire`), the C++ mirror is `include/irmv/protocol/types.hpp`.

The [external producer interface](#external-producer-interface) below is a
second, independent contract between the *application* and a producer process,
frozen in the same module: `protocol/producer.idl` and
`include/irmv/protocol/producer.hpp`. It does not change any of the five
messages or topics on this page.

## Encoding and bounds

* `protocol_version` is `2`. A packet with any other value is rejected
  (`VersionMismatch`) and never changes runtime state. Version 2 adds the
  `condition` field to `FaultDetail`; the topic set, QoS and the remaining
  message layouts are unchanged.
* Up to **64 joints** per packet. Collections are fixed-size IDL arrays
  (`JointValues[64]`, `JointFeedback[64]`, `FaultDetail[64]`, `Text64Bytes[64]`,
  `Text256Bytes[256]`) plus an explicit count, so a receiver never allocates for
  an inbound sample and the count is validated against the configuration layout.
* Identities are 16 raw bytes (`Id`). An **all-zero id means "none"**: an
  absent session, producer or runtime filter.
* Text is a count plus fixed bytes: 64 bytes for names/layout ids, 256 bytes for
  reason messages, counted and bounded in **UTF-8 bytes**, not code points.
* Every struct is `@final`; a field that is not listed below does not exist, and
  an unknown joint index outside the effective count is rejected.

## Topics and QoS

Five topics carry the whole contract. The QoS values are frozen; the runtime
creates its readers and writers with exactly these policies, and
`dds_codec_test` reads them back from the live endpoints.

| Topic | Direction (runtime) | Reliability | Durability | History | Resource limits (max samples / instances / per instance) | `max_blocking_time` |
|---|---|---|---|---|---|---|
| `irmv/v1/execution_request` | reads | RELIABLE | VOLATILE | KEEP_LAST 32 | 32 / 1 / 32 | 10 ms |
| `irmv/v1/execution_result` | writes | RELIABLE | VOLATILE | KEEP_LAST 32 | 32 / 1 / 32 | 10 ms |
| `irmv/v1/joint_command` | reads | BEST_EFFORT | VOLATILE | KEEP_LAST 1 | 1 / 1 / 1 | – |
| `irmv/v1/robot_state` | writes | BEST_EFFORT | VOLATILE | KEEP_LAST 1 | 1 / 1 / 1 | – |
| `irmv/v1/supervisor_heartbeat` | reads | BEST_EFFORT | VOLATILE | KEEP_LAST 8 | 8 / 1 / 8 | – |

Notes:

* Durability is VOLATILE on every channel: a late joiner is never replayed old
  samples. Nothing survives a restart on the wire, which is why a restarted
  runtime is a new instance and never resumes a previous session.
* The two reliable channels are the request/answer lane, where a lost answer
  must be recoverable by a retry. The status lanes stay best-effort and
  keep-last-1: a stale target or state is worthless. The heartbeat lane is
  best-effort as well but is **queued, not replaced**: the runtime hands the
  state machine every received heartbeat in receive order, up to the eight a
  tick can consume, because only the machine can tell the owner's heartbeat from
  a foreign or reordered one. A full lane refuses the newest sample and counts
  it instead of disposing of an older one.
* `max_blocking_time` is 10 ms on the reliable channels only; every other
  channel never waits, and the control loop never blocks on DDS at all.
* The domain comes from `dds.domain` (or `--domain`), the participant is bound
  to `dds.interface`. CycloneDDS caches the configuration of a domain inside the
  process and the first creator wins; an interface that does not exist makes
  participant creation fail instead of silently using another one.

## Messages

### `ExecutionRequest` — `irmv/v1/execution_request`

The only way a supervisor changes anything. One operation per request, answered
by exactly one `ExecutionResult`.

| Field | Type | Meaning |
|---|---|---|
| `protocol_version` | `unsigned long` | 2 |
| `supervisor_id` | `Id` | the requesting supervisor; the issued session is bound to it |
| `request_id` | `unsigned long long` | request identity, unique per supervisor; the retry key |
| `session_id` | `Id` | the session the request acts on; **empty for `AcquireSession`** |
| `runtime_id` | `Id` | the live instance; any other value, the empty id included, is `WrongInstance` |
| `expected_epoch` | `unsigned long long` | the authorization watermark the requester believes is current |
| `proposed_epoch` | `unsigned long long` | the new watermark a `Tracking` request proposes |
| `producer_id` | `Id` | the source of the target commands (`Tracking` only) |
| `fault_id` | `unsigned long long` | the fault a `Reset` clears |
| `operation` | `Operation` | see below |
| `first_command` | `JointCommand` | the first target frame of a `Tracking` request; every other operation leaves it unused, and an empty one (`joints.count == 0`) is accepted for them and ignored by the runtime |

### `ExecutionResult` — `irmv/v1/execution_result`

| Field | Type | Meaning |
|---|---|---|
| `protocol_version` | `unsigned long` | 2 |
| `supervisor_id`, `request_id` | `Id`, `unsigned long long` | echo of the answered request |
| `session_id` | `Id` | the session the answer belongs to |
| `runtime_id` | `Id` | the runtime instance that answered |
| `execution_state` | `ExecutionState` | state after the request |
| `output_action` | `OutputAction` | action the runtime applied with it |
| `producer_id` | `Id` | authorizing producer, empty when none |
| `epoch` | `unsigned long long` | the watermark after the request |
| `fault_id` | `unsigned long long` | latched fault, 0 when none |
| `stamp_us` | `unsigned long long` | runtime monotonic stamp |
| `status` | `ResultStatus` | `Applied` or `Rejected` |
| `reason` | `RejectReason` | `REJECT_NONE` when applied |

`Applied` means **the host send path accepted the output that implements the
request**. It is never a device acknowledgement; this phase has no device.

### `JointCommand` — `irmv/v1/joint_command`

The continuous target lane, one producer per session.

| Field | Type | Meaning |
|---|---|---|
| `protocol_version` | `unsigned long` | 2 |
| `runtime_id`, `session_id`, `producer_id` | `Id` | routing identity; all three must match the authorized session and producer |
| `epoch` | `unsigned long long` | must equal the current watermark (`EpochMismatch` otherwise) |
| `command_seq` | `unsigned long long` | strictly increasing per producer; older or repeated values are `StaleRequest` |
| `source_state_seq` | `unsigned long long` | the state snapshot the target was computed from |
| `stamp_us` | `unsigned long long` | producer stamp |
| `layout_id` | `Text64` | joint layout the target is expressed in; must match the configuration |
| `joints` | `JointVector` | `count` plus `q`, `dq`, `tau_ff`, `kp`, `kd` arrays |

### `SupervisorHeartbeat` — `irmv/v1/supervisor_heartbeat`

| Field | Type | Meaning |
|---|---|---|
| `protocol_version` | `unsigned long` | 2 |
| `supervisor_id`, `session_id`, `runtime_id` | `Id` | the owner of the session and the instance it talks to |
| `heartbeat_seq` | `unsigned long long` | **strictly increasing**; an equal or older value does not refresh the lease |
| `stamp_us` | `unsigned long long` | sender stamp |

### `RobotState` — `irmv/v1/robot_state`

Published by the runtime as the machine state of record.

| Field | Type | Meaning |
|---|---|---|
| `protocol_version` | `unsigned long` | 2 |
| `runtime_id`, `session_id`, `producer_id` | `Id` | live instance, session and authorizing producer |
| `epoch` | `unsigned long long` | authorization watermark |
| `state_seq` | `unsigned long long` | strictly increasing state sequence |
| `stamp_us` | `unsigned long long` | publication stamp |
| `execution_state` | `ExecutionState` | `BOOT`, `ZERO_TORQUE`, `DAMPING`, `TRACKING`, `FAULT` |
| `output_action` | `OutputAction` | `OUTPUT_ZERO_TORQUE`, `OUTPUT_DAMPING`, `OUTPUT_TRACKING` |
| `stop_reason` | `StopReason` | why the last stop was applied |
| `fault_id` | `unsigned long long` | latched fault, 0 when none |
| `last_applied_command_seq` | `unsigned long long` | command sequence of the applied target |
| `healthy`, `damping_eligible` | `boolean` | gate results of the last tick |
| `config_id`, `layout_id` | `Text64` | configuration digest and joint layout |
| `feedback` | `FeedbackSnapshot` | `snapshot_seq`, `stamp_us`, `count`, `joints[64]`, `imu` |
| `faults` | `FaultReport` | `count` plus `details[64]` |

`JointFeedback` carries `valid`, `connected`, `q`, `dq`, `tau_est`,
`rx_stamp_us`, `freq_hz`, `error_code`. `ImuFeedback` carries `valid`,
`orientation_rpy[3]`, `angular_velocity[3]`, `linear_acceleration[3]`,
`rx_stamp_us`. `FaultDetail` carries `reason`, `condition`, `joint_index`,
`error_code`, `observed_value`, `limit`, `sample_stamp_us`,
`detected_stamp_us`, `output_action` and a 256-byte `message`; a joint index
that names no joint is the all-ones value. `reason` remains the aggregate
category that drives the fault priority and latch semantics; `condition` refines
it without changing either. A decoder rejects a condition outside the frozen
set.

## Enums

```
ExecutionState { BOOT=0, ZERO_TORQUE=1, DAMPING=2, TRACKING=3, FAULT=4 }
OutputAction   { OUTPUT_ZERO_TORQUE=0, OUTPUT_DAMPING=1, OUTPUT_TRACKING=2 }
Operation      { ACQUIRE_SESSION=0, REQUEST_DAMPING=1, REQUEST_ZERO_TORQUE=2,
                 REQUEST_TRACKING=3, RESET=4 }
ResultStatus   { APPLIED=0, REJECTED=1 }
```

Reject reasons (frozen set, `REJECT_NONE` for an applied request):

| Reason | Raised when |
|---|---|
| `MALFORMED` | a field violates its bounds: version-independent layout, counts, joint indices, text bounds, missing identity |
| `VERSION_MISMATCH` | `protocol_version` is not 2 |
| `WRONG_INSTANCE` | `runtime_id` is neither empty nor the live instance |
| `WRONG_SESSION` | the addressed session is not the live one (or the operation needs one and none is given), or the request does not come from the supervisor the session is bound to |
| `SESSION_BUSY` | `ACQUIRE_SESSION` while another supervisor owns the session |
| `STALE_REQUEST` | a command or request sequence that is not strictly newer than what was already accepted |
| `REQUEST_CONFLICT` | the same `request_id` of the same supervisor with a different payload |
| `EPOCH_MISMATCH` | `expected_epoch` does not equal the watermark, or `proposed_epoch` does not exceed it |
| `INVALID_TRANSITION` | the operation is not allowed in the current state |
| `INVALID_COMMAND` | the target payload is invalid for the configured layout (unknown or repeated joints, non-finite values, count mismatch) |
| `STALE_SOURCE` | the source state the target was computed from is older than the source-state deadline |
| `DISCONTINUITY` | the first frame, or a jump beyond the configured continuity limits |
| `RECOVERY_NOT_READY` | the request needs the recovery/healthy window that has not elapsed yet |
| `FAULT_MISMATCH` | a `RESET` that does not match the latched fault (instance, session, `fault_id`) |
| `WRITE_FAILED` | the backend refused the torque write implementing the request |
| `QUEUE_FULL` | the bounded handoff queue had no room for the sample |

Fault reasons: `FAULT_NONE`, `BOOT_FAILED`, `FEEDBACK_INVALID`, `DRIVER_ERROR`,
`IMU_INVALID`, `JOINT_POSITION_EXCEEDED`, `JOINT_VELOCITY_EXCEEDED`,
`COMMAND_INVALID`, `FAULT_WRITE_FAILED`, `PROTOCOL_EXHAUSTED`.

Fault conditions (fixed values, `FAULT_CONDITION_NONE=0`):
`FAULT_CONDITION_NONE`, `FAULT_CONDITION_UNSPECIFIED`,
`FAULT_CONDITION_JOINT_DISCONNECTED`, `FAULT_CONDITION_JOINT_FEEDBACK_INVALID`,
`FAULT_CONDITION_JOINT_FEEDBACK_NON_FINITE`,
`FAULT_CONDITION_JOINT_FEEDBACK_STALE`, `FAULT_CONDITION_JOINT_FREQUENCY_LOW`,
`FAULT_CONDITION_JOINT_DRIVER_ERROR`, `FAULT_CONDITION_JOINT_POSITION_EXCEEDED`,
`FAULT_CONDITION_JOINT_VELOCITY_EXCEEDED`,
`FAULT_CONDITION_IMU_FEEDBACK_INVALID`,
`FAULT_CONDITION_IMU_FEEDBACK_NON_FINITE`, `FAULT_CONDITION_IMU_FEEDBACK_STALE`,
`FAULT_CONDITION_FEEDBACK_SHAPE_INVALID`, `FAULT_CONDITION_BOOT_FAILED`,
`FAULT_CONDITION_COMMAND_INVALID`, `FAULT_CONDITION_PROTOCOL_EXHAUSTED`,
`FAULT_CONDITION_PRIMARY_WRITE_FAILED`,
`FAULT_CONDITION_FALLBACK_WRITE_FAILED`.

Stop reasons: `STOP_NONE`, `STOP_REQUESTED`, `COMMAND_TIMEOUT`,
`SOURCE_STATE_TIMEOUT`, `SUPERVISOR_TIMEOUT`.

## Identity, sessions and the authorization watermark

* The runtime has one identity (`runtime_id`) per process run and publishes its
  `config_id` (a digest of the configuration in effect, domain included) and
  `layout_id`. A restarted runtime is a **different instance** with an empty
  session: packets of the previous instance are answered as foreign and never
  re-authorize anything.
* `ACQUIRE_SESSION` is the only bootstrap. It is accepted **only with an empty
  `session_id`** (a bootstrap that carries one is `MALFORMED`), is answered by
  the runtime with the session it created, and binds that session to the
  requesting supervisor. A second supervisor asking for a session while one is
  owned gets `SESSION_BUSY`.
* A retry of the same `ACQUIRE_SESSION` with the same `request_id` from the same
  supervisor returns **the same session** without creating a second one; the
  same retry rule covers the answers of every other operation (a lost result is
  recoverable without re-authorizing anything).
* Every other operation is answered only for the supervisor the session is bound
  to. A request that echoes the session id of another supervisor is answered
  `WRONG_SESSION` before the request record is consulted: it is not recorded as
  that session's newest request, it does not move the request watermark, and it
  changes no authorization - whether it asks for `TRACKING`, a stop or a
  `RESET`. The runtime's own stop is not a request and carries no supervisor
  identity at all (see "Control loop and telemetry").
* The supervisor lease expires **300 ms** after the last accepted heartbeat, and
  only a **strictly increasing** `heartbeat_seq` refreshes it. Losing the lease
  revokes the session.
* The authorization watermark (`epoch`) is monotonic. A `Tracking` request must
  carry `expected_epoch` equal to the published watermark and `proposed_epoch`
  strictly greater than it; the accepted target then carries the new epoch.
  A stop (`REQUEST_DAMPING`, `REQUEST_ZERO_TORQUE`, a timeout, a fault) advances
  the watermark **even when the execution state does not change**, so a target
  captured before the stop can never be replayed after it.
* A stop accepted in a control tick owns that tick: a `TRACKING` request handled
  later in the same tick is answered `INVALID_TRANSITION`, even when it carries
  exactly the watermark the stop produced and a continuous first frame. The rule
  expires with the tick - the next tick authorizes the same frame - and refused
  traffic never triggers it.
* A `TRACKING` request from the producer that already owns the authorization is
  an ordinary guarded re-authorization: `expected_epoch`, `proposed_epoch`, the
  first frame and the continuity bounds decide it, so a component entering a new
  behaviour execution stage moves to a new epoch per the rule above without a
  `DAMPING` stop in between. Producer identity is a component, not a mode. The
  old epoch stops being executable the moment the switch is committed, and a
  true replay is answered from the stored result without re-authorizing.

Across backends, runtime position and speed protection use measured joint
feedback (`JointPositionExceeded` and `JointVelocityExceeded`). Finite tracking
targets are not rejected or clamped merely for their magnitude. Tracking
handoff continuity and torque limits still apply. The application can still
use `target_velocity_limits` to bound initialization trajectories; it is not a
runtime command rejection threshold.

## Timeouts and fallbacks

| Deadline | Measured from | Default | On expiry |
|---|---|---|---|
| command | the **transport receive stamp** of the accepted frame | 100 ms | `Tracking` → `Damping`, session kept, `STOP_COMMAND_TIMEOUT` |
| source state | the stored generation time of the last accepted state sequence | 150 ms | `Tracking` → `Damping`, session kept, `STOP_SOURCE_STATE_TIMEOUT` |
| supervisor lease | the last accepted strictly-increasing heartbeat | 300 ms | session revoked: `Tracking` → `Damping`, `STOP_SUPERVISOR_TIMEOUT`; `Damping`, `ZeroTorque` and a latched `Fault` keep their output |
| feedback | the sample receive stamp | 20 ms | feedback invalid → fault |
| IMU | the sample receive stamp | 20 ms | IMU invalid (only when the configuration requires it) |
| boot grace / healthy window / boot deadline | process start | 1 s / 100 ms / 2 s | boot failure → `FAULT` |
| recovery window | the last fault transition | 100 ms | the `RESET` of a matching fault clears it |

Inside the boot grace window a **required sample that has never been observed at
all** - no receive stamp, no driver error, and the documented default zeros -
keeps the machine in `BOOT` commanding zero torque and publishing
`healthy=false` until the first frame arrives. Waiting for a first frame is
neither health nor a fault: the runtime never fabricates a valid zero
measurement, and a frame that merely *claims* absence while carrying values is
judged by those values like any other observation. Observed driver errors,
non-finite values, an observed disconnection and overspeed fault immediately,
inside the window as well, and a machine that never receives a frame faults when
the grace or the boot deadline expires.

The excuse is **per required input and lasts exactly until its first frame**:
the runtime records whether each joint and the required IMU has ever delivered
a sample, so a joint or an IMU that goes missing *after* it was observed - its
sample reverting to that same all-default shape - is an observed loss and
faults at once, inside the window as well. A first frame that is itself
erroneous never earns a grace tick of its own: it is judged, not excused.

The command deadline is deliberately anchored to the transport receive stamp,
not to the sender's `stamp_us`: a delayed packet is judged by when the runtime
received it. A stalled producer therefore costs the session its target, not its
session; a dead supervisor costs it the session.

## External producer interface

The five messages above are the contract between `robot_runtime` and its
supervisor. An **external producer** speaks a second, independent interface with
the *application*: it never acquires a session, never requests a mode, never
resets and never publishes a supervision heartbeat, and the runtime never learns
that a target came from one. The IDL is `protocol/producer.idl` (the same
`irmv::protocol::wire` module, compiled with `execution.idl` included so the
identity, the bounded texts, `JointCommand` and `StopReason` have exactly one
definition), and the C++ mirror is `include/irmv/protocol/producer.hpp`.

| Topic | Direction | RELIABILITY | DURABILITY | HISTORY | Resource limits | `max_blocking_time` |
|---|---|---|---|---|---|---|
| `irmv/v1/producer/announcement` | producer → application | RELIABLE | VOLATILE | KEEP_LAST 8 | 8 / 1 / 8 | 10 ms |
| `irmv/v1/producer/prepare_request` | application → producer | RELIABLE | VOLATILE | KEEP_LAST 8 | 8 / 1 / 8 | 10 ms |
| `irmv/v1/producer/prepare_result` | producer → application | RELIABLE | VOLATILE | KEEP_LAST 8 | 8 / 1 / 8 | 10 ms |
| `irmv/v1/producer/authorization` | application → producer | RELIABLE | VOLATILE | KEEP_LAST 8 | 8 / 1 / 8 | 10 ms |
| `irmv/v1/producer/stop` | application → producer | RELIABLE | VOLATILE | KEEP_LAST 8 | 8 / 1 / 8 | 10 ms |
| `irmv/v1/producer/velocity_intent` | application → producer | BEST_EFFORT | VOLATILE | KEEP_LAST 1 | 1 / 1 / 1 | – |

The joint-level target is not a seventh message: a producer publishes
`JointCommand` on `irmv/v1/joint_command`, exactly like the initialization
generator and the local policy. These six messages carry only the lifecycle
around it.

| Message | Fields |
|---|---|
| `ProducerAnnouncement` | `protocol_version`, `producer_process_id`, `runtime_id`, `layout_id`, `target_period_us`, `reads_robot_state`, `status`, `reason`, `detail` |
| `PrepareRequest` | `protocol_version`, `producer_process_id`, `prepare_id`, `runtime_id`, `session_id`, `producer_id`, `epoch`, `target_period_us`, `intent` |
| `PrepareResult` | the request's association fields, `status`, `reason`, `detail`, `first_command` |
| `AuthorizationNotice` | `protocol_version`, `producer_process_id`, `prepare_id`, `runtime_id`, `session_id`, `producer_id`, `epoch`, `first_command_seq` |
| `StopNotice` | `protocol_version`, `producer_process_id`, `runtime_id`, `session_id`, `producer_id`, `epoch`, `reason` |
| `VelocityIntentMessage` | `protocol_version`, `producer_process_id`, `seq`, `forward`, `lateral`, `yaw`, `send_stamp_us`, `valid_for_us` |

### Rules

* **The association key** of a preparation is
  `{producer_process_id, prepare_id, runtime_id, session_id}`. A result or a
  notice whose key does not match the preparation the application opened belongs
  to another run and is dropped; an equal `prepare_id` from another process
  instance is never accepted. `prepare_id` is never zero.
* **A registration is taken while the application is idle in `Ready`.** A
  process that announces itself while a locomotion context exists - preparing,
  prepared, authorized or damping out of one - is ignored rather than accepted:
  the frames of a live context belong to the process whose frame the runtime
  adopted, and replacing it would tell that process to stop mid-context. The
  announcement is not lost, because a producer announces itself on its own
  cadence and is taken from the `Ready` that follows. The supervisor refuses the
  replacement itself while a preparation or a candidate of the selected producer
  is in flight, so the rule holds whatever the caller does.
* **Ready means a frame.** A `PrepareResult` may only be `READY` or `FAILED`. A
  `READY` carries the complete first frame that the application will put into
  its `TRACKING` request; a `READY` without one, or with a frame the execution
  codec refuses, is malformed. A `FAILED` carries a reason.
* **The authorization notice follows the runtime's answer**, never leads it. It
  is sent only after the `TRACKING` request was applied and a later state showed
  the context, and its `epoch`/`producer_id` are the ones that request proposed
  and the frame carried. A producer that receives it may publish frames with a
  strictly increasing `command_seq` from the first frame's sequence on.
* **The stop notice is a notice.** It tells a producer that its context ended so
  it can stop computing, but it is not what withdraws the right to publish: the
  runtime withdraws the epoch on its own when the target times out, the
  supervisor heartbeats stop, the context is replaced or the session ends. A
  producer that never receives a stop notice, or that ignores one, is refused by
  the runtime all the same the moment it publishes a frame of a withdrawn epoch.
* **The velocity intent expires.** `valid_for_us` is non-zero, and a producer
  that sees no fresh intent inside it tracks zero. The intent travels inside the
  `PrepareRequest` as well, so a producer prepares for the velocity it is being
  asked to track rather than for the last one it happened to see.
* **Identity fields are not authentication.** `producer_process_id`,
  `producer_id` and the session identify a run so that a stale or duplicated
  sender cannot be mistaken for the current one; they do not prove who sent a
  sample. The default deployment trusts its DDS network, and a deployment that
  admits untrusted participants needs sender authentication of its own.

### The application's side of the lifecycle

`--locomotion-source external` makes the application the party above: it opens
the six producer endpoints, selects and prepares a producer, and asks the
runtime for the authorization the producer will publish under. The sequence is
fixed and each step is gated on the one before it:

1. **Selection.** Every `ProducerAnnouncement` is offered to the supervisor,
   which accepts it only when it is `READY`, carries this protocol version, this
   layout, a `target_period_us` below `runtime.timeouts.command_us` and a
   non-zero process identity. A second producer replaces the first, and the
   replaced one is told to stop.
2. **Preparation.** Once a producer is selected, a session is live and the
   behaviour is `Ready` - the only state a locomotion authorization can be asked
   from - the application sends one `PrepareRequest`, carrying the operator's
   current velocity intent. A preparation that is not answered inside
   `application.external_producer.prepare_timeout_us` is over; a result that
   arrives after that deadline is refused on its own, so nothing depends on the
   caller expiring the preparation first. The producer stays registered.
3. **The candidate.** A `READY` result whose frame has this deployment's layout
   and joint count, every value finite, non-negative gains and a
   `source_state_seq` this deployment has actually published becomes the
   application's candidate. It is not published: it is held as the first frame
   of the next authorization, and it stops being usable after
   `application.external_producer.prepare_timeout_us`, at which point the
   application prepares again.
4. **Authorization.** The operator's locomotion key asks the behaviour to
   propose a `TRACKING` request; the candidate travels inside it as
   `first_command`, under the producer identity the announcement registered. The
   runtime applies it as it applies any other request. Only once a later state
   showed that context does the application send the `AuthorizationNotice`, with
   the epoch the request proposed, the producer identity it named and the
   command sequence of the frame the request carried.
5. **The stream.** The producer publishes the frames from that sequence on. The
   application publishes none: the authorization has exactly one writer, and a
   second sequence interleaved into it would be a discontinuity the runtime's
   command gate is right to refuse.
6. **Revocation.** A stop, a fault, the exit, or the runtime withdrawing the
   context by itself ends the authorization: the application tells the producer
   to stop and drops the preparation and its candidate, so nothing of a
   withdrawn context can prepare the next one. A producer that keeps publishing
   is refused by the runtime from the first frame of the withdrawn epoch.

The application's own registration window,
`application.external_producer.registration_timeout_us`, is a diagnostic: a
producer that has not announced itself by then is reported once
(`producer=missing timeout_us=...`), the loop keeps supervising its link exactly
as before, and a producer that starts later is still selected, prepared and
authorized. Selecting this source therefore needs no producer to be running at
startup, and never falls back to another source.

## Faults

A fault latches. The runtime commands the degraded output of the fault
(`OUTPUT_ZERO_TORQUE`, or damping while it is still eligible) and stays in
`FAULT` until a `RESET` for **this** fault
(`runtime_id`/`session_id`/`fault_id` match) is accepted after a continuous
**100 ms healthy window**; the accepted reset returns the machine to
`ZERO_TORQUE`. A fault is never cleared by a stop, a timeout or the exit of the
process. `damping_eligible` records whether damping may still be used as a
degraded output; once a fault disqualifies damping, the runtime falls back to
zero torque permanently.

## Control loop and telemetry

The control loop runs at `runtime.control_hz` (500 Hz in the fake deployment)
against an absolute deadline, publishes one `RobotState` per tick and reads no
clock inside the state machine. Logging is **not** part of this protocol: the
runtime writes structured records to stderr and to a rotating local log file
(`telemetry.*`), and a blocked log consumer can only cost log records, never
control ticks. Its own diagnostics - including the clean-shutdown summary - are
written without waiting for a reader, so a jammed stderr pipe can cost a
diagnostic but never the safe exit.

The runtime also stops itself when the process is told to leave. That local stop
is **not** a protocol message: it carries no supervisor identity, it is never
answered by an `ExecutionResult`, it never displaces a received stop, and it
never clears a latched fault. It reaches `ZERO_TORQUE` from `DAMPING` and
`TRACKING` only, and the process leaves only after that state and the torque
write of its tick were accepted.

## Recorded runs

Recording is local and is not part of the wire contract. When
`telemetry.recorder.enabled` is true, the runtime and the application each open
an independent recorder under `telemetry.recorder.directory` (a `null`
directory means the log directory), and each writes a run directory of its own.
Neither process waits for the other's disk, and neither stream is a superset of
the other. The application's velocity recorder creates its configured base
directory recursively when it is missing, then its own uniquely named run
directory below it; the runtime recorder still requires its base to exist.

| Writer | Directory | Contents |
|---|---|---|
| `robot_runtime` | `run_<monotonic-microseconds>` | `meta.json`, `index.json`, `chunk_<n>.bin`, `sessions.json`, `faults.jsonl` |
| `application` | `app_run_<UTC>_<process-id>-<counter>` | `meta.json`, `velocity.jsonl`, `index.json`, `policy_sessions.jsonl` |

Both use the configured `telemetry.recorder.sample_hz`, which must divide
`runtime.control_hz` exactly when the runtime records. That shared number is
**not** a shared clock: the runtime records control ticks on its own stride, and
the application records the body-velocity intent it held on its own sampling
deadlines. The two phases start independently, and a dropped or delayed sample
is never replayed. Rows of the two directories are therefore never paired by
row index; association uses the monotonic stamps plus the identities below.

### Runtime run: schema 4

`meta.json` declares `schema_version: 4`, the configuration and layout digests,
the backend, `control_hz`, `sample_hz`, the joint count and names, the units,
the build revision, and the clock identity. A reader that finds schema 3 or any
other value refuses the run by name instead of reinterpreting its bytes; the
chunk layout, the row width and the validity block changed in schema 4.

Each chunk row is one coherent control tick: the published state, the command
in force, the feedback that tick used and the output it produced. The exported
columns and their units are:

| Columns | Meaning and unit |
|---|---|
| `stamp_us`, `state_seq` | controller monotonic time (µs) and published state sequence |
| `execution_state`, `output_action`, `stop_reason` | protocol enums, numeric |
| `fault_id`, `healthy`, `damping_eligible` | latched fault identity and the tick's gate results |
| `command_present`, `producer_id`, `epoch`, `command_seq`, `source_state_seq`, `command_rx_us` | the command identity in force; `command_rx_us` is the host receive time (µs) |
| `feedback_seq`, `feedback_stamp_us`, `feedback_valid`, `imu_valid` | feedback identity and validity |
| `imu_orientation_rpy` | roll, pitch, yaw in that order, rad |
| `imu_angular_velocity` | rad/s |
| `imu_linear_acceleration` | m/s² |
| `imu_rx_stamp_us` | host monotonic receive time of the IMU frame, µs |
| `send_accepted`, `output_valid` | the host send path's answer and whether this tick produced an output |
| `q`, `dq`, `tau_ff` | applied target: rad, rad/s, Nm |
| `kp`, `kd` | gains: Nm/rad, Nm·s/rad |
| `measured_q`, `measured_dq` | measured feedback: rad, rad/s |
| `measured_tau_est` | torque estimate the feedback carried, Nm |
| `output_torque` | torque the output stage asked the backend to apply, Nm |
| `joint_valid` | the raw per-joint validity flag, one byte per joint in the chunk |

The metadata states the limits of what the recorder knows:

* **IMU conventions.** `orientation_rpy` is roll, pitch, yaw in radians and the
  other two vectors are rad/s and m/s², but the driver contract fixes neither
  their axis frame, their axis order, nor whether `linear_acceleration`
  includes gravity. `meta.json` writes those as `unknown`; the recorder does not
  transform or guess at a measurement.
* **Torque estimate.** `measured_tau_est` is the estimate carried by the
  feedback snapshot, not a torque sensor reading. It is not the commanded
  torque (`output_torque`) and not evidence that a motor produced anything.
* **Validity.** `joint_valid` is the raw per-joint flag. It does not replace
  `feedback_valid`, the freshness checks or the per-input observation history:
  a run consumer must still treat a value behind a false flag as not a
  measurement.
* **Send result.** `send_accepted` is the host send path's answer, never a
  device acknowledgement. `output_torque` is what this tick asked the backend
  to apply.

`sessions.json` records the authorization contexts the runtime adopted (runtime,
session, producer, epoch, first-command gains and timestamps). The runtime holds
no model, so its `model_sha256` is always the literal `unknown`; the model
digest belongs to the application and travels in the application's own sidecar.

### Application run: velocity schema 1

The application's `meta.json` declares `schema` = `irmv.application_velocity`,
`version` = `1`, `sdk_git_sha`, the application and runtime configuration
digests, the layout, the sample rate, the loop period and whether sampling was
paced by the deadline, the clock identity, the units and an explicit model
block. `model.status` is `loaded` (local model: the actual file name, path and
SHA-256), `external` (a producer process supplied the stream), `none` (no
locomotion source) or `unloaded` (local source without a loaded model). Only
`loaded` carries a name, path or digest; the others carry `null` rather than a
fabricated model identity.

`velocity.jsonl` has one row per accepted sampling deadline: `stamp_us`,
`sample_seq`, `identity_known`, `runtime_id`, `session_id`,
`latest_state_seq`, `forward` (m/s), `lateral` (m/s) and `yaw` (rad/s). Before
the runtime is discovered, or before a session exists, `identity_known` is false
and the identities are the all-zero value. The index reports the rows, the loss
spans (`queue_full`, `write_failed`) and a final `complete` marker.
`policy_sessions.jsonl` sits in the same `app_run_*` directory and carries the
application's own model/gain/identity facts per confirmed context. Each line
declares `schema` `irmv.policy_session` and `version` 2 (`model.name` was added
in version 2; other versions are refused by the reader).

### Association and clock identity

Both metadata files carry
`clock = {"clock": "linux_clock_monotonic", "boot_id": "<kernel boot id>",
"available": true|false}`. `linux_clock_monotonic` is a per-boot counter, so
stamps are comparable only when the two runs declare the same clock and the same
boot id. Association additionally requires the runtime record's own session
identity (`runtime_id`, `session_id` in `sessions.json`) to appear in an
application row with `identity_known: true`. The tools apply exactly these
rules:

* both metadata files must declare the same boot id, and both clock identities
  must be available;
* rows of another runtime instance, session or unknown identity are excluded
  and counted, never merged;
* if no row matches, association is refused with the reason, and the runtime
  archive is still written without application columns.

There is **no cross-host clock synchronisation**. Two hosts have unrelated
monotonic counters even if their boot-id strings happen to match, and a run
whose boot id could not be read cannot be associated automatically. Nor is
there any retry, interpolation or padding of a missed sample: an absent sample
is a gap, not fabricated history.

### Fault history

`RobotState.faults` remains the protocol's state of record: it reports the
**first latched fault** (one identity and its detail) and does not change until a
matching Reset clears it. `faults.jsonl` is the runtime's separate observation
history of every distinct condition established by the safety checks:

* it is an occurrence stream, not a level: the first appearance of a key is a
  line; a sustained condition is not repeated on later ticks; after it
  disappears for at least one tick, a recurrence is a new line;
* the key is `{condition, joint_index}`; driver-error and write-failure keys
  additionally include `error_code`, so a different driver error is a new
  occurrence while the same one persists;
* each line carries `schema_version: 1`, the fault line's own version (separate
  from the binary chunk schema 4), `record_seq` (the runtime's ordinal),
  the published `runtime_id`/`state_seq`/`fault_id`, the numeric reason and
  condition with their stable names, the full `FaultDetail`, and both the
  human-readable `message` and `message_hex` (the exact raw bytes);
* non-finite `observed_value`/`limit` values are the explicit JSON strings
  `"NaN"`, `"+Inf"` and `"-Inf"`; `null` is never used for a measurement.

The index's `faults` block accounts for what did not become a row:
`generation_dropped` (the observation batch could not enumerate it),
`queue_dropped` (the recorder handoff was full) and `write_failures` (the line
was accepted but could not be written), each with its loss spans in `losses`.
A fault occurrence can therefore be missing while the run is otherwise readable;
the loss table names the record-sequence range and the reason.

The completion marker means the worker finalized the stream: the index carries
`faults.complete: true` and no accepted line failed to write. A killed run has no
finalized marker (its index says `complete: false` and carries an empty fault
loss table), so its fault history must be treated as incomplete even when the
lines that exist are well-formed. The same rule applies to the sample chunks:
`read_record` reports `complete`, `truncated` and `corrupt` separately, and a
chunk that fails its header or payload CRC contributes no rows.

### Reading and exporting

`read_record` dispatches on the run's `meta.json` and prints one JSON object per
row followed by a summary line. It refuses an unknown schema or a metadata file
that declares neither stream rather than guessing. It only reads; it opens no
DDS endpoint.

```bash
# Runtime run: sample and fault rows in [from, to), then the summary.
build/read_record --run logs/run_<id> --from-us 1000000 --to-us 2000000
build/read_record --run logs/run_<id> --summary-only

# Application run: velocity and policy-session rows, then the summary.
build/read_record --run logs/app_run_<UTC>_<unique> --from-us 1000000 --to-us 2000000
build/read_record --run logs/app_run_<UTC>_<unique> --summary-only
```

`--from-us` is inclusive and `--to-us` exclusive, both on the row's monotonic
`stamp_us`; omitting them reads the whole run. Fault rows are filtered by
`detected_stamp_us`; a velocity row by its `stamp_us`. The summary reports
`rows_in_range`, the first and last stamps and sequences, every gap or loss,
`complete`, `truncated` and `corrupt`; non-finite values in rows are the same
explicit strings the fault file uses.

`tools/record_to_npz.py` converts a runtime run through `read_record` (never a
second decoder) into a NumPy archive:

```bash
# Runtime run alone.
python3 tools/record_to_npz.py logs/run_<id> run.npz --reader build/read_record

# Runtime run plus an associated application run.
python3 tools/record_to_npz.py logs/run_<id> run.npz \
  --reader build/read_record \
  --application-record logs/app_run_<UTC>_<unique> \
  --from-us 1000000 --to-us 2000000
```

The window selects the exported runtime, fault and application rows exactly as
`read_record` does. Association and hold-forward are computed over the whole
application recording first, so a runtime row at the start of the window can
still use an application sample taken before it. `velocity_*` arrays carry the
selected application rows (each with its own length, not the runtime row count);
`velocity_hold_forward/lateral/yaw` are attached to the runtime rows and are
`NaN` outside a contiguous recorded segment, including before the first sample,
past one planned period after the last, and across an indexed loss or cadence
gap. `fault_*` arrays likewise have their own length, and a joint-less fault uses
`-1` rather than an object. `association_ok` and `association_reason` state
whether the application stream joined; a refused association still exports the
runtime archive and names the reason.

### What a run does not prove

* `sdk_git_sha` identifies the commit `HEAD` of the build's source tree when
  that tree is the Git root; it does **not** describe a clean or exact source
  tree. Uncommitted changes are not captured. Without Git, a repository root or a
  commit, the value is `unknown`, never a guessed hash.
* No write is fsynced. A crash can lose recent lines and leaves the completion
  marker false; durability is not promised.
* The recorder and the exported stamps describe this host and this boot. They
  are not synchronised with another host, a device clock or a wall clock.
* The Fake backend has no physical-execution evidence: `send_accepted` and
  `output_torque` describe the host send path, never a motor.
