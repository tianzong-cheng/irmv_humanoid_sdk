# Runtime protocol (version 1)

The wire contract between `robot_runtime` and a supervision client. Everything
in this document is frozen for the phase: the topics, the QoS, the field layout,
the identity rules and the timing semantics are the ones the implementation and
its real-DDS tests enforce. The IDL is `protocol/execution.idl` (module
`irmv::protocol::wire`), the C++ mirror is `include/irmv/protocol/types.hpp`.

## Encoding and bounds

* `protocol_version` is `1`. A packet with any other value is rejected
  (`VersionMismatch`) and never changes runtime state.
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
  keep-last-1: a stale target or state is worthless.
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
| `protocol_version` | `unsigned long` | 1 |
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
| `protocol_version` | `unsigned long` | 1 |
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
| `protocol_version` | `unsigned long` | 1 |
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
| `protocol_version` | `unsigned long` | 1 |
| `supervisor_id`, `session_id`, `runtime_id` | `Id` | the owner of the session and the instance it talks to |
| `heartbeat_seq` | `unsigned long long` | **strictly increasing**; an equal or older value does not refresh the lease |
| `stamp_us` | `unsigned long long` | sender stamp |

### `RobotState` — `irmv/v1/robot_state`

Published by the runtime as the machine state of record.

| Field | Type | Meaning |
|---|---|---|
| `protocol_version` | `unsigned long` | 1 |
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
`rx_stamp_us`. `FaultDetail` carries `reason`, `joint_index`, `error_code`,
`observed_value`, `limit`, `sample_stamp_us`, `detected_stamp_us`,
`output_action` and a 256-byte `message`; a joint index that names no joint is
the all-ones value.

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
| `VERSION_MISMATCH` | `protocol_version` is not 1 |
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

The command deadline is deliberately anchored to the transport receive stamp,
not to the sender's `stamp_us`: a delayed packet is judged by when the runtime
received it. A stalled producer therefore costs the session its target, not its
session; a dead supervisor costs it the session.

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
