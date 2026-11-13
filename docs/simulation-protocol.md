# Local simulation bridge protocol v1

This engine-neutral contract is separate from frozen DDS protocol v2. It adds
no DDS topics or fields. A local socket endpoint, coordinator and independent
engine runner are implemented; SDK loop/inference wiring remains Task 7.
All participant queues must be bounded to 32 frames;
this codec does not own a queue. A packet is at most 65,536 bytes, including its
header. A phase inventory has at most 64 manifests, independently of the
32-frame transport queue. Joint arrays have at most 64 entries. All encode/decode operations in
C++ use fixed storage and are `noexcept`, without allocation, locks, or I/O.

## Packet representation

All integers are unsigned, little endian. Doubles are IEEE-754 binary64 little
endian. Booleans are exactly one byte, 0 or 1. There is no alignment padding,
terminator, native structure copying, CDR encapsulation, or trailing data.
IDs are exactly 16 raw bytes (`protocol::Id`); SHA-256 digests are 32 raw bytes,
not hexadecimal ASCII. No codec rule treats a zero ID as a missing identity.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u32 | Complete packet byte count, including this field |
| 4 | u16 | Local protocol version, exactly 1 |
| 6 | u16 | Payload type, 1 through 16 |
| 8 | byte[16] | Run ID |
| 24 | u64 | Control step index |
| 32 | u32 | Immutable phase identifier |
| 36 | u64 | Logical time, microseconds |
| 44 | u64 | Physics dt, microseconds |
| 52 | u64 | Control dt, microseconds |
| 60 | typed payload | Fields below, in declaration order |

The header reuses `StepIdentity`: run_id, control_step, phase, logical_us.
Both dt values are positive; physics dt divides control dt. Logical time is
exactly `1 + control_step * control_dt_us`, with checked overflow. This is the
Task 2 clock origin contract. Phase numbers are opaque u32 identities here;
assigning their state-machine meanings belongs to the endpoint/coordinator
work. Receivers bind DDS business stamps to this immutable phase; host arrival
time is diagnostic only. Codecs do not infer ordering, freshness, authorization,
or completion from timestamps.

## Payloads and API

`include/irmv/simulation/protocol.hpp` is the C++ field-order reference;
`tools/simulation/protocol.py:SCHEMA` gives the identical Python field order.
Nested structures are encoded inline. Fixed numeric arrays contain their
exact number of doubles. `Joints` is a u32 count followed by count doubles;
only active entries are serialized. `Text256` is a u32 byte count followed by
count bytes, at most 256; embedded NULs are permitted. Arbitrary reason bytes
are not implicitly UTF-8 decoded.

| Type | ID | Payload and intent |
| --- | --- | --- |
| Hello | 1 | Role; config/application/engine digests; runtime ID; physics/control dt |
| Feedback | 2 | Physics step, engine time seconds, q/dq/tau, quaternion[4], angular velocity[3], specific force[3], base position[3], base velocity[6], feedback/IMU validity |
| Torque | 3 | Write ID, local accepted flag, state seq, exact command identity, final tau |
| Manifest | 4 | Sender role and one exact DDS message identity |
| Ack | 5 | Actual consumer role, disposition, complete echoed Manifest |
| ReleaseArm | 6 | Exact Tracking request, first command, Applied result, Tracking state identities |
| LeaveReady | 7 | Exact stop request, stop result, stopped state identities |
| LeaveCommitted | 8 | Echoed LeaveReady proof; authorization revoked, session closed, heartbeat resolved flags |
| Abort | 9 | Numeric code and bounded reason bytes |
| AppliedToStep | 10 | Write ID, inclusive physics start index and exclusive physics end index |
| PhaseSeal | 11 | Sender role, u32 manifest_count (0..64), ordered manifest SHA-256 digest[32] |
| PhaseAck | 12 | Actual consumer role, ingress/consumed disposition, echoed PhaseSeal |
| InventoryReady | 13 | Actual consumer role and exact installed PhaseSeal |
| PublishPermit | 14 | PhaseSeal authorized for DDS publication |
| TickPermit | 15 | Role and stage (1 runtime tick; 2 application processing) |
| FirstCommandProof | 16 | Exact outer Request identity and nested first-command identity; auxiliary local evidence, not DDS inventory |

Feedback mirrors Task 1 `engine.Feedback` without converting its coordinate
conventions: q/dq/tau are output-side rad, rad/s, N m; tau is actuator generalized
force, quaternion is wxyz in world, angular velocity and specific force are in
the IMU site frame, base position is world metres, base velocity is world linear
and local angular. The three joint counts must agree. The two validity flags
carry delivery validity; they do not fabricate motor connectivity or ACKs.

`Torque.accepted` means acceptance into the local bounded send path only.
`AppliedToStep` is a distinct engine completion fact, never DDS
`ExecutionResult::Applied`. Its end index cannot precede its start index.

Roles: Coordinator=1, Runtime=2, Application=3. Dispositions: Ingress=1,
BusinessConsumed=2. Neither a Manifest nor a sender's ACK is proof of consumer
completion. The authenticated endpoint role and the expected disposition must
be checked by later barrier code. Ingress reserves the message for the next
runtime tick; it does not assert business execution.

C++ `Payload` is a fixed-capacity `std::variant` of these sixteen structures.
`Frame` contains identity, physics_dt_us, control_dt_us, payload. The variant's
index plus one is the type number. No type-erased byte payload is exposed.

- `bool Encode(const Frame&, ByteBuffer*) noexcept`
- `bool Decode(ByteView, Frame*) noexcept`
- `ByteBuffer`: array of 65,536 bytes and used `size`; `ByteView`: pointer/size.
- `bool MatchesHello(const Hello&, const Hello&) noexcept`: exact equality of
  all three digests, runtime ID and dt values; role is intentionally different
  for different peers. Call this against the expected startup configuration
  after structural decoding. Digest mismatch is a startup rejection, not a
  stateless decode error. Endpoints must also match the header run ID.

Null arguments, malformed lengths, extra bytes, truncation, unknown local
version/type/enums, invalid bools, oversized counts, nonfinite doubles, conflicting
dt, or logical-time overflow are rejected. NaNs are never normalized or given
business meaning. Decoder failure leaves the destination unchanged. Encoder
failure does not provide a usable packet; callers must check its return value.

Python `Frame` is a frozen dataclass with `kind`, `payload` (field-name dict),
`run_id`, `control_step`, `phase`, `logical_us`, `physics_dt_us`, `control_dt_us`.
`encode(Frame) -> bytes` and `decode(bytes) -> Frame` raise ValueError for invalid
frames. Dict keys must exactly match the schema. This module imports only the
Python standard library, not the engine or GPU packages.

## Exact DDS identities and digests

`MessageIdentity` fields, in wire order:
kind(u32), runtime/supervisor/session/producer IDs, request_id(u64), epoch(u64),
proposed_epoch(u64), sequence(u64), source_state_seq(u64), fault_id(u64),
stamp_us(u64), operation/status/reason/execution_state/output_action(u32 each),
content_digest(byte[32]). Message kinds: Request=1, Result=2, State=3,
Heartbeat=4, Command=5.

| DDS sample | Required identity projections |
| --- | --- |
| ExecutionRequest | All four IDs, request_id, epoch=expected_epoch, proposed_epoch, fault_id, operation |
| ExecutionResult | All four IDs, request_id, epoch, fault_id, stamp_us, status, reason, execution_state, output_action |
| RobotState | runtime/session/producer IDs, epoch, sequence=state_seq, source_state_seq=last_applied_command_seq (historical), fault_id, stamp_us, reason=stop_reason, execution_state, output_action |
| SupervisorHeartbeat | runtime/supervisor/session IDs, sequence=heartbeat_seq, stamp_us |
| JointCommand | runtime/session/producer IDs, epoch, sequence=command_seq, source_state_seq, stamp_us |

Unused projections are zero. Values such as status=Applied (DDS value 0) are
preserved literally, with no sentinel interpretation. Identity projections
support matching; the digest binds **all** content, including first_command,
feedback, faults, layout/config, and state last_applied_command_seq. A request
ID alone cannot identify a result: status, reason and resulting execution state
are included, and the digest distinguishes any other changed content.

Content digest recipe for future DDS barrier workers: SHA-256 of a canonical
unpadded traversal of the frozen `protocol/execution.idl` sample, in IDL field
order, recursively. Scalars use the LE encodings above, DDS enums use u32,
IDs use their 16 bytes. Count-bounded text/joint/feedback/fault arrays encode the
u32 count and **only active elements**; JointVector encodes count once and then
q, dq, tau_ff, kp, kd active arrays in that order. Fixed vectors always encode
all entries. Ignore inactive storage tails. No CDR header or host padding is
hashed. Reject invalid counts/nonfinite values before digesting. The codec
transports the supplied digest; it does not serialize DDS samples or compute
SHA-256. Producer and actual consumer must independently compute and compare
this digest before generating a completion ACK.

A Manifest represents one exact sample. A **PhaseSeal** explicitly closes its
sender's inventory for the run/control_step/phase in its frame header, including
an empty inventory (`manifest_count=0`). `kMaxPhaseManifestEntries=64` in C++ and
`MAX_PHASE_MANIFEST_ENTRIES=64` in Python are distinct from queue capacity 32:
one runtime tick may emit 32 results plus its state. Counts over 64 are rejected
on encode and decode. A seal has exactly 40 payload bytes; a PhaseAck has 48.
Types 1..10 keep their existing wire representation; these append types 11/12
within the new local v1. Frozen DDS is unchanged.

The ordered inventory digest is precisely
`SHA256(C(M[0]) || C(M[1]) || ... || C(M[count-1]))`. Each `C(M)` is the
180-byte **Manifest payload**: sender role as LE u32, followed by the complete
176-byte MessageIdentity in the order above (including the per-sample content
digest). Equivalently, it is bytes `[60:]` of that Manifest's canonical encoded
packet. Exclude the local packet header, counts, length prefixes, padding, ACKs,
and seal itself from the digest input. Preserve sender declaration order; do
not sort by message kind or DDS arrival time. Retransmission of an identical
entry does not add a new inventory entry. Each entry must belong to the same
sender and immutable phase header as the seal. For zero entries the digest is
SHA256 of empty bytes:
`e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`.

A seal identity is its complete frame header's run/control_step/phase,
logical_us, physics_dt_us, control_dt_us plus its sender, manifest_count and
manifest_digest. **PhaseAck** uses that same header (not the later host receive
time), echoes all three PhaseSeal fields in `seal`, and adds the actual
`consumer` and required `disposition` (Ingress=1 or BusinessConsumed=2).
Thus it attests one exact sealed inventory, not a count alone. Count 0 also
requires a consumer PhaseAck; silence does not close an empty phase.

Task 5 owns cryptographic generation and exact inventory validation, including
entry ordering, duplication/conflicts, actual consumer provenance, matching the
required disposition and complete seal identity, and whether all expected
business messages have reached their required stage. The stateless codec
carries all these fields and bounds the count but cannot establish those facts.
No per-sample ACK, last received ACK, sender seal, or merely decodable PhaseAck
is by itself proof of whole-phase business completion.

ReleaseArm must bind the matching run/runtime/session/producer, new epoch,
Locomotion Tracking request, first-command sequence/source state, consumed
Applied result and fresh Tracking state. LeaveCommitted requires runtime proof
of a Damping or Fault stop, revoked motion authorization, and resolved session /
heartbeat obligations. The codec carries these facts; only the later consumer
state machine can establish them. Structural decoding alone never authorizes
release or removal from a barrier.

## Verification

`simulation_codec_test` checks all sixteen frame types, truncation at every byte,
structural failures, and encode/decode with heap allocation forbidden.
`simulation_protocol_test` exchanges independently constructed nonzero fixtures
in both directions with that C++ binary, compares literal `GOLDEN_HEX` packets,
and checks malformed inputs in both languages. No simulator, DDS peer, or GPU
is needed. Run `ctest --test-dir build -R '^simulation_(codec|protocol)_test$'
--output-on-failure`.

## Consumer barrier API (Task 5)

`simulation/barrier.hpp` defines control-thread-owned `PhaseManifest`,
`PhaseReceipt`, and `Barrier`. Inventories contain at most 64 distinct sample
identities, in declaration order. `Add(MessageIdentity)` deduplicates identical
declarations and refuses conflicting content. Producer `Close()` generates the
seal. A receiver sets the expected header, sender, consumer and required stage,
uses `Add(Frame)` for each Manifest, then `Close(Frame)` for the received seal.
These overloads validate the complete immutable phase header and sender;
`Valid()` independently checks count, ordered digest, roles, kinds and closure.
A seal without its complete inventory is never sufficient.

`Identify(const protocol::{ExecutionRequest,ExecutionResult,RobotState,
SupervisorHeartbeat,JointCommand}&, MessageIdentity*) noexcept -> bool` hashes
all canonical content with bounded streaming SHA-256; it returns false for
invalid counts or nonfinite active values. Projection fields follow the table
above exactly, including literal zero enum values. Inactive tails are ignored.
No allocating configuration SHA function is called. `PhaseReceipt` carries
`StepIdentity identity`, both dt values, consumer, disposition and exact message.
It attests a stage, never FSM acceptance.

`Barrier(Role local)` retains at most 32 phase inventories. `Begin(manifest)`
validates closure and monotonic identities; evicted/old phases cannot be
reintroduced. `Observe(receipt)` matches every identity and stage independently;
content conflicts poison the phase. `MakeAck(manifest, Frame*)` is available
only to the actual local consumer, after all required receipts, including an
explicit call for an empty phase. `Complete(manifest)` also requires that ACK.
A remote checker uses `ObserveAck(frame, authenticated_peer_role)` and verifies
the complete header, seal, consumer and required disposition. The caller must
obtain peer role from its authenticated endpoint, never from the ACK itself.

Both frozen transports expose these control-thread APIs:

```cpp
bool ConfigureSimulation(const simulation::LogicalClock&); // once, before Start
bool BeginSimulationPhase(const simulation::PhaseManifest&) noexcept;
bool SimulationPhaseInstalled(const simulation::PhaseManifest&) noexcept;
bool TrySimulationReceipt(simulation::PhaseReceipt*) noexcept;
bool ReportSimulationProcessed(const simulation::MessageIdentity&) noexcept;
bool TrySimulationAck(simulation::Frame*) noexcept;
bool SimulationFailed() const noexcept;
```

The injected clock is borrowed and must outlive the transport. Configuration
allocates simulation-only FIFO storage before Start. The existing sole DDS
worker receives inventories through an SPSC channel. BeginSimulationPhase only
confirms enqueueing. After actually installing the inventory, WorkerUpdate
publishes a separate fixed installation record through a capacity-2 worker-to-
control SPSC channel. SimulationPhaseInstalled drains this channel on the control
owner and returns true only for the exact current closed inventory: complete
run/step/phase/logical time, both dt values, sender, consumer, required disposition,
seal count and ordered digest. Stale notifications are ignored, and installation
notification overflow latches SimulationFailed. This query is nonblocking and
allocation-free; no helper DDS owner or thread is introduced. An empty inventory
also requires this worker observation. Ingress/business PhaseAck is a separate
stage and cannot substitute for installation readiness. The worker independently
computes sample identities, deduplicates before business handoff, and emits Ingress
receipts only after successful storage. A matching sample receives the sealed
phase's logical time, with the clock checked against that phase; host passage,
retransmission and cached reads cannot refresh it. Identical cached resends are
ignored; changed/undeclared or evicted old samples fail closed. A frozen sample
cannot be redeclared for execution in a later phase. There is no DDS helper
thread. Existing stop lanes and frozen DDS QoS/topics are unchanged.

TakeInput/TakeEvents record only dequeue eligibility. After actual business
handling (including explicit rejection), the control thread must call
ReportSimulationProcessed for each exact delivered identity. This drains ingress
receipts and records BusinessConsumed; a false return must be handled/retried
under the host deadline, not treated as completion. TrySimulationAck also drains
ingress receipts. TrySimulationReceipt exposes ingress records to a driver;
it is not a second consumer and must be called on that same control thread.
All simulation methods except the atomic Failed query have the stated single
owner; lifecycle calls must not race them. Keep the current phase until it is
ACKed and its business lanes drained. Enqueue the next phase, then poll
SimulationPhaseInstalled for that exact inventory under a bounded host deadline before authorizing any publication.
Enqueue success, elapsed host time, and even an empty PhaseAck do not prove
installation. Early new-phase traffic can permanently latch failure against the
previous active inventory; no retry/recovery guarantee is provided for sends
before readiness. After readiness, bounded identical retransmission for DDS loss
and host deadlines remain driver responsibilities. Simulation FIFO refusal or
receipt overflow latches failure. Ingress ACK proves bounded admission; a
BusinessConsumed ACK additionally requires explicit processing reports.

Task 7 must wire those reports after actual handling in production loops, using
identities of the samples actually taken, not latest sequence counters. Runtime
publication inventory must enumerate only the returned tick's state and
`results[0:result_count]`, including emitted rejections/duplicate-request replies;
a pending future Applied result is not part of that tick. Per-topic arrival
order and DDS write success cannot close any consumer barrier. Coordinator
scheduling, resend deadlines, endpoint frame relay and authenticated role routing
remain future driver/coordinator obligations. Current production loops are
unchanged. The real-DDS test runs serially on reserved domain 222; the scenario
allocator excludes it (`tests/integration/dds_domains.json`).

## Coordinator scheduling and driver contract (Task 6)

`python3 -m tools.simulation.main --bootstrap NORMALIZED.json --socket PATH`
starts only the independent coordinator/engine runner. It does not start or
signal either SDK process. Parent PATH must already be owned by the current
user with mode 0700. The runner owns a Unix SOCK_SEQPACKET listener, checks
SO_PEERCRED, accepts exactly one Runtime and one Application, bounds each output
queue to 32 packets, and only removes its own socket inode on exit. Existing
files, symlinks and stale sockets are never removed to make startup succeed.

The bootstrap JSON is an explicit normalized manifest, **not sdk.json**. Required
keys are `run_id`, `runtime_id` (32 hex digits each), `config_digest`,
`application_digest`, `engine_digest` (64 hex digits each), `physics_dt_us`,
`control_dt_us`, and `engine_config` (adapter configuration object). Optional
integer microsecond keys: `startup_timeout_us` (120000000), `phase_timeout_us`
(30000000), `max_duration_us` (3600000000, never above one logical hour).
The independent launcher must generate fresh, independent run and simulation
runtime IDs before starting either SDK process, and pass these SAME identities
and normalized digests to all three processes. It must obtain canonical digests
from the C++ loader normalization interface (Task 9); Python never reimplements
canonical configuration JSON. A new run requires new processes/endpoints.
Runtime must use this supplied runtime ID from its first DDS output. Hello does
not wait for or discover a runtime ID from a DDS state.

Both SDK peers send Hello at step=0, phase=0, logical_us=1, with role=Runtime(2)
or Application(3), after their configuration/model/transport preparation is
complete (application policy load/prewarm included). Coordinator replies Hello
with role=Coordinator(1), matching every digest, ID and dt. Engine initialization
and valid initial completed feedback at physics_step=0 are additionally required
before any tick permit. Initial engine time is 0, SDK time starts at 1 us.

New **local v1 only** types (old type numbers/layouts and all DDS remain unchanged):

| Type | Number | Ordered payload fields |
|---|---:|---|
| InventoryReady | 13 | Role consumer; PhaseSeal seal |
| PublishPermit | 14 | PhaseSeal seal |
| TickPermit | 15 | Role role; u32 stage (1 or 2) |
| FirstCommandProof | 16 | MessageIdentity request; MessageIdentity command |

All repeat the exact run/step/phase/time/dt header being authorized. Stage is a
computation permission; numeric phase identifies an inventory or physics
handoff. They are deliberately different concepts:

| Header phase | Owner/inventory | Required consumer completion |
|---|---|---|
| 0 | startup Hello only, step 0 | both process Hellos plus completed engine feedback |
| 1 | runtime state and actual tick results at t | application BusinessConsumed (2) |
| 2 | application requests/heartbeat/command computed at t | runtime Ingress (1), retained for the next normal tick |
| 3 | engine handoff for [t,t+control_dt) | completed engine feedback, exact integer physics step |

For each control step, drivers MUST follow this order:

1. Coordinator sends Runtime Feedback then TickPermit(role=Runtime, stage=1),
   both phase 1 at `1 + step*control_dt_us`. Runtime installs that feedback and
   performs **one** normal tick with the preceding application phase's ingress.
   Runtime computes final torque and all state/results without publishing DDS.
2. Runtime sends Torque, then one Manifest per actual state/result, then
   PhaseSeal, all phase 1. Torque write_id increases; accepted must be true.
   Task 7 enriches the Task 4 backend TorqueFrame with actual state_seq and
   command identity (including the bootstrap runtime_id, even when no command
   is active). The sole state identity must match torque.state_seq and time t;
   initialization/stop/Fault states are still normal states, not release proofs.
3. Coordinator streams manifests to Application, verifies the ordered inventory
   hash/count on seal, and forwards seal. The receiving driver assembles the
   exact PhaseManifest, calls BeginSimulationPhase, then **bounded-polls
   SimulationPhaseInstalled(the exact manifest)**. Only after this succeeds may
   it send InventoryReady(consumer=Application, same seal), phase 1.
4. Coordinator sends PublishPermit(same seal) to Runtime AND
   TickPermit(role=Application, stage=2) to Application, **both phase 1**.
   This stage-2 permission must precede application business processing, not
   wait for its completion ACK. Runtime now publishes/retransmits only its
   buffered exact DDS outputs. Application waits for the exact inventory,
   processes results/state through the real business path, runs its per-tick
   processing and any due inference at frozen t, and buffers all outputs.
   ReportSimulationProcessed uses the actual delivered identities; a false
   handoff is retried with the SAME identity under a host deadline, not a new
   dequeue. Once actual business consumption completes, Application sends
   PhaseAck(consumer=Application, disposition=BusinessConsumed, original seal),
   still phase 1. Receiving state alone is not completion.
5. On the SAME application socket, after that ACK, send the buffered output
   Manifest entries and PhaseSeal under **phase 2**, at the same step/time.
   Coordinator forwards these to Runtime. Runtime calls BeginSimulationPhase,
   waits for exact SimulationPhaseInstalled, then sends InventoryReady under
   phase 2. Coordinator replies PublishPermit to Application under phase 2.
   Only now does Application publish/retransmit its exact DDS outputs.
6. Runtime sends PhaseAck(disposition=Ingress, exact phase-2 seal) only after
   the declared messages are in its bounded ingress. These messages are NOT
   consumed by an extra same-time normal tick. Even an empty application output
   inventory needs seal -> installed -> ready -> permit -> empty phase ACK.
7. Coordinator gives the engine worker the OLD tick's final torque, held across
   `control_dt/physics_dt` substeps. It accepts only completed coherent feedback
   at `(step+1)*substeps`. It reports AppliedToStep to Runtime using the OLD
   step's phase-3 header/write_id and exact [begin,end) physics indices, then
   sends next step's phase-1 Feedback/TickPermit. A target generated at t is
   first usable by the normal runtime tick at t+control_dt, never during the
   physics interval just authorized.

A sender must retain exact output payloads/identities until the corresponding
consumer completion. BestEffort retry after PublishPermit must reuse identity,
payload and original logical timestamps; drivers supply bounded retries for
DDS loss (Task 7). A coordinator control frame is reliable local transport and
is not evidence that DDS was consumed. Per-message Ack is not a substitute for
PhaseAck. Manifest maximum is 64; recent completed phase caches are 32; bounded
socket queues remain 32 independently of inventory size. Exact duplicates are
idempotent without extending deadlines, including members of a full inventory.
Conflicting identities/seals, unexpected roles/phases/runs/digests or late
uncached phases fail the run. A runtime inventory cannot omit its tick state.

`SimulationCoordinator.advance(Event, host_now_us) -> tuple[Action,...]` is pure
with respect to I/O and clocks; it mutates only its owned state. Event fields:
`kind`, `peer` (authenticated 2/3), `frame`, `feedback`, `reason`. Kinds are
`start`, `poll`, `frame`, `feedback`, `engine_error`, `disconnect`, `stop`.
Call start once, poll under bounded host time, deliver worker feedback only
for initialization or an outstanding physics action. Action kinds:
`initialize` (use bootstrap.engine_config), `send` (peer + complete Frame),
`step` (immutable torque tuple + positive substeps), `cancel` (reason).
Unknown actions/events fail; none stands in for a real SDK call or DDS send.
The runner executes these actions; only its sole daemon engine worker creates,
initializes, reads, writes, steps or closes the adapter. Selectors/control/host
phase deadlines continue while GPU work blocks. Integer SDK time never uses
engine time. Adapter checks actual GPU synchronization and capacity each step.
MuJoCo Warp time checks compare incremental float32 additions **exactly**, at
EVERY completed physical substep, before advancing the host physics counter.
The pinned engine requires integer physics_dt_us >= 1000 for its one-hour range;
smaller timesteps fail model validation before CUDA startup. Coordinator's
finite/coherent feedback, integer indices, quaternion and coarse per-handoff
time bound are sanity diagnostics, not execution proof; the adapter's synchronized
device time/capacity checks establish the handoff contract.

Lifecycle extension points are explicit: Handshake, RuntimeTick,
ApplicationPhase, IngressWait, PhysicsAdvance are active normal states;
Stopping/Failed implement fail-closed termination today. RuntimeOnly/Closed and
successful Leave/ZeroTorque shutdown require Task 8 proofs; this runner never
claims successful shutdown from a stop request. ReleaseArm/LeaveReady are
rejected until those proofs exist, and no Tracking state can release the base.
A stop/death/deadline cancels future scheduling, sends best-effort local Abort,
and exits nonzero. A hung GPU thread may remain alive after the bounded join;
Python cannot kill it. This is logical cancellation, with physical stop
**unproven**. Late results after failure do not restart physics. No recovery,
reset, UI or release mechanism is implemented here.


### Fix-round-1 exact provenance and ownership contract

No existing Frame/MessageIdentity layout, DDS layout, 180-byte inventory hash
recipe or receipt semantics changed. `Identify(RobotState)` now projects the
actual `last_applied_command_seq` into that kind's previously unused
`MessageIdentity.source_state_seq` slot. This is historical state metadata, NOT
an assertion that a target is currently active. For Command this slot continues
to mean the command's source state sequence.

Application sends one `FirstCommandProof` (type16) under phase2 for each declared
Tracking Request (operation3), before that phase's seal. Its payload is the exact
outer Request MessageIdentity and `Identify(request.first_command)` computed
from that very buffered request. It MUST NOT add an extra Command Manifest for
a nested command that was never independently published as JointCommand DDS.
Proofs are auxiliary coordinator input: no forwarding, inventory count, hash
member or consumer receipt is synthesized for them. Same header/run/phase/dt
validation applies. At actual runtime Ingress PhaseAck2, coordinator pairs each
proof with its exact sealed Request identity, including outer content digest,
and checks nested runtime/session/producer/proposed_epoch context plus an actual
previously emitted and application-consumed source-state fact. Missing,
conflicting, unsealed or unknown-source proofs fail. At most64 proofs per phase;
phase2 dedup storage accommodates at most134 frames (other phases70). Transport
queue32, inventory64 and completed-phase cache32 remain separate bounds.

For standalone Command manifests, completed PhaseAck2 records the exact identity
and any observed source-state fact. Unknown-source/rejected standalone inputs
are not turned into accepted targets by the coordinator: if a later Tracking
Torque claims one as active, it must have an actual source-state fact. Runtime
safety still decides acceptance, freshness, discontinuity, ownership, and faults.
Coordinator does not reproduce those rules or reinterpret an ingress receipt as
Applied. Candidate evidence is bounded to32 completed application phases, up to
128 command/proof candidates each; the actual state index is bounded to8192,
the runtime's maximum source window. No freshness/time-limit decisions are made
from these caches.

Runtime Task7 MUST set `Torque.command` from its independently observed actual
FSM/CommandGate.current_target(), never from the coordinator's last ACK or the
latest application declaration. At phase1 completion a Tracking output requires
execution_state=Tracking(3), output_action=Tracking(2), exact completed-ingress
command identity (all fields AND content digest), and actual source-state fact.
State runtime/session/producer/epoch must equal the current target's fields;
state's projected historical last-applied sequence must equal target.sequence.
The selected (command,source-state) pair is pinned independently of BOTH phase
cache and candidate cache rotation until replaced or cleared. Holding a target
does not require fake ingress or refreshed business timestamps.

For non-Tracking output, the exact no-active metadata is a MessageIdentity with
kind=Command(5), runtime_id=bootstrap runtime ID, every other ID/digest byte zero,
and every other scalar zero. C++ construction: value-initialize MessageIdentity,
set kind=Command and runtime_id; do NOT Identify(default JointCommand), which
would create a content hash for a nonexistent sample. This sentinel is not a DDS
inventory member. Damping/ZeroTorque/Fault can retain a NONZERO historical state
last-applied sequence with this no-active metadata. No FSM fields are reset or
changed. Clearing also discards previously active candidate evidence; fresh
completed ingress is required to claim an active target again.

All Action frames returned by advance are deeply detached caller-owned payloads.
Mutating a returned Hello, Manifest, PhaseSeal, nested PublishPermit, or the
original input cannot change retained coordinator facts or later ACK matching.

The Warp3.9 `_next_time` kernel uses float32 time and timestep and one binary32
addition per physical substep. Host prediction rounds the sum of those binary32
operands back to binary32, with exact comparison (no ULP allowance). For dt>=1ms,
one logical hour entails at most3.6million additions. Even bounding each rounding
error below8192s by half its largest float32 ULP (2^-12s), and timestep conversion
by relative2^-24, total diagnostic time stays below
3600*(1+2^-24)+3600000*2^-12 < 4480s < 8192s. Thus that bound is self-consistent;
minimum timestep exceeds two ULPs in that range and no addition can stall.
Exact validation rejects a single missed update, including the review's
100us/1024s one-ULP arithmetic probe; that timestep itself is now explicitly
outside this engine's startup range. This restriction belongs to the pinned
Warp adapter, not the generic integer SDK clock or a claim about all engines.
Short real-GPU checkpoint probes verify the pinned addition behavior; injected
missing clock updates fail before host physics count advancement. These probes
are not an hour-long rollout, malicious-peer authentication, motor execution,
or a substitute for all numerical/physical validation.
