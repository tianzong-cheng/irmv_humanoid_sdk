# Local simulation bridge protocol v1

This engine-neutral contract is separate from frozen DDS protocol v2. It adds
no DDS topics or fields. No socket, coordinator, runtime driver, or inference
code is implemented here. All participant queues must be bounded to 32 frames;
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
| 6 | u16 | Payload type, 1 through 12 |
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

C++ `Payload` is a fixed-capacity `std::variant` of these twelve structures.
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
| RobotState | runtime/session/producer IDs, epoch, sequence=state_seq, fault_id, stamp_us, reason=stop_reason, execution_state, output_action |
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

`simulation_codec_test` checks all twelve frame types, truncation at every byte,
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
bool TrySimulationReceipt(simulation::PhaseReceipt*) noexcept;
bool ReportSimulationProcessed(const simulation::MessageIdentity&) noexcept;
bool TrySimulationAck(simulation::Frame*) noexcept;
bool SimulationFailed() const noexcept;
```

The injected clock is borrowed and must outlive the transport. Configuration
allocates simulation-only FIFO storage before Start. The existing sole DDS
worker receives inventories through an SPSC channel, independently computes
sample identities, deduplicates before business handoff, and emits Ingress
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
ACKed and its business lanes drained. Install the next phase before sending its
samples; any early packet is refused, so bounded identical retransmission and
host deadlines remain driver responsibilities. Simulation FIFO refusal or
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
