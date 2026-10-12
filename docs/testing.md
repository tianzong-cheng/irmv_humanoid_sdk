# Testing

Two suites prove this phase: a deterministic core suite that runs as one
process per test with no peer and no network, and a real-DDS suite in which the
runtime and independent client processes really talk over CycloneDDS. The
coverage is stated per test, and the section [What is not covered](#what-is-not-covered)
says explicitly what no test here proves.

## How to run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2

ctest --test-dir build --output-on-failure          # everything (56 tests)
ctest --test-dir build -L core --output-on-failure  # deterministic core suite
ctest --test-dir build -L integration --output-on-failure
ctest --test-dir build -R integration_normal --output-on-failure
```

The whole matrix takes about four minutes on a developer machine
(`ctest -N` reports 23 core and 33 integration entries) and each integration
entry is bounded by its own CTest timeout (30 s for `dds_codec_test`, 180 s for
`runtime_cli_test` and `application_cli_test`, 300 s for `application_client_test`
and per scenario, 600 s for `application_flow_test`).

For manual work the harness of the scenario matrix also runs the whole matrix in
one process, printing one summary line per scenario:

```bash
python3 tools/run_integration.py --all \
    --runtime build/robot_runtime --client build/runtime_client \
    --application build/application --source .
```

The channel handoff is also checked for data races, which no functional test can
prove: a stress run can only show that a torn payload was observed, never that
the interleaving which would produce one cannot happen. The entry that owns the
handoff is run under ThreadSanitizer:

```bash
cmake -S . -B build-tsan -DCMAKE_BUILD_TYPE=Debug -DIRMV_SANITIZE=thread
cmake --build build-tsan --target channel_test -j2
setarch "$(uname -m)" -R ./build-tsan/channel_test
```

`setarch -R` disables address-space randomisation for that process, which
ThreadSanitizer needs on hosts where it refuses to start otherwise (without it
every ThreadSanitizer binary aborts with `unexpected memory mapping`). This run
is not part of CI yet; it is the check that found the eviction race, and
`bounded_channel.hpp` must stay clean under it.

The phase-3 review ran both sanitizers over the deterministic core suite, which
is the shape the plan asks for - targeted concurrency plus the safety paths:

```bash
cmake -S . -B build-sanitize -DCMAKE_BUILD_TYPE=Debug \
      -DIRMV_SANITIZE=address,undefined
cmake --build build-sanitize -j6 && ctest --test-dir build-sanitize -L core
setarch "$(uname -m)" -R ctest --test-dir build-tsan -L core
```

* `address,undefined`: 23/23 pass. The run caught one latent
  heap-use-after-free in `application_fsm_test` (`RequestTimeoutFaults` kept a
  pointer into the sink's request vector across the Step that begins the retry
  and moves it), which the plain build survived only because the freed memory
  still held the old value; the pointer is now copied out.
* `thread`: 22/23 pass. The one failure is `logger_test`, which warns `unlock of
  an unlocked mutex` inside `WriteDiagnosticBlocking` - a `try_lock_for` whose
  failure returns early and whose success is followed by `unlock`, so the warning
  is the known ThreadSanitizer/libstdc++ false positive on `timed_mutex`, not a
  race in this code. `channel_test`, `inference_test` and `application_fsm_test`
  - the two-thread handoff, the inference worker whose wakeup this round fixed,
  and the behaviour - are clean under it.

### DDS domain policy

Every integration entry isolates itself on its **own DDS domain**, always below
the 230 bound the configuration contract enforces:

* The `run_integration.py` harness takes the domain declared by the scenario
  (201–212 for the phase-1 failure matrix, 213–218 for the six application
  scenarios), claims it with an exclusive lock file, and refuses to start unless
  **every RTPS port of that domain** can be bound right now. If the declared
  domain is taken it walks up to the next free one, so two scenarios – or two
  developers – never share a domain. A lock left behind by a killed run is
  reclaimed once its process is gone.
* `tests/integration/runtime_cli_test.py` derives its domain from its own
  process id (`20 + pid % 190`), so parallel runs stay apart, and overrides the
  domain of its configuration file on the command line – the startup line proves
  the override took effect. `application_cli_test` walks inside a band of its own
  (`1 + pid % 19`), one domain per case, which cannot overlap the band the other
  two process-level entries share.
* `application_flow_test.py` claims one domain per case through the same
  `choose_domain` of the harness (imported as a module) and passes the shared
  `integration` directory as the lock directory, so its locks live in the one
  namespace every scenario entry uses and a case and a scenario entry never take
  the same domain at the same time; the `application_normal` case it runs as a
  subprocess claims its own domain the way every scenario entry does.
* Waits are bounded everywhere. On timeout the harness kills what it started
  (ordered shutdown first, a hard kill as the last resort) and fails the entry
  with the collected evidence: the runtime log, the application log, every
  client's JSON-lines observations and stderr, the per-tick output trace, and
  the domain used. A step that did not run is never reported as a pass.

### The application is driven through a terminal, never through a test client

The real `application` binary owns a terminal: it puts it into raw mode and
reads the keyboard from it. Every test that drives it therefore starts it the
way an operator does, on a **pty pair**, and writes the bytes of a key to the
master – the bytes the application reads are exactly the bytes a keyboard would
produce. The harness has an `Application` process class for the scenarios
(`--application BIN`, steps `start_application`, `stop_application`,
`kill_application`, `restart_application`, `wait_application_log`,
`application_keys`) and `tests/integration/application_flow_test.py` has its own
for the cross-process cases.

* stdout and stderr are merged into that terminal (one stream, like the
  runtime's merged log), so the status lines, the exit line and a startup
  rejection all appear in the same evidence file (`application_runN.log`).
* The slave is closed in the parent right after the spawn, so the master
  reports end-of-file the moment the application leaves; on Linux the master
  reports the slave's termios, so `termios_attrs()` still reads the live
  attributes and the harness can prove the terminal was handed back.
* A wait in a scenario starts scanning at the position of the last key that
  scenario wrote, so a status line printed minutes earlier can never satisfy a
  wait that is about the key just sent.
* The application and the runtime of one scenario always share the generated
  `config.json` and the chosen domain – a scenario whose `application` section
  describes another configuration is refused instead of failing later as an
  unexplained handshake – and `runtime_client` is never a stand-in for the
  application.

The scenario entry points are also what CI runs; `--all` is a convenience for
manual runs and is never used by CTest.

## Deterministic core suite (`-L core`, 23 tests)

| Test | What it proves | Design §9 |
|---|---|---|
| `protocol_test` | The frozen wire contract of the core types: every enum validator accepts exactly its own values and rejects everything else, and the command shape rules (joint count against the configured layout, layout-id bounds, non-finite values, negative gains) return exactly the frozen rejection reasons. | "all legal and rejected transitions", "missing/duplicate joints, non-finite intermediate computations" |
| `config_test` | Configuration loading and validation: the shipped configuration is loaded field by field, malformed or contradictory documents are rejected with the offending field path (uniqueness and completeness of joint names, position-range ordering, boot-deadline ordering, minimum frequency versus control rate, state-window coverage), the numeric contract keeps exact decimal tokens, the SHA-256 known-answer vectors hold, and the configuration identity digest (`config_id`) covers the configuration in effect including a command line domain override. | "the configuration is executable and versioned" |
| `fake_backend_test` | The fake backend's contract: snapshot stamps, connection/error reporting, write acceptance and refusal, the fixed output ring, and that the last output reports the latest *attempted* output including a refused one. | Fake backend contract (§8) |
| `safety_test` | The safety gates on feedback and targets: freshness exactly at and beyond every timeout, IMU coherence, control-frequency bounds, driver error codes, position and velocity limits in both directions at and beyond the threshold, multiple joints, non-finite samples, the boot grace waiting for a first frame without calling absence healthy or faulting on it, the observation history that narrows the grace to inputs that never delivered a frame (an observed joint or required IMU that goes missing faults at once, a never-observed one keeps waiting, and a first erroneous frame is judged), the same grace never excusing an observed invalid sample, driver error or overspeed, the priority between simultaneous reasons, the complete fault report, and the recovery window. | "forward/reverse/equal-threshold/multi-joint overspeed", "feedback freshness, IMU coherence, complete error codes, recovery window reset" |
| `command_gate_test` | The command gate: sequence monotonicity, queue wait followed by the transport receive stamp as the validity origin, repeated sequences not refreshing the deadline, the source-state index window (unknown, ignored, stale, fresh), the command and source-state deadlines measured per tick with no new packet, the independent supervisor lease, and that a rejected owner frame refreshes nothing. | "per-tick timeout with no new packet, old packets do not feed the watchdog, source-state window and independent supervisor lease", "single-source authorization" |
| `execution_fsm_test` | The state machine: every legal and rejected transition, boot grace/deadline rules with the per-input observation history (an observed joint or required IMU that goes missing faults inside the grace window, a never-observed one keeps waiting to the deadline), the full normal path, write failure, request idempotency and conflicts, stop ordering and the authorization watermark (including a stop that does not change the state), session takeover and lease expiry, a heartbeat batch that mixes the owner's sample with a foreign or reordered one refreshing the lease, first-frame continuity, foreign packets, fault output degradation latching to zero torque, the recovery window with an explicit Reset, epoch exhaustion, the session's supervisor binding, same-tick stop ownership, a guarded same-producer epoch change, and the runtime's own trusted stop. | "all legal and rejected transitions, boot grace/deadline, explicit Reset", "pure feedforward, PD + feedforward with the total torque limit", "request retry/different payload/out-of-order, atomic first-frame and gain switch, stop preemption", "fault first-tick output, damping disqualification degrades permanently to zero torque" |
| `recorder_test` | The run recorder: a recorder that was never started accepts nothing and creates nothing, while a started one creates exactly one run directory with its metadata and refuses a directory that does not exist; a paced run of 910 samples is written as one full chunk and one remainder, every chunk renamed into place with no partial file left behind, and the index reports the rows, the joint count and each chunk's sequence and time range; a deliberate hole in the published sequence becomes a gap with its exact range; a burst that fills the bounded handoff costs the refused samples, which are counted *and* reported as a gap of their own, because the worker never saw them and they could not show up as a hole between two samples it did write; stopping twice is harmless and keeps naming the run. | §8.2, §8.4 |
| `record_reader_test` | The read side of a recorded run, on bytes the *recorder* really produced rather than on a fixture: a run read whole returns the metadata it declares and every field of every row, a time range returns exactly the rows inside it and reports the gap that overlaps it on the recorded grid (with a stride of five, two missing samples are reported as two, not as the eight ticks between them), an empty range is an empty answer and not an error, a chunk that was cut short is reported as a truncation with the rows it did hold, and a run of another schema version is refused whole instead of reinterpreted. | §8.2, §8.4 |
| `logger_test` | The bounded asynchronous logger: queue bound and drop accounting, level filtering, FIFO order, the drain on stop, an event racing with the stop, the 256-byte message bound, rotation of the default sink, the formatted line and the error path - plus the bounded diagnostic writer the sink and the exit path share, against a pipe the test fills itself. | §8 (bounded telemetry) |
| `channel_test` | The bounded handoff between the control thread and the transport thread: FIFO order, capacity, the untouched state of a refused push, the keep-last-1 handoff of the latest-value channels (a published value is replaced and counted only while no consumer holds it, and a taken one is never counted as a loss), a separate stop lane, and two-thread stress on both shapes whose payloads each carry their own generator, so a torn or overwritten copy fails the test. This is the entry that is also run under ThreadSanitizer; the eviction race it used to hide was found that way. | §2 (thread model), §4 (lane separation) |
| `config_schema_contract_test` | The shipped `config/sdk.schema.json` against the shipped `config/sdk.json`, the custom `irmv-joint-name` format (non-empty, at most 64 **UTF-8 bytes**), and that standard JSON Schema validity and the loader's own checks agree. | "the configuration is executable and versioned" |
| `application_config_test` | The upper layer's own configuration: the shipped `application`, `input` and supervision sections load field by field, the trajectory, retry, heartbeat, state-window and intent deadlines are validated as a whole (ordering, non-zero periods, an initialization deadline that leaves room for the whole stable window after the ramp, the state window staying inside the runtime's own source-state deadline so `fresh` and the initializer's usability rule can never disagree, tolerance bounds, the `emergency_stop` binding being the space character), and a document that would make the upper layer unable to supervise is rejected with the offending field path. | phase-2 configuration contract |
| `robot_client_test` | The official supervisor client as a pure function of the states, results and times it is handed: one request at a time, the retry cadence and the final deadline, a rejection resolving immediately, an `Applied` only confirming against a state published at or after it (both arrival orders, and a terminal refusal after a contradicting state), a reordered, repeated or zero `state_seq` within one runtime instance ignored whole before it can touch the snapshot, the receive age or the session - while a new instance rebuilds the baseline and a legal next frame still witnesses an answer -, the independent lease and instance/session loss, target publication gated on a live session, the epoch proposal never repeating a value, and an abandoned request never resuming. | phase-2 client contract |
| `application_loop_time_test` | The supervision loop's decision instant: it is never earlier than the newest receive stamp of the transport batch the iteration acts on (a newer stamp wins, an older one keeps the clock read after the drain, equal instants agree, and neither end of the unsigned range wraps). This is the structural replacement for the microsecond race that let a legal state be fresh for the client and stale for the initializer. | phase-2 loop contract |
| `application_initialization_test` | The initialization trajectory against the runtime's own formula: the first frame stays inside the continuity bounds and the target velocity limits, a plan the runtime would refuse is refused before anything is sent, completion requires the position and velocity tolerances over the whole stability window and is bounded by the deadline: a move past `trajectory.deadline_us` fails, latches and emits nothing, while a window that completed exactly on the deadline is still `Ready`, and the Ready hold keeps publishing fresh frames or reports the sample unusable without failing. It drives a real `ExecutionFsm` through the runtime fixture, so the generator and the lower layer agree on the same numbers. | phase-2 initialization …
| `application_fsm_test` | The behaviour FSM and the serial supervision order as a pure function over a real `RobotClient` and a recording sink: the full path with its confirmations, the emergency stop latching and preempting, the exit path, `UnexpectedContext` and `RuntimeRestarted` latching, the timeout causes, the Reset clearing the latch only into `Standby` on a fresh `ZeroTorque`, the locomotion source arming from `Ready` and entering `Locomotion` only on the context the runtime granted, a frame of another epoch never published, a late result latching `LocalPolicyFailed`, the source identity taken from the candidate (an external producer is not this application), and the replaced context staying expected while a request is in flight. | phase-2 behaviour contract |
| `producer_supervisor_test` | The external producer supervisor, as a pure decision component: a producer is selected only when it reports itself Ready with this deployment's layout and a period inside the runtime's own target deadline (another version, a failed status, an empty process identity, another layout and a period of zero are all refused), the producer is authorized under its own process identity, one preparation is outstanding at a time with a number that never repeats, a reply of another process, another preparation or another session is ignored rather than applied, the next preparation never reuses a number, a candidate frame must have this deployment's layout and joint count with finite values and non-negative gains, a frame generated from a state this deployment never published is a failure, a replacement drops the previous producer's preparation and candidate and queues its stop notice, the authorization notice names the preparation and the granted identity, a revocation clears both and names the context, the producer stays registered for the next authorization, and `Forget` makes a restarted process register again. | §6.3 |
| `input_router_test` | The keyboard router: one press is one semantic event, the repeat guard a terminal without key-up needs, the batch priority (emergency stop first, velocity intent last), and the single body-velocity intent with its deadline. | phase-2 input contract |

| `deployment_test` | The frozen deployment contract of a policy ONNX: two valid documents (a two-joint one whose three joint lists are in three different orders, and a three-joint one with a non-null action clip, a scaled and a clipped observation term) parse into the contract the SDK implements, `BuildJointMapping` resolves each list against the deployment table by exact name and refuses a model joint the table does not have, and about fifty mutations are refused with the exact JSON field path of their first violation - a missing or unknown field, schema version 2, a wrong policy type, a joint list of the wrong length, a duplicate joint name, a wrong unit, frame, clip or pipeline order, a negative gain, an unsupported normalizer, a null provenance field that `source.missing` does not explain, and a document that is not readable JSON (including the NaN/Infinity tokens and a number that overflows the double range). | §3.4, review focus 1 and 2 |
| `deployment_schema_contract_test` | The same two fixtures against `schema/onnx-deployment-v1.schema.json`: the document that the C++ parser accepts must also satisfy the schema, a value the schema pins must be refused when changed, an undeclared field must be refused, and the frozen facts (the six term positions with their name, unit and frame, the four action stages, the nullable digest pattern, the non-negative gain) must be spelled out in the schema exactly as the parser implements them. | §3.4 |
| `policy_math_test` | The two pure transformations against the frozen fixtures: the observation is the six contracted segments in order, with the command, the gravity projected into the base frame from roll and pitch, the base angular velocity, the two joint terms in *their own* orders with their own references and the previous action; a term's clip is applied before its scale; an invalid IMU snapshot, a joint count that disagrees, an invalid joint sample and a non-finite value are refused. The action applies the frozen pipeline (wrapper clip, scale, offset, term clip) and scatters the result into the deployment's canonical order with the resolved gains; a wrong action count, a non-finite action and a state of another joint count are refused. The gain merge applies only the override set whose complete digest matches, records the model value and the applied value of every difference, and refuses a model that does not command every configured joint. | §3.1-3.3 |
| `onnx_model_test` | The model loader against ten generated models: the valid one loads with its contract, mapping and digest, and one inference returns a finite action; a model with no contract, another schema version, an input width or output name the graph contradicts, a dynamic width, two inputs, an unknown joint or a non-finite output is refused, as is a file that does not exist and a model whose observation is of the wrong width. The fixtures are generated from the frozen document, so no binary blob is checked in. | §3.4, review focus 1 |
| `inference_test` | The inference worker: a new worker runs nothing until something is submitted, one request produces one response that carries the identity of its request (generation, epoch, state sequence and stamp, submission instant), a burst of submissions never blocks and is compressed rather than queued (every accepted request is either replaced or run, and replaced ones are counted), a model that cannot answer produces a failed step rather than a silent zero action, and the judgement of one result is pinned at its boundaries (`JudgePolicyResult`: the whole step identity, the deadline inclusive at the edge and failing one microsecond past it; `PolicySourceStateUsable`: a sequence ahead of the newest state, a zero sequence or stamp, and a source exactly at the runtime's window and one microsecond past it). `Stop()` is idempotent and joins. | §3.4, §8.4 |

All twenty-three run without a peer, without a network and without a wall-clock
deadline: they assert exact tick semantics and counter values, and the only
wall-clock waits in `logger_test` are hang guards.

## Real-DDS suite (`-L integration`, 32 tests)

### Process-level entries

| Test | What it proves |
|---|---|
| `dds_codec_test` | Against a live endpoint: the codec round trip and field-by-field equality of the core types - including the empty, unused first command of a session, stop or reset request and the rejection of an over-long or missing joint vector - codec rejection of malformed samples, that the running transport's readers and writers really carry the frozen QoS (durability, reliability, history depth and kind, resource limits, and `max_blocking_time` on the reliable channels) as read back from the endpoints, and that a peer process sees the published samples. It also proves the transport's own lane policy for heartbeats: a batch that receives a foreign sample after the owner's still hands the owner sample over first, and a lane that is full refuses what does not fit and counts it instead of displacing an older entry. |
| `runtime_cli_test` | The process contract of `robot_runtime`: every startup rejection (missing/malformed configuration, `backend: real`, a broken scenario, unknown argument, out-of-range domain) exits non-zero with a message and without starting DDS; the normal path reaches `ZeroTorque` and exits 0 with a clean shutdown on `SIGTERM`; an unsupervised runtime keeps running; a latched fault keeps running and refuses to leave, and a hard kill is reported as such. |
| `record_export_test` | The recorded run, exported end to end: a real runtime records for four seconds with the recorder enabled, `tools/read_record` reads the run back and `tools/record_to_npz.py` turns it into an archive. The archive must carry every column by name, the joint table in the order the joint columns use, the units, the schema version, the configuration and layout digests and both rates; the row rate must be the configured 100 Hz rather than the 500 Hz control rate; the sampling stride must be constant; and a window of the run must return exactly the rows of that window - the same data, not a different run. | §8.2, §8.4, §11.1 |
| `producer_codec_test` | The external producer interface: the six messages round-trip field by field, and every malformed one is refused whole - another protocol version, an unknown status or reason, a text count above the wire capacity, a preparation number of zero, a velocity intent with no validity period, a `READY` result without a first frame, a nested frame the execution codec refuses, and a `READY` producer that reports a failure reason at the same time. It also pins the six frozen topic names and QoS, and reads the policies back from a writer and a reader built with the frozen descriptors (a lifecycle endpoint is reliable, volatile, keep-last eight with a 10 ms blocking time; the velocity intent is best effort, volatile, keep-last one), then publishes a real `PrepareRequest` and receives it. The transport section drives the application's own `ProducerTransport` against a raw producer peer: the peer's announcement and `READY` result arrive through `TakeEvents` with their fields intact, the preparation request, the authorization, the stop notice and the velocity intent arrive at the peer, only the newest intent is written, and a publish on a stopped transport is refused and counted. | §6.3 |
| `policy_package_test` | The SDK half of the ONNX deployment contract: `tools/verify_policy.py` accepts a package whose document agrees with the model it is embedded in and is numerically unchanged, and refuses a model without the contract, with the contract twice, with a document of another policy type, a document claiming another input, a document with an unsupported observation frame, a model whose weights moved, and a model saved at an ONNX IR version above the one the pinned runtime loads. The models are built in a temporary directory, so the check needs no simulator and no checked-in binary. | §3.4, §11.1 |
| `policy_locomotion_test` | The acceptance of the local policy, end to end: a generated twelve-joint Fake deployment (a configuration and a policy ONNX whose document commands exactly those joints) is started as a real runtime and a real application, and the application is driven through its terminal from Standby into Damping, through Initializing into Ready, into Locomotion, and out through the ordered exit. It asserts that the policy was loaded from the configured path, that the runtime really entered Tracking, that holding the behaviour for far longer than the target and source-state deadlines produces neither a `LocalPolicyFailed` latch nor a runtime target timeout, and that the exit is confirmed. The model's weights are small on purpose: its first frame has to be a continuous target, and the runtime's own continuity guard - never a relaxed test configuration - is what enforces that. | §11 and §11.1 (local ONNX locomotion) |
| `confirmation_test` | The test-only confirmation helper: a request is only proven when a later-published `Applied` result and an agreeing `RobotState` identify it, in both arrival orders, and a late `Applied` after a `Fault` never confirms anything. Pure protocol logic, no DDS. |
| `application_cli_test` | The process contract of `application` against a real runtime: every startup rejection (missing/malformed configuration, unknown argument, out-of-range domain) exits with the usage or configuration code and message before DDS exists; the normal exit and the signalled exit are the same confirmed ordered stop (`stop=confirmed state=Damping`, exit 0); a latched lower fault is reported as Fault and leaves with `stop=unknown lower=Fault`, exit 3, without claiming a stop; an unreachable lower leaves with `stop=unknown lower=Unknown`, exit 3, inside its bound; the terminal is restored on every exit path; and a lost input device is the same bounded exit. |
| `application_client_test` | The official client against a real standalone runtime, driven by hand: the same transport and request/answer rules the application ships, with the arrival order of an answer and its witness state made deterministic, so a confirmation in both orders, a session takeover, a stop preempting a pending request and the epoch watermark are proven against the real lower layer rather than only against a recording sink. |
| `application_flow_test` | The cross-process phase acceptance: a runtime replaced under a running application (the upper reports `Fault`, binds the new instance and never returns to a tracking behaviour by itself), a `SIGSTOP`/`SIGCONT` of the runtime (the upper reports `Unknown` inside the state window plus slack, latches its own `Fault` and stays latched while the runtime keeps running), a `SIGSTOP`ped application (the runtime falls back on its own command timeout, measured from its own log) and a `SIGKILL`ed application (the runtime expires the supervisor lease, proven by its own event and by the all-zero session of its per-tick trace), a normal application exit with the runtime surviving (run through the harness on `application_normal`), the phase invariant that no run ever shows `behavior=Locomotion` (recorded as the third-phase boundary, not as a pass), and that every case claimed its own domain and left no process behind. |

| `external_producer_test` | The external producer acceptance: the real application, the real runtime and `examples/external_producer.cpp` as three processes over real DDS. The application is driven from Damping into Ready, the producer registers, is selected and prepares a first frame, the locomotion key authorizes the context, and the producer's own stream is compared with the runtime's output trace: 104 adopted frames with strictly increasing sequences under the granted producer identity, none of them before the authorization, the target moving under a forward intent, and the stop that ends it. |
| `application_record_test` | The application-driven recorded run (§11.1): the recorder is enabled in a copy of the shipped configuration, the real application supervises one session, and the run is read back - 381 rows at 100 Hz on the 500 Hz loop with the command in force, the feedback and the output of the same tick in every tracking row, and the authorization contexts in the run's own `sessions.json`. |
| `application_stability_test` | The state-switching stability run (§11.1): `Standby -> Damping -> Ready` in a loop for a configurable wall-clock time, with the memory, thread and descriptor counts of both processes sampled from `/proc`. The CTest entry runs it for twenty seconds as a regression guard; the thirty minutes of the acceptance are the same driver with `--duration-s 1800`. |

### The scenario matrix

Each scenario is one independent CTest entry, one domain, one runtime process
and one or more client processes. The list is the frozen failure matrix of the
phase.

| Scenario | What it proves | Design §9 |
|---|---|---|
| `normal` (201) | The full round trip over real DDS: `Boot → ZeroTorque → Damping → Tracking → Damping → ZeroTorque`, confirmed in both arrival orders, with the per-tick output acceptance of a 50 Hz producer against the 500 Hz control loop – every control tick recomputes a different torque from a new measured velocity, and the trace proves there is no lower-layer interpolation. | "full round trip", "50 Hz target against 500 Hz control, per-tick output, no interpolation" |
| `request_retry` (202) | A lost answer: the identical tracking request is re-sent with the same `request_id` and is answered from the stored result without a second authorization and without applying anything twice. | "result lost, retried with the same id" |
| `stale_context` (203) | Replay of a recorded session after the lease expired: old session, old epoch, old commands and old heartbeats re-authorize nothing, the target is never refreshed, the machine never returns to `Tracking`. A replayed bootstrap is answered for the same recorded supervisor – the documented bootstrap, not a re-authorization. | "restart/replay must not re-authorize" |
| `supervisor_killed` (204) | The supervisor is `SIGKILL`ed while tracking: although an independent producer keeps sending, the runtime leaves `Tracking` on the supervisor timeout and revokes the session; replayed packets of the dead supervisor extend nothing. The runtime then deliberately refuses to leave damping without a session on its own, so the scenario kills it and reports the refusal. | "pause/kill the supervisor while the producer stays alive, revoke by deadline" |
| `supervisor_paused` (205) | `SIGSTOP`ped supervisor: the lease lapses exactly on the supervisor timeout, no target survives it, and after `SIGCONT` the supervisor has to run the whole handshake again. | "pause/kill the supervisor, revoke by deadline" |
| `producer_paused` (206) | `SIGSTOP`ped producer: the command timeout returns the machine to `Damping` on the receive deadline **keeping the session**; after `SIGCONT` a new authorization is needed before the runtime commands a target again. | "command timeout falls back to damping keeping the session" |
| `runtime_restarted` (207) | A restarted runtime is a new instance: packets of the previous instance are answered as foreign, and the fresh instance never enters `Tracking` on its own. | "after a restart, the old instance/session/epoch are refused" |
| `command_flood` (208) | A flood of ordinary tracking requests starves neither the stop lane nor the control loop: the stop is applied while the flood runs, the target keeps being served, and the runtime reports the requests it dropped instead of hiding the loss. | "target flood must not block control" |
| `slow_logger` (209) | The log consumer is blocked on purpose: the logger drops events, the control loop keeps its deadlines and the per-tick output trace stays complete. | "a slow log must not block control" |
| `stderr_backpressure` (211) | The log consumer is never released and the pipe is actually full when `SIGTERM` arrives, so the logger is parked inside its sink - and the machine is in `Tracking` on a live supervision session at that moment, so the runtime has to stop itself while every diagnostic it would write is being dropped. It leaves with exit 0 inside the bound without the reader resuming, closes its trace completely and keeps its control deadlines; the clean-shutdown line itself may be dropped by the full pipe, which is why the exit guard, the complete trace and the holding supervisor's own confirmations are the assertions. | "the shutdown path is bounded", "a slow log must not block control", "the runtime stops itself when the process leaves" |
| `local_stop` (212) | The runtime's own guarded stop is the only way out: a supervisor holds a granted tracking authorization on a live session and a producer keeps the target fresh, and neither asks for a stop before `SIGTERM`. The machine is proven to be in `Tracking` when the signal arrives, and the run then has to reach `ZeroTorque` by itself, accept that torque write and report a clean shutdown - the operator's signal does not depend on a supervisor being reachable, and the process cannot leave without the stop having happened. | "the runtime stops itself when the process leaves", "`Applied` is a host-path acceptance" |
| `write_refused` (210) | The fake backend refuses the write of a tracking tick: the refusal latches as a fault, the runtime commands zero torque instead of the refused target, and only a matching `Reset` clears the fault once the write path is healthy again. The trace carries the refused attempt and the effective write of every such tick. | "fault first-tick output", "`Applied` is a host-path acceptance, never a device ack" |

### The application matrix (real upper process over real DDS)

Each entry drives the shipped `application` binary through a PTY against a real
`robot_runtime` on one claimed domain, with one generated `config.json` shared
by both processes. The upper layer is never simulated by `runtime_client`, and
no scenario in this group asserts anything about motor execution.

| Scenario | What it proves |
|---|---|
| `application_normal` (213) | The documented operator path of the two-process deployment: `Starting → Standby → Damping → Initializing → Ready → Damping → Standby → Damping → Initializing → Ready` by keyboard over DDS – the feedback file is `tests/support/fake_static.json`, whose joints never move, which is what lets the same path be walked twice. The first process is then restarted with `x` **from `Ready` while the runtime is still `Tracking`**: the ordered exit stops generating, keeps the tracking context (no `behavior=Fault` appears before the exit line), and is confirmed by a fresh lower state (`stop=confirmed state=Damping`, exit 0). The runtime outlives the application and keeps damping without a supervisor, and the **second** application process then takes that left-over damping state over: it acquires a session, asks for `ZeroTorque`, reaches `Standby` on the confirmed fresh state – never into a behaviour that could move – and leaves through its own confirmed ordered exit. The runtime, left damping without a supervisor again, deliberately refuses to leave on `SIGTERM`, which the scenario reports. |
| `application_ready_hold` (214) | A `Ready` hold of 1.5 s is longer than every runtime deadline the upper layer could hide behind: the per-tick trace proves 1060 tracked ticks with 106 distinct, strictly advancing applied command sequences over 2.1 s, the runtime's log holds no `CommandTimeout` and no `SourceStateTimeout`, its control deadlines hold, and it then stops cleanly from the `ZeroTorque` the upper layer asked for while still supervised. The application process, whose lower layer is gone by then, is killed: the bounded exit is `application_unknown`'s subject, not this scenario's. |
| `application_repeat_and_reset` (215) | The initialization runs twice from `Damping` (a repeated `i` starts a new trajectory and reaches `Ready` again), the software emergency stop on a healthy link latches the upper `Fault` while the lower layer keeps its own `Damping` output, and the explicit `Reset` is the only way back – through a confirmed fresh `ZeroTorque` into `Standby`, never beyond it. |
| `application_stop_preempt` (216) | A stop preempts a pending request: with the tracking request on the wire (`pending=Tracking`) the `ZeroTorque` key replaces it with a strictly larger request id (`pending=ZeroTorque`), the upper layer ends in `Standby` on a confirmed fresh `ZeroTorque` and never reaches `Ready`, and the runtime reports the stop it was asked for rather than a target deadline. |
| `application_fault` (217) | The fake backend refuses the torque write, the runtime latches `WriteFailed` while it keeps running and opens its recovery window, and the real application reports `behavior=Fault` with the lower `runtime=Fault` – it sends no target and never claims a stop. Once the window is open, **one** `r` sends the `Reset` and reaches `Standby` on the confirmed fresh `ZeroTorque` (the runtime's own `ResetApplied`), never beyond it. The software emergency stop on the healthy link then latches the upper `Fault` while the lower layer keeps its own `Damping` output, and a second `r` is the only way back. The runtime stops cleanly from the `ZeroTorque` it was left at, and the application, whose lower layer is gone by then, exits bounded and honest: `stop=unknown lower=Unknown`, exit 3. |
| `application_unknown` (218) | The lower process is `SIGKILL`ed while the application supervises it: the application reports `runtime=Unknown` (and `fresh=no`) 148–150 ms after the kill, i.e. inside the 150 ms state window plus slack, latches its own `Fault` instead of presenting the last remembered state as the runtime's present fact, and leaves with `stop=unknown lower=Unknown`, exit 3, 1.01 s after the exit key – never reporting a stop it could not confirm. |
| `application_overspeed` (219) | The other half of the lower-fault acceptance: `tests/support/fake_overspeed.json` feeds a measured velocity above `runtime.safety.joint_velocity_limits` (the same case the phase-1 runtime CLI test uses), so the runtime latches `JointVelocityExceeded` before it leaves `Boot` and the write path stays accepted. The real application reports the lower fault in step – `behavior=Fault runtime=Fault` with `cause=RuntimeFault` and no target – never starts a move it would have to take back (`init=-`), never claims a stop, and exits 3 after its lower layer is killed, naming the fault it still sees (`stop=unknown lower=Fault`); the runtime's log holds no `WriteFailed`, no `ResetApplied` and no clean shutdown, so the latch was left alone. |

## Traceability of the design report §9

| Design §9 acceptance item | Where it is proven |
|---|---|
| All legal and rejected state transitions, boot grace/deadline, explicit Reset | `execution_fsm_test` |
| Pure feedforward, PD + feedforward with the total torque limit, missing/duplicate joints, non-finite intermediate computations | `protocol_test`, `safety_test`, `execution_fsm_test` |
| Request retry, different payload, out of order; single-source authorization; atomic first-frame and gain switch; stop preemption | `execution_fsm_test`, `command_gate_test`, `request_retry` |
| Per-tick timeout with no new packet, old packets never feeding the watchdog, source-state window, independent supervisor lease | `command_gate_test`, `execution_fsm_test`, `producer_paused`, `supervisor_paused` |
| Forward/reverse/equal-threshold/multi-joint overspeed, fault output on the first tick, damping disqualification degrading permanently to zero torque | `safety_test`, `execution_fsm_test`, `write_refused` |
| Feedback freshness, IMU coherence, complete error codes, recovery-window reset | `safety_test`, `execution_fsm_test` |
| Runtime and client as independent processes over the fixed DDS profile | every `integration_*` entry, `runtime_cli_test`, `dds_codec_test` |
| Full round trip `Boot → ZeroTorque → Damping → Tracking → Damping → ZeroTorque` | `normal` |
| Both result/state arrival orders, a lost result retried by the same id, a late result | `confirmation_test`, `normal`, `request_retry` |
| 50 Hz target under 500 Hz control with per-tick output and target-age checks, no lower-layer interpolation | `normal` (scripted measured-velocity ramp) |
| Pausing/killing the supervisor while an independent producer or the DDS threads stay alive, revocation by deadline | `supervisor_paused`, `supervisor_killed`, `producer_paused` |
| After a runtime restart, old instance/session/epoch refused; target flood and slow logging must not block control | `runtime_restarted`, `stale_context`, `command_flood`, `slow_logger`, `stderr_backpressure` |
| The runtime's own stop leaves the machine only at zero torque, from the state it was found in, and the process leaves only then | `local_stop`, `execution_fsm_test`, `runtime_cli_test` |
| Configuration and versioning executable, installable runtime, format check, CI | `config_test`, `config_schema_contract_test`, `runtime_cli_test`, `format-check` and the CI workflow |

## Traceability of the phase-2 two-process acceptance (§11)

| Acceptance item (§11) | Where it is proven |
|---|---|
| `Standby → Damping → Initializing → Ready → Damping → Standby` per tick with the upper state checked | `application_normal` (the behaviour path), `application_initialization_test` (the trajectory and its tolerances) |
| An emergency stop latches the upper `Fault`, preempts the pending request, and only an explicit `Reset` reaches `Standby`/`ZeroTorque` | `application_fsm_test`, `application_repeat_and_reset` (both keys over real DDS) |
| A `Locomotion` initialization request is refused and creates no return trajectory | **not accepted in this phase** – see the boundary below |
| An ordered application exit leaves the runtime damping; an unconfirmable stop is never reported as a stop | `application_normal`, `application_cli_test`, `application_unknown`, `application_fault` |
| Lower-layer refusal / write failure, lost or late results, same-id retry, older-request refusal, out-of-order states, stop preempting a pending request | `write_refused`, `request_retry`, `stale_context`, `application_client_test`, `application_stop_preempt`, `application_fault` |
| Killing/pausing the application while the DDS threads stay alive: the runtime still falls back inside the configured deadlines and no old packet feeds the watchdog | `application_flow_test` (`app_paused`, `app_killed`), `supervisor_paused`, `supervisor_killed` |
| A runtime restart or a one-way state-link interruption: the upper layer does not automatically resume `Tracking`, and `Unknown` is never presented as a stopped machine | `application_flow_test` (`restart`, `state_link`), `application_unknown`, `runtime_restarted`, `stale_context` |
| Upper `Fault` with the lower layer damping; the lower layer's own fault reported in step | `application_repeat_and_reset`, `application_fault` (`WriteFailed`), `application_overspeed` (`JointVelocityExceeded`), `application_cli_test` |
| The upper layer keeps publishing targets across several lower-layer deadline windows | `application_ready_hold` |
| No real hardware, no ONNX, no external producer, no `Recorder`, no `Locomotion` entry, no motor acknowledgement | **not covered** – see the list below |

## Timing claims

The deterministic suite asserts exact tick semantics; the integration suite
records measured timings and compares them against the configured deadlines with
an explicit scheduling tolerance (100 ms on the fault and fallback observations,
bounded final waits of 10 s per scenario). The two-process acceptance adds the
upper layer's own reactions, each measured from the signal that caused it and
compared against the configured window plus slack: the unknown lower state
inside `state_timeout_us` (150 ms) plus 450 ms and no earlier than half the
window, the runtime's command timeout and supervisor lease expiry read from the
runtime's own log, and the bounded exit inside `request_deadline_us` plus slack.
Those tolerances account for ordinary operating-system scheduling **on the test
host**. They are not a hard real-time proof and this phase does not claim one:
no test here demonstrates a bounded worst-case latency under load, and none is
run on the target hardware.

## Phase 3: the ONNX deployment contract

The policy model, its contract and the runtime that reads it are in place and
verified against the model the training side really exported. What is *not* in
place is the behaviour that would drive the machine with it, and this section
says exactly where the line is.

### The contract

`schema/onnx-deployment-v1.schema.json` freezes the document a policy ONNX
carries in its single model-level metadata key `irmv.deployment`: the model
interface, the policy period, the action joint list with its aligned default
q/dq, gains, scale, resolved offset, both clip layers and the frozen pipeline
order, the six observation terms of the training policy group with their unit,
frame, scale, clip and history, the two joint reference lists, the normalizer
and the provenance block. `irmv::policy::ParseDeployment` implements it and
refuses a document it cannot honour with the JSON field path of the first
violation; `schema/onnx-deployment-v1.schema.json` and the parser are kept in
step by `deployment_schema_contract_test`, which validates the same fixtures
against the schema.

### The model that was exported

`scripts/rsl_rl/play.py` in the training repository now writes the document from
the *resolved* values of the running environment after the ONNX export it
already performed. Running it against the recorded checkpoint
(`logs/rsl_rl/irmv_flat/2026-09-08_09-56-44`, checkpoint `model_2999.pt`) with

```bash
# in irmv_humanoid_locomotion, with env_humanoid active
python scripts/rsl_rl/play.py --task Irmv-Flat-v0 --headless --num_envs 1 \
    --load_run 2026-09-08_09-56-44
```

produced a package whose contract is:

| Field | Value |
|---|---|
| model | `obs[1,45] -> actions[1,12]` |
| policy period | 20000 us (50 Hz, `sim.dt * decimation`) |
| action joints | 12, in the PhysX dof order the action term resolved |
| kp / kd | 100 / 6 for hip pitch, roll, yaw and knee; 60 / 4 for ankle pitch and roll |
| scale, offset | 0.5, and the resolved default pose (hip_pitch -0.3, knee 0.6, ankle_pitch -0.3, rest 0) |
| clip layers | both absent (no wrapper clip, no term clip) |
| observation | command 3, projected_gravity 3, base_ang_vel 3, joint_pos_rel 12, joint_vel_rel 12, last_action 12 = 45 |
| normalizer | disabled and not embedded |
| model SHA-256 | `0dea6741905a0531ca91f9b83423503d358de6f6325e54495e8cce12cde0e97d` |
| provenance | checkpoint, env and agent configuration digests, the exporting commit and the asset digest, `missing` empty |

`tools/verify_policy.py` accepts the package against the very same file before
the document was written: the ONNX checker passes, the document conforms to the
schema, the declared interface agrees with the graph, the graph and the weights
are byte-identical between the two files, ONNX Runtime reads the document back
out of the saved file, and both sessions produce bit-identical outputs for the
same fixed input. The SDK's own C++ loader then accepts the same file with the
deployment's twelve joint names, assembles a 45-component observation from a
synthetic coherent snapshot, runs one inference and builds one complete
twelve-joint frame (printed `q` values are exactly `action * 0.5 + offset` and
the gains land on the joints the document names).

### What is in place

* The contract, its schema, the parser and the joint mapping (`deployment_test`,
  `deployment_schema_contract_test`).
* The offline package gate (`tools/verify_policy.py`,
  `tools/deployment_contract.py`, `policy_package_test`).
* The application-side configuration (`application.policy` and
  `application.policy_overrides`, matched by the model's complete SHA-256).
* The pinned ONNX Runtime dependency (1.22.0 by URL and archive SHA-256, with a
  provenance manifest and an offline prefix override) and the model loader.
* The observation and action transformations and the gain merge
  (`policy_math_test`); the `last_action` observation follows the contract's
  stage and starts from its `initial` values, and the value the frame carries
  and the value the next observation reads come from one implementation of the
  frozen pipeline.
* The inference worker on its own thread with a bounded keep-last-1 handoff
  (`inference_test`), and the judgement of one result as pure functions: the
  identity of the step (submission instant, local generation, authorization
  epoch), its own deadline, and the source state it was computed from.
* The `Locomotion` behaviour end to end on the local model
  (`policy_locomotion_test`).
* The external producer lifecycle: the six frozen endpoints, the supervisor,
  the example process and the whole exchange driven by the supervision loop -
  selection, preparation, the authorization after the runtime applied the
  context, the producer's own frame stream, the intent lane and the revocation
  (`integration_application_external_absent` for the mode with nobody to run
  it, `external_producer_test` for the stream itself).
* The recorder and the run reader: one coherent sample per sampled tick with the
  measured feedback and the final output of that tick, per-stretch gaps, a
  `sessions.json` sidecar for the authorization contexts, time-range reading and
  the NPZ export (`recorder_test`, `record_reader_test`, `record_export_test`).
* `ldd build/robot_runtime` names no ONNX Runtime: the lower process still loads
  nothing but its own configuration. `ldd build/external_producer` names none
  either.

### The installed tree

`cmake --install build --prefix <tmp>` produces the two processes, the pinned
CycloneDDS pair, the pinned ONNX Runtime and the configuration under
`<prefix>/etc/irmv`. Verified after this phase:

* both installed binaries run from a directory that is not the source tree;
* the installed `bin/application` resolves `libonnxruntime.so.1` through its own
  `$ORIGIN/../lib` entry, and the installed `bin/robot_runtime` names no ONNX
  Runtime at all (`ldd`);
* every installed artifact's only rpath entry is that relative one, which the
  install rules check with `file(RPATH_CHECK)`;
* the installed configuration has no model path, so the installed application
  starts with no locomotion source - `policy=none source=none` - and supervises
  the runtime through its ordered states.

Adding the policy layer made the first version of this fail: the ONNX Runtime
library is not a system library and was only resolved through the *build* rpath,
so the installed application could not start. The install rules now name the
pinned ONNX Runtime prefix as a dependency search directory, exactly like the
two DDS prefixes, so the resolution no longer depends on the build tree and an
unresolved dependency is a configure-time error instead of a binary that fails
on another machine.

### What is not in place yet

* **A *shipped* policy.** The model this phase was accepted against is the one
  the training repository exported from a real checkpoint, and this SDK now
  reproduces its numbers element by element ("The training-side reference sample"
  below), but no model is installed with the deployment and the shipped
  configuration points at none: a machine runs a policy only when an operator
  names one.
* **Any bench or machine validation.** Every gain, tolerance and limit in this
  contract is a nominal training or demonstration value; nothing here is a
  validated machine limit, and the phase-4 bench acceptance is still owed.
* **A no-source deployment is still a legal configuration.** A deployment that
  configures no `policy.model_path` and selects no source runs with no
  locomotion source at all, which is what the shipped Fake one does: pressing
  `l` arms nothing and the behaviour keeps holding `Ready`.
* **The upper layer's hold can lose a race against the runtime's own target
  deadline, and the run below measures how often.** While a stop is pending the
  behaviour publishes no further target on purpose - a frame published then would
  belong to the authorization the stop is withdrawing - so the only thing
  keeping the tracking context alive is the runtime's `command_us` deadline. If
  the stop is not applied inside that window, the runtime leaves `Tracking` on
  its own (`stop=CommandTimeout`), the behaviour follows the lower layer into
  `Damping`, and the operator sees `Damping` where a `ZeroTorque` press would
  normally land in `Standby`. Nothing faults, nothing moves, and the operator
  recovers by pressing the key again; the run below reports the rate, and the
  finding is recorded as an open risk rather than a fixed defect.

### The ten-minute recorded run (§11.1)

A twelve-joint Fake deployment (`tools/make_test_model.py --fake-deployment`)
was recorded for exactly ten minutes at 100 Hz on the 500 Hz control loop:

| Measure | Value |
|---|---|
| wall clock | 10:00.01 (`/usr/bin/time -v`, 23.7 s user + 8.2 s system) |
| control ticks | 299,988 in the shutdown line, 17 overruns, worst 13.0 ms |
| recorded rows | 59,998 in 67 chunks (`chunk_0.bin` … `chunk_66.bin`) |
| measured rate | 99.997 Hz from the stamps, 100.001 Hz from the tick stride |
| sampling stride | exactly 5 control ticks per row, constant across the run (every chunk spans 4,496 or 2,986 ticks for its 900 or 598 rows) |
| gaps | none: 0 missing samples, 0 write failures |
| run size | 35.2 MB for 12 joints, 586 bytes per row |
| peak RSS | 13.8 MB (`/usr/bin/time -v`) |

The run's own `index.json` and `meta.json` carry all of it, and
`tools/read_record` returns the same numbers for the whole run and for any
window of it. What this does *not* measure is a machine: it is the FakeBackend,
so the numbers describe the host loop and the recorder, never a device.

What is still owed from §11.1 is the thirty-minute state-switching stability run
with its memory, thread and handle statistics, and a recorded *application*
run - the recorder samples the runtime, which is where the coherent snapshot
lives. Both are now in place; the two subsections below record them.

### The thirty-minute state-switching run (§11.1)

`tests/integration/application_stability_test.py` walks the documented path -
`d` (Damping), `i` (Initialize) to `Ready`, `z` (ZeroTorque) back to `Standby` -
and waits for the behaviour it asked for before asking for the next, sampling
the memory, thread and descriptor counts of both processes from `/proc` as it
goes. The CTest entry runs it for twenty seconds as a regression guard; the
acceptance run is the same driver with `--duration-s 1800`.

A two-hundred-second run with the shipped configuration and an otherwise idle
host, tracing the runtime's per-tick output for the analysis:

| Measure | Value |
|---|---|
| wall clock | 200.3 s (`--duration-s 200`) |
| rounds | 300, no key re-asked |
| runtime-deadline stops | `CommandTimeout` 0, `SourceStateTimeout` 0 |
| faults | none (`behavior=Fault` never appeared) |
| application | RSS 12.8 -> 14.6 MB, threads 8 flat, descriptors 8 flat |
| runtime | RSS 11.7 -> 11.8 MB, threads 10 flat, descriptors 10 flat |
| adopted targets | 297 contexts, 8 to 32 frames each, largest gap between two frames of one context 16 ticks (32 ms) against a `command_us` of 100 ms |

The last row is the one worth stating: every context that was authorized
received its next frame well inside the runtime's own target deadline, so the
runtime never had to fall back on it. The counts that *can* end a context are
reported rather than failed on, because `CommandTimeout` is a legal runtime
fallback; an earlier run of the same driver on a heavily loaded host (the whole
tree was being rebuilt beside it) reported 21 of them in 265 rounds, which is
what the race described under "What is not in place yet" costs when the host
cannot keep the schedule. Nothing faulted, nothing moved, and every round
completed.

### The application-driven recorded run (§11.1)

`tests/integration/application_record_test.py` turns the recorder on in a copy
of the shipped configuration, supervises one session with the real application
(`Standby -> Damping -> Initializing -> Ready -> hold -> Damping -> exit`) and
reads the run back:

| Measure | Value |
|---|---|
| rows | 381 at 100 Hz on the 500 Hz control loop, schema version 2 |
| authorized window | 266 tracking rows, one producer identity and one non-zero epoch |
| per row | the command in force (identity, epoch, sequence, source state), the feedback of that tick and the output it produced, all valid and host-accepted |
| session table | 4 events in `sessions.json`, one per authorization context, with the epoch, the producer, the gains and the model digest written as `unknown` |

The run's rows and its session table join on `{producer_id, epoch}`, which is
what makes a recorded stretch traceable to the context that produced it.

### The external producer acceptance (Task 6)

`tests/integration/external_producer_test.py` runs the real application, the
real runtime and `examples/external_producer.cpp` as three processes over real
DDS and drives the documented path through the application's terminal:

```bash
# what the test does, by hand
robot_runtime --config config/sdk.json --domain 229 &
application  --config config/sdk.json --domain 229 --locomotion-source external &
external_producer --config config/sdk.json --domain 229 --rate 50 &
# d -> Damping, i -> Ready (the loop selects and prepares the producer),
# l -> the authorization, w -> a forward intent, d -> the stop, x -> the exit
```

What it asserts, and what the run observed:

| Measure | Value |
|---|---|
| lifecycle | `producer=announced`, `selected`, `preparing`, `prepared`, `authorized epoch=N first_seq=1`, `publishing seq=2..`, `stopped`, for every producer of the run |
| producers | four: the happy path's, a restarted one, one that replaces it, and the one started after the previous was killed - each announced, prepared and authorized exactly once |
| authorizations | epochs 3, 6, 9 and 12, one per context, every one of them naming the process that was selected at that moment |
| frames before the authorization | none, asserted from the producer's log and from the runtime's trace |
| contexts adopted by the runtime | eight: four initialization holds (epochs 2, 5, 8, 11, 85 to 88 frames each) and four producer contexts (epochs 3, 6, 9, 12) |
| the producer's stream | epoch 3: 104 adopted frames, sequences 1..104 strictly increasing, one producer identity, 105 frames published (the first travelled inside the request) |
| the walk | 26 distinct targets on the first joint, 0.00 .. 0.05 rad, inside the configuration's continuity delta |
| the replacement | the context that was authorized after the replacement names the *replacing* producer in the runtime's trace |
| the producer's death | the runtime falls back on its own command deadline, the application follows it into `Damping` without latching anything, and a new producer is prepared and authorized afterwards |
| faults | none for the whole run |

The frames the test compares are read from the *runtime's* output trace, not
from the command topic: a frame the producer published but nobody accepted -
which is exactly what a missing routing identity produces - cannot appear in it,
and did not, before the defect below was fixed.

### The training-side reference sample (§11.1)

The package gate of Task 2 proved the model's *contract* against the training
export; what it could not prove is that this SDK reproduces the training
*numbers*. That is what the reference sample is for, and it is now generated,
checked and green.

`scripts/rsl_rl/reference_sample.py` in `irmv_humanoid_locomotion` instantiates
the matched environment from the exported checkpoint headless (12-DOF IRMV V3,
`Irmv-Flat-v0`, seed 42, one environment, closed loop on the exported ONNX),
drives it to four deterministic states and writes
`exported/reference_sample.json` beside the model it describes:

| Field | Value |
|---|---|
| artifact | `logs/rsl_rl/irmv_flat/2026-09-08_09-56-44/exported/reference_sample.json` |
| artifact SHA-256 | `38f50f1031284f15a9f2631a8c8bfbcc3db422c9b775be8f14efb8a6bcc7e44f` |
| model | `policy.onnx`, SHA-256 `0dea6741905a0531ca91f9b83423503d358de6f6325e54495e8cce12cde0e97d`, ONNX IR 8 |
| provenance | checkpoint `model_2999.pt`, training commit `05f83c9`, env-config digest, asset revision, IsaacLab 0.54.2, torch 2.7.0+cu128, onnx 1.22.0 |
| samples | `stand_zero`, `forward_half`, `turn_left`, `forward_turn`, each with the raw state, the velocity command, the `last_action` the training observed, and the four expected vectors |
| pipeline | scale 0.5, the resolved default pose as offset, both clips absent, `last_action.stage = raw` |

Its own test (`scripts/rsl_rl/test_reference_sample.py`) pins the document's
shape, the finiteness of every value, the contract it embeds, the model digest
against the file's bytes, the pipeline order, and that two regenerations agree
within 1e-6.

The SDK side is `tools/verify_policy_sample`, registered as
`policy_reference_test` when the artifact is present (a configure-time variable
names the directory; without it the entry is not registered and the configure
step says so, so its absence is never a silent pass). It builds the observation
from the recorded state and command with `BuildObservation`, runs the same model
file through the SDK's `OnnxModel`, builds the frame with `BuildCommand`, and
compares all three vectors element by element:

```
verify_policy_sample: model=0dea6741905a0531ca91f9b83423503d358de6f6325e54495e8cce12cde0e97d samples=4 tolerance=1e-05
verify_policy_sample: stand_zero     observation=3.725e-09 action=3.725e-08 target=2.980e-08 ok
verify_policy_sample: forward_half   observation=0.000e+00 action=0.000e+00 target=1.490e-08 ok
verify_policy_sample: turn_left      observation=9.313e-10 action=5.960e-08 target=2.980e-08 ok
verify_policy_sample: forward_turn   observation=3.725e-09 action=1.192e-07 target=6.706e-08 ok
verify_policy_sample: every vector matches within 1e-05
```

The residuals are float32 rounding, which is the whole claim the comparison can
make: the SDK's observation, the model's action and the resulting joint position
target are the training environment's own numbers for the same state and the
same velocity command.

### The installed tree runs the external lifecycle

`cmake --install build --prefix <tmp>` produces the four binaries, the pinned
CycloneDDS pair, the pinned ONNX Runtime and the configuration under
`<prefix>/etc/irmv`. Verified after this phase:

* the installed `application`, `robot_runtime` and `external_producer`, started
  from a directory that is neither the source nor the build tree, complete the
  whole external lifecycle - registration, preparation, the authorization, the
  producer's frame stream and the stop (`external_producer_test` driven with the
  installed binaries);
* every installed binary's only rpath entry is the relative one, which the
  install rules check with `file(RPATH_CHECK)` - including `external_producer`,
  which failed to load its DDS libraries from `<prefix>/bin` before that entry
  was corrected;
* `ldd` resolves ONNX Runtime through `<prefix>/bin/../lib` for the application
  only: the runtime, the producer and the record reader name none.

## Defects found and fixed by the phase-3 review

The review of the phase-3 delivery listed twelve items. Each one is closed here
with the evidence that closes it; the one that is not a defect of the code - the
bench validation, which belongs to phase 4 - is recorded as a gate, not as a
pass.

| # | Item | Fix and evidence |
|---|---|---|
| P0 | The external producer was never wired: `--locomotion-source external` was a startup refusal | The supervision loop opens the six producer endpoints in that mode and drives the whole exchange; `examples/external_producer.cpp` is a real producer process; `integration_application_external_absent` drives the mode with nobody to run it, `external_producer_test` drives registration, preparation, the authorization, the producer's own stream (cross-checked against the runtime's output trace) and the stop |
| P0 | Task 9's acceptance evidence was missing | This section, the recorded run and the stability run below |
| P1 | A local inference result was checked against its `generation` only, and a result of a step already written off as late could still drive a frame | `JudgePolicyResult` (identity whole, deadline of the result itself) and `PolicySourceStateUsable` (a state the runtime can still look the target up against, inclusive window like the runtime's own), both pure functions with unit tests, wired into the loop before it touches `last_action` or a candidate |
| P1 | The inference worker could lose a wakeup | The take of a request and the reset of the flag it was woken for are one critical section of the mutex `Submit` sets the flag under; a churn test submits into that window |
| P1 | Recorder gaps were inaccurate: a failed chunk write discarded accepted rows silently, and queue refusals were merged into one gap over the whole run | Every loss is now its own segment with a real sequence and stamp range - per refusal stretch, per failed chunk write, and the tail at `Stop()` - in `Recorder::gaps()` and the index alike |
| P1 | A truncated chunk could be read out as rows whose missing joint columns defaulted to zero | A truncation inside the column-major joint columns returns no rows and reports the truncation; a truncation inside the fixed per-row fields still returns the complete rows before it |
| P1 | The record did not carry a complete snapshot, and no policy session | Schema version 2 adds the measured feedback and the final output of the same tick; the runtime records one event per authorization context on its own bounded channel, with the model digest written as the literal `unknown` |
| P2 | The `last_action` contract was accepted by the parser and ignored by the application | `ResetLastAction` starts an authorization from the contract's `initial` values and `LastActionFromAction` stores each step in the stage the contract names, with numeric tests for both stages |
| P2 | `Observe` ignored the instant it was given, so an expired preparation reply could still become a candidate | The deadline is applied inside `Observe` itself; a caller no longer has to expire the preparation first, and `ExpirePreparation` remains the way a caller reports it |
| P2 | A recorder sampling rate that did not divide the control rate was rounded, so a run recorded another rate than the one it declared | `LoadConfig` refuses the combination (200 Hz on a 500 Hz loop), and the runtime derives the stride from the two periods instead of rounding their ratio |
| P2 | The NPZ archive lost the run's units and the per-row producer identity, and narrowed microsecond stamps to float64 | The reader prints both, the archive carries them (the identity losslessly, the stamps as `uint64`), and the export test re-reads every column against the reader's own output, including a stamp above 2**53 |
| P2 | The application could not be configured for the external producer at all | `application.external_producer.{prepare_timeout_us,registration_timeout_us}` are configuration like any other, validated and digested |

Two further defects were found by the new acceptance runs rather than by the
review, and both are fixed and pinned:

1. **A locomotion request faulted the behaviour on a legal state.** Asking for a
   new context while the previous one was still live - which is what both
   sources do from `Ready` - read the runtime's own state, published before it
   applied the request, as an authorization nobody asked for, and latched
   `UnexpectedContext`; the external acceptance could not reach its
   authorization at all. The FSM now remembers the context it had confirmed
   until it proposes the request and accepts it while the request is pending,
   dropping the memory on every path that withdraws the context.
   `LocomotionRequestToleratesTheReplacedContext` pins both halves.
   The external acceptance then found the second half of the same hole: a
   request the runtime *refuses* leaves the context it refused for exactly
   where it was, and the next proposal was written over the memory of it, so the
   same fault came back one refusal later - the behaviour sat in `Ready` and
   every later proposal faulted. The memory is now only written when there is a
   confirmed context to record, and never by a refusal.
2. **The example producer published into a context it did not name.** Its frames
   carried the producer, the epoch, the sequence and the source state but
   neither the runtime identity nor the session, so the runtime's command gate
   dropped every one of them as a foreign sample: the context kept the first
   frame of the application's request, its source aged out, and the runtime
   ended it with `SourceStateTimeout` while the producer was logging a healthy
   stream. The acceptance reads the runtime's own trace of adopted targets, so a
   frame nobody accepted cannot pass; it failed before the fix and passes after
   it.

## What is not covered

* **Real hardware.** There is no `RealBackend`, no CAN or EtherCAT transport and
  no device in this phase; every test above runs the fake backend.
* **Physical execution confirmation.** `Applied` proves that the host send path
  accepted an output, never that a device acted on it. No test verifies actual
  motion, torque or brake behaviour, and no safety number here is a validated
  machine limit.
* **Firmware loss behaviour** and the recovery of a real drive are untested.
* **What phase two did not cover, and what phase three added.** The two-process
  acceptance drives the real `application` and the real `robot_runtime`; the list
  it did *not* accept, and the part of it phase three has closed so far:
  * **The ONNX policy contract is in place, its behaviour is not.** The model
    contract, the loader, the observation and action transformations and the
    inference worker are implemented and tested, but the supervision loop does
    not yet drive a `Locomotion` behaviour with them - see
    [Phase 3 so far](#phase-3-so-far-the-onnx-deployment-contract). The earlier
    statement that the upper layer ships no policy at all is therefore no longer
    accurate.
  * **No external producer lifecycle**: a producer that registers, warms up and
    publishes velocity intents is not part of this phase. The one body-velocity
    intent the keyboard maintains and expires has no consumer.
  * **No `Recorder`**: nothing records a long run, and no statistical check of
    the host receive rate, gaps or latency exists.
  * **No real backend**: every process runs the fake backend with scripted
    feedback, so no CAN/EtherCAT transport, no device, no real drive recovery
    and no bench value is covered.
  * **`Locomotion` is not accepted.** The enumerator exists and
    `StartLocomotion()` refuses (`application_fsm_test`), and
    `application_flow_test` asserts that no run of this acceptance ever shows
    `behavior=Locomotion`; the phase ships no locomotion target generator, no
    `Locomotion` entry key and no locomotion acceptance. `application_normal`
    therefore ends at `Ready`.
  * **No motor acknowledgement**: `Applied` is the host send path accepting an
    output, `ZeroTorque` is a torque request and never a power cut, and no test
    here observes physical execution.
  * **No real hardware emergency stop**: the `space` key is a software latch on
    this host and this DDS link.
* Later phases stay out of scope and are not simulated as done: ONNX inference
  consistency, the lifecycle of an external producer, long-running recorder
  tests and the hardware acceptance of the overall report are not covered by any
  test in this phase.
* The test client `runtime_client`, the confirmation helper, the flow test's
  own helpers and the fake scenario files under `tests/support/` are test
  artifacts. They are not a delivered client API, they are not installed, and
  nothing in the runtime calls into them. `tests/support/fake_static.json` is
  the scenario a *repeated* initialization needs: the shipped
  `tests/support/fake_scenario.json` steps the measured position to 0.05 rad two
  seconds in, which is outside the shipped completion tolerance, so the second
  initialization could never complete against it.
* Telemetry is not part of the protocol: `logger_test`, `slow_logger` and
  `stderr_backpressure` cover the bounded logger's own contract and the bounded
  exit diagnostics, not a wire format. A diagnostic that a jammed pipe destroys
  is dropped by design, so `stderr_backpressure` asserts the exit, the confirmed
  state and the trace rather than the line.

## Defects found and fixed by the two-process acceptance

Driving the two real processes over DDS surfaced two deviations from the
documented behaviour of the upper layer. Both were found by this acceptance,
fixed in `src/application/behavior_fsm.{hpp,cpp}` (the first also needed the
repeated latch to stop abandoning the request the latched behaviour itself
made), and are now pinned by deterministic unit tests in
`tests/unit/application_fsm_test.cpp`; no scenario records them as an accepted
deviation any more.

1. **The ordered exit from a tracking behaviour latched an upper `Fault`.** With
   the application in `Ready` and the runtime still `Tracking`, pressing `x`
   printed `[application] behavior=Fault runtime=Tracking ...` before the honest
   `application: exit stop=confirmed state=Damping`. `RequestExit` cleared the
   tracking context, so the next iteration read the lower layer's still-`Tracking`
   state - our own authorization, with our own Damping stop on the wire - as a
   context nobody authorized (`UnexpectedContext`). The exit now stops generating
   without clearing that context (the exit owns the iteration and skips the
   target phase), and the context is only dropped when the lower layer really
   leaves `Tracking` or the stop resolves.
   * `ExitFromTrackingKeepsTheContext` - a `Ready` + `Exit` sequence with the
     lower layer reporting `Tracking` for eight further Steps never latches, and
     the exit completes with `exit_stop_confirmed` once a fresh state shows
     `Damping`. It fails without the fix (`CHECK failed: step.state ==
     BehaviorState::Ready`).
   * `StopFromMotionKeepsTheContext` - the same for an ordinary Damping stop
     from `Ready` (a guard: this path was already correct).
   * `application_normal` now exits **from `Ready` while the runtime is still
     `Tracking`** and asserts that no `behavior=Fault` appears before the exit
     line.

2. **A `Reset` the runtime applied did not bring the application to `Standby`
   on the first attempt.** With a latched lower fault and the recovery window
   open, one `r` sent the `Reset`, the runtime answered
   `ResetApplied {event=15 from="Fault" to="ZeroTorque"}`, and the application
   stayed in `Fault`; a second `r` reached `Standby` through the explicit
   `ZeroTorque` stop. The confirmation was read in an iteration whose state came
   from before the transition - and every iteration that still observed the
   runtime's fault state re-ran the latch, which abandoned the pending `Reset`
   before its outcome was consumed. The latch now only clears what was in flight
   for a *new* latch (the Reset that leaves it and the handover that reconnects
   it are the latched behaviour's own requests), and a confirmed transition the
   state has not witnessed yet is held until a fresh state shows it - bounded by
   `supervision.request_deadline_us`, and dropped by a latch, an emergency stop
   and a new request.
   * `ResetConfirmationWaitsForTheState` - a `Reset` whose `Applied` confirmation
     is read while the state still carries the pre-reset facts reaches `Standby`
     on that first `Reset` once the fresh `ZeroTorque` state arrives. It fails
     without the fix (`CHECK failed: step.state == BehaviorState::Standby`).
   * `HeldConfirmationIsDropped` - the held confirmation is dropped by an
     emergency stop and by a session loss: a later fresh `ZeroTorque` state must
     not promote the behaviour behind the latch (a guard for the fix's bound).
   * `application_fault` presses `r` **once** from the latched lower fault and
     asserts `behavior=Standby runtime=ZeroTorque` plus the runtime's own
     `ResetApplied`, then exercises the emergency stop on the healthy link and a
     second Reset.
