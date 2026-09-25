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

ctest --test-dir build --output-on-failure          # everything (23 tests)
ctest --test-dir build -L core --output-on-failure  # deterministic core suite
ctest --test-dir build -L integration --output-on-failure
ctest --test-dir build -R integration_normal --output-on-failure
```

The whole matrix takes about a minute on a developer machine and each
integration entry is bounded by its own CTest timeout (30 s for `dds_codec_test`,
180 s for `runtime_cli_test`, 300 s per scenario).

For manual work the harness of the scenario matrix also runs the whole matrix in
one process, printing one summary line per scenario:

```bash
python3 tools/run_integration.py --all \
    --runtime build/robot_runtime --client build/runtime_client --source .
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
ThreadSanitizer needs on hosts where it refuses to start otherwise. This run is
not part of CI yet; it is the check that found the eviction race, and
`bounded_channel.hpp` must stay clean under it.

### DDS domain policy

Every integration entry isolates itself on its **own DDS domain**, always below
the 230 bound the configuration contract enforces:

* The `run_integration.py` harness takes the domain declared by the scenario
  (201–211 for the shipped eleven), claims it with an exclusive lock file, and
  refuses to start unless **every RTPS port of that domain** can be bound right
  now. If the declared domain is taken it walks up to the next free one, so two
  scenarios – or two developers – never share a domain. A lock left behind by a
  killed run is reclaimed once its process is gone.
* `tests/integration/runtime_cli_test.py` derives its domain from its own
  process id (`20 + pid % 190`), so parallel runs stay apart, and overrides the
  domain of its configuration file on the command line – the startup line proves
  the override took effect.
* Waits are bounded everywhere. On timeout the harness kills what it started
  (ordered shutdown first, a hard kill as the last resort) and fails the entry
  with the collected evidence: the runtime log, every client's JSON-lines
  observations and stderr, the per-tick output trace, and the domain used. A
  step that did not run is never reported as a pass.

The scenario entry points are also what CI runs; `--all` is a convenience for
manual runs and is never used by CTest.

## Deterministic core suite (`-L core`, 9 tests)

| Test | What it proves | Design §9 |
|---|---|---|
| `protocol_test` | The frozen wire contract of the core types: every enum validator accepts exactly its own values and rejects everything else, and the command shape rules (joint count against the configured layout, layout-id bounds, non-finite values, negative gains) return exactly the frozen rejection reasons. | "all legal and rejected transitions", "missing/duplicate joints, non-finite intermediate computations" |
| `config_test` | Configuration loading and validation: the shipped configuration is loaded field by field, malformed or contradictory documents are rejected with the offending field path (uniqueness and completeness of joint names, position-range ordering, boot-deadline ordering, minimum frequency versus control rate, state-window coverage), the numeric contract keeps exact decimal tokens, the SHA-256 known-answer vectors hold, and the configuration identity digest (`config_id`) covers the configuration in effect including a command line domain override. | "the configuration is executable and versioned" |
| `fake_backend_test` | The fake backend's contract: snapshot stamps, connection/error reporting, write acceptance and refusal, the fixed output ring, and that the last output reports the latest *attempted* output including a refused one. | Fake backend contract (§8) |
| `safety_test` | The safety gates on feedback and targets: freshness exactly at and beyond every timeout, IMU coherence, control-frequency bounds, driver error codes, position and velocity limits in both directions at and beyond the threshold, multiple joints, non-finite samples, the boot grace waiting for a first frame without calling absence healthy or faulting on it, the same grace never excusing an observed invalid sample, driver error or overspeed, the priority between simultaneous reasons, the complete fault report, and the recovery window. | "forward/reverse/equal-threshold/multi-joint overspeed", "feedback freshness, IMU coherence, complete error codes, recovery window reset" |
| `command_gate_test` | The command gate: sequence monotonicity, queue wait followed by the transport receive stamp as the validity origin, repeated sequences not refreshing the deadline, the source-state index window (unknown, ignored, stale, fresh), the command and source-state deadlines measured per tick with no new packet, the independent supervisor lease, and that a rejected owner frame refreshes nothing. | "per-tick timeout with no new packet, old packets do not feed the watchdog, source-state window and independent supervisor lease", "single-source authorization" |
| `execution_fsm_test` | The state machine: every legal and rejected transition, boot grace/deadline rules, the full normal path, write failure, request idempotency and conflicts, stop ordering and the authorization watermark (including a stop that does not change the state), session takeover and lease expiry, first-frame continuity, foreign packets, fault output degradation latching to zero torque, the recovery window with an explicit Reset, epoch exhaustion, the session's supervisor binding, same-tick stop ownership, a guarded same-producer epoch change, and the runtime's own trusted stop. | "all legal and rejected transitions, boot grace/deadline, explicit Reset", "pure feedforward, PD + feedforward with the total torque limit", "request retry/different payload/out-of-order, atomic first-frame and gain switch, stop preemption", "fault first-tick output, damping disqualification degrades permanently to zero torque" |
| `logger_test` | The bounded asynchronous logger: queue bound and drop accounting, level filtering, FIFO order, the drain on stop, an event racing with the stop, the 256-byte message bound, rotation of the default sink, the formatted line and the error path - plus the bounded diagnostic writer the sink and the exit path share, against a pipe the test fills itself. | §8 (bounded telemetry) |
| `channel_test` | The bounded handoff between the control thread and the transport thread: FIFO order, capacity, the untouched state of a refused push, the keep-last-1 handoff of the latest-value channels (a published value is replaced and counted only while no consumer holds it, and a taken one is never counted as a loss), a separate stop lane, and two-thread stress on both shapes whose payloads each carry their own generator, so a torn or overwritten copy fails the test. This is the entry that is also run under ThreadSanitizer; the eviction race it used to hide was found that way. | §2 (thread model), §4 (lane separation) |
| `config_schema_contract_test` | The shipped `config/sdk.schema.json` against the shipped `config/sdk.json`, the custom `irmv-joint-name` format (non-empty, at most 64 **UTF-8 bytes**), and that standard JSON Schema validity and the loader's own checks agree. | "the configuration is executable and versioned" |

All nine run without a peer, without a network and without a wall-clock
deadline: they assert exact tick semantics and counter values, and the only
wall-clock waits in `logger_test` are hang guards.

## Real-DDS suite (`-L integration`, 14 tests)

### Process-level entries

| Test | What it proves |
|---|---|
| `dds_codec_test` | Against a live endpoint: the codec round trip and field-by-field equality of the core types - including the empty, unused first command of a session, stop or reset request and the rejection of an over-long or missing joint vector - codec rejection of malformed samples, that the running transport's readers and writers really carry the frozen QoS (durability, reliability, history depth and kind, resource limits, and `max_blocking_time` on the reliable channels) as read back from the endpoints, and that a peer process sees the published samples. |
| `runtime_cli_test` | The process contract of `robot_runtime`: every startup rejection (missing/malformed configuration, `backend: real`, a broken scenario, unknown argument, out-of-range domain) exits non-zero with a message and without starting DDS; the normal path reaches `ZeroTorque` and exits 0 with a clean shutdown on `SIGTERM`; an unsupervised runtime keeps running; a latched fault keeps running and refuses to leave, and a hard kill is reported as such. |
| `confirmation_test` | The test-only confirmation helper: a request is only proven when a later-published `Applied` result and an agreeing `RobotState` identify it, in both arrival orders, and a late `Applied` after a `Fault` never confirms anything. Pure protocol logic, no DDS. |

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
| `stderr_backpressure` (211) | The log consumer is never released and the pipe is actually full when `SIGTERM` arrives, so the logger is parked inside its sink: the runtime still reaches `ZeroTorque`, confirmed over DDS, leaves with exit 0 inside the bound without the reader resuming, closes its trace completely and keeps its control deadlines. The final clean-shutdown diagnostic may be dropped by the full pipe - the exit itself, the confirmed state and the complete trace are the assertions. | "the shutdown path is bounded", "a slow log must not block control" |
| `write_refused` (210) | The fake backend refuses the write of a tracking tick: the refusal latches as a fault, the runtime commands zero torque instead of the refused target, and only a matching `Reset` clears the fault once the write path is healthy again. The trace carries the refused attempt and the effective write of every such tick. | "fault first-tick output", "`Applied` is a host-path acceptance, never a device ack" |

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
| Configuration and versioning executable, installable runtime, format check, CI | `config_test`, `config_schema_contract_test`, `runtime_cli_test`, `format-check` and the CI workflow |

## Timing claims

The deterministic suite asserts exact tick semantics; the integration suite
records measured timings and compares them against the configured deadlines with
an explicit scheduling tolerance (100 ms on the fault and fallback observations,
bounded final waits of 10 s per scenario). Those tolerances account for ordinary
operating-system scheduling **on the test host**. They are not a hard
real-time proof and this phase does not claim one: no test here demonstrates a
bounded worst-case latency under load, and none is run on the target hardware.

## What is not covered

* **Real hardware.** There is no `RealBackend`, no CAN or EtherCAT transport and
  no device in this phase; every test above runs the fake backend.
* **Physical execution confirmation.** `Applied` proves that the host send path
  accepted an output, never that a device acted on it. No test verifies actual
  motion, torque or brake behaviour, and no safety number here is a validated
  machine limit.
* **Firmware loss behaviour** and the recovery of a real drive are untested.
* Later phases stay out of scope and are not simulated as done: the real
  application's two-layer acceptance, ONNX inference consistency, the lifecycle
  of an external producer, long-running recorder tests and the hardware
  acceptance of the overall report are not covered by any test in this phase.
* The test client `runtime_client`, the confirmation helper and the fake
  scenario files are test artifacts. They are not a delivered client API, they
  are not installed, and nothing in the runtime calls into them.
* Telemetry is not part of the protocol: `logger_test`, `slow_logger` and
  `stderr_backpressure` cover the bounded logger's own contract and the bounded
  exit diagnostics, not a wire format. A diagnostic that a jammed pipe destroys
  is dropped by design, so `stderr_backpressure` asserts the exit, the confirmed
  state and the trace rather than the line.
