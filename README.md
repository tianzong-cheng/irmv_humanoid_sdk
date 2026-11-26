# IRMV humanoid SDK — execution runtime and supervising application

The humanoid runtime: the standalone execution runtime (`robot_runtime`), the
supervising upper process (`application`) that drives it, the frozen protocol
they speak over real DDS, and the deterministic and real-DDS suites that prove
both. The default build is the **Fake** deployment. An opt-in real build adds
the CAN-FD/IMU backend and the maintenance program; it has no hardware evidence
yet and makes no physical-execution claim.

## Build

Requirements: a C++17 compiler, CMake ≥ 3.22, Python 3 (test harness and
tools; `numpy` and `onnx` are needed when the application is built, the
default, because the build generates the policy model fixtures), and network
access on the first configure — the pinned matched CycloneDDS 11.0.1 pair (C
runtime and C++ binding) and the pinned nlohmann/json are fetched and built
from their pinned commits, and ONNX Runtime 1.22.0 is downloaded and SHA-256
verified when the application is configured, unless a prebuilt prefix is
given. The real-backend submodule requires CMake ≥ 3.26.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

A prebuilt CycloneDDS prefix (built by the two-stage recipe in
`cmake/Dependencies.cmake`) shortens configuration:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
      -DIRMV_CYCLONEDDS_PREFIX=/path/to/cyclonedds-prefix
```

`robot_runtime` and `application` are the two processes these phases ship.
`read_record` is always installed; the application build adds
`external_producer` and `verify_policy_sample`. `runtime_client`,
`confirmation_test` and the `*_test` binaries are test-only and are never
installed.

## Launch an installed SDK

Use `cmake --preset sim -DIRMV_SIMULATION_PYTHON=/path/to/sim-venv/bin/python`
for a workstation, or `cmake --preset robot` for a robot (with the pinned
hardware dependencies available). Build with `cmake --build --preset sim` or
`cmake --build --preset robot`, then install
with `cmake --install build/sim --prefix /path/to/sim-sdk` or
`cmake --install build/robot --prefix /path/to/robot-sdk`. Use a separate,
empty prefix for each package: CMake install does not remove files left by
another preset. These commands use the standard `bin`, `etc`, `lib` and
`share` layout.

Provision the deployment's `etc/irmv/deployment-sdk.json` once: a
[prepared simulation config](docs/simulation.md) for `sim`, or an explicit
hardware config for `robot`. Use stable asset and model paths. The ordinary
development build instead defaults to the shipped Fake `etc/irmv/sdk.json`.

```sh
/path/to/sim-sdk/bin/irmv-sdk                 # default deployment
/path/to/sim-sdk/bin/irmv-sdk --web --viewer  # local Studio and native viewer
/path/to/sim-sdk/bin/irmv-sdk --config /path/to/sdk.json --domain 3
```

The `sim` package includes Studio but starts it only with `--web`; the
`robot` package excludes Studio and simulation tooling by default. Viewer
flags and unavailable features are rejected before starting the SDK. In
simulation, `x` leaves the application while Ctrl-C requests whole-experiment
shutdown. Starting a robot deployment does not automatically start motion.
For live Studio, enable `telemetry.recorder.enabled` in the deployment config.
Studio uses the configured recorder directory and prints its local URL.

## Run the default Fake deployment

The shipped configuration describes a two-joint fake robot at 500 Hz, DDS
domain 1 on interface `lo`, and logging into the relative `logs/` directory of
the process working directory:

```bash
build/robot_runtime --config config/sdk.json
```

```text
usage: robot_runtime --config PATH [--domain N] [--fake-scenario PATH] [--output-trace PATH]
```

* `--config PATH` is mandatory. The file is validated by the loader's own
  checks - `config/sdk.schema.json` documents them and a contract test keeps the
  two in step, but the schema file itself is not consulted at runtime. In the
  default build `backend` must be `fake`, and a configuration that selects
  `real` is refused before any device, DDS or log directory is touched.
* `--domain N` overrides `dds.domain` for this run (0..230). The override is
  applied by the configuration layer, so the published `config_id` describes
  the configuration actually in effect, domain included.
* `--fake-scenario PATH` injects scripted feedback and write acceptance into
  the fake backend (`tests/support/fake_scenario.json` is a documented
  example). It never adds a protocol topic and is refused for a non-fake
  backend.
* `--output-trace PATH` writes one JSON line per torque write attempt, so a tick
  whose write was refused appears twice: the refused attempt and the effective
  zero-torque fallback. It is a test-only option of the fake deployment; without
  it nothing is written.

Startup is ordered and fails loudly: parse the command line, load and validate
the configuration, build the runtime identity, initialise the fake backend,
construct the state machine and the logger, start the logger, then start the
transport. Every failure before the control loop reports on stderr and exits
non-zero (exit 1 for configuration, scenario and startup errors, exit 2 for a
usage error). A failure at the transport stage can already have created DDS
entities of this process; no command is sent.

The runtime reports its state on stderr through the structured logger
(`[runtime] runtime started`, `BootCompleted`, `status`, `shutdown`, …); the
same records are written into `logs/run_<utcstamp>/humanoid.log`.

### It is one process among independent processes

`robot_runtime` owns the control loop, the safety gates and the DDS endpoints.
A supervisor is a **separate process** that speaks the frozen protocol on the
frozen topics — it acquires a session, sends heartbeats and target commands,
and reads results and robot state. Nothing in the runtime calls into a client
and no client shares state with it: the phase ties them together only through
DDS, which is why the integration tests run the real binary against a real
client process. There is no in-process client library in this phase; the test
client (`tests/integration/runtime_client.cpp`) is a test artifact, not a
deliverable API.

### Leaving a behaviour is not shutting the process down

Two different things, deliberately kept apart:

* A **behaviour** is left when a deadline or an operator request takes the
  machine out of the state it was in - a command timeout falls back from
  `Tracking` to `Damping` keeping the session, a supervisor timeout revokes the
  session and damping without authorization, a fault latches. The runtime keeps
  running in every one of those cases.
* A **shutdown** is the process ending. `SIGINT`/`SIGTERM` asks for it, and the
  runtime exits with code 0 only once it is in `ZeroTorque` *and* the torque
  write of the tick was accepted. From `Boot`, `Tracking` or damping without a
  session it requests the protocol stop it can request, reports why it is still
  running, and keeps controlling until the confirmation arrives; under a
  latched fault it refuses to leave at all (a Reset is required, an exit
  request never clears a fault and never reports a clean stop). A hard kill is
  always possible and always reported as such: it is not a clean shutdown.

### Backends

The default build selects the fake backend; `deployment: fake_only` and
`backend: fake` remain the shipped demo. A second pair, `deployment: real` and
`backend: real`, is accepted only when the file also carries the hardware table
described in `docs/hardware-config.md`, and only in a build configured with
`-DIRMV_ENABLE_REAL_BACKEND=ON`; otherwise the runtime refuses to start before
touching a device or DDS.

```bash
cmake -S . -B build-real -DCMAKE_BUILD_TYPE=Debug \
  -DIRMV_ENABLE_REAL_BACKEND=ON \
  -DIRMV_IRMV_CONTROL_SOURCE_DIR=$PWD/deps/irmv_control \
  -DIRMV_YIS_IMU_SOURCE_DIR=$PWD/deps/yis_imu_driver
cmake --build build-real -j2
```

The real build links the pinned driver submodules, takes an exclusive device
lock before opening a device, and reports the complete 32-bit device error and
the host send result. Opening a device sends no frame; enabling is explicit and
per motor. The vendor send is synchronous and its worst-case wait must be
measured on the bench before claiming a control-period bound. `Applied` still
means the host send path accepted the output, never that a motor applied it. There is **no physical-execution
evidence** yet: see `docs/hardware-migration.md`, `docs/maintenance.md` and
`docs/bench-acceptance.md`.

A runtime-only build (`-DIRMV_BUILD_APPLICATION=OFF`) builds `robot_runtime`
without ONNX Runtime or the upper process. The maintenance program is built with
`-DIRMV_BUILD_MAINTENANCE=ON` (requires the real backend). `tools/sdk_launch.py`
starts the two processes and, unless `--stop-all` is given, leaves a runtime
that outlives the application running. Before starting the application it
prints the configured key bindings (`input.bindings`), so the operator sees the
actual map even when a key is rebound.

## Run the application: the supervising upper process

`application` is the upper layer of the two-process fake deployment. It never
starts, stops, signals or holds the process id of the runtime: it enters through
DDS alone, and the lower layer keeps running - and keeps damping - after the
upper one is gone. Start the runtime first, then the application, on the same
domain and with the same configuration file:

```bash
build/robot_runtime --config config/sdk.json --domain 3
build/application  --config config/sdk.json --domain 3
```

```text
usage: application --config PATH [--domain N] [--locomotion-source local|external|none]
```

`--config` is mandatory and `--domain N` (0..230) overrides `dds.domain` exactly
like the runtime's option, so both processes describe the same configuration
digest and the same instance. The file carries the runtime section plus the
`application` and `input` sections of the upper layer.

### Keys

| Key | Action |
| --- | --- |
| `space` | emergency stop: latch `OperatorEmergencyStop`, enter `Fault`, request `Damping`. The latch is left only by an explicit Reset |
| `d` | damping stop: `Standby`/`Initializing`/`Ready`/`Locomotion` → `Damping` |
| `i` | initialize: `Damping` → `Initializing` (a trajectory towards `application.initialization.target_position`), then `Ready` once the target is reached and stable |
| `l` | locomotion: from `Ready`, arm the source this run was started with - the local policy or the external producer process - and let the first frame it produces become the first frame of the Tracking request; the confirmed context enters `Locomotion`. Refused from any other behaviour |
| `z` | zero torque: request a zero-torque stop; a confirmed `ZeroTorque` brings a non-faulted behaviour back to `Standby` |
| `r` | reset: the only way out of a latched `Fault`, and only into `Standby`, on a confirmed fresh `ZeroTorque` |
| `x` | exit (see below) |
| `w` / `s` | body-velocity intent forward / backward |
| `a` / `f` | body-velocity intent left / right |
| `q` / `e` | body-velocity intent yaw left / right |

The bindings live in `input.bindings` of the configuration, never in the code: a
byte the file does not bind is inert, and `emergency_stop` must be the space
character. `w`/`a`/`s`/`f`/`q`/`e` maintain and expire the one body-velocity
intent (`input.intent_timeout_us`), which is the command observation the local
policy tracks while it is in `Locomotion` and the intent the application
publishes to an external producer; with no source selected and no `l` pressed,
no intent ever becomes a joint target. `start_locomotion` may stay
unbound, and is then inert like every other unbound byte.

### Status line

```text
[application] behavior=<X> runtime=<Y> output=<Z> session=<owned|lost|none> pending=<operation|-> reject=<reason|-> fresh=<yes|no> init=<ok|refused|-> cause=<fault cause|-> stop=<lower stop reason|-> fault=<runtime fault reasons|->
```

`behavior` is the upper behaviour (`Starting`, `Standby`, `Damping`,
`Initializing`, `Ready`, `Locomotion`, `Fault`; `Locomotion` is entered only
after a configured local policy or an external producer supplies a first frame
and the runtime confirms the tracking context), `runtime`/`output` the lower
layer's own `execution_state`/`output_action` (`Unknown` while no current state
is known), `pending` the operation of the request the application waits on,
`reject` the reason of the last refused request, `fresh` whether a `RobotState`
arrived inside `application.supervision.state_timeout_us`, and `init` reports the
initialization: `ok` while it runs or holds, `refused` for the iteration that
refused an un-runnable plan.

`cause` is the application's own latched fault cause (`-` while it holds none,
otherwise `OperatorEmergencyStop`, `RuntimeFault`, `RuntimeUnreachable`,
`RuntimeRestarted`, `InitializationFailed`, `RequestTimeout` or
`UnexpectedContext`), and `stop` is the lower layer's own reason for the last
stop it made, as its current state reports it (`-` when it reports none or is
not known, otherwise `Requested`, `CommandTimeout`, `SourceStateTimeout` or
`SupervisorTimeout`). `fault` shows the distinct latched reasons from fresh
runtime feedback, for example `cause=RuntimeFault fault=JointVelocityExceeded`.
An application-only fault still reports its own `cause` without inventing a
runtime failure. The interactive simulation launcher aligns the leading
`behavior`, `runtime`, `output` and `session` columns, drops the fields that do
not apply, and colors readiness, warnings and faults; `NO_COLOR` or a
non-terminal stream keeps the plain line. `application.log` is always plain.
The line is printed once initially, then only when the
displayed fields change. A single
request may produce multiple lines as those fields change; a repeated request
that leaves them unchanged produces none. Idle operation does not repeat the
line.

### Exit

`x`, `Ctrl+C` and `SIGTERM` are the same exit: the signal handler writes one
atomic flag, and the loop turns it into exactly the ordered exit the key raises.
Target generation stops, the pending request is cancelled, the runtime is asked
for `Damping`, and the application waits for the `Result` **and** a fresh
`RobotState` that shows the stop, or for
`application.supervision.request_deadline_us`, whichever comes first:

```text
application: loop iterations=N overruns=M worst_gap_us=X worst_rx_us=R worst_tx_us=T period_us=P
application: link results_dropped=A states_dropped=B requests_dropped=C commands_dropped=D heartbeats_dropped=E inbound_rejected=F read_failures=G
application: exit stop=confirmed state=Damping   # the stop was proven, exit 0
application: exit stop=unknown lower=Fault       # the runtime was already in Fault, exit 3
application: exit stop=unknown lower=Unknown     # the stop was never proven, exit 3
```

The first line is the loop's own pacing, printed before the exit line the way
the runtime prints `overruns` and its worst overrun in the clean-shutdown line:
an iteration counts as an overrun when it started more than two loop periods
after the one before it - it skipped a whole turn - and the worst gap is what a
supervision stall looks like from this side. The one after it splits that stall
into the two halves of the transport: the worst time the thread spent draining
the inbound lanes (`worst_rx_us`) and the worst time it spent in the sends the
behaviour asked for (`worst_tx_us`). A gap far larger than both of them is a
stall of the thread itself, not of the link, and the line after it is the link's
own evidence: the samples a lane refused, the samples that failed validation, and
the reads that ended a loop. A silence on the wire with a healthy gap and empty
link counters is neither process stalling - it is a lane that lost samples
between two processes that were both on time.

Exit codes: `0` a confirmed stop, `1` a configuration or startup error (the
hardware was never commanded - a startup that failed at the transport can
already have created DDS entities of this process, it just never sent a
command), `2` a usage error, `3` the bounded exit that could not prove a stop.
A `3` never says `damped`, `stopped` or `safe`: the runtime keeps its own
output rule - the damping it was already commanding, or the fault output it
latched - and this process exits without touching it. The terminal attributes are
restored on every exit path this process can observe, the signal path included.
Exiting the application is **not** shutting the system down; stopping the runtime
and releasing the device is a separate, explicit operation.

### `ZeroTorque` is not a power cut

`ZeroTorque` is a request for zero output torque: it is never a power-off, never
a brake and never a physical guarantee. `space` is a **software** emergency stop
that depends on this host and on the DDS link to the runtime; it is not a
hardware emergency-stop circuit. `Applied` in a `Result` means the host send path
accepted the request, never that a motor acknowledged or executed it. A lost
host, a broken link or a killed process leaves the machine to the runtime's own
watchdogs, which run independently of this process.

### The initialization bound the operator has to respect

`application.initialization.trajectory.duration_us` fixes the largest move the
initialization can admit: its first frame must satisfy the runtime's continuity
bounds and no frame may exceed the runtime's target velocity limits, so a start
position too far from `application.initialization.target_position` for that
duration is **refused** - the application prints `init=refused`, stays in
`Damping` and sends nothing - rather than attempted. Raising `duration_us` (and
`deadline_us` with it) admits a larger move. The shipped values are
demonstration values for the fake deployment and are **not bench-approved**.

### What these phases do not ship

The shipped configuration carries no trained policy and selects the Fake
backend (`application.policy.model_path` is null); the opt-in real backend has
no physical-execution evidence yet. A `Locomotion` behaviour therefore needs a
locomotion source chosen at startup: the local model the configuration points
at, or an external producer process. The run recorder is shipped but disabled
in the shipped configuration. Nothing here is a validated machine limit - every
gain, tolerance and limit is a nominal training or demonstration value - and the
phase-4 bench acceptance is still owed. The upper layer never re-implements a
lower-layer safety rule: the runtime's limits, watchdogs, fault rules and
damping stay independent of whether the application runs at all.


## The policy model contract (phase 3)

A local locomotion policy is a `policy.onnx` that carries its deployment
parameters in one model-level metadata key, `irmv.deployment`, whose document is
frozen by `schema/onnx-deployment-v1.schema.json`. The model is the single
source of the trained values - the ordered action joints, the default pose, the
per-joint kp/kd, the action scale and resolved offset, both clip layers, the six
observation terms and the normalizer - and `config/sdk.json` stays the single
source of the deployment's own facts (the joint table, the wiring, the limits,
the initialization gains, the fallback damping). Neither file silently
overrides the other, and the runtime never reads a model at all.

```bash
# Point the application at a model and the experimental overrides that match it.
#   "application": { "policy": { "model_path": "policy.onnx",
#                                "inference_timeout_us": 100000 } }
python3 tools/verify_policy.py before.onnx policy.onnx
```

The source of the locomotion behaviour is chosen at startup and never hot
swapped. With no `--locomotion-source` the application follows its
configuration: a configured `model_path` selects the local model, and a
configuration without one - the shipped Fake deployment - runs with no source at
all, exactly as the earlier phases did, so pressing `l` arms nothing and the
behaviour keeps holding `Ready`. `--locomotion-source local` demands a model
path and fails at startup without one. `--locomotion-source external` runs the
producer lifecycle of `docs/runtime-protocol.md` - a separate producer process
registers, is prepared, and is authorized after the runtime applied the context
this application asked for - and `none` states the no-source case explicitly.
No mode is a hot swap: changing the source is a stop, an initialization and a
new authorization, which is the next section.

A recorded run carries a second file beside the samples: on every confirmed
locomotion context the application appends one line (schema
`irmv.policy_session` v2) to
`<telemetry.recorder.directory>/app_run_<UTC>_<unique>/policy_sessions.jsonl`
with the identity the runtime granted the context (`runtime_id`, `session_id`,
`producer_process_id`, `epoch`) and what produced it - for a local model, its
digest, its path, its I/O names, the digest of the `irmv.deployment` document it
declared and one row per gain the operator overrode, with the model's value and
the value that was sent; for an external producer, the process the runtime gated
and the gains its first frame declared. The runtime's own `sessions.json` cannot
carry that: the runtime never reads a deployment contract and links no
inference, so it names the model `unknown` and the application's file is what a
reader joins it to.

`tools/verify_policy.py` is the gate between the training export and this
deployment: it checks the model with the ONNX checker, refuses a model whose
document does not conform to the schema or does not agree with the graph it is
embedded in, proves the graph and the weights did not move when the document was
written, and compares the outputs of the two files on a fixed input. The
training side writes that document from the instantiated environment; see
`docs/testing.md` for the contract the recorded run produced and for what is
still missing before a `Locomotion` behaviour drives real hardware. The Fake
deployment enters `Locomotion` when a source is configured; no
physical-execution claim is made.

## Run an external producer (phase 3)

The other locomotion source is a *producer process*: the application selects it,
asks it to prepare a first frame, asks the runtime to authorize the context that
frame belongs to, and tells it when that context ends. Nothing of the
preparation reaches the execution topic, and the frames of the authorized
context are published by the producer itself - the application publishes none of
them, so the authorization has exactly one writer. The lifecycle, the six
endpoints and their QoS are frozen in `docs/runtime-protocol.md`.

```bash
# one shell: the runtime, as always
robot_runtime --config config/sdk.json --domain 1
# a second shell: the application, with the producer source
application --config config/sdk.json --domain 1 --locomotion-source external
# a third shell: the example producer, installed next to the two processes
external_producer --config config/sdk.json --domain 1 --rate 50
```

Then the documented path: `d` (Damping), `i` (Initialize) to `Ready`, and `l`.
The application reports `producer=selected`, then `producer=prepared` once the
first frame is in hand, and `producer=authorized` when the runtime applied the
context; the producer reports `producer=publishing` per frame from the sequence
the request carried on. `d` ends the context and tells the producer to stop.

A producer that is not running yet is not a startup error: the application
supervises its link as always, reports `producer=missing timeout_us=...` once
after `application.external_producer.registration_timeout_us`, and selects a
producer that starts later.

`examples/external_producer.cpp` is an example and a test peer, not a machine
program: the gains, amplitudes and tolerances it uses are demonstration values
of the Fake deployment and are not a validated machine limit. It is also the
peer of the external acceptance test, which drives the whole lifecycle through
the real application, the real runtime and this process over real DDS.

## Recorded runs

Recording is disabled in the shipped configuration; it writes megabytes per
minute and is an explicit choice. Enable it in the shared `telemetry.recorder`
section (the runtime and the application read the same one):

```json
"telemetry": {
  "recorder": {
    "enabled": true,
    "sample_hz": 100,
    "queue_capacity": 256,
    "directory": "logs/runs"
  }
}
```

`sample_hz` must divide `runtime.control_hz` for the runtime recorder; the
application's velocity recorder uses the same rate with its own schedule. Both
processes write independent directories under `directory` (a `null` directory
means the log directory):

* the runtime writes `run_<monotonic-microseconds>/` with `meta.json`,
  `index.json`, `chunk_<n>.bin`, `sessions.json` and `faults.jsonl`;
* the application writes `app_run_<UTC>_<unique>/` with `meta.json`,
  `velocity.jsonl`, `index.json` and `policy_sessions.jsonl`.

The runtime's chunks are **schema 4**; a run whose metadata, index or chunk
declares schema 3 or an unknown version is refused by name, never reinterpreted.
Each row is one coherent control tick: the published state, the command in
force, the feedback that tick used and the output it produced. Joint target and
measurement columns are in the configuration's joint order (`q`/`dq`/`tau_ff`,
`kp`/`kd`, `measured_q`/`measured_dq`, `measured_tau_est`, `output_torque`,
plus the per-joint `joint_valid` flag). IMU columns are
`imu_orientation_rpy` (roll, pitch, yaw; rad), `imu_angular_velocity` (rad/s),
`imu_linear_acceleration` (m/s²) and `imu_rx_stamp_us`. Three limits matter:

* the driver contract fixes neither the IMU axis frame/order nor whether the
  acceleration includes gravity; `meta.json` records those as `unknown` and the
  recorder never transforms a measurement;
* `measured_tau_est` is the feedback's torque *estimate*, not a torque-sensor
  reading and not the commanded torque;
* `joint_valid` is the raw per-joint flag; it does not replace `feedback_valid`
  or the freshness checks. `send_accepted` is the host send path's answer, never
  a device acknowledgement.

The two directories are **never joined by row index**. Both metadata files carry
`{linux_clock_monotonic, boot_id, available}`; association requires the same
clock and boot, and an application row with `identity_known: true` whose
`runtime_id`/`session_id` appears in the runtime run's `sessions.json`. Rows of
another boot, instance, session or unknown identity are excluded and counted.
There is no cross-host clock synchronisation and no interpolation of a missed
sample.

`faults.jsonl` is the complete occurrence history, separate from the wire:
`RobotState.faults` remains the **first latched fault** and does not change until
a matching Reset. The sidecar records the first appearance of each key
(`{condition, joint_index}`, plus `error_code` for driver and write failures), a
sustained condition only once, and a recurrence after it disappears as a new
line. Non-finite values are the explicit strings `"NaN"`, `"+Inf"` and `"-Inf"`.
The index's `faults` block carries generation/queue/write loss spans and a
`complete` marker; a killed run is incomplete (`complete: false`), and a
`faults.jsonl` line that was cut short is reported as truncation, not data.
There is no fsync or durability promise.

Both metadata files carry `sdk_git_sha`: the commit `HEAD` of the build's source
tree when that tree is the Git root, or `unknown` without Git/a commit. It
identifies the build's commit, **not** a clean or exact source tree - uncommitted
changes are not captured. The application's metadata states
`model.status = loaded | external | none | unloaded`; only `loaded` carries a
model name, path and SHA-256, and that SHA-256 is the application's own loaded
model. An external source is a producer process and leaves the application's
model block null rather than inventing a name.

Read and export a recorded run with the installed reader and the Python tool
(the exporter consumes `read_record`'s output; it is a source tool, not an
installed binary, and needs `numpy`):

```bash
# Runtime run: rows in [from_us, to_us) and the summary.
build/read_record --run logs/runs/run_<id> --summary-only
build/read_record --run logs/runs/run_<id> --from-us 1000000 --to-us 2000000

# Application run: velocity and policy-session rows.
build/read_record --run logs/runs/app_run_<UTC>_<unique> --summary-only

# NPZ of the runtime run, optionally associated with the application run.
python3 tools/record_to_npz.py logs/runs/run_<id> run.npz \
  --reader build/read_record
python3 tools/record_to_npz.py logs/runs/run_<id> run.npz \
  --reader build/read_record \
  --application-record logs/runs/app_run_<UTC>_<unique> \
  --from-us 1000000 --to-us 2000000
```

`--from-us` is inclusive and `--to-us` exclusive, both on the monotonic row
stamp. A window does not change association: the exporter associates and
computes hold-forward over the whole application run first, then selects the
rows to write. `velocity_*` arrays keep their own length;
`velocity_hold_forward/lateral/yaw` are attached to the runtime rows and are
`NaN` outside a contiguous recorded segment (before the first sample, past one
period after the last, across a gap or an indexed loss). `fault_*` arrays also
have their own length and use `-1` for a joint-less fault.
`association_ok`/`association_reason` state whether the application stream
joined; a refused association still writes the runtime archive.

A recorded run is evidence about this host, this boot and the code that wrote
it, not a device: the Fake backend has no physical-execution evidence. See
[`docs/runtime-protocol.md`](docs/runtime-protocol.md) for the complete schema,
association rules and fault-history contract, and
[`docs/testing.md`](docs/testing.md) for the tests that prove them.

## Install

```bash
cmake --install build --prefix /tmp/irmv-sdk-stage
```

The installed tree is self-contained and portable — it can be copied to another
machine and run from any working directory:

```text
<prefix>/bin/robot_runtime                      the runtime
<prefix>/bin/application                        the supervising upper layer
<prefix>/bin/external_producer                  the example producer
<prefix>/bin/read_record                        the recorded-run reader
<prefix>/bin/verify_policy_sample               the policy reference tool
<prefix>/bin/irmv_maintenance                   the bench tool, when built
<prefix>/etc/irmv/sdk.json                      the default Fake configuration
<prefix>/etc/irmv/sdk.schema.json               its schema
<prefix>/lib/libddscxx.so*, libddsc.so*         the DDS libraries, when they
                                                are not provided system-wide
<prefix>/lib/libonnxruntime.so*                 the inference runtime, when the
                                                application is built
<prefix>/lib/                                   the pinned vendor libraries,
                                                when the real backend is built
```

The installed binaries find their libraries through the relative rpath
`$ORIGIN/../lib`; no installed artifact carries a build or source path in its
dynamic section (a Debug build still embeds source paths in its DWARF debug
information, so ship a release build if the tree must be path-clean). The
installation bundles every shared library the processes resolve outside the
platform directories and fails if one cannot be resolved, and it verifies that
each installed binary that needs the bundle has exactly that one relative rpath
entry.

```bash
mkdir -p /tmp/run && cd /tmp/run
/tmp/irmv-sdk-stage/bin/robot_runtime \
    --config /tmp/irmv-sdk-stage/etc/irmv/sdk.json --domain 227
# ... BootCompleted {from="Boot" to="ZeroTorque"} ...
# In a second shell, the same staged configuration and the same domain:
/tmp/irmv-sdk-stage/bin/application \
    --config /tmp/irmv-sdk-stage/etc/irmv/sdk.json --domain 227
# [application] behavior=Standby runtime=ZeroTorque output=ZeroTorque ...
# Ctrl-C in *this* shell (the application's): the upper process exits through
# its ordered stop, and the runtime in the other shell is not touched:
# application: exit stop=confirmed state=Damping
# It is the runtime's own shell that stops the runtime:
# (there) Ctrl-C -> robot_runtime: clean shutdown: state=ZeroTorque ticks=... overruns=...
```

## Formatting and CI

Repository formatting is managed by pre-commit:

| Scope | Formatter |
| --- | --- |
| First-party C/C++ | clang-format 18.1.8 with the root `.clang-format` |
| First-party `.py` files | Ruff import sorting, default lint checks, and formatting |
| `web/` JavaScript, CSS and HTML (including tests) | Prettier (single quotes in JavaScript) |

Generated code and vendored dependencies are excluded. With Python 3.10+
and Node.js 20+ (`node` and `npm`) available:

```bash
python3 -m venv .tools/format
.tools/format/bin/pip install -r tools/requirements-format.txt
.tools/format/bin/pre-commit install --install-hooks
.tools/format/bin/pre-commit run --all-files
```

The commit hook formats staged files. If it rewrites anything, review and
stage the formatted files before retrying the commit. CI runs the same hooks
on all tracked files and fails if any formatter makes changes. Web formatting
does not add a build step or browser runtime dependencies.

For C/C++ only, the existing CMake targets remain available after configuring
the build:

```bash
cmake --build build --target format-check  # read-only; prints diffs
cmake --build build --target format        # formats in place
```

Both targets use `tools/format.py`, the same pinned formatter and the same
first-party source selection as the C/C++ hook. CI does not run a second
CMake format check.

## Documentation

* `docs/runtime-protocol.md` — the frozen topics, message fields, enums, QoS
  and the identity/session/epoch, timeout and fault semantics, plus the external
  producer interface and the application's side of its lifecycle.
* `docs/testing.md` — every deterministic and real-DDS test with what it
  proves, how to run the suites, the DDS domain policy, the phase-3 acceptance
  runs with their numbers, and what this phase does not cover.
* `docs/hardware-config.md` — the real-backend configuration, loader rules,
  unit conversion and device-lease semantics.
* `docs/hardware-migration.md` — provenance of the hardware facts and the
  candidate wiring; `docs/bench-acceptance.md` — the software evidence and the
  still-unrun bench matrix.
* `docs/maintenance.md` — the `irmv_maintenance` commands, dry-run rules and
  device ownership.
* `docs/superpowers/plans/` — the phase plans and the per-phase handoff reports
  (a local, untracked directory; the handoff names the commits, the evidence and
  the remaining risks).
