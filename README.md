# IRMV humanoid SDK — runtime foundation

Phases one and two of the humanoid runtime: the standalone execution runtime
(`robot_runtime`) for the **Fake** deployment only, the supervising upper process
(`application`) that drives it, the frozen protocol they speak with each other
over real DDS, and the deterministic and real-DDS test suites that prove both.
There is no real robot backend and no real hardware emergency stop in these
phases.

## Build

Requirements: a C++17 compiler, CMake ≥ 3.22, Python 3 (test harness and
tools) and network access on the first configure — the pinned CycloneDDS
0.10.5 and its C++ binding and the pinned nlohmann/json are fetched and built
from their pinned commits unless a prebuilt prefix is given.

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

`robot_runtime` and `application` are the two executables these phases ship;
`runtime_client`, `confirmation_test` and the `*_test` binaries are test-only and
are never installed.

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
  two in step, but the schema file itself is not consulted at runtime. `backend`
  must be `fake`, and a configuration that selects `real` is refused before any
  device, DDS or log directory is touched.
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
usage error) without starting DDS.

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

### No real hardware in this phase

The backend is the fake one; `deployment: fake_only` and `backend: fake` are
enforced by the configuration contract. There is **no RealBackend, no CAN
transport and no physical execution confirmation**: `Applied` means the host
send path accepted the output implementing a request, never that a device
acknowledged it. Real hardware, real safety limits, firmware-loss behaviour and
the physical-execution acceptance belong to later phases.

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
usage: application --config PATH [--domain N]
```

`--config` is mandatory and `--domain N` (0..230) overrides `dds.domain` exactly
like the runtime's option, so both processes describe the same configuration
digest and the same instance. The file carries the runtime section plus the
`application` and `input` sections of the upper layer.

### Keys

| Key | Action |
| --- | --- |
| `space` | emergency stop: latch `OperatorEmergencyStop`, enter `Fault`, request `Damping`. The latch is left only by an explicit Reset |
| `d` | damping stop: `Standby`/`Initializing`/`Ready` → `Damping` |
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
[application] behavior=<X> runtime=<Y> output=<Z> session=<owned|lost|none> pending=<operation|-> reject=<reason|-> fresh=<yes|no> init=<ok|refused|-> cause=<fault cause|-> stop=<lower stop reason|->
```

`behavior` is the upper behaviour (`Starting`, `Standby`, `Damping`,
`Initializing`, `Ready`, `Locomotion`, `Fault`; `Locomotion` has an enumerator
and an entry guard but this phase never enters it), `runtime`/`output` the lower
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
`SupervisorTimeout`). The two together are what tells an operator emergency stop
from an unreachable runtime, and a lower command timeout from a supervisor
timeout; without them a `behavior=Fault` line names no cause at all. The line is
printed whenever a field changes and at most every 500 ms otherwise.

### Exit

`x`, `Ctrl+C` and `SIGTERM` are the same exit: the signal handler writes one
atomic flag, and the loop turns it into exactly the ordered exit the key raises.
Target generation stops, the pending request is cancelled, the runtime is asked
for `Damping`, and the application waits for the `Result` **and** a fresh
`RobotState` that shows the stop, or for
`application.supervision.request_deadline_us`, whichever comes first:

```text
application: loop iterations=N overruns=M worst_gap_us=X period_us=P
application: exit stop=confirmed state=Damping   # the stop was proven, exit 0
application: exit stop=unknown lower=Fault       # the runtime was already in Fault, exit 3
application: exit stop=unknown lower=Unknown     # the stop was never proven, exit 3
```

The first line is the loop's own pacing, printed before the exit line the way
the runtime prints `overruns` and its worst overrun in the clean-shutdown line:
an iteration counts as an overrun when it started more than two loop periods
after the one before it - it skipped a whole turn - and the worst gap is what a
supervision stall looks like from this side.

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

No trained policy of its own and no real backend. The shipped configuration
carries no model (`application.policy.model_path` is null) and every process
runs the Fake backend, so a `Locomotion` behaviour needs a locomotion source
chosen at startup: the local model the configuration points at, or an external
producer process. The run recorder is shipped but disabled in the shipped
configuration. Nothing here is a validated machine limit - every gain, tolerance
and limit is a nominal training or demonstration value - and the phase-4 bench
acceptance is still owed. The upper layer never re-implements a lower-layer
safety rule: the runtime's limits, watchdogs, fault rules and damping stay
independent of whether the application runs at all.


## The policy model contract (phase 3, in progress)

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

`tools/verify_policy.py` is the gate between the training export and this
deployment: it checks the model with the ONNX checker, refuses a model whose
document does not conform to the schema or does not agree with the graph it is
embedded in, proves the graph and the weights did not move when the document was
written, and compares the outputs of the two files on a fixed input. The
training side writes that document from the instantiated environment; see
`docs/testing.md` for the contract the recorded run produced and for what is
still missing before a `Locomotion` behaviour drives the machine.

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

## Install

```bash
cmake --install build --prefix /tmp/irmv-sdk-stage
```

The installed tree is self-contained and portable — it can be copied to another
machine and run from any working directory:

```text
<prefix>/bin/robot_runtime                      the runtime
<prefix>/bin/application                        the supervising upper layer
<prefix>/etc/irmv/sdk.json                      the default Fake configuration
<prefix>/etc/irmv/sdk.schema.json               its schema
<prefix>/lib/libddscxx.so*, libddsc.so*         the DDS libraries, when they
                                                are not provided system-wide
```

`robot_runtime` and `application` find their libraries through the relative rpath
`$ORIGIN/../lib`; no installed artifact carries a build or source path in its
dynamic section (a Debug build still embeds source paths in its DWARF debug
information, so ship a release build if the tree must be path-clean). The
installation bundles every shared library either of them resolves outside the
platform directories and fails if one cannot be resolved, and it verifies that
both installed binaries have exactly that one relative rpath entry.

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

The whole first-party C/C++ tree is formatted with clang-format 18.1.8 and the
single `.clang-format` at the repository root (`BasedOnStyle: Google`, two-space
indentation). Third-party dependencies and generated sources — the generated
IDL sources of `execution_idl` in particular — are never formatted.

```bash
python3 -m venv .tools/format
.tools/format/bin/pip install -r tools/requirements-format.txt
cmake --build build --target format-check   # read-only, non-zero on any diff
cmake --build build --target format          # formats in place
```

Both targets run `tools/format.py`, which refuses any clang-format other than
the pinned version, reports the version and path it found, and prints a unified
diff for every file that is not formatted. `.github/workflows/ci.yml` runs
configure, build, `ctest --output-on-failure`, `format-check` and the install
step on Ubuntu 24.04, plus a second job that builds with
`-DIRMV_SANITIZE=address,undefined` and runs the deterministic core suite
(`ctest -L core`). No stage is skipped silently: a failed dependency fetch, a
failed build and a failed format check all fail the job, and logs are kept as
artifacts on failure.

## Documentation

* `docs/runtime-protocol.md` — the frozen topics, message fields, enums, QoS
  and the identity/session/epoch, timeout and fault semantics, plus the external
  producer interface and the application's side of its lifecycle.
* `docs/testing.md` — every deterministic and real-DDS test with what it
  proves, how to run the suites, the DDS domain policy, the phase-3 acceptance
  runs with their numbers, and what this phase does not cover.
* `docs/superpowers/plans/` — the phase plans and the per-phase handoff reports
  (a local, untracked directory; the handoff names the commits, the evidence and
  the remaining risks).
