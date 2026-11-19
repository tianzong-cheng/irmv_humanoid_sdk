# Local lockstep simulation

Simulation is an opt-in two-process SDK deployment with an independent engine
owner. Default `config/sdk.json` remains Fake. `robot_runtime` commands torque;
`application` owns behavior and policy and never starts, stops or signals the
runtime. Neither runtime nor its hardware backend imports Python, MuJoCo, Warp
or ONNX. The engine extension is the worker-owned `EngineAdapter` interface in
`tools/simulation/engine.py`; a future Isaac Sim adapter must implement the same
coherent feedback, completed-step, torque, release and close contracts. No
alternate engine has been validated.

Use the pinned isolated environment described in
[simulation dependencies](../tools/simulation/requirements.txt) and explicit
prepared assets. No CPU substitute counts as GPU acceptance. Dependency pins
are MuJoCo 3.9.0, MuJoCo Warp 3.9.0 and Warp 1.15.0. Asset preparation attests
resolved mesh/include bytes, conversion choices, source URDF, policy and initial
stance. The binding refuses changed bytes and ambiguous file resolution. The
initial configuration must equal the converter's admitted flat-floor stance;
an arbitrary finite base-pose override is not admission. An explicit preparation
stance may be supplied with `--initial-pose` as a JSON object (inline or a file
path) mapping every controlled joint name to a finite angle within its URDF
limits. It drives both the compiled support-height computation and the
application initialization target; without it the policy `default_q` remains
the stance. Re-prepare old assets explicitly to obtain an initial-geometry
declaration. No automatic pose lift, blend, reset, warmup support or
post-release refixing is provided.
Live launch requires asset-manifest `collision_support_version=1`; older
declarations must be regenerated with `prepare_simulation_asset.py`, not patched
or grandfathered. Historical records remain available for offline analysis.

The DDS dependency is the matched CycloneDDS 11.0.1 pair (C runtime and C++
binding, commits recorded in `cmake/Dependencies.cmake`). It carries the upstream
fix for the listener-argument use-after-free that could crash a process during
endpoint construction while discovery matched a live peer: 200 measured
startups on the old 0.10.5 pin crashed 4 times, 20 fresh startups plus 1000
create/destroy churn rounds on 11.0.1 crashed none. `0.10.x` has no release
containing that fix, so it cannot be pinned safely.

11.0.1 is not wire-compatible with an `0.10.5`-built peer: the two IDL
generators publish different XTypes type information, and the `lo` discovery
assumption differs, so mixed pairs never match. Build `robot_runtime`,
`application` and `external_producer` from the same tree; an old prefix or
manifest is refused by configure rather than silently accepted.


```sh
# All paths are explicit; consult --help for required preparation arguments.
python3 tools/prepare_simulation_asset.py --help
python3 tools/sim_launch.py --config /path/to/prepared/sdk.json \
  --runtime "$PWD/build/robot_runtime" --application "$PWD/build/application" \
  --python "$PWD/build/simulation-venv/bin/python"
```

The launcher verifies binding, reserves a new private directory, initializes the
engine and only then starts SDK peers. Application `x` completes Damping and
leaves the runtime alive. Ctrl-C to the independent launcher requests a fresh
ZeroTorque write and engine close. A crash/disconnect without that proof is
stopped-unproven; a host-accepted write never proves physical motor execution.
Keep the experiment root short enough for the loader's 80-byte socket-directory
bound (for example a new private root under `/tmp`). Do not reuse an existing
run/socket directory. Preserve failed experiments.

SDK logical time is integer microseconds, beginning at 1. Physics dt divides
control dt and policy dt is a control-period multiple. One command drives the
following completed physics interval: the baseline is one control cycle.
Runtime state sequence is not a physics step counter. Host monotonic operation
and watchdog durations remain separately named host measurements. Slower
rendering/inference/physics must not manufacture another application cycle or
age logical commands while the lockstep phase is waiting. Frozen termination
can accept a fresh zero write without advancing physics/logical time.

The base is held by a solver weld, not mathematical fixation. Release requires
actual application Locomotion intent, the matching first command accepted by
runtime, and application consumption of Applied plus Tracking. The next worker
interval disables the weld once; q/dq/base state remain continuous. Feet/plane
placement and finite feedback are experiment admission requirements, not added
runtime safety rules or evidence of balance.

## Acceptance evidence

Ordinary builds register standard-library deterministic and Fake/process tests
without optional GPU packages. Real socket, host-watchdog and subprocess tests
are integration tests. Run DDS tests serially; reserve unique domains. GPU
capability and short lifecycle checks require explicit configuration:

```sh
cmake -S . -B build-gpu -DIRMV_TEST_SIMULATION_GPU=ON \
  -DIRMV_SIMULATION_PYTHON=/path/to/pinned/python \
  -DIRMV_SIMULATION_TEST_CONFIG=/path/to/fresh/prepared/sdk.json
WARP_CACHE_PATH="$PWD/build/warp-cache" ctest --test-dir build-gpu -L gpu --output-on-failure
```

Four-scenario acceptance additionally requires
`IRMV_TEST_SIMULATION_POLICY_ACCEPTANCE=ON` and
`IRMV_SIMULATION_ACCEPTANCE_MANIFEST=/path/to/spec.json`. The registered test
has TIMEOUT 900 and RUN_SERIAL. The specification supplies `config`,
`training_env`, extracted `deployment` contract JSON, exclusive `output`,
`scope` (`trained_policy` or `backend_integration_fixture`), `intent_bounds`,
`fall` (body, world frame, origin_offset_m, minimum_height_m), and optional
`host_timeout_s` (default 180 per scenario). Paths are explicit operator inputs.
The runner creates one new process experiment per supported scenario; no reset
API, automatic recovery, calibration or safety relaxation. Required duration is
60 logical seconds after release, independently for standing/forward/lateral/yaw.
InputRouter unit keys must fit the source bounds. Held intent is refreshed on
logical time, not wall time. An unsupported axis is not a successful zero command.
A byte-bound `known_failed_prerequisite` (reason plus path/sha256 files) can
produce explicit failed prerequisite reports without repeating an unchanged
failed startup. Such reports say no current rollout executed.

Each output includes a run manifest binding config/bootstrap/sidecar/owner
outcome, policy/contract/source, engine bytes/versions, prepared closure and SDK
reader exports by SHA-256. Evaluate it with:

```sh
PYTHONPATH=tools python3 -m simulation.evaluate /experiment/manifest.json \
  --output /experiment/evaluation.json
```

The evaluator streams the new `simulation.record.read(visitor=...)` API;
materializing evidence through its convenience list is capped at 65,536 rows.
It replays coordinator events to reject missing/extra command cycles, broken
phase seals, missing first-command/release proofs and inconsistent applied write
identities. Complete file/footer, coherent bootstrap-through-final measurements,
matching run/digests, actual SDK application/policy and final stop proof are
required. Missing, nonfinite, fall, protection, skipped GPU or incomplete data
cannot succeed. Binary-reader exports supply coherent measured-versus-target
joint errors and configured torque saturation. These are measurements, not
invented safety thresholds. Saturation refers to the **configured SDK torque
limit**, whose physical motor validity is not established by this experiment.
World linear velocity is rotated into the gravity-aligned yaw frame. Local
angular qvel is rotated by full measured base orientation to world yaw rate.
Application intent coverage is required on the entire configured sampling grid
inside the released interval, anchored at logical origin 1. Each row must have
the grid's sample sequence and name the exact runtime state and session consumed
for that control step, after the runtime phase acknowledgement and before
application phase publication. The evaluator merges a second streaming replay
with the next velocity row; an internally complete export with a missing prefix,
suffix or interior interval cannot prove persistent intent. First/last stamps,
expected/observed row counts and maximum observed gap are reported. Full-horizon
velocity tracking RMSE stays `null` until both measured and intent coverage are
proved; manifest intentions and completeness flags alone are insufficient.

Failure does not discard observations. Reports retain validated-prefix release,
applied intervals, feedback duration, measured torque, observed fall/fault and
protection facts, plus file/replay/rollout/intent coverage flags. `null` means
unavailable: no observed fault during incomplete rollout coverage is not a measured zero;
no release means fall coverage is unavailable. An observed fall/fault remains
true even if later evidence is incomplete. Counts under incomplete coverage are
observed lower bounds. Separate `recorded_runtime_fault_count` and
`recorded_protection_count` name counts over the available recorded interval;
a known zero there does not establish zero events over an unrecorded 60-second
horizon. Maximum measured torque covers all validated feedback,
including startup; minimum base height covers observed released intervals.
Joint errors and saturation use available SDK numeric rows with a separate
coverage flag. Before attributing any SDK metric or fault, a bounded streaming
scan validates that export's own summary run/configuration/clock identity and
sampling rate (including the application configuration identity for intent).
A manifest SHA identifies the bytes, not their run. Foreign or unidentified SDK
exports contribute no measurements or fault flags; independently validated
engine facts remain available. Identity validation does not require a complete
file: same-run exports with readable identity retain valid numeric/intent
prefixes despite incompleteness or later bad rows. Without a readable identity
summary, SDK quantities remain unavailable rather than being attributed to the
sidecar's run by assumption. Failed-owner outcomes keep these facts and still
fail acceptance.
An application initialization fault is distinguished from a runtime fault.

Host operation metrics are never labeled logical control latency. See
[recording](simulation-recording.md) for units, association and relocation.

Existing artifact groups can be re-evaluated without starting any experiment:

```sh
python3 tests/integration/simulation_policy_acceptance_test.py \
  --reevaluate /path/to/original-group --output /path/to/new-derived-group
```

The output directory must be new. Original manifests, records and reports remain
unchanged and byte-bound; derived reports name their sources. Only the recorded
standing attempt has observed metrics. Forward/yaw reports retain its failed
prerequisite reference and do not inherit standing measurements as their own;
unsupported lateral retains unavailable metrics. This command never creates an
engine, changes a configuration or reruns a model.

## Current evidence and limits

The discovered trained model's saved env SHA-256 is
`fa24a787b854a4a4dd121c08cdbccd7550d87c92393b1f8c9ff9d244c6df75b5`.
It permits forward [0,1.2] m/s, lateral [0,0] m/s and yaw [-1,1] rad/s.
Nonzero lateral acceptance is unsupported; zero lateral is not sideways success.
Training terminates below root world height 0.4m. Bind the measured free-base
origin to the training root before using that condition; do not invent a roll
angle threshold. Tracking rewards use yaw-frame XY and world-frame angular Z.
Historical mesh/simulator binaries are not fully attested; URDF hash matching
alone does not prove equivalent training physics.

Original actual-asset startup with .02rad initialization completion tolerance
failed at approximately .02475rad static compliance. One separately authorized
.03rad completion calibration reached Ready, then Locomotion handoff was refused
with Discontinuity and CommandTimeout; it did not release. These are observed
startup/SDK constraint incompatibilities before a completed policy rollout.
A fresh binding with the current recorder reproduced InitializationFailed at
observed step 1223 (2,446,001 logical microseconds), with measured maximum
initialization error 0.0248077512rad and no release. Forward/yaw reports reference
that same current failed startup; lateral is unsupported. A separate constant
contract fixture completed 200ms after release using the previously empirical
.03rad upper completion tolerance; the trained configuration stayed .02rad.
That short backend result cannot establish 60-second trained-policy reliability.
They do not establish learned-policy unreliability or physical motor unsafety.
The generated configuration partly inherits 20Nm/10rad/s limits and continuity
settings from the two-joint example; these are not verified motor limits.
No additional startup calibration, gain/physics/timing/limit tuning is justified
merely to obtain a passing report. Preserve both original negative artifacts.

The continuation found a separate conversion defect in the old stance: primitive
collision geoms have `geom_dataid=-1`, which the converter had incorrectly used
as the final mesh index. For the actual robot, the box feet were about 84.66mm
too high. Collision support now uses oriented box half-extents, cylinder
radius/half-length, sphere radius or compiled mesh vertices as appropriate;
unknown geometry fails preparation. Corrected default-stance base height is
0.744224409m rather than 0.828880589m, preserving the explicit 2mm clearance.
The old startup figures above describe the old geometry, not the corrected run.
A freshly prepared corrected-geometry CUDA/SDK run still failed initialization
at control step 1077 (2154001 logical us): maximum goal error 0.0218897462rad,
with joint speeds below 0.00035rad/s. The application reported
InitializationFailed; the base remained fixed. Evidence replay and measured
feedback were complete for the observed startup, and independent shutdown
proved runtime ZeroTorque plus engine closure. This is not a successful policy
lifecycle. Standing failed before release; forward/yaw reference that same
byte-bound failed prerequisite without another identical startup, and lateral
is unsupported. No 60-second trained rollout ran. Evidence is retained in
`build/nov16-policy-evidence/` (original group `/tmp/irmv-n16-policy/`).

After explicit user authorization, a separate corrected-geometry experiment
changed only the upper initialization completion tolerance to .03rad. It
reached Ready, then the first actual policy takeover was rejected Discontinuity
at control step 369 (738001 logical us). The first-command proof names source
state 369 (736001us); an offline reconstruction from that exact feedback and
initial last_action gives the following checks against the next runtime state:

| Check | Reconstructed value | Unchanged configured limit |
|---|---:|---:|
| Left ankle pitch position gap | 0.120951rad | 0.1rad |
| Left ankle pitch torque jump | 6.914758Nm | 5Nm |
| Left knee torque jump | 6.479910Nm | 5Nm |

`verify_policy_sample` matched the reconstructed observation within 2.22e-16 and
the action/target exactly. These are reconstructed first-action quantities,
not raw-action recording or physical motor measurements. No blend, gain,
timing, physics or continuity-limit change was made. The base never released;
the owner completed proven ZeroTorque/engine shutdown and both children exited
0. Standing failed at handoff, forward/yaw were blocked by that same byte-bound
prerequisite, and lateral remained unsupported. No 60-second rollout ran.

`build/nov16-completion-003/authorization.json` binds the user authorization and
the original/candidate config hashes; `outcome.json` records the .03rad
disposition and evidence hashes. Raw evidence is copied under its `evidence/`
directory (original group `/tmp/irmv-n16-c003/`). Raw provenance prose inherits
the .02rad baseline description followed by the explicit .03rad override;
the bound per-run SDK configuration and `outcome.json` unambiguously name the
actual .03rad experiment. The original .02rad failure remains unchanged.

The user subsequently authorized disabling Discontinuity for an isolated
diagnostic. `build/nov16-no-continuity/sdk.json` sets all three continuity maps
to finite nonbinding 1e100, retains .03rad completion and leaves guard code,
default/real configuration, gains, physics, torque/speed limits and watchdogs
unchanged. This also removes initialization takeover continuity gating, not
just the policy takeover gate; initialization was admitted earlier, so this
is not a single-policy-handoff-only comparison.

The actual trained-policy CUDA short lifecycle passed: Ready, Locomotion,
one continuous base release at observed control step357, 200ms of policy
rollout, application Damping leave with runtime retained, then independent
ZeroTorque/engine closure. Maximum observed torque was 18.0766869Nm and
minimum base height over the whole short experiment was 0.720734835m.
Configuration/authorization/logs are under `build/nov16-no-continuity/`;
owner evidence is `/tmp/irmv-n16-no-cont-run/evidence.json`.
This proves short startup/motion feasibility with continuity gates bypassed,
not balance, walking reliability, original-configuration acceptance or 60s
trained-policy acceptance. No broader protection bypass was applied.


Continuation diagnosis separates a rejected policy takeover from the subsequent
SDK defect: a rejected Locomotion request now disarms that attempt and retains
the existing Ready generator and its authorized producer/epoch. It does not
retry without another operator request, release the base, or manufacture a
command timeout by abandoning that hold. Unit coverage includes both
local and external candidates; the real-SDK/Fake-engine lockstep case verifies
that the old target stream continues beyond that watchdog interval. No runtime
continuity limit, initialization completion criterion or trained model changed.

An offline first-action replay from the preserved trained initialization sample
(state 1224, logical 2446001us, initial last_action) gives a left ankle pitch
position gap of 0.12999667rad against 0.1rad, and torque jumps of 7.3068246Nm at
that ankle and 7.1011379Nm at the left knee against 5Nm. The SDK
`verify_policy_sample` reproduced the independent observation to 2.22e-16 and
the model action/target exactly. This is a saved-state deployment compatibility
diagnostic, not the original calibrated handoff state or a new policy rollout.
Fixing refusal handling does not make that first action admissible.
Task10 changed adapter bytes: old engine hashes are stale and cannot be bypassed.
Reference-vector agreement proves component numerics, not GPU/DDS reliability.
A contract/toy fixture on the actual robot can prove backend integration only;
it cannot upgrade trained-policy acceptance. The separate opt-in
`simulation_observation_test` now exercises the joined GPU-to-observation path
through production IPC/drivers, SimBackend, DDS, RobotClient and
BuildObservation. Two CUDA runs with distinct joints and gyro XYZ cover
roll/pitch/yaw, reversed observation joint ordering, torque-excited velocities
and SDK yaw-invariant projected gravity. Dedicated test business loops replace
the runtime/application loops; this is feedback-chain proof, not actual
trained-policy inference/lifecycle or long-horizon reliability. There is no HIL
or physical motor execution evidence and no overall trained-policy acceptance.

Installation scans pinned private shared-library directories explicitly. CMake
may warn that transitive `libddsc` was found in the supplied search directory:
the pinned C++ DDS library has no rpath of its own. This is resolved by bundling
the located library and verifying the moved prefix with relative executable
rpath; unresolved dependencies fail installation. No dependency pin change is
needed. Use `ldd` on the moved runtime to verify resolution and absence of ONNX,
Python, MuJoCo and Warp.

## Native real-time viewer

Native visualization is explicit and optional:

```sh
python3 tools/sim_launch.py --config /path/to/fresh/prepared/sdk.json \
  --runtime "$PWD/build/robot_runtime" --application "$PWD/build/application" \
  --python "$PWD/build/simulation-venv/bin/python" --viewer
```

Use `--viewer-follow` instead for an initially base-following camera (it also
enables the viewer). `--viewer-no-shadow` opens the viewer without the cast
ground shadow; lighting, sky gradient and the checker ground stay. That pass
costs about 0.24 ms per control interval on the same GPU the physics engine
uses, which is the difference between roughly 0.9x and 1.0x real time on the
laptop GPU measured here, so it is an explicit opt-in rather than the default.
Native left-drag rotates, right-drag pans and the mouse
wheel zooms. Send SDK behavior keys from the launching terminal, not the viewer
window. Closing the window closes visualization only; application `x` still
leaves through Damping, and Ctrl-C in the launcher requests whole-experiment
shutdown through runtime ZeroTorque and engine close.

Interactive launch prints the configured key map and follows application status
in the launching terminal while retaining the full `run/application.log`.
Keys are immediate: Enter is not required. Wait for Standby first. With the
example bindings, press `d`, wait for `[application] behavior=Damping`, press
`i`, then wait for the actual `[application] behavior=Ready` status before
pressing `l`.
Locomotion input during Initializing is deliberately ignored, not queued for
later execution. Initialization uses logical time and can take longer in wall
time when physics is slow. The viewer's BASE HELD label does not mean keyboard
input was lost: it remains held through initialization and Ready.

The application continues to own terminal mode and stdin; the launcher only
mirrors the existing child-owned log, with bounded reads rather than placing a
pipe in the SDK output path. Application `x` restores terminal attributes and
retains the runtime; launcher Ctrl-C still requires independent stop proof.

`simulation_terminal_test` exercises this sequence through a real controlling
PTY with real SDK/DDS and fake physics, including terminal restoration after
application exit and active-experiment Ctrl-C shutdown. It is not GPU policy
acceptance evidence.

The viewer process owns a separate CPU model/data, uses CPU forward kinematics
only, and displays immutable completed GPU qpos snapshots. It cannot command or
perturb authoritative physics. Logical time and BASE HELD/RELEASED are displayed;
the initial view explicitly says it is waiting for a coherent GPU pose.
Display publication is capped near30Hz of host time, nonblocking and bounded.
A slow display drops visual samples; it cannot add a control barrier or age
logical commands. Display time must not be used as the authoritative evidence
stream. Native UI edits affect only its disposable display copy.

Explicit native startup failure is reported before SDK peers start. Later
window closure/failure retires visualization without cancelling the experiment;
the launcher records separate viewer status and owns bounded child cleanup.
Without either flag, no renderer process or optional rendering import is used.
No pause, single-step, reset, force injection or graphical replay is provided.
This adapter revision changes the engine binding: explicitly re-prepare assets
before launching; never patch or bypass an old digest. Visualization does not
select calibration or continuity-bypass configurations automatically.

Native validation used an actual local desktop and CUDA engine: the robot was
rendered from completed GPU snapshots, the diagnostic policy released the base,
and closing its native window left both SDK processes running. Application
Damping leave and independent ZeroTorque/engine closure then completed with
both children exiting0. Separate native desktop checks exercised rotate/pan/zoom
against a stationary display pose and proved that abrupt owner exit closes the
orphan renderer through socket EOF. Artifacts/screenshots:
`build/viewer-smoke/{visual-proof,camera-proof,orphan-proof}.json` and PNGs.
The motion smoke explicitly used the previously authorized .03rad/continuity-
bypass diagnostic configuration, not product defaults or long-run acceptance.

The display keeps one newest pending pose and sends independently of physics,
so a saturated viewer eventually receives the final completed pose even if
physics subsequently waits. Owner lifetime uses EOF-capable local packets;
unexpected nonzero renderer exit remains an error, not normal window closure.
Standard-library regression coverage includes stalled rendering without further
physics, owner EOF, reordered/foreign/nonfinite poses and delayed child failure.

### Display appearance

The native child adds a daylight headlight/directional light, a sky gradient,
and a light checker material on planes in its private model spec. Robot
materials are retained. Ground friction, collision geometry, solver settings,
joint dynamics and authoritative assets are not changed.
The configured-pose spec loader is shared with the engine so display resources
do not introduce a second pose convention. That refactor changes the adapter
byte binding: explicitly re-prepare assets rather than editing an old digest.
The current local diagnostic was freshly prepared at
`build/viewer-daylight-assets/sdk.json`; use it as the template when generating
a new run directory, not the older `build/viewer-smoke/sdk-final.json`.
It retains the previously authorized diagnostic tolerances/continuity settings,
not product-default or long-run acceptance settings.

Native before/after screenshots are in `build/simulation-timing/lighting-*.png`.
A CPU comparison found 28 physical model arrays, dimensions and timestep
unchanged. Native closure verification exposed an existing daemon GUI-thread
exit race; the child now joins its GUI thread with a bounded deadline before
interpreter teardown. The actual native probe then closed with exit0.

### Simulation time versus wall time

The overlay shows completed **logical simulation time**, not wall time. Current
warm local RTX3070 measurements with MuJoCo/MuJoCo Warp3.9.0, Warp1.15.0 and the
matched CycloneDDS 11.0.1 pair, held base, Standby and zero torque:

|Mode|Mean host milliseconds per 2 ms control interval|Simulation/wall time|
|---|---:|---:|
|Full SDK, headless|1.74|1.15|
|Full SDK, viewer with shadow|2.20|0.91|
|Full SDK, viewer `--viewer-no-shadow`|1.95|1.02|

Each row is a 10 s steady window after 100 warm intervals; all runs ended
`Closed` with independent stop proof, SDK exits[0,0] and a clean viewer exit.
This is a throughput measurement of the SDK pipeline, not locomotion or 60 s
policy acceptance. Viewer cost scales with the retained render passes, not with
the control period: the shadow pass alone is ≈0.24 ms per interval, and no
lossless scene flag (haze, reflection, idle-skip, smaller window) moved the
cycle outside its ±0.04 ms noise band. The host is also thermally throttled
(≈86 °C, 1.75 GHz late in a session) and carries desktop IO pressure, so the same
tree measured 0.98x earlier when cooler.

Getting here from 60.6 ms/cycle took three independent changes, each measured
and regression-covered rather than guessed:

- **Wakeups instead of polling** (`src/simulation/endpoint.cpp`,
  `tools/simulation/main.py`, DDS transports, simulation loop drivers): an
  eventfd now carries control→worker work, a wake-only DDS listener reports
  inbound data to the sole transport thread, worker results wake the
  coordinator selector, and driven waits replace fixed 5/10 ms sleeps. Ready
  work is never parked behind a timeout.
- **CUDA graphs and compact readback** (`tools/simulation/mujoco_warp_engine.py`):
  each control interval replays one captured physics-step graph per substep plus
  a tail graph (endpoint forward, compact gather) with four small copies, and a
  single pinned control upload. Per-substep clock/overflow checks still run
  before the next substep. The engine operation fell from 14.6–16.4 ms to
  1.20 ms standalone while staying bit-identical to the eager path over 300
  cycles including release.
- **Physics/application overlap** (`tools/simulation/coordinator.py`): the
  interval `[t, t+dt)` is issued once the runtime's torque is accepted and its
  phase-1 inventory is proven, and the application phase handshake of the same
  step runs concurrently. The next tick still requires both phase
  completions and the completed feedback; see the protocol document for the
  release offset and the `physics_in_flight` failure semantics this implies.

Raw evidence: `build/simulation-timing/{report,sdk-comparison,final2-headless,final2-viewer,noshadow-viewer,render-opt-sweep-final}.json`,
`engine-profile.txt`, hardware/version metadata and native screenshots.
Older recordings with missing shutdown proof remain failures; the new
successful timing runs do not retrospectively certify them.

## Suspended flat-sole initialization experiment

An explicitly prepared non-training stance under `build/suspended-stance/`
compensates the oblique hip axes. Its nominal sole planes are horizontal and
their lowest collision points are 0.03 m above the ground; the held base is at
0.765292818 m. Training action offsets and model metadata are unchanged.

The actual SDK/MuJoCo Warp run `/tmp/irmv-flat3cm-pf6ymfa1/` reached Ready.
Over 100 subsequent control intervals, left/right sole clearance was
0.02611–0.02738 m and maximum sole tilt was about 0.90 degrees: finite-gain
gravity deflection means actual posture is not the exact kinematic target.
The measured startup/initialization trajectory stayed above 0.01662 m clearance.
Independent shutdown proved ZeroTorque and engine closure. This run did not
enter Locomotion and makes no release, landing or policy-stability claim.

This isolated configuration retains the previously authorized continuity
bypass and initialization tolerance, and position bounds expanded 0.2 rad from
URDF limits. It is not a change to the default Fake/Real deployment.
