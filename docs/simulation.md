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
an arbitrary finite base-pose override is not admission. Re-prepare old assets
explicitly to obtain an initial-geometry declaration. No automatic pose lift,
blend, reset, warmup support or post-release refixing is provided.

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
Host operation metrics are never labeled logical control latency. See
[recording](simulation-recording.md) for units, association and relocation.

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
Task10 changed adapter bytes: old engine hashes are stale and cannot be bypassed.
Reference-vector agreement proves component numerics, not GPU/DDS reliability.
A contract/toy fixture on the actual robot can prove backend integration only;
it cannot upgrade trained-policy acceptance. The joined all-axis GPU feedback-to-policy-observation path is not fully
established by the constant model, which ignores its inputs. There is no HIL or physical motor
execution evidence and no overall trained-policy acceptance claim.

Installation scans pinned private shared-library directories explicitly. CMake
may warn that transitive `libddsc` was found in the supplied search directory:
the pinned C++ DDS library has no rpath of its own. This is resolved by bundling
the located library and verifying the moved prefix with relative executable
rpath; unresolved dependencies fail installation. No dependency pin change is
needed. Use `ldd` on the moved runtime to verify resolution and absence of ONNX,
Python, MuJoCo and Warp.
