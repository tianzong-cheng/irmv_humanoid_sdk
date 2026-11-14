# Simulation recording and offline analysis

Binary records remain schema v4. `meta.json.clock.clock_domain` explicitly
selects `host_monotonic` or `simulation`. Old v4 metadata without this field
keeps its host meaning: `linux_clock_monotonic`, matched by available boot ID.
Simulation association requires the same available simulation clock, nonzero
`clock_run_id`, `logical_origin_us=1`, physics/control dt, and actual matching
runtime/session identities. A shared boot never authorizes a simulation join.

Runtime and velocity sample stamps in simulation use logical microseconds.
Sampling uses an integer logical deadline grid, with no replay of missed
samples and no duplicate scheduled sample during frozen stop time. Policy
session rows add explicit `logical_us`; their original `monotonic_us` remains
host steady time and `host_us` remains host Unix time. A simulation policy row
without logical time is refused, rather than relabeled as logical time.

The independent coordinator owns `simulation.jsonl` in its private socket/run
directory. SDK metadata carries a relative `simulation.sidecar` reference.
Keep the complete experiment group (runtime directories, application
directories, coordinator directory) together, preserving their relative layout
when moving it. Copying only one SDK directory can leave a missing sidecar;
readers explicitly report incomplete evidence. Source and installed tools use
the same references, without embedding their install/source locations.

The launcher lets SDK children inherit its working directory; it does not
colocate artifacts automatically. An inherited recorder directory under `logs`
may be separate from a coordinator socket directory under `/tmp`. That default
separated layout does **not** provide full sidecar support to a web service
rooted only at `logs`. Configure both recording directories and coordinator
under one common experiment root before normalization/startup, then use that
root for the web service and preserve its relative layout when moving it. CLI
and NPZ exports can consume a separated layout while the relative sidecar is
reachable; copying only SDK directories does not preserve that guarantee.
Exported NPZ embeds the event/measurement/metadata snapshot for offline analysis.

The sidecar schema is `irmv.simulation_evidence`, version 1:

- Header: simulation clock/run/origin/dt, runtime/config/application/engine
  identities, actual engine factory, dependency/Python versions, initial engine
  configuration and declared units. Asset/config digests live in SDK metadata;
  the application records the loaded ONNX model and deployment-contract digests.
  No random seed is invented when none is configured.
- Events: monotonic event sequence, exact control step/phase/logical time,
  coordinator outcome, protocol payloads, failure/release/stop facts. `state_seq`
  comes from the actual runtime payload; it is never derived from control step.
  `write_accepted` means the backend handoff accepted a write, while
  `AppliedToStep` names a completed worker physics interval for that write ID.
  Frozen phase 4 can accept a fresh zero-output write and advance FSM state
  sequence without advancing logical or physics time or emitting AppliedToStep.
- Bootstrap and once-after-step feedback: coherent measured joint q/dq/torque,
  IMU quaternion/gyro/specific force, free-base position/velocity and measured
  base quaternion. Base position and linear velocity use world coordinates;
  the final three base velocity components are local angular velocity. IMU gyro
  and specific force use the IMU site frame. Base/IMU quaternions are wxyz to
  world. `physics_step` starts at zero. `engine_time_s` is diagnostic, never the
  SDK business clock. `feedback_validated` separates accepted coherent feedback
  from rejected engine facts. No velocity or pose is reconstructed from targets
  or accelerometer values. A legacy adapter's empty base quaternion is explicitly
  unavailable and clears measured-feedback completeness. Loss, absent bootstrap,
  or missing completed-step feedback also clears measured-feedback completeness.
  Readers require exactly one validated measurement from bootstrap through the
  completed horizon evidenced by event control steps and AppliedToStep physics
  endpoints, including final facts. A recorded prefix with an absent final step
  is incomplete even when its footer counters agree. This flag never turns
  missing measurements into successful evidence.
- Host measurements: `host_observed_us` and `host_operation_start_us` use kernel
  boot monotonic time. `host_operation_duration_us`, `host_release_duration_us`,
  and `host_stop_duration_us` are elapsed host time for engine operations,
  release and zero-output/close respectively. They are not logical control
  latency. `released` and event kind identify applicable duration measurements.
- Footer: offered/written/dropped/write-failure counts. Offers use a fixed 256
  entry queue and 64 KiB maximum line; the worker exclusively writes the file.
  Overflow loses evidence and increments drop counts rather than delaying SDK
  control. Readers stream bounded lines and retain sequence gaps, missing tails,
  footer status and failure facts. A partial pre-footer write is incomplete;
  any bytes after a footer (even whitespace or an unterminated fragment) are
  rejected rather than allowed to retain completeness. Streaming Python
  `read(path, visitor=...)`,
  C++ ReadSimulationEvidence and web summaries impose no total-file/row limit;
  the 60-second/500-Hz horizon is tested with 240,008 events. Python's convenience
  materialization mode refuses after 65,536 rows with an explicit instruction
  to use streaming; it never returns a partial result as complete. File
  completeness does not assert a successful
  lifecycle, GPU execution, or physical motor execution: inspect final runner
  state/stop proof/failure and measurement availability as well.

`read_record --run DIR` emits `simulation_event` rows and a
`simulation_evidence` summary. Its time window uses event logical time; engine
feedback at the end of an interval has that completed step's logical time,
while AppliedToStep retains the originating command phase/time. Web contexts
expose the same validated summary and display simulation run/origin/dt. The web
records root must contain both SDK directories and the referenced sidecar;
references outside that root (including symlink escapes) are refused.

`record_to_npz.py RUNTIME OUT.npz --reader PATH --application-record APP`
retains existing gaps and columns, adds explicit `logical_stamp_us`, clock
identity and simulation metadata/summary JSON, and exports simulation event
logical/phase/step/sequence and separately named host-duration arrays. Full
`simulation_event_json` retains measured feedback, protocol/write identities,
physics intervals, release and failure facts without object arrays. Values in
these arrays are measurements only when the event kind/validity facts say so.

The adapter now copies measured base orientation from the same completed host
snapshot. Its bytes participate in `engine_identity`; old generated configs and
manifests must be rebound and remain rejected by the unchanged binding checks.
FakeEngine recording tests provide no MuJoCo Warp/GPU acceptance evidence.
