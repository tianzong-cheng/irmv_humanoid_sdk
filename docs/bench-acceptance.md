# Bench acceptance record

Status: **software verification in progress; no real-device item is accepted.**
Every hardware row below is `NOT RUN` until a field operator records the
evidence in `.superpowers/phase4/<run>/`.

## Evidence identity

| Item | Value |
| --- | --- |
| Branch | `feat/real-backend-maintenance` |
| Baseline | `main@425290f58735cd0deb16585dc1c9f28f4c1b66f6` |
| IRMVControl | `493275f87c809b369c409a84d6bf4abe604ccab6` (local pin) |
| YIS | `3b73395d823b92a5c9760a759cd58709fd9e31c8` (local pin) |
| Vendor library | SHA-256 `56beee8e...f0b2c`, x86-64 |
| Synthetic config | `tests/support/hardware/real_two_joint.json` (**not a machine config**) |
| Real config | none yet; values are unverified |

## Software verification

| Check | Result | Evidence |
| --- | --- | --- |
| Configuration accepts a real pair and rejects partial/duplicate/non-finite mappings | PASS | `hardware_config_test` |
| Device lease excludes a second process, releases on failure/exit, ignores bus renaming | PASS | `device_lease_test` |
| RealBackend lifecycle, invalid output refusal, lease-before-open, idempotent close | PASS | `real_backend_test` |
| Upstream codec: 32-bit error, boundary ids, short frames, position expansion, saturation | PASS | upstream `runtime_io_test` |
| Upstream dependency contract: fresh configure, no examples, no `-march=native` | PASS | upstream `dependency_contract_test` |
| Maintenance dry-run, raw error display, selected-joint load, can1 channel rule, zero read-back | PASS | `maintenance_test` |
| Launcher keeps a surviving runtime and reports a runtime that never becomes ready | PASS | `launcher_test` |
| Dependency pins, vendor digest/arch, runtime-only configure without ONNX Runtime | PASS | `hardware_package_test` |
| Movable install tree runs Fake from a moved prefix | PASS (manual, this branch) | `cmake --install` into a temporary prefix, then `mv` and run |
| Full default regression | PASS | `ctest -j2`: 64/64, 28 core + 36 integration |
| Sanitizer core suite | PASS | `IRMV_SANITIZE=address,undefined`, 28/28 core |
| Runtime-only build without ONNX Runtime | PASS | `IRMV_BUILD_APPLICATION=OFF`, `ldd` has no ONNX |
| RealBackend over real DDS with scripted port | PASS | `real_backend_contract_test`: read-only send log, edge/far future stamps, write refusal, driver error, overspeed, missing IMU |

## Bench matrix

Each row needs: prerequisites, the exact SDK/driver/YIS/vendor SHAs, the
configuration and model digest, steps, expected FSM/output, physical
observation, log and raw evidence path, and PASS/FAIL/NOT RUN.

| # | Item | Status | Required evidence |
| --- | --- | --- | --- |
| 1 | Read-only identity: every axis name/bus/id/direction/unit/offset, 32-bit errors, frequency, IMU pose/units | NOT RUN | per-axis table signed by the operator |
| 2 | Mechanical limits, hardware e-stop, firmware link-loss behaviour | NOT RUN | physical check and firmware documentation |
| 3 | Offline zero calibration, one joint and the full set; unspecified motors unchanged; reboot pose check | NOT RUN | before/after fresh feedback, config offsets, reboot observation |
| 4 | Boot, ZeroTorque, Damping on one axis then the full set; damping sign; output-side torque scaling and clamp | NOT RUN | tick log, measured torque/pose, worst send time |
| 5 | Stop sending, unplug/replug, driver error, IMU loss/recovery; Fault output on the first failing tick | NOT RUN | injected or recorded device events, output sequence |
| 6 | Standby→Damping→Initializing→Ready→Locomotion→Damping→Standby, local and external producer | NOT RUN | state log, gain continuity, physical observation |
| 7 | E-stop/stop preemption, application exit keeps damping, supervisor timeout, Reset only to Standby/ZeroTorque, runtime restart does not resume motion | NOT RUN | logs from both processes |
| 8 | Recorder at the bench control rate for at least 10 minutes; seq/gaps and digests | NOT RUN | run index, reader output, config/model/gain digests |
| 9 | At least 30 minutes of state switching; RSS/threads/fds, control overruns, slow log consumer | NOT RUN | `/proc` samples, run logs |

## Limits of the automated evidence

- Fault-injection tests do not replace a real unplug, a real e-stop or a real
  firmware timeout.
- The host send result and `Applied` prove the host path accepted a frame, not
  that a motor applied it.
- Zero reset cannot prove firmware persistence through this protocol.
- Only x86-64 was built; no aarch64 vendor claim is made.
- The local dependency commits are not yet confirmed on the Codeup remotes.

The phase is not accepted until every required row above has a signed PASS with
evidence, and the real configuration contains only verified values.
