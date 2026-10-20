# Hardware migration provenance

This document records where every hardware fact of the real backend comes from,
and which facts are still unverified. It is not a machine configuration: the
shipped `config/sdk.json` stays fake, and the synthetic fixture in
`tests/support/hardware/real_two_joint.json` contains test values only.

## Baselines

| Item | Value | Source |
| --- | --- | --- |
| SDK baseline | `main@425290f58735cd0deb16585dc1c9f28f4c1b66f6` | plan baseline |
| IRMVControl upstream | `6bc7ed0` before this branch | `/home/tianzong/Workspace/Humanoid/IRMVControl` |
| IRMVControl with runtime I/O | `493275f87c809b369c409a84d6bf4abe604ccab6` | this branch, `deps/irmv_control` |
| YIS IMU driver | `3b73395d823b92a5c9760a759cd58709fd9e31c8` | this branch, `deps/yis_imu_driver` |
| Vendor library | `libcontrolcanfd.so`, SHA-256 `56beee8e...f0b2c`, ELF x86-64 (machine 62) | `deps/irmv_control/ThirdParty/controlcanfd/libs` |
| Licence | IRMVControl ships the repository `LICENSE`; YIS ships `LICENSE` | each checkout |

Publishing the two commits to the Codeup remotes and recording the returned
remote SHAs is a delivery step that needs credentials. The local pins are what
this branch was built and tested against; they are not proof that the remote
branches contain the commits. `tools/verify_hardware_dependencies.py` checks the
local pins, the submodule URLs and the vendor digest/architecture.

## Legacy reference

The only read-only reference is `/home/tianzong/Workspace/Humanoid/irmv_humanoid_v3_sdk`.
The old driver's five bench programs were mapped to the one maintenance program;
they are not copied into the build.

## The twelve axes (candidate wiring, to be confirmed on the bench)

The legacy `DataManager` built two driver instances, one per bus. These are its
candidate mappings; they are **not** a validated hardware table. Every row must
be confirmed against the physical robot before a real configuration is written.

| Joint | Bus | `(bus, motor_id)` | Legacy direction | Reduction | Offset | Kt | Limits |
| --- | --- | --- | --- | --- | --- | --- | --- |
| left_hip_pitch | can0 (channel 0) | can0, 1 | +1 | unverified | 0 | 2.2 (legacy global) | unverified |
| left_hip_roll | can0 | can0, 2 | +1 | unverified | 0 | 2.2 | unverified |
| left_hip_yaw | can0 | can0, 3 | -1 | unverified | 0 | 2.2 | unverified |
| left_knee | can0 | can0, 4 | +1 | unverified | 0 | 2.2 | unverified |
| left_ankle_pitch | can0 | can0, 16 | -1 | unverified | 0 | 2.2 | unverified |
| left_ankle_roll | can0 | can0, 17 | +1 | unverified | 0 | 2.2 | unverified |
| right_hip_pitch | can1 (channel 1) | can1, 7 | -1 | unverified | 0 | 2.2 | unverified |
| right_hip_roll | can1 | can1, 8 | +1 | unverified | 0 | 2.2 | unverified |
| right_hip_yaw | can1 | can1, 9 | -1 | unverified | 0 | 2.2 | unverified |
| right_knee | can1 | can1, 10 | -1 | unverified | 0 | 2.2 | unverified |
| right_ankle_pitch | can1 | can1, 18 | -1 | unverified | 0 | 2.2 | unverified |
| right_ankle_roll | can1 | can1, 19 | -1 | unverified | 0 | 2.2 | unverified |

Bus options: vendor device type 41 (`USBCANFD_200U`), device index 0,
arbitration 1 Mbit/s, data 5 Mbit/s, ISO CAN-FD on, termination off. These match
the legacy constructors; they still need confirmation on the actual adapter.

Legacy motor strings (`MG6012E-i36`, `M10015E-i10`) are observations, not a
source of numeric ratios: the legacy code passed `1/22.64` as a driver-level
gear and `1` as the per-motor configuration gear, so it never encoded the model
ratio per joint. The adapter now applies one motor/joint ratio from the
configuration and never a second internal ratio.

## Protocol and units

The heartbeat frame `0x7NN` (11 bytes minimum) is decoded as:

| Bytes | Field | Conversion |
| --- | --- | --- |
| 0..3 | error code | kept as the complete `uint32_t`; unknown bits are reported raw |
| 4..5 | position, int16 | `/65536 * 2π` rad, single loop, expanded to multi-turn |
| 6..7 | speed, int16 | `/65536 * 2π * 1000` rad/s, rotor side |
| 8..9 | current, int16 | `* 3.3 / 0.002 / 65536 / 20` A |
| 10 | mode | reported, not interpreted |

The control frame is `0x400`, 64 bytes; byte 0 is the mode, and motor `N` at
bytes `1 + (N-1)*2`. The torque encoding is the legacy current scale
`tau / Kt / 3.3 * 0.002 * 65536 * 20`, clamped to `int16`. The repository does
not contain a per-joint `Kt`; the legacy value 2.2 is the only one seen and it
must be confirmed per motor model. Enable is `0x300` with `data[0]=0x01`;
single-motor disable/reset are `id=motor_id` with `data[0]=0x00`/`0xFF`.

`RuntimeIo::Open` opens devices and starts reception only. Enable is a separate,
per-motor command, and the initial zero torque frame is an explicit send. A
read-only tool therefore transmits nothing. The vendor send call is synchronous
and has no documented worst-case bound; the bench item must measure it before a
control-period claim is accepted. The old `motor_load_test_can1` used `0xF` and
aliased 0x710..0x713 onto 1..3; that variant is not a reference.

## Missing facts and gates

These are required before a real configuration can be written, and they block
the corresponding bench item:

- per-joint reduction ratio and sign, verified from raw frames and motion;
- per-joint torque constant and encoder offset;
- firmware link-loss timeout and required command rate;
- hardware emergency-stop behaviour and its effect on the bus;
- zero-point persistence evidence after a reboot (the protocol has no
  independent read-back acknowledgement);
- vendor library distribution permission for the deployed machines;
- an aarch64 vendor library: only x86-64 was available, so no aarch64 claim is
  made.

No guessed value is written into a runnable real configuration. The hardware
configuration schema, however, is exercised by synthetic fixtures.
