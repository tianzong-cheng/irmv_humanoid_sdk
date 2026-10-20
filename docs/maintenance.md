# Maintenance tool

`irmv_maintenance` replaces the five legacy bench programs with one program and
five subcommands. It parses the whole configuration and its own arguments before
acquiring the device lock and opening a device. Write commands are a dry run
unless `--execute` is given.

```
irmv_maintenance --config PATH <subcommand> [options]
```

| Subcommand | Purpose |
| --- | --- |
| `heartbeat-decode` | print fresh feedback per joint, the full 32-bit error, host age, frequency, and unsupported temperature/voltage |
| `motor-load-test` | send a constant torque to explicit joints for a finite duration |
| `motor-load-test-can1` | the same, restricted to joints whose configured bus has `channel=1` |
| `motor-zero-reset` | disable then reset zero for explicit joints, then verify a fresh read-back |
| `motor-zero-reset-single` | the same for exactly one joint |

Options:

- `--joint NAME` — repeatable, required for every write command and for the
  load tests. There is no "all joints" shorthand: a broadcast is refused because
  the controlled set cannot be proven from the protocol.
- `--torque-nm N` — magnitude is checked against `runtime.safety.torque_limits`
  of every selected joint.
- `--duration-ms N`, `--rate-hz N` — load window and send/read rate.
- `--execute` — perform writes. Without it, the plan is printed and no device is
  opened.
- `--verify-position-tolerance-rad`, `--verify-velocity-tolerance-rad-s`,
  `--verify-timeout-ms` — required positive values for zero reset. Missing,
  zero or non-finite values are refused.

## What zero reset proves

1. The joint has a fresh pre-reset sample. A silent motor writes nothing.
2. The disable command is accepted, then the zero-reset command is accepted.
3. A heartbeat **received after the reset** satisfies the position and velocity
   tolerances. An old cached frame, a wrong motor id, a missing frame or a
   refused send fails the command with a non-zero exit.

The protocol has no independent read-back of a persistent zero register. The
tool therefore reports "fresh readback passed", not "firmware persisted". A
bench reboot and a physical pose check are the evidence for persistence, and
they are recorded in `docs/bench-acceptance.md`.

## Load test

Only the selected joints carry the requested torque; every other slot in every
control frame is zero. At the end of the window, or on SIGINT/SIGTERM, the tool
sends a final all-zero frame. An uncatchable kill cannot promise a final frame;
the runtime's own stop path does not depend on this tool running.

The tool prints whether each frame was accepted by the host send path. That is
not an acknowledgement from the motor firmware.

## Device ownership

The tool takes the same exclusive lock as the runtime before opening a device.
Stop the runtime first. The lock is advisory for unrelated programs, as
described in `docs/hardware-config.md`.

## Building

The program needs the real driver:

```
cmake -S . -B build-real \
  -DIRMV_ENABLE_REAL_BACKEND=ON \
  -DIRMV_BUILD_MAINTENANCE=ON \
  -DIRMV_IRMV_CONTROL_SOURCE_DIR=$PWD/deps/irmv_control \
  -DIRMV_YIS_IMU_SOURCE_DIR=$PWD/deps/yis_imu_driver
cmake --build build-real --target irmv_maintenance
```

The default build does not link a vendor library and does not install this
program.
