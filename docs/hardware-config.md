# Hardware configuration

One file describes the deployment. `deployment` and `backend` select a pair:
`fake_only`/`fake` (the shipped demo) or `real`/`real`. A real deployment must
name its buses and the hardware facts of every joint; the loader refuses a
partial table instead of using defaults.

```json
{
  "deployment": "real",
  "backend": "real",
  "robot": {
    "units": {"position": "rad", "velocity": "rad_s", "torque": "Nm"},
    "buses": [
      {"name": "can0", "device_type": 41, "device_index": 0, "channel": 0,
       "arbitration_bps": 1000000, "data_bps": 5000000,
       "iso_mode": true, "resistance_enabled": false}
    ],
    "imu": {"hardware_id": "VID_1234&PID_5678", "baudrate": 921600},
    "joints": [
      {"name": "left_hip_pitch", "position_min": -2.0, "position_max": 2.0,
       "bus": "can0", "motor_id": 1, "direction": 1,
       "reduction_ratio": 36.0, "encoder_offset": 0.0,
       "torque_constant": 2.2}
    ]
  }
}
```

The rest of the file (runtime, application, input, telemetry, dds) is unchanged.

## Rules the loader enforces

- Bus names are unique, 1..64 bytes, and configuration-local.
- `(device_type, device_index, channel)` is unique; `channel` is 0 or 1.
- Every joint mapping names a configured bus; `motor_id` is 1..31; `(bus,
  motor_id)` is unique.
- `direction` is exactly `-1` or `1`; `reduction_ratio` and `torque_constant`
  are positive; every physical number is finite.
- `robot.imu` is optional. `runtime.imu_required=true` with `deployment=real`
  requires it.
- A joint with any hardware field requires all of them. A real deployment
  requires `robot.buses` and a full mapping for every joint.
- The schema and the loader are kept in step by `hardware_config_test` and
  `config_schema_contract_test`.

## Physical meaning and units

All protocol quantities are joint-output side: rad, rad/s, N·m, with kp in
N·m/rad and kd in N·m·s/rad. The adapter applies the conversion exactly once:

```
q         = direction * (position_rad - encoder_offset) / reduction_ratio
dq        = direction * speed_rad_s / reduction_ratio
tau_est   = direction * reduction_ratio * torque_constant * current_a
tau_motor = direction * tau_joint / reduction_ratio
```

`reduction_ratio` is motor side / joint side. It must be the physical ratio that
the protocol does not already apply; never divide by an internal ratio a second
time.

## Device ownership

The runtime and `irmv_maintenance` take an exclusive `flock` before opening a
device. The lock identity is the physical device `(device_type, device_index)`
and the IMU hardware id, not the configuration bus name. The default directory
is `/run/lock/irmv`; `IRMV_LOCK_DIR` overrides it. Install the directory with a
shared group, for example:

```
sudo install -d -m 0770 -g irmv /run/lock/irmv
```

The lock is advisory: it excludes the SDK processes that honour it, not an
unrelated program that opens the vendor device directly. Stop legacy tools
before starting the runtime.

## Freshness

Receive timestamps are host monotonic microseconds stamped when a complete
frame is parsed. Reading a cached snapshot never refreshes the stamp. The
upstream snapshot marks a sample invalid after its feedback timeout, and the
runtime applies the configured `timeouts.feedback_us` as well. Device ticks are
never compared with host time.

## IMU

The YIS driver now carries `host_tick_us`, the host monotonic receive time of
the parsed frame. The upstream runtime I/O maps device `euler` (pitch, roll,
yaw) to the frozen `orientation_rpy` order (roll, pitch, yaw) and passes gyro in
rad/s and acceleration in m/s². If the build has no IMU backend, an
`imu_present` configuration fails to open rather than running without the
required IMU.

## Synthetic fixture

`tests/support/hardware/real_two_joint.json` is a **test fixture**. Its ratios,
directions and torque constants are arbitrary and must never be copied to a
robot. A runnable real configuration is produced only after every value in
`docs/hardware-migration.md` has a verified source.
