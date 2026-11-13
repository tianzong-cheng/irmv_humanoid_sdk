"""Engine-neutral worker interface. Tuples are immutable, detached snapshots."""
from dataclasses import dataclass
from typing import Protocol, Sequence


@dataclass(frozen=True)
class Feedback:
    physics_step: int
    engine_time_s: float
    q: tuple[float, ...]
    dq: tuple[float, ...]
    tau: tuple[float, ...]  # Actuator generalized force only, output-side N m.
    imu_quaternion: tuple[float, ...]  # wxyz, IMU site orientation in world.
    imu_angular_velocity: tuple[float, ...]  # rad/s, local IMU site frame.
    imu_specific_force: tuple[float, ...]  # m/s^2, local frame, includes gravity at rest.
    base_position: tuple[float, ...]  # World metres.
    base_velocity: tuple[float, ...]  # Free-joint qvel (world linear, local angular).


class EngineAdapter(Protocol):
    """All calls belong to the single simulation worker, never a control thread."""
    def initialize(self, config: dict) -> None: ...
    def read_feedback(self) -> Feedback: ...
    def apply_torque(self, torque: Sequence[float]) -> None: ...
    def step(self, substeps: int) -> Feedback: ...
    def release_base(self) -> None: ...
    def close(self) -> None: ...


class EngineTimeDiagnostic:
    """Exact pinned-Warp binary32 addition check, never an SDK clock.

    The Warp 3.9 time kernel adds two float32 values once per physical substep.
    Host prediction rounds the float32 sum to float32; no tolerance is needed.
    This engine-specific one-hour contract requires integer dt >= 1000 us.
    Smaller timesteps are rejected at startup rather than allowing time stalls.
    """
    def __init__(self, physics_dt_us: int):
        import struct
        if type(physics_dt_us) is not int or not 1000 <= physics_dt_us <= 3600000000:
            raise ValueError('Warp one-hour diagnostic requires integer physics_dt_us >= 1000')
        self.dt_us = physics_dt_us
        self.steps = 0
        self.expected_s = 0.0
        self._round = lambda x: struct.unpack('<f', struct.pack('<f', x))[0]
        self.dt_s = self._round(physics_dt_us / 1000000)

    def advance(self):
        if (self.steps + 1) * self.dt_us > 3600000000:
            raise ValueError('3600 logical second experiment limit')
        self.expected_s = self._round(self.expected_s + self.dt_s)
        self.steps += 1

    def validate(self, actual_s):
        import math
        if not math.isfinite(actual_s) or actual_s != self.expected_s:
            raise RuntimeError('engine time differs from exact completed float32 additions')
