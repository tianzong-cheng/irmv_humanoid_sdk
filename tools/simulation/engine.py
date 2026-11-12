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
