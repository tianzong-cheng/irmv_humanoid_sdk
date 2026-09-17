#pragma once

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"

namespace irmv::runtime {

struct SafetyReport {
  bool healthy{false};
  bool fault{false};
  bool stale_feedback{false};
  bool frequency_low{false};
  bool feedback_invalid{false};
  bool driver_error{false};
  bool position_exceeded{false};
  bool velocity_exceeded{false};
  bool imu_invalid{false};
  bool damping_eligible{false};
  protocol::FaultReason reason{protocol::FaultReason::None};
  protocol::FaultReport details;
};

struct ControlOutput {
  bool valid{false};
  hardware::TorqueOutput output;
};

// Validates one tracking command against the protocol shape and the configured
// limits, without touching the measured state: VersionMismatch for a wrong
// protocol version, Malformed for a wrong joint count or layout, InvalidCommand
// for non-finite or negative gains, a non-finite motion target, a target
// position outside [position_min, position_max] or a target velocity above
// target_velocity_limits (equality is within limit).
protocol::ValidationResult ValidateTrackingCommand(
    const configuration::RuntimeConfig &config,
    const protocol::JointCommand &command);

// Evaluates one feedback snapshot against the configuration limits and
// timeouts. Inside the boot grace window stale samples and low control
// frequency are recorded but not treated as faults; already observed invalid
// samples, driver errors, position and velocity violations always fault.
SafetyReport CheckFeedback(const configuration::RuntimeConfig &config,
                           const protocol::FeedbackSnapshot &feedback,
                           protocol::TimeUs now_us, bool boot_grace);

// Computes the torque command for tracking. Any malformed, out of range or
// non-finite input rejects the whole command: no partial output is produced.
// The snapshot must report the configured joint count, otherwise the measured
// state is incomplete and no torque is produced. Freshness of the measured
// state is not evaluated here.
ControlOutput ComputeTracking(const configuration::RuntimeConfig &config,
                              const protocol::JointCommand &command,
                              const protocol::FeedbackSnapshot &feedback);

// Computes the safety damping torque for the measured joint velocities. Any
// invalid, disconnected or non-finite joint sample rejects the whole command.
// Freshness of the measured state is not evaluated here.
ControlOutput ComputeDamping(const configuration::RuntimeConfig &config,
                             const protocol::FeedbackSnapshot &feedback);

}  // namespace irmv::runtime
