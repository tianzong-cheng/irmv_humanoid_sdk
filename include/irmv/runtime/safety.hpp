#pragma once

#include <array>
#include <cstddef>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"

namespace irmv::runtime {

// Which required inputs have ever delivered a sample, one flag per possible
// joint plus one for the IMU. The boot grace window only excuses an input that
// was never observed at all: a sample that goes missing after the input has
// already delivered a frame is judged like any other invalid feedback, so a
// backend that goes quiet after its first frame faults at once instead of
// waiting out the window.
//
// The caller updates the history from the snapshot of the tick *before* it
// calls CheckFeedback, so the tick that carries an input's first sample already
// counts as observed. A first frame that is erroneous is therefore judged
// rather than excused: an unobserved input is the documented all-default shape
// a backend reports when it has never delivered a frame, not a frame that
// merely claims absence.
struct FeedbackObservation {
  std::array<bool, protocol::kMaxJoints> joints{};
  bool imu{false};

  // Marks every required input this snapshot carries a sample for. A sample is
  // an observation when it is not the documented all-default shape of an input
  // that has never delivered a frame.
  void Observe(const protocol::FeedbackSnapshot &feedback,
               std::size_t joint_count);
};

struct SafetyReport {
  bool healthy{false};
  bool fault{false};
  // The snapshot carries no observation at all for at least one required
  // sample inside the boot grace window, so this report is neither a fault nor
  // healthy: startup has to wait, keep commanding zero torque, and fault when
  // the configured grace/startup limit expires.
  bool awaiting_feedback{false};
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
//
// A sample that has never been observed - the all-default sample of a backend
// that has not delivered a frame yet - is neither a fault nor healthy inside
// the grace window: absence is no observation, so it cannot satisfy a startup
// check and the report sets awaiting_feedback with healthy false while the
// caller keeps commanding zero torque. The boot deadline/startup limit stays
// the backstop: once the grace window has closed an absent sample faults
// exactly like invalid feedback, so a machine that never receives a frame
// still faults.
//
// `observed` is the history of `FeedbackObservation::Observe` over the previous
// ticks including this one, and it narrows the excuse to inputs that were never
// observed: once a joint or the required IMU has delivered a sample, a later
// absent or invalid sample is an observation of a failure and faults
// immediately, inside the grace window as well.
SafetyReport CheckFeedback(const configuration::RuntimeConfig &config,
                           const protocol::FeedbackSnapshot &feedback,
                           protocol::TimeUs now_us, bool boot_grace,
                           const FeedbackObservation &observed);

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
