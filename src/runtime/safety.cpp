#include "irmv/runtime/safety.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "irmv/protocol/validation.hpp"

namespace irmv::runtime {
namespace {

// The detail array is bounded by the protocol payload. The flag booleans and
// the reported reason never depend on the details array having room: higher
// priority passes are appended first and later passes are dropped at capacity.
constexpr std::uint32_t kDetailCapacity =
    static_cast<std::uint32_t>(protocol::kMaxJoints);

// Lower value means lower priority. FeedbackInvalid > DriverError > ImuInvalid
// > JointPositionExceeded > JointVelocityExceeded.
int ReasonPriority(protocol::FaultReason reason) {
  switch (reason) {
    case protocol::FaultReason::JointVelocityExceeded:
      return 1;
    case protocol::FaultReason::JointPositionExceeded:
      return 2;
    case protocol::FaultReason::ImuInvalid:
      return 3;
    case protocol::FaultReason::DriverError:
      return 4;
    case protocol::FaultReason::FeedbackInvalid:
      return 5;
    default:
      return 0;
  }
}

void RaiseReason(protocol::FaultReason *reason,
                 protocol::FaultReason candidate) {
  if (ReasonPriority(candidate) > ReasonPriority(*reason)) {
    *reason = candidate;
  }
}

bool IsJointSampleFinite(const protocol::JointFeedback &joint) {
  return std::isfinite(joint.q) && std::isfinite(joint.dq) &&
         std::isfinite(joint.tau_est) && std::isfinite(joint.freq_hz);
}

// A sample carries no observation at all when the backend has never delivered
// a frame for it: unobserved and unconnected, no receive stamp and no driver
// status. The values must still be finite so garbage is never read as absence.
// Zero-valued defaults therefore stay distinguishable from a measurement of
// zero, which is why absence is neither healthy nor faulty.
bool IsJointSampleAbsent(const protocol::JointFeedback &joint) {
  return !joint.valid && !joint.connected && joint.rx_stamp_us == 0 &&
         joint.error_code == 0 && IsJointSampleFinite(joint);
}

bool IsImuSampleFinite(const protocol::ImuFeedback &imu) {
  for (const double value : imu.orientation_rpy) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : imu.angular_velocity) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  for (const double value : imu.linear_acceleration) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

// The IMU has no connection flag, so absence is the unobserved sample with no
// receive stamp and finite fields.
bool IsImuSampleAbsent(const protocol::ImuFeedback &imu) {
  return !imu.valid && imu.rx_stamp_us == 0 && IsImuSampleFinite(imu);
}

// A timestamp from the future is as unusable as one older than the configured
// timeout, so both are treated as stale.
bool IsStale(protocol::TimeUs rx_stamp_us, protocol::TimeUs now_us,
             protocol::TimeUs timeout_us) {
  return rx_stamp_us > now_us || now_us - rx_stamp_us > timeout_us;
}

// detected_stamp_us and output_action are filled in once the whole snapshot has
// been evaluated and the damping decision is known.
void AppendDetail(protocol::FaultReport *report, protocol::FaultReason reason,
                  std::uint32_t joint_index, std::uint32_t error_code,
                  double observed_value, double limit,
                  protocol::TimeUs sample_stamp_us, std::string_view message) {
  if (report->count >= kDetailCapacity) {
    return;
  }

  protocol::FaultDetail *detail = &report->details[report->count];
  *detail = protocol::FaultDetail{};
  detail->reason = reason;
  detail->joint_index = joint_index;
  detail->error_code = error_code;
  detail->observed_value = observed_value;
  detail->limit = limit;
  detail->sample_stamp_us = sample_stamp_us;
  static_cast<void>(detail->message.Assign(message));
  ++report->count;
}

double ClampToLimit(double value, double limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

}  // namespace

SafetyReport CheckFeedback(const configuration::RuntimeConfig &config,
                           const protocol::FeedbackSnapshot &feedback,
                           protocol::TimeUs now_us, bool boot_grace) {
  SafetyReport report;
  const std::size_t joint_count = config.joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints ||
      feedback.count != joint_count) {
    report.fault = true;
    report.feedback_invalid = true;
    report.reason = protocol::FaultReason::FeedbackInvalid;
    return report;
  }

  protocol::FaultReason reason = protocol::FaultReason::None;
  bool fault = false;

  // Pass A: signal integrity, freshness, driver status and control frequency.
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    const std::uint32_t joint_index = static_cast<std::uint32_t>(index);

    // Inside the boot grace window a sample that was never observed is
    // "waiting", not faulty and not healthy: there is no measurement to judge,
    // so its signal integrity, staleness, control frequency, position and
    // velocity are all skipped and its zero-valued defaults are never read as
    // observations. The boot deadline stays the backstop, because outside the
    // window an absent sample faults like any other invalid feedback.
    if (boot_grace && IsJointSampleAbsent(joint)) {
      report.awaiting_feedback = true;
      continue;
    }

    if (!joint.valid || !joint.connected || !IsJointSampleFinite(joint)) {
      report.feedback_invalid = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::FeedbackInvalid);
      AppendDetail(&report.details, protocol::FaultReason::FeedbackInvalid,
                   joint_index, 0, 0.0, 0.0, joint.rx_stamp_us,
                   "invalid joint feedback sample");
    }
    if (IsStale(joint.rx_stamp_us, now_us, config.timeouts.feedback_us)) {
      report.stale_feedback = true;
      if (!boot_grace) {
        fault = true;
        RaiseReason(&reason, protocol::FaultReason::FeedbackInvalid);
        AppendDetail(&report.details, protocol::FaultReason::FeedbackInvalid,
                     joint_index, 0, 0.0, 0.0, joint.rx_stamp_us,
                     "stale joint feedback sample");
      }
    }
    if (joint.error_code != 0) {
      report.driver_error = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::DriverError);
      AppendDetail(&report.details, protocol::FaultReason::DriverError,
                   joint_index, joint.error_code, 0.0, 0.0, joint.rx_stamp_us,
                   "driver error");
    }
    if (joint.freq_hz < config.boot.minimum_frequency_hz) {
      report.frequency_low = true;
      if (!boot_grace) {
        report.feedback_invalid = true;
        fault = true;
        RaiseReason(&reason, protocol::FaultReason::FeedbackInvalid);
        AppendDetail(&report.details, protocol::FaultReason::FeedbackInvalid,
                     joint_index, 0, joint.freq_hz,
                     config.boot.minimum_frequency_hz, joint.rx_stamp_us,
                     "control frequency below minimum");
      }
    }
  }

  // Pass B: position limits.
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    if (boot_grace && IsJointSampleAbsent(joint)) {
      continue;
    }
    if (joint.q < config.position_min[index] ||
        joint.q > config.position_max[index]) {
      report.position_exceeded = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::JointPositionExceeded);
      const double limit = joint.q < config.position_min[index]
                               ? config.position_min[index]
                               : config.position_max[index];
      AppendDetail(&report.details,
                   protocol::FaultReason::JointPositionExceeded,
                   static_cast<std::uint32_t>(index), 0, joint.q, limit,
                   joint.rx_stamp_us, "joint position limit exceeded");
    }
  }

  // Pass C: velocity limits. Equality is within limit; invalid or stale
  // velocities are still compared instead of being treated as zero.
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    if (boot_grace && IsJointSampleAbsent(joint)) {
      continue;
    }
    const double limit = config.joint_velocity_limits[index];
    if (!(std::abs(joint.dq) <= limit)) {
      report.velocity_exceeded = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::JointVelocityExceeded);
      AppendDetail(&report.details,
                   protocol::FaultReason::JointVelocityExceeded,
                   static_cast<std::uint32_t>(index), 0, joint.dq, limit,
                   joint.rx_stamp_us, "joint velocity limit exceeded");
    }
  }

  if (config.imu_required) {
    const protocol::ImuFeedback &imu = feedback.imu;
    if (boot_grace && IsImuSampleAbsent(imu)) {
      // An IMU that has never delivered a sample is waiting, exactly like an
      // absent joint; an observed invalid or stale IMU keeps faulting.
      report.awaiting_feedback = true;
    } else {
      const bool imu_invalid = !imu.valid || !IsImuSampleFinite(imu) ||
                               (!boot_grace && IsStale(imu.rx_stamp_us, now_us,
                                                       config.timeouts.imu_us));
      if (imu_invalid) {
        report.imu_invalid = true;
        fault = true;
        RaiseReason(&reason, protocol::FaultReason::ImuInvalid);
        AppendDetail(&report.details, protocol::FaultReason::ImuInvalid,
                     protocol::kNoJointIndex, 0, 0.0, 0.0, imu.rx_stamp_us,
                     "imu feedback invalid");
      }
    }
  }

  report.fault = fault;
  // Waiting for a first frame is neither healthy (the machine is not verified
  // yet) nor faulty (nothing was observed to judge), so the caller keeps
  // commanding zero torque until the grace window or boot deadline expires.
  report.healthy = !fault && !report.awaiting_feedback;
  report.reason = reason;
  report.damping_eligible =
      fault && !report.awaiting_feedback && !report.feedback_invalid &&
      !report.stale_feedback && !report.frequency_low && !report.driver_error;

  const protocol::OutputAction action =
      report.damping_eligible ? protocol::OutputAction::Damping
                              : protocol::OutputAction::ZeroTorque;
  for (std::uint32_t index = 0; index < report.details.count; ++index) {
    report.details.details[index].detected_stamp_us = now_us;
    report.details.details[index].output_action = action;
  }
  return report;
}

protocol::ValidationResult ValidateTrackingCommand(
    const configuration::RuntimeConfig &config,
    const protocol::JointCommand &command) {
  const std::size_t joint_count = config.joint_count();
  const protocol::ValidationResult shape =
      protocol::ValidateCommandShape(command, joint_count);
  if (!shape.ok) {
    return shape;
  }
  if (command.layout_id.View() != config.layout_id) {
    return {false, protocol::RejectReason::Malformed};
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double target_position = command.joints.q[index];
    const double target_velocity = command.joints.dq[index];
    if (target_position < config.position_min[index] ||
        target_position > config.position_max[index]) {
      return {false, protocol::RejectReason::InvalidCommand};
    }
    if (!(std::abs(target_velocity) <= config.target_velocity_limits[index])) {
      return {false, protocol::RejectReason::InvalidCommand};
    }
  }
  return {true, protocol::RejectReason::None};
}

ControlOutput ComputeTracking(const configuration::RuntimeConfig &config,
                              const protocol::JointCommand &command,
                              const protocol::FeedbackSnapshot &feedback) {
  ControlOutput result;
  const std::size_t joint_count = config.joint_count();
  if (feedback.count != joint_count) {
    return result;
  }
  if (!ValidateTrackingCommand(config, command).ok) {
    return result;
  }

  hardware::TorqueOutput candidate;
  candidate.count = static_cast<std::uint32_t>(joint_count);
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double target_position = command.joints.q[index];
    const double target_velocity = command.joints.dq[index];
    const double position_term =
        command.joints.kp[index] * (target_position - feedback.joints[index].q);
    const double velocity_term = command.joints.kd[index] *
                                 (target_velocity - feedback.joints[index].dq);
    if (!std::isfinite(position_term) || !std::isfinite(velocity_term)) {
      return result;
    }
    const double raw_torque =
        position_term + velocity_term + command.joints.tau_ff[index];
    if (!std::isfinite(raw_torque)) {
      return result;
    }
    candidate.torque[index] =
        ClampToLimit(raw_torque, config.torque_limits[index]);
  }

  result.output = candidate;
  result.valid = true;
  return result;
}

ControlOutput ComputeDamping(const configuration::RuntimeConfig &config,
                             const protocol::FeedbackSnapshot &feedback) {
  ControlOutput result;
  const std::size_t joint_count = config.joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints ||
      feedback.count != joint_count) {
    return result;
  }

  hardware::TorqueOutput candidate;
  candidate.count = static_cast<std::uint32_t>(joint_count);
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    if (!joint.valid || !joint.connected || !std::isfinite(joint.dq)) {
      return result;
    }
    const double damping_torque = -config.damping_kd[index] * joint.dq;
    if (!std::isfinite(damping_torque)) {
      return result;
    }
    candidate.torque[index] =
        ClampToLimit(damping_torque, config.torque_limits[index]);
  }

  result.output = candidate;
  result.valid = true;
  return result;
}

}  // namespace irmv::runtime
