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
// a frame for it: unobserved and unconnected, no receive stamp, no driver
// status, and the documented default shape the backend contract asks for - the
// zeros an untouched sample holds. Requiring that shape is what keeps a frame
// that merely *claims*
// absence while carrying values a measurement: it is judged by those values
// like any other observation instead of being excused by the grace window, and
// garbage is never read as absence. Zero-valued defaults therefore stay
// distinguishable from a measurement of zero, which is why absence is neither
// healthy nor faulty.
bool IsJointSampleAbsent(const protocol::JointFeedback &joint) {
  return !joint.valid && !joint.connected && joint.rx_stamp_us == 0 &&
         joint.error_code == 0 && joint.q == 0.0 && joint.dq == 0.0 &&
         joint.tau_est == 0.0 && joint.freq_hz == 0.0;
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
// receive stamp and the same documented default shape as an absent joint.
bool IsImuSampleAbsent(const protocol::ImuFeedback &imu) {
  if (imu.valid || imu.rx_stamp_us != 0 || !IsImuSampleFinite(imu)) {
    return false;
  }
  for (const double value : imu.orientation_rpy) {
    if (value != 0.0) {
      return false;
    }
  }
  for (const double value : imu.angular_velocity) {
    if (value != 0.0) {
      return false;
    }
  }
  for (const double value : imu.linear_acceleration) {
    if (value != 0.0) {
      return false;
    }
  }
  return true;
}

// A timestamp from the future is as unusable as one older than the configured
// timeout, so both are treated as stale.
bool IsStale(protocol::TimeUs rx_stamp_us, protocol::TimeUs now_us,
             protocol::TimeUs timeout_us) {
  return rx_stamp_us > now_us || now_us - rx_stamp_us > timeout_us;
}

protocol::FaultDetail MakeFaultDetail(
    protocol::FaultReason reason, protocol::FaultCondition condition,
    std::uint32_t joint_index, std::uint32_t error_code, double observed_value,
    double limit, protocol::TimeUs sample_stamp_us, std::string_view message) {
  protocol::FaultDetail detail;
  detail.reason = reason;
  detail.condition = condition;
  detail.joint_index = joint_index;
  detail.error_code = error_code;
  detail.observed_value = observed_value;
  detail.limit = limit;
  detail.sample_stamp_us = sample_stamp_us;
  static_cast<void>(detail.message.Assign(message));
  return detail;
}

// detected_stamp_us and output_action are filled in once the whole snapshot has
// been evaluated and the damping decision is known.
void AppendDetail(protocol::FaultReport *report,
                  const protocol::FaultDetail &detail) {
  if (report->count >= kDetailCapacity) {
    return;
  }
  report->details[report->count] = detail;
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

void FeedbackObservation::Observe(const protocol::FeedbackSnapshot &feedback,
                                  std::size_t joint_count) {
  if (joint_count > protocol::kMaxJoints) {
    joint_count = protocol::kMaxJoints;
  }
  const std::size_t count =
      feedback.count < joint_count ? feedback.count : joint_count;
  for (std::size_t index = 0; index < count; ++index) {
    // Anything that is not the all-default shape of an input that never
    // delivered a frame is a sample, so the first erroneous frame counts as
    // observed and is judged instead of being excused by the grace window.
    if (!IsJointSampleAbsent(feedback.joints[index])) {
      joints[index] = true;
    }
  }
  // The IMU is recorded even when the configuration does not require it: the
  // configuration is immutable, so a flag that can never be consulted is
  // harmless, and the rule stays independent of which config is loaded.
  if (!IsImuSampleAbsent(feedback.imu)) {
    imu = true;
  }
}

void CheckFeedback(const configuration::RuntimeConfig &config,
                   const protocol::FeedbackSnapshot &feedback,
                   protocol::TimeUs now_us, bool boot_grace,
                   const FeedbackObservation &observed, SafetyReport &report) {
  // Reset only the verdict fields and the container lengths. The detail
  // storage is overwritten entry by entry as observations are appended, so a
  // reused report neither reconstructs the 1024-entry batch on every tick nor
  // exposes a stale tail: only the first `count` entries are ever read.
  report.healthy = false;
  report.fault = false;
  report.awaiting_feedback = false;
  report.stale_feedback = false;
  report.frequency_low = false;
  report.feedback_invalid = false;
  report.driver_error = false;
  report.position_exceeded = false;
  report.velocity_exceeded = false;
  report.imu_invalid = false;
  report.damping_eligible = false;
  report.reason = protocol::FaultReason::None;
  report.details.count = 0;
  report.observations.count = 0;
  report.observations.dropped = 0;

  const std::size_t joint_count = config.joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints ||
      feedback.count != joint_count) {
    report.fault = true;
    report.feedback_invalid = true;
    report.reason = protocol::FaultReason::FeedbackInvalid;
    // The published report has no slot for a shape fault, but the complete
    // diagnostic keeps it as one observation without a joint.
    AppendFaultObservation(
        report.observations,
        MakeFaultDetail(protocol::FaultReason::FeedbackInvalid,
                        protocol::FaultCondition::FeedbackShapeInvalid,
                        protocol::kNoJointIndex, 0,
                        static_cast<double>(feedback.count),
                        static_cast<double>(joint_count), feedback.stamp_us,
                        "feedback snapshot shape invalid"));
    return;
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
    // observations. Only an input that has never delivered a frame qualifies;
    // one that was observed before is judged by this absent sample like any
    // other invalid feedback. The boot deadline stays the backstop, because
    // outside the window an absent sample faults anyway.
    if (boot_grace && !observed.joints[index] && IsJointSampleAbsent(joint)) {
      report.awaiting_feedback = true;
      continue;
    }

    const bool disconnected = !joint.connected;
    const bool invalid = !joint.valid;
    const bool non_finite = !IsJointSampleFinite(joint);
    if (disconnected || invalid || non_finite) {
      report.feedback_invalid = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::FeedbackInvalid);
      // The published report keeps one combined entry per joint, named after
      // the first established cause. The complete diagnostic records every
      // established cause separately, so a disconnected sample and a
      // non-finite value on a healthy connection stay distinguishable.
      protocol::FaultCondition first_condition =
          protocol::FaultCondition::JointDisconnected;
      std::string_view first_message = "joint disconnected";
      if (!disconnected && invalid) {
        first_condition = protocol::FaultCondition::JointFeedbackInvalid;
        first_message = "invalid joint feedback sample";
      } else if (!disconnected) {
        first_condition = protocol::FaultCondition::JointFeedbackNonFinite;
        first_message = "non-finite joint feedback sample";
      }
      AppendDetail(&report.details,
                   MakeFaultDetail(protocol::FaultReason::FeedbackInvalid,
                                   first_condition, joint_index, 0, 0.0, 0.0,
                                   joint.rx_stamp_us, first_message));
      if (disconnected) {
        AppendFaultObservation(
            report.observations,
            MakeFaultDetail(protocol::FaultReason::FeedbackInvalid,
                            protocol::FaultCondition::JointDisconnected,
                            joint_index, 0, 0.0, 0.0, joint.rx_stamp_us,
                            "joint disconnected"));
      }
      if (invalid) {
        AppendFaultObservation(
            report.observations,
            MakeFaultDetail(protocol::FaultReason::FeedbackInvalid,
                            protocol::FaultCondition::JointFeedbackInvalid,
                            joint_index, 0, 0.0, 0.0, joint.rx_stamp_us,
                            "invalid joint feedback sample"));
      }
      if (non_finite) {
        AppendFaultObservation(
            report.observations,
            MakeFaultDetail(protocol::FaultReason::FeedbackInvalid,
                            protocol::FaultCondition::JointFeedbackNonFinite,
                            joint_index, 0, 0.0, 0.0, joint.rx_stamp_us,
                            "non-finite joint feedback sample"));
      }
    }
    if (IsStale(joint.rx_stamp_us, now_us, config.timeouts.feedback_us)) {
      report.stale_feedback = true;
      if (!boot_grace) {
        fault = true;
        RaiseReason(&reason, protocol::FaultReason::FeedbackInvalid);
        const protocol::FaultDetail stale = MakeFaultDetail(
            protocol::FaultReason::FeedbackInvalid,
            protocol::FaultCondition::JointFeedbackStale, joint_index, 0, 0.0,
            0.0, joint.rx_stamp_us, "stale joint feedback sample");
        AppendDetail(&report.details, stale);
        AppendFaultObservation(report.observations, stale);
      }
    }
    if (joint.error_code != 0) {
      report.driver_error = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::DriverError);
      const protocol::FaultDetail driver = MakeFaultDetail(
          protocol::FaultReason::DriverError,
          protocol::FaultCondition::JointDriverError, joint_index,
          joint.error_code, 0.0, 0.0, joint.rx_stamp_us, "driver error");
      AppendDetail(&report.details, driver);
      AppendFaultObservation(report.observations, driver);
    }
    if (joint.freq_hz < config.boot.minimum_frequency_hz) {
      report.frequency_low = true;
      if (!boot_grace) {
        report.feedback_invalid = true;
        fault = true;
        RaiseReason(&reason, protocol::FaultReason::FeedbackInvalid);
        const protocol::FaultDetail slow = MakeFaultDetail(
            protocol::FaultReason::FeedbackInvalid,
            protocol::FaultCondition::JointFrequencyLow, joint_index, 0,
            joint.freq_hz, config.boot.minimum_frequency_hz, joint.rx_stamp_us,
            "control frequency below minimum");
        AppendDetail(&report.details, slow);
        AppendFaultObservation(report.observations, slow);
      }
    }
  }

  // Pass B: position limits.
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    if (boot_grace && !observed.joints[index] && IsJointSampleAbsent(joint)) {
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
      const protocol::FaultDetail position =
          MakeFaultDetail(protocol::FaultReason::JointPositionExceeded,
                          protocol::FaultCondition::JointPositionExceeded,
                          static_cast<std::uint32_t>(index), 0, joint.q, limit,
                          joint.rx_stamp_us, "joint position limit exceeded");
      AppendDetail(&report.details, position);
      AppendFaultObservation(report.observations, position);
    }
  }

  // Pass C: velocity limits. Equality is within limit; invalid or stale
  // velocities are still compared instead of being treated as zero.
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    if (boot_grace && !observed.joints[index] && IsJointSampleAbsent(joint)) {
      continue;
    }
    const double limit = config.joint_velocity_limits[index];
    if (!(std::abs(joint.dq) <= limit)) {
      report.velocity_exceeded = true;
      fault = true;
      RaiseReason(&reason, protocol::FaultReason::JointVelocityExceeded);
      const protocol::FaultDetail velocity =
          MakeFaultDetail(protocol::FaultReason::JointVelocityExceeded,
                          protocol::FaultCondition::JointVelocityExceeded,
                          static_cast<std::uint32_t>(index), 0, joint.dq, limit,
                          joint.rx_stamp_us, "joint velocity limit exceeded");
      AppendDetail(&report.details, velocity);
      AppendFaultObservation(report.observations, velocity);
    }
  }

  if (config.imu_required) {
    const protocol::ImuFeedback &imu = feedback.imu;
    if (boot_grace && !observed.imu && IsImuSampleAbsent(imu)) {
      // An IMU that has never delivered a sample is waiting, exactly like an
      // absent joint; an observed invalid, missing or stale IMU keeps faulting.
      report.awaiting_feedback = true;
    } else {
      const bool invalid = !imu.valid;
      const bool non_finite = !IsImuSampleFinite(imu);
      const bool stale = !boot_grace && IsStale(imu.rx_stamp_us, now_us,
                                                config.timeouts.imu_us);
      if (invalid || non_finite || stale) {
        report.imu_invalid = true;
        fault = true;
        RaiseReason(&reason, protocol::FaultReason::ImuInvalid);
        // Like a joint, the published report carries one combined entry named
        // after the first established cause, while the complete diagnostic
        // reports invalid, non-finite and stale samples separately.
        protocol::FaultCondition first_condition =
            protocol::FaultCondition::ImuFeedbackInvalid;
        std::string_view first_message = "imu feedback invalid";
        if (!invalid && non_finite) {
          first_condition = protocol::FaultCondition::ImuFeedbackNonFinite;
          first_message = "imu feedback non-finite";
        } else if (!invalid) {
          first_condition = protocol::FaultCondition::ImuFeedbackStale;
          first_message = "imu feedback stale";
        }
        AppendDetail(
            &report.details,
            MakeFaultDetail(protocol::FaultReason::ImuInvalid, first_condition,
                            protocol::kNoJointIndex, 0, 0.0, 0.0,
                            imu.rx_stamp_us, first_message));
        if (invalid) {
          AppendFaultObservation(
              report.observations,
              MakeFaultDetail(protocol::FaultReason::ImuInvalid,
                              protocol::FaultCondition::ImuFeedbackInvalid,
                              protocol::kNoJointIndex, 0, 0.0, 0.0,
                              imu.rx_stamp_us, "imu feedback invalid"));
        }
        if (non_finite) {
          AppendFaultObservation(
              report.observations,
              MakeFaultDetail(protocol::FaultReason::ImuInvalid,
                              protocol::FaultCondition::ImuFeedbackNonFinite,
                              protocol::kNoJointIndex, 0, 0.0, 0.0,
                              imu.rx_stamp_us, "imu feedback non-finite"));
        }
        if (stale) {
          AppendFaultObservation(
              report.observations,
              MakeFaultDetail(protocol::FaultReason::ImuInvalid,
                              protocol::FaultCondition::ImuFeedbackStale,
                              protocol::kNoJointIndex, 0, 0.0, 0.0,
                              imu.rx_stamp_us, "imu feedback stale"));
        }
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
