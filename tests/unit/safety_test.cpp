#include "irmv/runtime/safety.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::LoadConfig;
using irmv::configuration::RuntimeConfig;
using irmv::protocol::FaultDetail;
using irmv::protocol::FaultReason;
using irmv::protocol::FaultReport;
using irmv::protocol::FeedbackSnapshot;
using irmv::protocol::JointCommand;
using irmv::protocol::kMaxJoints;
using irmv::protocol::kNoJointIndex;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::OutputAction;
using irmv::protocol::TimeUs;
using irmv::runtime::CheckFeedback;
using irmv::runtime::ComputeDamping;
using irmv::runtime::ComputeTracking;
using irmv::runtime::ControlOutput;
using irmv::runtime::SafetyReport;

constexpr TimeUs kNowUs = 1'000'000;
constexpr TimeUs kFreshUs = kNowUs - 1'000;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

FeedbackSnapshot HealthyFeedback(const RuntimeConfig &config) {
  FeedbackSnapshot feedback;
  feedback.snapshot_seq = 7;
  feedback.stamp_us = kFreshUs;
  feedback.count = static_cast<std::uint32_t>(config.joint_count());
  for (std::size_t i = 0; i < config.joint_count(); ++i) {
    feedback.joints[i].valid = true;
    feedback.joints[i].connected = true;
    feedback.joints[i].q = 0.0;
    feedback.joints[i].dq = 0.0;
    feedback.joints[i].tau_est = 0.0;
    feedback.joints[i].rx_stamp_us = kFreshUs;
    feedback.joints[i].freq_hz = 500.0;
    feedback.joints[i].error_code = 0;
  }
  feedback.imu.valid = true;
  feedback.imu.orientation_rpy = {0.0, 0.0, 0.0};
  feedback.imu.angular_velocity = {0.0, 0.0, 0.0};
  feedback.imu.linear_acceleration = {0.0, 0.0, 9.81};
  feedback.imu.rx_stamp_us = kFreshUs;
  return feedback;
}

const FaultDetail *FindDetail(const FaultReport &report, FaultReason reason,
                              std::uint32_t joint_index) {
  for (std::uint32_t i = 0; i < report.count; ++i) {
    if (report.details[i].reason == reason &&
        report.details[i].joint_index == joint_index) {
      return &report.details[i];
    }
  }
  return nullptr;
}

std::uint32_t CountDetails(const FaultReport &report, FaultReason reason) {
  std::uint32_t count = 0;
  for (std::uint32_t i = 0; i < report.count; ++i) {
    if (report.details[i].reason == reason) {
      ++count;
    }
  }
  return count;
}

RuntimeConfig ManyJointConfig(std::size_t joint_count) {
  RuntimeConfig config;
  config.joint_names.reserve(joint_count);
  for (std::size_t i = 0; i < joint_count; ++i) {
    config.joint_names.push_back("joint_" + std::to_string(i));
  }
  config.position_min.fill(-2.0);
  config.position_max.fill(2.0);
  config.damping_kd.fill(2.0);
  config.joint_velocity_limits.fill(10.0);
  config.torque_limits.fill(20.0);
  config.target_velocity_limits.fill(10.0);
  config.timeouts.command_us = 100'000;
  config.timeouts.source_state_us = 150'000;
  config.timeouts.feedback_us = 20'000;
  config.timeouts.imu_us = 20'000;
  config.timeouts.supervisor_us = 300'000;
  config.boot.frequency_grace_us = 1'000'000;
  config.boot.healthy_window_us = 100'000;
  config.boot.deadline_us = 2'000'000;
  config.boot.minimum_frequency_hz = 400.0;
  config.layout_id = "wide_layout";
  return config;
}

JointCommand CommandFor(const RuntimeConfig &config) {
  JointCommand command;
  command.protocol_version = kProtocolVersion;
  command.stamp_us = kNowUs;
  command.joints.count = static_cast<std::uint32_t>(config.joint_count());
  static_cast<void>(command.layout_id.Assign(config.layout_id));
  return command;
}

}  // namespace

int main() {
  const RuntimeConfig config = LoadConfig("config/sdk.json");
  CHECK(config.joint_count() == 2);
  CHECK(config.torque_limits[0] == 20.0);
  CHECK(config.torque_limits[1] == 20.0);
  CHECK(config.timeouts.feedback_us == 20'000);

  // ---- CheckFeedback: healthy baseline ----
  FeedbackSnapshot feedback = HealthyFeedback(config);
  SafetyReport report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.healthy);
  CHECK(!report.fault);
  CHECK(!report.stale_feedback);
  CHECK(!report.frequency_low);
  CHECK(!report.feedback_invalid);
  CHECK(!report.driver_error);
  CHECK(!report.position_exceeded);
  CHECK(!report.velocity_exceeded);
  CHECK(!report.imu_invalid);
  CHECK(!report.damping_eligible);
  CHECK(report.reason == FaultReason::None);
  CHECK(report.details.count == 0);

  // IMU is ignored entirely when the config does not require it.
  feedback.imu.valid = false;
  feedback.imu.rx_stamp_us = 0;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.healthy);
  CHECK(!report.imu_invalid);

  // ---- CheckFeedback: count mismatch ----
  feedback = HealthyFeedback(config);
  feedback.count = 3;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(!report.healthy);
  CHECK(report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);
  CHECK(!report.damping_eligible);
  CHECK(report.details.count == 0);

  feedback = HealthyFeedback(config);
  feedback.count = 0;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);

  // ---- CheckFeedback: invalid samples ----
  feedback = HealthyFeedback(config);
  feedback.joints[1].valid = false;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);
  CHECK(!report.damping_eligible);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].reason == FaultReason::FeedbackInvalid);
  CHECK(report.details.details[0].joint_index == 1);
  CHECK(report.details.details[0].sample_stamp_us == kFreshUs);
  CHECK(report.details.details[0].detected_stamp_us == kNowUs);
  CHECK(report.details.details[0].output_action == OutputAction::ZeroTorque);
  CHECK(!report.details.details[0].message.View().empty());

  feedback = HealthyFeedback(config);
  feedback.joints[0].connected = false;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].joint_index == 0);

  feedback = HealthyFeedback(config);
  feedback.joints[0].q = kNaN;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);

  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = kInf;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);

  feedback = HealthyFeedback(config);
  feedback.joints[1].tau_est = kNaN;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);

  feedback = HealthyFeedback(config);
  feedback.joints[1].freq_hz = kNaN;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);

  // An invalid sample is never excused by the boot grace window.
  feedback = HealthyFeedback(config);
  feedback.joints[0].valid = false;
  report = CheckFeedback(config, feedback, kNowUs, true);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(!report.damping_eligible);

  // ---- CheckFeedback: freshness ----
  feedback = HealthyFeedback(config);
  feedback.joints[0].rx_stamp_us = kNowUs - config.timeouts.feedback_us - 1;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.stale_feedback);
  CHECK(!report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);
  CHECK(!report.damping_eligible);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].reason == FaultReason::FeedbackInvalid);
  CHECK(report.details.details[0].joint_index == 0);
  CHECK(report.details.details[0].sample_stamp_us ==
        kNowUs - config.timeouts.feedback_us - 1);

  feedback = HealthyFeedback(config);
  feedback.joints[0].rx_stamp_us = kNowUs + 1;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.stale_feedback);

  // Exactly at the timeout is still fresh.
  feedback = HealthyFeedback(config);
  feedback.joints[0].rx_stamp_us = kNowUs - config.timeouts.feedback_us;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.healthy);

  // Boot grace tolerates staleness but records it and never enables damping.
  feedback = HealthyFeedback(config);
  feedback.joints[0].rx_stamp_us = kNowUs - config.timeouts.feedback_us - 1;
  report = CheckFeedback(config, feedback, kNowUs, true);
  CHECK(!report.fault);
  CHECK(report.healthy);
  CHECK(report.stale_feedback);
  CHECK(!report.damping_eligible);
  CHECK(report.reason == FaultReason::None);
  CHECK(report.details.count == 0);

  // ---- CheckFeedback: control frequency ----
  feedback = HealthyFeedback(config);
  feedback.joints[1].freq_hz = config.boot.minimum_frequency_hz - 1.0;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.frequency_low);
  CHECK(!report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);
  CHECK(!report.damping_eligible);

  feedback = HealthyFeedback(config);
  feedback.joints[1].freq_hz = config.boot.minimum_frequency_hz;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.healthy);
  CHECK(!report.frequency_low);

  feedback = HealthyFeedback(config);
  feedback.joints[1].freq_hz = config.boot.minimum_frequency_hz - 1.0;
  report = CheckFeedback(config, feedback, kNowUs, true);
  CHECK(!report.fault);
  CHECK(report.frequency_low);
  CHECK(report.reason == FaultReason::None);
  CHECK(!report.damping_eligible);

  // ---- CheckFeedback: driver errors ----
  feedback = HealthyFeedback(config);
  feedback.joints[0].error_code = 0x80000000u;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.driver_error);
  CHECK(!report.feedback_invalid);
  CHECK(report.reason == FaultReason::DriverError);
  CHECK(!report.damping_eligible);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].reason == FaultReason::DriverError);
  CHECK(report.details.details[0].joint_index == 0);
  CHECK(report.details.details[0].error_code == 0x80000000u);
  CHECK(report.details.details[0].sample_stamp_us == kFreshUs);

  // FeedbackInvalid outranks DriverError when both are observed.
  feedback = HealthyFeedback(config);
  feedback.joints[0].error_code = 0x80000000u;
  feedback.joints[1].rx_stamp_us = kNowUs - config.timeouts.feedback_us - 1;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.reason == FaultReason::FeedbackInvalid);
  CHECK(report.driver_error);
  CHECK(report.stale_feedback);
  CHECK(report.details.count == 2);
  CHECK(FindDetail(report.details, FaultReason::DriverError, 0) != nullptr);
  CHECK(FindDetail(report.details, FaultReason::FeedbackInvalid, 1) != nullptr);

  // The boot grace window does not excuse driver errors.
  feedback = HealthyFeedback(config);
  feedback.joints[1].error_code = 5;
  report = CheckFeedback(config, feedback, kNowUs, true);
  CHECK(report.fault);
  CHECK(report.driver_error);
  CHECK(report.reason == FaultReason::DriverError);
  CHECK(!report.damping_eligible);

  // ---- CheckFeedback: position limits ----
  feedback = HealthyFeedback(config);
  feedback.joints[0].q = config.position_max[0] + 0.5;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.position_exceeded);
  CHECK(!report.feedback_invalid);
  CHECK(!report.velocity_exceeded);
  CHECK(report.reason == FaultReason::JointPositionExceeded);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].reason == FaultReason::JointPositionExceeded);
  CHECK(report.details.details[0].joint_index == 0);
  CHECK(report.details.details[0].observed_value == 2.5);
  CHECK(report.details.details[0].limit == 2.0);
  CHECK(report.details.details[0].sample_stamp_us == kFreshUs);
  CHECK(report.details.details[0].detected_stamp_us == kNowUs);
  CHECK(report.details.details[0].output_action == OutputAction::Damping);
  CHECK(report.damping_eligible);

  feedback = HealthyFeedback(config);
  feedback.joints[0].q = config.position_min[0] - 0.5;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.position_exceeded);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].observed_value == -2.5);
  CHECK(report.details.details[0].limit == -2.0);

  feedback = HealthyFeedback(config);
  feedback.joints[0].q = config.position_min[0];
  feedback.joints[1].q = config.position_max[1];
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.healthy);

  // The boot grace window does not excuse an observed position violation.
  feedback = HealthyFeedback(config);
  feedback.joints[0].q = config.position_max[0] + 0.5;
  report = CheckFeedback(config, feedback, kNowUs, true);
  CHECK(report.fault);
  CHECK(report.position_exceeded);

  // ---- CheckFeedback: velocity limits ----
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = config.joint_velocity_limits[0];
  feedback.joints[1].dq = -config.joint_velocity_limits[1];
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.healthy);
  CHECK(!report.velocity_exceeded);

  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = config.joint_velocity_limits[0] + 0.01;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.velocity_exceeded);
  CHECK(!report.feedback_invalid);
  CHECK(report.reason == FaultReason::JointVelocityExceeded);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].reason == FaultReason::JointVelocityExceeded);
  CHECK(report.details.details[0].joint_index == 0);
  CHECK(report.details.details[0].observed_value == 10.01);
  CHECK(report.details.details[0].limit == 10.0);
  CHECK(report.damping_eligible);
  CHECK(report.details.details[0].output_action == OutputAction::Damping);

  feedback = HealthyFeedback(config);
  feedback.joints[1].dq = -config.joint_velocity_limits[1] - 0.01;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.velocity_exceeded);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].joint_index == 1);
  CHECK(report.details.details[0].observed_value == -10.01);
  CHECK(report.details.details[0].limit == 10.0);

  // Non-finite velocity is a feedback fault, never a zero velocity.
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = kNaN;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.feedback_invalid);
  CHECK(report.reason == FaultReason::FeedbackInvalid);
  CHECK(!report.damping_eligible);

  // Stale velocity is still evaluated against the limit (not treated as zero)
  // and the stale signal blocks damping eligibility.
  feedback = HealthyFeedback(config);
  feedback.joints[1].dq = 12.0;
  feedback.joints[1].rx_stamp_us = kNowUs - config.timeouts.feedback_us - 1;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.stale_feedback);
  CHECK(report.velocity_exceeded);
  CHECK(FindDetail(report.details, FaultReason::JointVelocityExceeded, 1) !=
        nullptr);
  CHECK(!report.damping_eligible);

  // The boot grace window does not excuse an observed velocity violation.
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = config.joint_velocity_limits[0] + 0.01;
  report = CheckFeedback(config, feedback, kNowUs, true);
  CHECK(report.fault);
  CHECK(report.velocity_exceeded);

  // ---- CheckFeedback: IMU ----
  RuntimeConfig imu_config = config;
  imu_config.imu_required = true;
  feedback = HealthyFeedback(imu_config);
  report = CheckFeedback(imu_config, feedback, kNowUs, false);
  CHECK(report.healthy);
  CHECK(!report.imu_invalid);

  feedback = HealthyFeedback(imu_config);
  feedback.imu.valid = false;
  report = CheckFeedback(imu_config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.imu_invalid);
  CHECK(!report.feedback_invalid);
  CHECK(report.reason == FaultReason::ImuInvalid);
  CHECK(report.details.count == 1);
  CHECK(report.details.details[0].reason == FaultReason::ImuInvalid);
  CHECK(report.details.details[0].joint_index == kNoJointIndex);
  CHECK(report.damping_eligible);
  CHECK(report.details.details[0].output_action == OutputAction::Damping);

  feedback = HealthyFeedback(imu_config);
  feedback.imu.angular_velocity[1] = kInf;
  report = CheckFeedback(imu_config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.imu_invalid);
  CHECK(report.reason == FaultReason::ImuInvalid);

  feedback = HealthyFeedback(imu_config);
  feedback.imu.linear_acceleration[2] = kNaN;
  report = CheckFeedback(imu_config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.imu_invalid);

  feedback = HealthyFeedback(imu_config);
  feedback.imu.rx_stamp_us = kNowUs - imu_config.timeouts.imu_us - 1;
  report = CheckFeedback(imu_config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.imu_invalid);
  CHECK(report.reason == FaultReason::ImuInvalid);

  feedback = HealthyFeedback(imu_config);
  feedback.imu.rx_stamp_us = kNowUs - imu_config.timeouts.imu_us;
  report = CheckFeedback(imu_config, feedback, kNowUs, false);
  CHECK(report.healthy);

  // Boot grace tolerates stale IMU samples but not invalid ones.
  feedback = HealthyFeedback(imu_config);
  feedback.imu.rx_stamp_us = kNowUs - imu_config.timeouts.imu_us - 1;
  report = CheckFeedback(imu_config, feedback, kNowUs, true);
  CHECK(!report.fault);
  CHECK(!report.imu_invalid);

  feedback = HealthyFeedback(imu_config);
  feedback.imu.valid = false;
  report = CheckFeedback(imu_config, feedback, kNowUs, true);
  CHECK(report.fault);
  CHECK(report.imu_invalid);

  // ---- CheckFeedback: simultaneous offenders and priority ----
  const RuntimeConfig three_joint = ManyJointConfig(3);
  feedback = HealthyFeedback(three_joint);
  feedback.joints[0].dq = 10.5;
  feedback.joints[1].dq = -12.0;
  feedback.joints[2].error_code = 0x80000000u;
  report = CheckFeedback(three_joint, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.velocity_exceeded);
  CHECK(report.driver_error);
  CHECK(report.reason == FaultReason::DriverError);
  CHECK(!report.damping_eligible);
  CHECK(report.details.count == 3);
  CHECK(CountDetails(report.details, FaultReason::DriverError) == 1);
  CHECK(CountDetails(report.details, FaultReason::JointVelocityExceeded) == 2);
  const FaultDetail *driver =
      FindDetail(report.details, FaultReason::DriverError, 2);
  CHECK(driver != nullptr);
  CHECK(driver->error_code == 0x80000000u);
  CHECK(driver->output_action == OutputAction::ZeroTorque);
  CHECK(FindDetail(report.details, FaultReason::JointVelocityExceeded, 0) !=
        nullptr);
  CHECK(FindDetail(report.details, FaultReason::JointVelocityExceeded, 1) !=
        nullptr);

  // Every simultaneous fresh overspeed joint is visible in the details.
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = 11.0;
  feedback.joints[1].dq = -11.0;
  report = CheckFeedback(config, feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.reason == FaultReason::JointVelocityExceeded);
  CHECK(report.damping_eligible);
  CHECK(report.details.count == 2);
  CHECK(report.details.details[0].joint_index == 0);
  CHECK(report.details.details[1].joint_index == 1);
  CHECK(report.details.details[0].output_action == OutputAction::Damping);
  CHECK(report.details.details[1].output_action == OutputAction::Damping);

  // ---- CheckFeedback: detail capacity ----
  const RuntimeConfig max_joint_config = ManyJointConfig(kMaxJoints);
  FeedbackSnapshot wide_feedback = HealthyFeedback(max_joint_config);
  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    wide_feedback.joints[i].dq = 20.0;
  }
  report = CheckFeedback(max_joint_config, wide_feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.velocity_exceeded);
  CHECK(report.reason == FaultReason::JointVelocityExceeded);
  CHECK(report.damping_eligible);
  CHECK(report.details.count == kMaxJoints);
  CHECK(CountDetails(report.details, FaultReason::JointVelocityExceeded) ==
        kMaxJoints);

  // Driver errors (higher-priority pass) keep their slots when overspeed
  // details would overflow the report capacity; flags and reason are intact.
  wide_feedback = HealthyFeedback(max_joint_config);
  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    wide_feedback.joints[i].dq = 20.0;
    wide_feedback.joints[i].error_code = 0x10;
  }
  report = CheckFeedback(max_joint_config, wide_feedback, kNowUs, false);
  CHECK(report.fault);
  CHECK(report.driver_error);
  CHECK(report.velocity_exceeded);
  CHECK(report.reason == FaultReason::DriverError);
  CHECK(!report.damping_eligible);
  CHECK(report.details.count == kMaxJoints);
  CHECK(CountDetails(report.details, FaultReason::DriverError) == kMaxJoints);

  // ---- ComputeTracking ----
  JointCommand command = CommandFor(config);
  ControlOutput output = ComputeTracking(config, command, feedback);
  CHECK(output.valid);
  CHECK(output.output.count == 2);
  CHECK(output.output.torque[0] == 0.0);
  CHECK(output.output.torque[1] == 0.0);

  // Plan example: raw 33 is clamped to the configured torque limit.
  command = CommandFor(config);
  command.joints.q[0] = 1.0;
  command.joints.dq[0] = 0.0;
  command.joints.kp[0] = 30.0;
  command.joints.kd[0] = 2.0;
  command.joints.tau_ff[0] = 5.0;
  command.joints.tau_ff[1] = 5.0;
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = 1.0;
  output = ComputeTracking(config, command, feedback);
  CHECK(output.valid);
  CHECK(output.output.count == 2);
  CHECK(output.output.torque[0] == 20.0);
  CHECK(output.output.torque[1] == 5.0);

  // Unclamped proportional plus derivative plus feedforward.
  command = CommandFor(config);
  command.joints.q[0] = 1.0;
  command.joints.dq[0] = 1.0;
  command.joints.kp[0] = 1.0;
  command.joints.kd[0] = 1.0;
  feedback = HealthyFeedback(config);
  output = ComputeTracking(config, command, feedback);
  CHECK(output.valid);
  CHECK(output.output.torque[0] == 2.0);

  // Pure feedforward, including the clamp.
  command = CommandFor(config);
  command.joints.tau_ff[0] = 5.0;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);
  CHECK(output.output.torque[0] == 5.0);

  command.joints.tau_ff[0] = 100.0;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);
  CHECK(output.output.torque[0] == 20.0);

  command.joints.tau_ff[0] = -100.0;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);
  CHECK(output.output.torque[0] == -20.0);

  command.joints.tau_ff[0] = -5.0;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);
  CHECK(output.output.torque[0] == -5.0);

  // Command shape rejections: no partial output on rejection.
  command = CommandFor(config);
  command.joints.count = 1;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);
  CHECK(output.output.count == 0);
  CHECK(output.output.torque[0] == 0.0);

  command = CommandFor(config);
  command.protocol_version = kProtocolVersion + 1;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);
  CHECK(output.output.count == 0);

  command = CommandFor(config);
  static_cast<void>(command.layout_id.Assign("other_layout"));
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);
  CHECK(output.output.count == 0);

  command = CommandFor(config);
  command.joints.q[0] = kNaN;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.dq[0] = kInf;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.tau_ff[0] = kNaN;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.kp[0] = kInf;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.kd[0] = kNaN;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.kp[0] = -1.0;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.kd[0] = -0.5;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  // Target position must stay inside the configured range.
  command = CommandFor(config);
  command.joints.q[0] = config.position_max[0] + 0.5;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.q[0] = config.position_min[0] - 0.5;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.q[0] = config.position_max[0];
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);

  command = CommandFor(config);
  command.joints.q[0] = config.position_min[0];
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);

  // Target velocity limit is inclusive.
  command = CommandFor(config);
  command.joints.dq[0] = config.target_velocity_limits[0];
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(output.valid);

  command = CommandFor(config);
  command.joints.dq[0] = config.target_velocity_limits[0] + 0.01;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  command = CommandFor(config);
  command.joints.dq[0] = -config.target_velocity_limits[0] - 0.01;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);

  // A finite but huge gain that overflows a product must not produce a
  // clamped or NaN torque.
  command = CommandFor(config);
  command.joints.q[0] = config.position_max[0];
  command.joints.kp[0] = 1e308;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);
  CHECK(output.output.count == 0);

  // A finite but huge bias that overflows the sum must not be clamped.
  command = CommandFor(config);
  command.joints.q[0] = 1.0;
  command.joints.kp[0] = 1e308;
  command.joints.tau_ff[0] = 1e308;
  output = ComputeTracking(config, command, HealthyFeedback(config));
  CHECK(!output.valid);
  CHECK(output.output.count == 0);

  // Non-finite measured state must not leak into the torque output.
  command = CommandFor(config);
  command.joints.q[0] = 1.0;
  command.joints.kp[0] = 1.0;
  feedback = HealthyFeedback(config);
  feedback.joints[0].q = kNaN;
  output = ComputeTracking(config, command, feedback);
  CHECK(!output.valid);

  // ---- ComputeDamping ----
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = 1.0;
  feedback.joints[1].dq = -3.0;
  output = ComputeDamping(config, feedback);
  CHECK(output.valid);
  CHECK(output.output.count == 2);
  CHECK(output.output.torque[0] == -2.0);
  CHECK(output.output.torque[1] == 6.0);

  feedback.joints[0].dq = 10.0;
  feedback.joints[1].dq = -100.0;
  output = ComputeDamping(config, feedback);
  CHECK(output.valid);
  CHECK(output.output.torque[0] == -20.0);
  CHECK(output.output.torque[1] == 20.0);

  // Stale feedback is accepted here: damping does not check freshness.
  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = 2.0;
  feedback.joints[0].rx_stamp_us = 0;
  output = ComputeDamping(config, feedback);
  CHECK(output.valid);
  CHECK(output.output.torque[0] == -4.0);

  feedback = HealthyFeedback(config);
  feedback.joints[1].valid = false;
  output = ComputeDamping(config, feedback);
  CHECK(!output.valid);
  CHECK(output.output.count == 0);

  feedback = HealthyFeedback(config);
  feedback.joints[0].connected = false;
  output = ComputeDamping(config, feedback);
  CHECK(!output.valid);

  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = kNaN;
  output = ComputeDamping(config, feedback);
  CHECK(!output.valid);

  feedback = HealthyFeedback(config);
  feedback.joints[0].dq = kInf;
  output = ComputeDamping(config, feedback);
  CHECK(!output.valid);

  feedback = HealthyFeedback(config);
  feedback.count = 1;
  output = ComputeDamping(config, feedback);
  CHECK(!output.valid);

  return 0;
}
