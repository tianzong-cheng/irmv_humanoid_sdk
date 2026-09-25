#include "irmv/application/initialization.hpp"

#include <cmath>
#include <cstdint>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

Initialization::Initialization(const configuration::ApplicationConfig &config)
    : config_(&config) {}

bool Initialization::Usable(const protocol::RobotState &fresh,
                            protocol::TimeUs now_us) const {
  if (fresh.protocol_version != protocol::kProtocolVersion) {
    return false;
  }
  const std::size_t joint_count = config_->joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints) {
    return false;
  }
  if (fresh.feedback.count != joint_count) {
    return false;
  }
  if (fresh.stamp_us == 0 || now_us < fresh.stamp_us) {
    return false;
  }
  if (now_us - fresh.stamp_us > config_->runtime.timeouts.source_state_us) {
    return false;
  }
  if (have_last_ && now_us < last_us_) {
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = fresh.feedback.joints[index];
    if (!joint.valid || !joint.connected) {
      return false;
    }
    if (!std::isfinite(joint.q) || !std::isfinite(joint.dq)) {
      return false;
    }
  }
  return true;
}

bool Initialization::WithinCompletionTolerance(
    const protocol::RobotState &fresh) const {
  const configuration::TrajectoryConfig &trajectory =
      config_->initialization.trajectory;
  for (std::size_t index = 0; index < config_->joint_count(); ++index) {
    if (std::fabs(fresh.feedback.joints[index].q -
                  config_->initialization.target_q[index]) >
        trajectory.position_tolerance) {
      return false;
    }
    if (std::fabs(fresh.feedback.joints[index].dq) >
        trajectory.velocity_tolerance) {
      return false;
    }
  }
  return true;
}

bool Initialization::BeginFrame(protocol::JointCommand *command,
                                const protocol::RobotState &fresh) {
  if (!command->layout_id.Assign(config_->layout_id())) {
    return false;
  }
  command->protocol_version = protocol::kProtocolVersion;
  command->joints = protocol::JointVector{};
  command->joints.count = static_cast<std::uint32_t>(config_->joint_count());
  command->producer_id = producer_id_;
  command->epoch = epoch_;
  command->command_seq = ++command_seq_;
  command->source_state_seq = fresh.state_seq;
  return true;
}

void Initialization::FillTarget(protocol::JointCommand *command) const {
  for (std::size_t index = 0; index < config_->joint_count(); ++index) {
    command->joints.q[index] = config_->initialization.target_q[index];
    command->joints.dq[index] = 0.0;
    command->joints.tau_ff[index] = 0.0;
    command->joints.kp[index] = config_->initialization.gains.kp[index];
    command->joints.kd[index] = config_->initialization.gains.kd[index];
  }
}

void Initialization::FillRamp(protocol::JointCommand *command,
                              protocol::TimeUs elapsed) const {
  const configuration::TrajectoryConfig &trajectory =
      config_->initialization.trajectory;
  const double fraction = static_cast<double>(elapsed) /
                          static_cast<double>(trajectory.duration_us);
  const double duration_seconds =
      static_cast<double>(trajectory.duration_us) / 1'000'000.0;
  for (std::size_t index = 0; index < config_->joint_count(); ++index) {
    const double delta =
        config_->initialization.target_q[index] - start_q_[index];
    command->joints.q[index] = start_q_[index] + delta * fraction;
    command->joints.dq[index] = delta / duration_seconds;
    command->joints.tau_ff[index] = 0.0;
    command->joints.kp[index] = config_->initialization.gains.kp[index];
    command->joints.kd[index] = config_->initialization.gains.kd[index];
  }
}

void Initialization::Fail() {
  active_ = false;
  ready_ = false;
  failed_ = true;
  stable_open_ = false;
  stable_since_ = 0;
}

bool Initialization::Start(const protocol::RobotState &fresh,
                           protocol::Sequence epoch, protocol::Id producer_id,
                           protocol::TimeUs now_us) {
  if (active_) {
    return false;
  }
  if (producer_id == protocol::kNoId) {
    return false;
  }
  if (!Usable(fresh, now_us)) {
    return false;
  }
  for (std::size_t index = 0; index < config_->joint_count(); ++index) {
    start_q_[index] = fresh.feedback.joints[index].q;
  }
  producer_id_ = producer_id;
  epoch_ = epoch;
  command_seq_ = 0;
  start_us_ = now_us;
  stable_since_ = 0;
  stable_open_ = false;
  have_last_ = true;
  last_us_ = now_us;
  active_ = true;
  failed_ = false;
  ready_ = false;
  return true;
}

InitializationStatus Initialization::Step(const protocol::RobotState &fresh,
                                          protocol::TimeUs now_us,
                                          protocol::JointCommand *command) {
  if (!active_) {
    return InitializationStatus::Failed;
  }
  if (command == nullptr) {
    return InitializationStatus::Failed;
  }
  if (!Usable(fresh, now_us)) {
    Fail();
    return InitializationStatus::Failed;
  }
  last_us_ = now_us;
  const configuration::TrajectoryConfig &trajectory =
      config_->initialization.trajectory;
  const protocol::TimeUs elapsed = now_us - start_us_;

  // Completion is judged before the deadline: a move that reached its target
  // and held it for the whole stable window is ready even at the last moment
  // before the deadline. Before the ramp finished the window is never opened.
  if (elapsed >= trajectory.duration_us && WithinCompletionTolerance(fresh)) {
    if (!stable_open_) {
      stable_open_ = true;
      stable_since_ = now_us;
    }
    if (now_us - stable_since_ >= trajectory.stable_window_us) {
      if (!BeginFrame(command, fresh)) {
        Fail();
        return InitializationStatus::Failed;
      }
      FillTarget(command);
      active_ = false;
      ready_ = true;
      stable_open_ = false;
      stable_since_ = 0;
      return InitializationStatus::Ready;
    }
  } else if (stable_open_) {
    // One out-of-tolerance sample after the ramp finished throws the
    // accumulated stability away; the window restarts at the next in-tolerance
    // sample.
    stable_open_ = false;
    stable_since_ = 0;
  }

  if (elapsed > trajectory.deadline_us) {
    // The command stays untouched: a call the caller may not send must never
    // hand it a frame.
    Fail();
    return InitializationStatus::Failed;
  }

  if (!BeginFrame(command, fresh)) {
    Fail();
    return InitializationStatus::Failed;
  }
  if (elapsed >= trajectory.duration_us) {
    FillTarget(command);
  } else {
    FillRamp(command, elapsed);
  }
  return InitializationStatus::Running;
}

InitializationStatus Initialization::Hold(const protocol::RobotState &fresh,
                                          protocol::TimeUs now_us,
                                          protocol::JointCommand *command) {
  if (!ready_) {
    return InitializationStatus::Failed;
  }
  if (command == nullptr) {
    return InitializationStatus::Failed;
  }
  if (!Usable(fresh, now_us)) {
    return InitializationStatus::Failed;
  }
  last_us_ = now_us;
  if (!BeginFrame(command, fresh)) {
    Fail();
    return InitializationStatus::Failed;
  }
  FillTarget(command);
  return InitializationStatus::Ready;
}

}  // namespace irmv::application
