#include <cstddef>

#include "irmv/runtime/execution_fsm.hpp"

namespace irmv::runtime {

// Boot: no motion authorization, zero torque, startup self-check. The boot
// start is stamped on the first tick; the healthy window starts at the first
// tick whose measured state is fully usable and restarts on every unhealthy
// tick. A safety fault latched before this step already left the state.

void ExecutionFsm::EnterBoot(TickScope &scope) {
  boot_started_ = true;
  boot_start_us_ = scope.now_us;
  boot_window_started_ = false;
  boot_window_start_us_ = 0;
  output_action_ = protocol::OutputAction::ZeroTorque;
}

void ExecutionFsm::LeaveBoot(TickScope &scope) {
  static_cast<void>(scope);
  boot_window_started_ = false;
  boot_window_start_us_ = 0;
}

bool ExecutionFsm::BootReady(TickScope &scope) const {
  if (scope.report->fault) {
    return false;
  }
  const protocol::FeedbackSnapshot &feedback = *scope.feedback;
  const std::size_t joint_count = config_.joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints ||
      feedback.count != joint_count) {
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = feedback.joints[index];
    if (!joint.valid || !joint.connected) {
      return false;
    }
    if (joint.rx_stamp_us > scope.now_us ||
        scope.now_us - joint.rx_stamp_us > config_.timeouts.feedback_us) {
      return false;
    }
    if (!(joint.freq_hz >= config_.boot.minimum_frequency_hz)) {
      return false;
    }
  }
  if (config_.imu_required) {
    const protocol::ImuFeedback &imu = feedback.imu;
    if (!imu.valid) {
      return false;
    }
    if (imu.rx_stamp_us > scope.now_us ||
        scope.now_us - imu.rx_stamp_us > config_.timeouts.imu_us) {
      return false;
    }
  }
  return true;
}

hardware::TorqueOutput ExecutionFsm::StepBoot(TickScope &scope) {
  if (!boot_started_) {
    boot_started_ = true;
    boot_start_us_ = scope.now_us;
  }
  output_action_ = protocol::OutputAction::ZeroTorque;
  if (scope.now_us - boot_start_us_ > config_.boot.deadline_us) {
    LatchFault(scope, protocol::FaultReason::BootFailed, EventCode::BootFailed,
               false,
               MakeFaultReport(protocol::FaultReason::BootFailed, scope.now_us,
                               0, "boot deadline exceeded"));
    return ZeroTorqueOutput();
  }
  if (BootReady(scope)) {
    if (!boot_window_started_) {
      boot_window_started_ = true;
      boot_window_start_us_ = scope.now_us;
    }
    if (scope.now_us - boot_window_start_us_ >=
        config_.boot.healthy_window_us) {
      SetState(protocol::ExecutionState::ZeroTorque, scope,
               EventCode::BootCompleted);
    }
  } else {
    boot_window_started_ = false;
  }
  return ZeroTorqueOutput();
}

protocol::RejectReason ExecutionFsm::BootAllows(
    protocol::Operation operation) const {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      // Session bootstrap is handled before the state gate and is legal in
      // every state, including Boot.
      return protocol::RejectReason::None;
    case protocol::Operation::Damping:
    case protocol::Operation::ZeroTorque:
    case protocol::Operation::Tracking:
    case protocol::Operation::Reset:
      return protocol::RejectReason::InvalidTransition;
  }
  return protocol::RejectReason::InvalidTransition;
}

}  // namespace irmv::runtime
