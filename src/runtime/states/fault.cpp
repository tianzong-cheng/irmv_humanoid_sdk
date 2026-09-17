#include <cmath>
#include <cstddef>

#include "irmv/runtime/execution_fsm.hpp"

namespace irmv::runtime {

// Fault: latched until a matching Reset succeeds. The state reports the fault
// while the output may still be damping; damping eligibility, once lost, never
// comes back within the same fault.

bool ExecutionFsm::DampingEligible(const SafetyReport &report) {
  return !report.feedback_invalid && !report.stale_feedback &&
         !report.frequency_low && !report.driver_error;
}

void ExecutionFsm::EnterFault(TickScope &scope) {
  static_cast<void>(scope);
  recovery_window_started_ = false;
  recovery_window_start_us_ = 0;
  recovery_ready_ = false;
}

void ExecutionFsm::LeaveFault(TickScope &scope) {
  static_cast<void>(scope);
  recovery_window_started_ = false;
  recovery_window_start_us_ = 0;
  recovery_ready_ = false;
}

hardware::TorqueOutput ExecutionFsm::StepFault(TickScope &scope) {
  if (fault_damping_eligible_) {
    const ControlOutput control = ComputeDamping(config_, *scope.feedback);
    if (control.valid) {
      output_action_ = protocol::OutputAction::Damping;
      return control.output;
    }
  }
  output_action_ = protocol::OutputAction::ZeroTorque;
  return ZeroTorqueOutput();
}

protocol::RejectReason ExecutionFsm::FaultAllows(
    protocol::Operation operation) const {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      return protocol::RejectReason::None;
    case protocol::Operation::Reset:
      // The remaining Reset guards run in HandleReset.
      return protocol::RejectReason::None;
    case protocol::Operation::Damping:
    case protocol::Operation::ZeroTorque:
      // A latched fault is left only through Reset.
      return protocol::RejectReason::InvalidTransition;
    case protocol::Operation::Tracking:
      return protocol::RejectReason::InvalidTransition;
  }
  return protocol::RejectReason::InvalidTransition;
}

void ExecutionFsm::LatchFault(TickScope &scope, protocol::FaultReason reason,
                              EventCode code, bool damping_eligible,
                              const protocol::FaultReport &report) {
  if (state_ == protocol::ExecutionState::Fault) {
    // The first latched fault wins; the new observation is reported but never
    // replaces the latched identity or its recovery window.
    EmitEvent(scope, EventCode::FaultLatched, protocol::ExecutionState::Fault,
              protocol::ExecutionState::Fault, protocol::RejectReason::None,
              reason);
    if (code != EventCode::FaultLatched && code != EventCode::None) {
      EmitEvent(scope, code, protocol::ExecutionState::Fault,
                protocol::ExecutionState::Fault, protocol::RejectReason::None,
                reason);
    }
    return;
  }
  fault_id_ = ++fault_counter_;
  fault_reason_ = reason;
  fault_report_ = report;
  fault_damping_eligible_ = damping_eligible;
  fault_output_degraded_ = !damping_eligible;
  stop_reason_ = protocol::StopReason::None;
  const protocol::ExecutionState previous = state_;
  ClearMotion(scope);
  SetState(protocol::ExecutionState::Fault, scope, EventCode::None);
  // The latch is always reported; a latch with a specific cause is additionally
  // reported under that cause.
  EmitEvent(scope, EventCode::FaultLatched, previous,
            protocol::ExecutionState::Fault, protocol::RejectReason::None,
            reason);
  if (code != EventCode::FaultLatched && code != EventCode::None) {
    EmitEvent(scope, code, previous, protocol::ExecutionState::Fault,
              protocol::RejectReason::None, reason);
  }
}

void ExecutionFsm::UpdateFaultEligibility(TickScope &scope) {
  const SafetyReport &report = *scope.report;
  // 1. Permanent degradation: a fault that was damping-eligible loses damping
  //    output for the rest of the fault as soon as the measured state stops
  //    being usable, or as soon as the send path failed.
  const bool degraded_now =
      !last_write_accepted_ || (report.fault && !DampingEligible(report));
  if (fault_damping_eligible_ && degraded_now) {
    fault_damping_eligible_ = false;
    fault_output_degraded_ = true;
    EmitEvent(scope, EventCode::FaultOutputDegraded,
              protocol::ExecutionState::Fault, protocol::ExecutionState::Fault);
  }

  // 2. Recovery window over consecutive healthy ticks.
  if (!RecoveryTickHealthy(scope)) {
    recovery_window_started_ = false;
    recovery_window_start_us_ = 0;
    recovery_ready_ = false;
    return;
  }
  if (!recovery_window_started_) {
    recovery_window_started_ = true;
    recovery_window_start_us_ = scope.now_us;
  }
  if (!recovery_ready_ && scope.now_us - recovery_window_start_us_ >=
                              config_.recovery.healthy_window_us) {
    recovery_ready_ = true;
    EmitEvent(scope, EventCode::RecoveryReady, protocol::ExecutionState::Fault,
              protocol::ExecutionState::Fault);
  }
}

bool ExecutionFsm::RecoveryTickHealthy(TickScope &scope) const {
  const SafetyReport &report = *scope.report;
  if (report.fault || report.stale_feedback || report.frequency_low ||
      report.feedback_invalid || report.driver_error ||
      report.position_exceeded || report.velocity_exceeded ||
      report.imu_invalid) {
    return false;
  }
  if (!last_write_accepted_) {
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
    if (!(joint.freq_hz >= config_.recovery.minimum_frequency_hz)) {
      return false;
    }
    if (!(std::fabs(joint.dq) <= config_.joint_velocity_limits[index])) {
      return false;
    }
    if (joint.q < config_.position_min[index] ||
        joint.q > config_.position_max[index]) {
      return false;
    }
  }
  if (config_.imu_required && !feedback.imu.valid) {
    return false;
  }
  return true;
}

void ExecutionFsm::DegradeForWriteFailure(TickScope &scope,
                                          std::uint32_t error_code) {
  if (state_ != protocol::ExecutionState::Fault) {
    LatchFault(scope, protocol::FaultReason::WriteFailed,
               EventCode::FaultLatched, false,
               MakeFaultReport(protocol::FaultReason::WriteFailed, scope.now_us,
                               error_code, "torque write refused"));
    return;
  }
  if (fault_damping_eligible_) {
    fault_damping_eligible_ = false;
    fault_output_degraded_ = true;
    EmitEvent(scope, EventCode::FaultOutputDegraded,
              protocol::ExecutionState::Fault, protocol::ExecutionState::Fault);
  }
}

protocol::FaultReport ExecutionFsm::MakeFaultReport(
    protocol::FaultReason reason, protocol::TimeUs now_us,
    std::uint32_t error_code, const char *message) {
  protocol::FaultReport report;
  report.count = 1;
  protocol::FaultDetail &detail = report.details[0];
  detail = protocol::FaultDetail{};
  detail.reason = reason;
  detail.joint_index = protocol::kNoJointIndex;
  detail.error_code = error_code;
  detail.detected_stamp_us = now_us;
  detail.output_action = protocol::OutputAction::ZeroTorque;
  static_cast<void>(detail.message.Assign(message));
  return report;
}

}  // namespace irmv::runtime
