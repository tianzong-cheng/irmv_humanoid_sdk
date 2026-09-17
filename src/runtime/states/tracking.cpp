#include "irmv/runtime/execution_fsm.hpp"

namespace irmv::runtime {

// Tracking: one producer owns one epoch and the last accepted full frame is
// held. The commanded torque is recomputed from the fresh measured state on
// every tick; the target deadlines are checked by the tick after the request
// phase, and the exit to Damping is driven from there or by a stop request.

void ExecutionFsm::EnterTracking(TickScope &scope) {
  static_cast<void>(scope);
  output_action_ = protocol::OutputAction::Tracking;
}

void ExecutionFsm::LeaveTracking(TickScope &scope) {
  static_cast<void>(scope);
}

hardware::TorqueOutput ExecutionFsm::StepTracking(TickScope &scope) {
  if (!gate_.has_target()) {
    // Without an accepted frame there is nothing to hold: the state must never
    // keep commanding a stale or partial target.
    output_action_ = protocol::OutputAction::ZeroTorque;
    LatchFault(scope, protocol::FaultReason::CommandInvalid,
               EventCode::FaultLatched, false,
               MakeFaultReport(protocol::FaultReason::CommandInvalid,
                               scope.now_us, 0,
                               "no accepted tracking target"));
    return ZeroTorqueOutput();
  }
  const ControlOutput control =
      ComputeTracking(config_, gate_.current_target(), *scope.feedback);
  if (!control.valid) {
    // A target that can no longer be executed rejects the whole command.
    output_action_ = protocol::OutputAction::ZeroTorque;
    LatchFault(scope, protocol::FaultReason::CommandInvalid,
               EventCode::FaultLatched, DampingEligible(*scope.report),
               MakeFaultReport(protocol::FaultReason::CommandInvalid,
                               scope.now_us, 0,
                               "accepted tracking target not executable"));
    return ZeroTorqueOutput();
  }
  output_action_ = protocol::OutputAction::Tracking;
  return control.output;
}

protocol::RejectReason ExecutionFsm::TrackingAllows(
    protocol::Operation operation) const {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      return protocol::RejectReason::None;
    case protocol::Operation::Damping:
    case protocol::Operation::ZeroTorque:
      return protocol::RejectReason::None;
    case protocol::Operation::Tracking:
      // Switching the source is a guarded, atomic authorization; the guards run
      // in HandleTracking.
      return protocol::RejectReason::None;
    case protocol::Operation::Reset:
      return protocol::RejectReason::InvalidTransition;
  }
  return protocol::RejectReason::InvalidTransition;
}

}  // namespace irmv::runtime
