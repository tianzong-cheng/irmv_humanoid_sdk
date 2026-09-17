#include "irmv/runtime/execution_fsm.hpp"

namespace irmv::runtime {

// Damping: the safe motion state. The damping torque is recomputed from the
// current feedback on every tick, no tracking command is consumed and no target
// deadline is tracked. The state is left only by a stop request or by a granted
// tracking authorization.

void ExecutionFsm::EnterDamping(TickScope &scope) {
  static_cast<void>(scope);
  output_action_ = protocol::OutputAction::Damping;
}

void ExecutionFsm::LeaveDamping(TickScope &scope) {
  static_cast<void>(scope);
}

hardware::TorqueOutput ExecutionFsm::StepDamping(TickScope &scope) {
  const ControlOutput control = ComputeDamping(config_, *scope.feedback);
  if (!control.valid) {
    // The measured state cannot be damped: no partial or unclamped torque is
    // produced, and the unusable input fault disables damping output.
    output_action_ = protocol::OutputAction::ZeroTorque;
    LatchFault(scope, protocol::FaultReason::FeedbackInvalid,
               EventCode::FaultLatched, false,
               MakeFaultReport(protocol::FaultReason::FeedbackInvalid,
                               scope.now_us, 0, "damping output unavailable"));
    return ZeroTorqueOutput();
  }
  output_action_ = protocol::OutputAction::Damping;
  return control.output;
}

protocol::RejectReason ExecutionFsm::DampingAllows(
    protocol::Operation operation) const {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      return protocol::RejectReason::None;
    case protocol::Operation::Damping:
    case protocol::Operation::ZeroTorque:
      return protocol::RejectReason::None;
    case protocol::Operation::Tracking:
      // The remaining tracking entry guards run in HandleTracking.
      return protocol::RejectReason::None;
    case protocol::Operation::Reset:
      return protocol::RejectReason::InvalidTransition;
  }
  return protocol::RejectReason::InvalidTransition;
}

}  // namespace irmv::runtime
