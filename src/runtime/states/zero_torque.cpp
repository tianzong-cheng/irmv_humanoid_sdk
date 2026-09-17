#include "irmv/runtime/execution_fsm.hpp"

namespace irmv::runtime {

// ZeroTorque: no motion authorization and no command consumption; the joints
// receive zero torque until a stop request moves the machine to Damping. There
// is no per-state data to set up or tear down, so the enter and leave steps are
// deliberately empty.

void ExecutionFsm::EnterZeroTorque(TickScope &scope) {
  static_cast<void>(scope);
  output_action_ = protocol::OutputAction::ZeroTorque;
}

void ExecutionFsm::LeaveZeroTorque(TickScope &scope) {
  static_cast<void>(scope);
}

hardware::TorqueOutput ExecutionFsm::StepZeroTorque(TickScope &scope) {
  static_cast<void>(scope);
  output_action_ = protocol::OutputAction::ZeroTorque;
  return ZeroTorqueOutput();
}

protocol::RejectReason ExecutionFsm::ZeroTorqueAllows(
    protocol::Operation operation) const {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      return protocol::RejectReason::None;
    case protocol::Operation::Damping:
      return protocol::RejectReason::None;
    case protocol::Operation::ZeroTorque:
      // Idempotent stop: the watermark still advances.
      return protocol::RejectReason::None;
    case protocol::Operation::Tracking:
      // Motion authorization must be requested from Damping.
      return protocol::RejectReason::InvalidTransition;
    case protocol::Operation::Reset:
      // Reset only leaves a latched fault.
      return protocol::RejectReason::InvalidTransition;
  }
  return protocol::RejectReason::InvalidTransition;
}

}  // namespace irmv::runtime
