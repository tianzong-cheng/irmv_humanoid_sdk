#include "irmv/protocol/validation.hpp"

#include <cmath>

namespace irmv::protocol {

bool IsValid(ExecutionState value) {
  switch (value) {
    case ExecutionState::Boot:
    case ExecutionState::ZeroTorque:
    case ExecutionState::Damping:
    case ExecutionState::Tracking:
    case ExecutionState::Fault:
      return true;
  }
  return false;
}

bool IsValid(OutputAction value) {
  switch (value) {
    case OutputAction::ZeroTorque:
    case OutputAction::Damping:
    case OutputAction::Tracking:
      return true;
  }
  return false;
}

bool IsValid(Operation value) {
  switch (value) {
    case Operation::AcquireSession:
    case Operation::Damping:
    case Operation::ZeroTorque:
    case Operation::Tracking:
    case Operation::Reset:
      return true;
  }
  return false;
}

bool IsValid(ResultStatus value) {
  switch (value) {
    case ResultStatus::Applied:
    case ResultStatus::Rejected:
      return true;
  }
  return false;
}

bool IsValid(RejectReason value) {
  switch (value) {
    case RejectReason::None:
    case RejectReason::VersionMismatch:
    case RejectReason::Malformed:
    case RejectReason::WrongInstance:
    case RejectReason::WrongSession:
    case RejectReason::SessionBusy:
    case RejectReason::StaleRequest:
    case RejectReason::RequestConflict:
    case RejectReason::EpochMismatch:
    case RejectReason::InvalidTransition:
    case RejectReason::InvalidCommand:
    case RejectReason::StaleSource:
    case RejectReason::Discontinuity:
    case RejectReason::RecoveryNotReady:
    case RejectReason::FaultMismatch:
    case RejectReason::WriteFailed:
    case RejectReason::QueueFull:
      return true;
  }
  return false;
}

bool IsValid(FaultReason value) {
  switch (value) {
    case FaultReason::None:
    case FaultReason::BootFailed:
    case FaultReason::FeedbackInvalid:
    case FaultReason::DriverError:
    case FaultReason::ImuInvalid:
    case FaultReason::JointPositionExceeded:
    case FaultReason::JointVelocityExceeded:
    case FaultReason::CommandInvalid:
    case FaultReason::WriteFailed:
    case FaultReason::ProtocolExhausted:
      return true;
  }
  return false;
}

bool IsValid(ProducerStatus value) {
  switch (value) {
    case ProducerStatus::Ready:
    case ProducerStatus::Failed:
      return true;
  }
  return false;
}

bool IsValid(ProducerFailureReason value) {
  switch (value) {
    case ProducerFailureReason::None:
    case ProducerFailureReason::Busy:
    case ProducerFailureReason::Incompatible:
    case ProducerFailureReason::PrepareFailed:
    case ProducerFailureReason::Internal:
      return true;
  }
  return false;
}

bool IsValid(StopReason value) {
  switch (value) {
    case StopReason::None:
    case StopReason::Requested:
    case StopReason::CommandTimeout:
    case StopReason::SourceStateTimeout:
    case StopReason::SupervisorTimeout:
      return true;
  }
  return false;
}

ValidationResult ValidateCommandShape(const JointCommand &command,
                                      std::size_t expected_count) {
  if (command.protocol_version != kProtocolVersion) {
    return {false, RejectReason::VersionMismatch};
  }
  if (command.joints.count != expected_count || expected_count == 0 ||
      expected_count > kMaxJoints) {
    return {false, RejectReason::Malformed};
  }
  if (command.layout_id.count > kMaxNameBytes) {
    return {false, RejectReason::Malformed};
  }
  for (std::size_t index = 0; index < expected_count; ++index) {
    if (!std::isfinite(command.joints.q[index]) ||
        !std::isfinite(command.joints.dq[index]) ||
        !std::isfinite(command.joints.tau_ff[index]) ||
        !std::isfinite(command.joints.kp[index]) ||
        !std::isfinite(command.joints.kd[index]) ||
        command.joints.kp[index] < 0.0) {
      return {false, RejectReason::InvalidCommand};
    }
    if (command.joints.kd[index] < 0.0) {
      return {false, RejectReason::InvalidCommand};
    }
  }
  return {true, RejectReason::None};
}

}  // namespace irmv::protocol
