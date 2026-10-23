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

bool IsValid(FaultCondition value) {
  switch (value) {
    case FaultCondition::None:
    case FaultCondition::Unspecified:
    case FaultCondition::JointDisconnected:
    case FaultCondition::JointFeedbackInvalid:
    case FaultCondition::JointFeedbackNonFinite:
    case FaultCondition::JointFeedbackStale:
    case FaultCondition::JointFrequencyLow:
    case FaultCondition::JointDriverError:
    case FaultCondition::JointPositionExceeded:
    case FaultCondition::JointVelocityExceeded:
    case FaultCondition::ImuFeedbackInvalid:
    case FaultCondition::ImuFeedbackNonFinite:
    case FaultCondition::ImuFeedbackStale:
    case FaultCondition::FeedbackShapeInvalid:
    case FaultCondition::BootFailed:
    case FaultCondition::CommandInvalid:
    case FaultCondition::ProtocolExhausted:
    case FaultCondition::PrimaryWriteFailed:
    case FaultCondition::FallbackWriteFailed:
      return true;
  }
  return false;
}

const char *FaultConditionName(FaultCondition value) {
  switch (value) {
    case FaultCondition::None:
      return "None";
    case FaultCondition::Unspecified:
      return "Unspecified";
    case FaultCondition::JointDisconnected:
      return "JointDisconnected";
    case FaultCondition::JointFeedbackInvalid:
      return "JointFeedbackInvalid";
    case FaultCondition::JointFeedbackNonFinite:
      return "JointFeedbackNonFinite";
    case FaultCondition::JointFeedbackStale:
      return "JointFeedbackStale";
    case FaultCondition::JointFrequencyLow:
      return "JointFrequencyLow";
    case FaultCondition::JointDriverError:
      return "JointDriverError";
    case FaultCondition::JointPositionExceeded:
      return "JointPositionExceeded";
    case FaultCondition::JointVelocityExceeded:
      return "JointVelocityExceeded";
    case FaultCondition::ImuFeedbackInvalid:
      return "ImuFeedbackInvalid";
    case FaultCondition::ImuFeedbackNonFinite:
      return "ImuFeedbackNonFinite";
    case FaultCondition::ImuFeedbackStale:
      return "ImuFeedbackStale";
    case FaultCondition::FeedbackShapeInvalid:
      return "FeedbackShapeInvalid";
    case FaultCondition::BootFailed:
      return "BootFailed";
    case FaultCondition::CommandInvalid:
      return "CommandInvalid";
    case FaultCondition::ProtocolExhausted:
      return "ProtocolExhausted";
    case FaultCondition::PrimaryWriteFailed:
      return "PrimaryWriteFailed";
    case FaultCondition::FallbackWriteFailed:
      return "FallbackWriteFailed";
  }
  return "Unknown";
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
