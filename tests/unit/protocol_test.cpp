#include <limits>
#include <string>

#include "irmv/protocol/validation.hpp"
#include "support/check.hpp"

using irmv::protocol::ExecutionState;
using irmv::protocol::FaultReason;
using irmv::protocol::IsValid;
using irmv::protocol::JointCommand;
using irmv::protocol::kMaxJoints;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::Operation;
using irmv::protocol::OutputAction;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::StopReason;
using irmv::protocol::ValidateCommandShape;

int main() {
  JointCommand command{};
  command.joints.count = 1;
  command.joints.kp[0] = -1.0;
  CHECK(!ValidateCommandShape(command, 1).ok);

  command.joints.kp[0] = 0.0;
  command.joints.tau_ff[0] = 3.0;
  CHECK(ValidateCommandShape(command, 1).ok);
  CHECK(!ValidateCommandShape(command, 2).ok);

  command.protocol_version = kProtocolVersion + 1;
  CHECK(!ValidateCommandShape(command, 1).ok);
  CHECK(ValidateCommandShape(command, 1).reason ==
        RejectReason::VersionMismatch);
  command.protocol_version = kProtocolVersion;

  command.joints.q[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(!ValidateCommandShape(command, 1).ok);
  command.joints.q[0] = 0.0;

  command.joints.dq[0] = std::numeric_limits<double>::infinity();
  CHECK(!ValidateCommandShape(command, 1).ok);
  command.joints.dq[0] = 0.0;

  command.joints.tau_ff[0] = -std::numeric_limits<double>::infinity();
  CHECK(!ValidateCommandShape(command, 1).ok);
  command.joints.tau_ff[0] = 3.0;

  command.joints.kd[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(!ValidateCommandShape(command, 1).ok);
  command.joints.kd[0] = -1.0;
  CHECK(!ValidateCommandShape(command, 1).ok);
  CHECK(ValidateCommandShape(command, 1).reason ==
        RejectReason::InvalidCommand);
  command.joints.kd[0] = 0.0;

  command.joints.count = 0;
  CHECK(!ValidateCommandShape(command, 0).ok);
  CHECK(ValidateCommandShape(command, 0).reason == RejectReason::Malformed);
  command.joints.count = kMaxJoints + 1;
  CHECK(!ValidateCommandShape(command, kMaxJoints + 1).ok);
  command.joints.count = 1;

  command.joints.q[1] = std::numeric_limits<double>::quiet_NaN();
  CHECK(ValidateCommandShape(command, 1).ok);

  CHECK(command.layout_id.Assign(
      std::string(irmv::protocol::kMaxNameBytes, 'x')));
  CHECK(command.layout_id.View().size() == irmv::protocol::kMaxNameBytes);
  CHECK(!command.layout_id.Assign(
      std::string(irmv::protocol::kMaxNameBytes + 1, 'x')));
  CHECK(command.layout_id.View().size() == irmv::protocol::kMaxNameBytes);
  command.layout_id.count = irmv::protocol::kMaxNameBytes + 1;
  CHECK(!ValidateCommandShape(command, 1).ok);
  CHECK(ValidateCommandShape(command, 1).reason == RejectReason::Malformed);

  CHECK(IsValid(ExecutionState::Boot));
  CHECK(IsValid(ExecutionState::Fault));
  CHECK(!IsValid(static_cast<ExecutionState>(99)));
  CHECK(IsValid(OutputAction::ZeroTorque));
  CHECK(IsValid(OutputAction::Tracking));
  CHECK(!IsValid(static_cast<OutputAction>(99)));
  CHECK(IsValid(Operation::AcquireSession));
  CHECK(IsValid(Operation::Reset));
  CHECK(!IsValid(static_cast<Operation>(99)));
  CHECK(IsValid(ResultStatus::Applied));
  CHECK(IsValid(ResultStatus::Rejected));
  CHECK(!IsValid(static_cast<ResultStatus>(99)));
  CHECK(IsValid(RejectReason::None));
  CHECK(IsValid(RejectReason::QueueFull));
  CHECK(!IsValid(static_cast<RejectReason>(99)));
  CHECK(IsValid(FaultReason::None));
  CHECK(IsValid(FaultReason::ProtocolExhausted));
  CHECK(!IsValid(static_cast<FaultReason>(99)));
  CHECK(IsValid(StopReason::None));
  CHECK(IsValid(StopReason::SupervisorTimeout));
  CHECK(!IsValid(static_cast<StopReason>(99)));
  return 0;
}
