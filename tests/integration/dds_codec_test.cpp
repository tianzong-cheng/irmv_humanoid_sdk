#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <dds/dds.hpp>

#include "execution.hpp"
#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/execution_context.hpp"
#include "irmv/transport/codec.hpp"
#include "irmv/transport/dds_transport.hpp"
#include "support/check.hpp"

namespace {

namespace wire = irmv::protocol::wire;
namespace codec = irmv::transport::codec;

using irmv::configuration::LoadConfig;
using irmv::configuration::RuntimeConfig;
using irmv::protocol::ExecutionRequest;
using irmv::protocol::ExecutionResult;
using irmv::protocol::ExecutionState;
using irmv::protocol::FaultDetail;
using irmv::protocol::FaultReason;
using irmv::protocol::FaultReport;
using irmv::protocol::FeedbackSnapshot;
using irmv::protocol::Id;
using irmv::protocol::ImuFeedback;
using irmv::protocol::JointCommand;
using irmv::protocol::JointFeedback;
using irmv::protocol::JointVector;
using irmv::protocol::kMaxJoints;
using irmv::protocol::kMaxNameBytes;
using irmv::protocol::kMaxReasonBytes;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::Operation;
using irmv::protocol::OutputAction;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::StopReason;
using irmv::protocol::SupervisorHeartbeat;
using irmv::protocol::TimeUs;
using irmv::protocol::TopicDescriptor;
using irmv::runtime::TickInput;
using irmv::runtime::TickResult;
using irmv::transport::DdsTransport;

constexpr const char *kFixtureConfigPath = "config/sdk.json";

// Domains stay below 230: the default RTPS port mapping overflows above that.
constexpr std::uint32_t kDefaultDomain = 42;
constexpr std::uint32_t kDomainAttempts = 4;

constexpr Sequence kAcquireRequestId = 0x1122334455ULL;
constexpr Sequence kStopRequestId = 0x6655443322ULL;
constexpr Sequence kCommandSeq = 0xABCDEF0123456789ULL;
constexpr Sequence kResultRequestId = 0x99AA99AA99ULL;
constexpr Sequence kInvalidRequestId = 0xDEADBEEFULL;
constexpr Sequence kStateSeq = 4242;
constexpr Sequence kFreshStateSeq = 4243;
constexpr Sequence kFlushStateSeq = 4244;

// Deliberately not a plausible host monotonic value, so a receive stamp copied
// from the sender would be visible as such.
constexpr TimeUs kSenderStampUs = 1'000'000'000'000'000ULL;

// The 32 bit error code from the plan: the high bit must survive the wire.
constexpr std::uint32_t kHighBitErrorCode = 0x80000000u;

TimeUs NowUs() {
  return static_cast<TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::string FullText(std::size_t size, char fill) {
  return std::string(size, fill);
}

Id MakeId(std::uint8_t seed) {
  Id id{};
  id.fill(seed);
  return id;
}

// Alternates a push and a bounded check until the check succeeds or the
// deadline expires. Every step of every scenario in this file is bounded this
// way, so a missing peer fails the test instead of hanging it.
template <typename Push, typename Check>
bool RepeatUntil(Push push, Check check, TimeUs timeout_us) {
  const TimeUs deadline = NowUs() + timeout_us;
  for (;;) {
    push();
    if (check()) {
      return true;
    }
    if (NowUs() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// ---- expected payloads ----------------------------------------------------

JointCommand MakeCommand() {
  JointCommand command;
  command.protocol_version = kProtocolVersion;
  command.runtime_id = MakeId(0x11);
  command.session_id = MakeId(0x22);
  command.producer_id = MakeId(0x33);
  command.epoch = 7;
  command.command_seq = kCommandSeq;
  command.source_state_seq = 21;
  command.stamp_us = kSenderStampUs;
  command.layout_id.Assign(FullText(kMaxNameBytes, 'L'));
  command.joints.count = static_cast<std::uint32_t>(kMaxJoints);
  for (std::size_t index = 0; index < kMaxJoints; ++index) {
    const double step = static_cast<double>(index);
    command.joints.q[index] = 0.25 * step;
    command.joints.dq[index] = -0.5 * step;
    command.joints.tau_ff[index] = 100.0 + step;
    command.joints.kp[index] = 50.0 + step;
    command.joints.kd[index] = 1.0 + 0.5 * step;
  }
  return command;
}

ExecutionRequest MakeRequest(Operation operation, Sequence request_id) {
  ExecutionRequest request;
  request.protocol_version = kProtocolVersion;
  request.supervisor_id = MakeId(0x55);
  request.request_id = request_id;
  request.session_id = MakeId(0x22);
  request.runtime_id = MakeId(0x11);
  request.expected_epoch = 6;
  request.proposed_epoch = 7;
  request.producer_id = MakeId(0x33);
  request.fault_id = 0;
  request.operation = operation;
  request.first_command = MakeCommand();
  return request;
}

ExecutionResult MakeResult(ExecutionState state, OutputAction output_action,
                           ResultStatus status, RejectReason reason) {
  ExecutionResult result;
  result.protocol_version = kProtocolVersion;
  result.supervisor_id = MakeId(0x55);
  result.request_id = kResultRequestId;
  result.session_id = MakeId(0x22);
  result.runtime_id = MakeId(0x11);
  result.execution_state = state;
  result.output_action = output_action;
  result.producer_id = MakeId(0x33);
  result.epoch = 7;
  result.fault_id = 3;
  result.stamp_us = 1'700'000;
  result.status = status;
  result.reason = reason;
  return result;
}

SupervisorHeartbeat MakeHeartbeat() {
  SupervisorHeartbeat heartbeat;
  heartbeat.protocol_version = kProtocolVersion;
  heartbeat.supervisor_id = MakeId(0x55);
  heartbeat.session_id = MakeId(0x22);
  heartbeat.runtime_id = MakeId(0x11);
  heartbeat.heartbeat_seq = 99;
  heartbeat.stamp_us = kSenderStampUs;
  return heartbeat;
}

RobotState MakeState(Sequence state_seq) {
  RobotState state;
  state.protocol_version = kProtocolVersion;
  state.runtime_id = MakeId(0x11);
  state.session_id = MakeId(0x22);
  state.producer_id = MakeId(0x33);
  state.epoch = 7;
  state.state_seq = state_seq;
  state.stamp_us = 1'700'001;
  state.execution_state = ExecutionState::Tracking;
  state.output_action = OutputAction::Tracking;
  state.stop_reason = StopReason::None;
  state.fault_id = 0;
  state.last_applied_command_seq = kCommandSeq;
  state.healthy = true;
  state.damping_eligible = false;
  state.config_id.Assign("config-1");
  state.layout_id.Assign(FullText(kMaxNameBytes, 'L'));

  state.feedback.snapshot_seq = 99;
  state.feedback.stamp_us = 1'700'000;
  state.feedback.count = static_cast<std::uint32_t>(kMaxJoints);
  for (std::size_t index = 0; index < kMaxJoints; ++index) {
    JointFeedback &joint = state.feedback.joints[index];
    joint.valid = true;
    joint.connected = true;
    joint.q = 0.5 + static_cast<double>(index);
    joint.dq = -0.125 * static_cast<double>(index);
    joint.tau_est = 3.0 + static_cast<double>(index);
    joint.rx_stamp_us = 1'699'000 + index;
    joint.freq_hz = 500.0;
    joint.error_code = index == 0 ? kHighBitErrorCode
                                  : static_cast<std::uint32_t>(index);
  }
  state.feedback.imu.valid = true;
  state.feedback.imu.orientation_rpy = {0.1, -0.2, 0.3};
  state.feedback.imu.angular_velocity = {0.4, -0.5, 0.6};
  state.feedback.imu.linear_acceleration = {0.7, -0.8, 0.9};
  state.feedback.imu.rx_stamp_us = 1'699'500;

  state.faults.count = 1;
  FaultDetail &detail = state.faults.details[0];
  detail.reason = FaultReason::JointVelocityExceeded;
  detail.joint_index = 3;
  detail.error_code = kHighBitErrorCode;
  detail.observed_value = 9.5;
  detail.limit = 4.0;
  detail.sample_stamp_us = 1'698'000;
  detail.detected_stamp_us = 1'698'500;
  detail.output_action = OutputAction::Damping;
  detail.message.Assign(FullText(kMaxReasonBytes, 'F'));
  return state;
}

// ---- field-by-field comparison of the core types --------------------------

template <std::size_t Capacity>
bool SameText(const irmv::protocol::BoundedText<Capacity> &left,
              const irmv::protocol::BoundedText<Capacity> &right) {
  return left.count == right.count && left.View() == right.View();
}

bool Same(const JointVector &left, const JointVector &right) {
  return left.count == right.count && left.q == right.q && left.dq == right.dq &&
         left.tau_ff == right.tau_ff && left.kp == right.kp &&
         left.kd == right.kd;
}

bool Same(const JointCommand &left, const JointCommand &right) {
  return left.protocol_version == right.protocol_version &&
         left.runtime_id == right.runtime_id &&
         left.session_id == right.session_id &&
         left.producer_id == right.producer_id && left.epoch == right.epoch &&
         left.command_seq == right.command_seq &&
         left.source_state_seq == right.source_state_seq &&
         left.stamp_us == right.stamp_us &&
         SameText(left.layout_id, right.layout_id) &&
         Same(left.joints, right.joints);
}

bool Same(const ExecutionRequest &left, const ExecutionRequest &right) {
  return left.protocol_version == right.protocol_version &&
         left.supervisor_id == right.supervisor_id &&
         left.request_id == right.request_id &&
         left.session_id == right.session_id &&
         left.runtime_id == right.runtime_id &&
         left.expected_epoch == right.expected_epoch &&
         left.proposed_epoch == right.proposed_epoch &&
         left.producer_id == right.producer_id &&
         left.fault_id == right.fault_id && left.operation == right.operation &&
         Same(left.first_command, right.first_command);
}

bool Same(const ExecutionResult &left, const ExecutionResult &right) {
  return left.protocol_version == right.protocol_version &&
         left.supervisor_id == right.supervisor_id &&
         left.request_id == right.request_id &&
         left.session_id == right.session_id &&
         left.runtime_id == right.runtime_id &&
         left.execution_state == right.execution_state &&
         left.output_action == right.output_action &&
         left.producer_id == right.producer_id && left.epoch == right.epoch &&
         left.fault_id == right.fault_id && left.stamp_us == right.stamp_us &&
         left.status == right.status && left.reason == right.reason;
}

bool Same(const SupervisorHeartbeat &left, const SupervisorHeartbeat &right) {
  return left.protocol_version == right.protocol_version &&
         left.supervisor_id == right.supervisor_id &&
         left.session_id == right.session_id &&
         left.runtime_id == right.runtime_id &&
         left.heartbeat_seq == right.heartbeat_seq &&
         left.stamp_us == right.stamp_us;
}

bool Same(const JointFeedback &left, const JointFeedback &right) {
  return left.valid == right.valid && left.connected == right.connected &&
         left.q == right.q && left.dq == right.dq &&
         left.tau_est == right.tau_est &&
         left.rx_stamp_us == right.rx_stamp_us &&
         left.freq_hz == right.freq_hz &&
         left.error_code == right.error_code;
}

bool Same(const ImuFeedback &left, const ImuFeedback &right) {
  return left.valid == right.valid &&
         left.orientation_rpy == right.orientation_rpy &&
         left.angular_velocity == right.angular_velocity &&
         left.linear_acceleration == right.linear_acceleration &&
         left.rx_stamp_us == right.rx_stamp_us;
}

bool Same(const FeedbackSnapshot &left, const FeedbackSnapshot &right) {
  if (left.snapshot_seq != right.snapshot_seq ||
      left.stamp_us != right.stamp_us || left.count != right.count ||
      !Same(left.imu, right.imu)) {
    return false;
  }
  for (std::size_t index = 0; index < left.count; ++index) {
    if (!Same(left.joints[index], right.joints[index])) {
      return false;
    }
  }
  return true;
}

bool Same(const FaultDetail &left, const FaultDetail &right) {
  return left.reason == right.reason && left.joint_index == right.joint_index &&
         left.error_code == right.error_code &&
         left.observed_value == right.observed_value &&
         left.limit == right.limit &&
         left.sample_stamp_us == right.sample_stamp_us &&
         left.detected_stamp_us == right.detected_stamp_us &&
         left.output_action == right.output_action &&
         SameText(left.message, right.message);
}

bool Same(const FaultReport &left, const FaultReport &right) {
  if (left.count != right.count) {
    return false;
  }
  for (std::size_t index = 0; index < left.count; ++index) {
    if (!Same(left.details[index], right.details[index])) {
      return false;
    }
  }
  return true;
}

bool Same(const RobotState &left, const RobotState &right) {
  return left.protocol_version == right.protocol_version &&
         left.runtime_id == right.runtime_id &&
         left.session_id == right.session_id &&
         left.producer_id == right.producer_id && left.epoch == right.epoch &&
         left.state_seq == right.state_seq && left.stamp_us == right.stamp_us &&
         left.execution_state == right.execution_state &&
         left.output_action == right.output_action &&
         left.stop_reason == right.stop_reason &&
         left.fault_id == right.fault_id &&
         left.last_applied_command_seq == right.last_applied_command_seq &&
         left.healthy == right.healthy &&
         left.damping_eligible == right.damping_eligible &&
         SameText(left.config_id, right.config_id) &&
         SameText(left.layout_id, right.layout_id) &&
         Same(left.feedback, right.feedback) && Same(left.faults, right.faults);
}

// ---- codec round trips ----------------------------------------------------

bool RoundTripSamples() {
  const JointCommand command = MakeCommand();
  wire::JointCommand wire_command;
  codec::ToWire(command, wire_command);
  if (wire_command.joints().count() != kMaxJoints ||
      wire_command.layout_id().count() != kMaxNameBytes ||
      wire_command.joints().q()[kMaxJoints - 1] != command.joints.q[kMaxJoints - 1]) {
    std::cerr << "codec: JointCommand wire fields do not match the source\n";
    return false;
  }
  JointCommand restored_command;
  if (!codec::FromWire(wire_command, restored_command) ||
      !Same(command, restored_command)) {
    std::cerr << "codec: JointCommand did not round trip\n";
    return false;
  }

  const ExecutionRequest request =
      MakeRequest(Operation::AcquireSession, kAcquireRequestId);
  wire::ExecutionRequest wire_request;
  codec::ToWire(request, wire_request);
  ExecutionRequest restored_request;
  if (!codec::FromWire(wire_request, restored_request) ||
      !Same(request, restored_request)) {
    std::cerr << "codec: ExecutionRequest did not round trip\n";
    return false;
  }

  const ExecutionResult result = MakeResult(
      ExecutionState::Tracking, OutputAction::Tracking, ResultStatus::Applied,
      RejectReason::None);
  wire::ExecutionResult wire_result;
  codec::ToWire(result, wire_result);
  ExecutionResult restored_result;
  if (!codec::FromWire(wire_result, restored_result) ||
      !Same(result, restored_result)) {
    std::cerr << "codec: ExecutionResult did not round trip\n";
    return false;
  }

  const SupervisorHeartbeat heartbeat = MakeHeartbeat();
  wire::SupervisorHeartbeat wire_heartbeat;
  codec::ToWire(heartbeat, wire_heartbeat);
  SupervisorHeartbeat restored_heartbeat;
  if (!codec::FromWire(wire_heartbeat, restored_heartbeat) ||
      !Same(heartbeat, restored_heartbeat)) {
    std::cerr << "codec: SupervisorHeartbeat did not round trip\n";
    return false;
  }

  const RobotState state = MakeState(kStateSeq);
  wire::RobotState wire_state;
  codec::ToWire(state, wire_state);
  RobotState restored_state;
  if (!codec::FromWire(wire_state, restored_state) || !Same(state, restored_state)) {
    std::cerr << "codec: RobotState did not round trip\n";
    return false;
  }
  if (restored_state.feedback.count != kMaxJoints ||
      restored_state.feedback.joints[0].error_code != kHighBitErrorCode ||
      restored_state.faults.count != 1 ||
      restored_state.faults.details[0].error_code != kHighBitErrorCode ||
      restored_state.faults.details[0].message.count != kMaxReasonBytes ||
      restored_state.layout_id.count != kMaxNameBytes) {
    std::cerr << "codec: boundary values did not survive the RobotState round trip\n";
    return false;
  }

  // Every enum value of every frozen message survives both directions.
  const Operation operations[] = {
      Operation::AcquireSession, Operation::Damping, Operation::ZeroTorque,
      Operation::Tracking, Operation::Reset};
  for (const Operation operation : operations) {
    const ExecutionRequest expected = MakeRequest(operation, kAcquireRequestId);
    wire::ExecutionRequest wire;
    codec::ToWire(expected, wire);
    ExecutionRequest restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: operation " << static_cast<std::uint32_t>(operation)
                << " did not round trip\n";
      return false;
    }
  }

  const ExecutionState states[] = {
      ExecutionState::Boot, ExecutionState::ZeroTorque, ExecutionState::Damping,
      ExecutionState::Tracking, ExecutionState::Fault};
  for (const ExecutionState value : states) {
    const ExecutionResult expected = MakeResult(
        value, OutputAction::Damping, ResultStatus::Rejected,
        RejectReason::QueueFull);
    wire::ExecutionResult wire;
    codec::ToWire(expected, wire);
    ExecutionResult restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: execution state "
                << static_cast<std::uint32_t>(value) << " did not round trip\n";
      return false;
    }
  }

  const OutputAction actions[] = {OutputAction::ZeroTorque,
                                  OutputAction::Damping,
                                  OutputAction::Tracking};
  for (const OutputAction value : actions) {
    const ExecutionResult expected = MakeResult(ExecutionState::Fault, value,
                                               ResultStatus::Applied,
                                               RejectReason::None);
    wire::ExecutionResult wire;
    codec::ToWire(expected, wire);
    ExecutionResult restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: output action " << static_cast<std::uint32_t>(value)
                << " did not round trip\n";
      return false;
    }
  }

  const ResultStatus statuses[] = {ResultStatus::Applied, ResultStatus::Rejected};
  for (const ResultStatus value : statuses) {
    const ExecutionResult expected =
        MakeResult(ExecutionState::Fault, OutputAction::ZeroTorque, value,
                   RejectReason::FaultMismatch);
    wire::ExecutionResult wire;
    codec::ToWire(expected, wire);
    ExecutionResult restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: result status " << static_cast<std::uint32_t>(value)
                << " did not round trip\n";
      return false;
    }
  }

  const RejectReason reasons[] = {
      RejectReason::None,          RejectReason::VersionMismatch,
      RejectReason::Malformed,     RejectReason::WrongInstance,
      RejectReason::WrongSession,  RejectReason::SessionBusy,
      RejectReason::StaleRequest,  RejectReason::RequestConflict,
      RejectReason::EpochMismatch, RejectReason::InvalidTransition,
      RejectReason::InvalidCommand, RejectReason::StaleSource,
      RejectReason::Discontinuity, RejectReason::RecoveryNotReady,
      RejectReason::FaultMismatch, RejectReason::WriteFailed,
      RejectReason::QueueFull};
  for (const RejectReason value : reasons) {
    const ExecutionResult expected = MakeResult(
        ExecutionState::Tracking, OutputAction::ZeroTorque,
        ResultStatus::Rejected, value);
    wire::ExecutionResult wire;
    codec::ToWire(expected, wire);
    ExecutionResult restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: reject reason " << static_cast<std::uint32_t>(value)
                << " did not round trip\n";
      return false;
    }
  }

  const FaultReason fault_reasons[] = {
      FaultReason::None,          FaultReason::BootFailed,
      FaultReason::FeedbackInvalid, FaultReason::DriverError,
      FaultReason::ImuInvalid,    FaultReason::JointPositionExceeded,
      FaultReason::JointVelocityExceeded, FaultReason::CommandInvalid,
      FaultReason::WriteFailed,   FaultReason::ProtocolExhausted};
  for (const FaultReason value : fault_reasons) {
    RobotState expected = MakeState(kStateSeq);
    expected.faults.details[0].reason = value;
    wire::RobotState wire;
    codec::ToWire(expected, wire);
    RobotState restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: fault reason " << static_cast<std::uint32_t>(value)
                << " did not round trip\n";
      return false;
    }
  }

  const StopReason stop_reasons[] = {
      StopReason::None, StopReason::Requested, StopReason::CommandTimeout,
      StopReason::SourceStateTimeout, StopReason::SupervisorTimeout};
  for (const StopReason value : stop_reasons) {
    RobotState expected = MakeState(kStateSeq);
    expected.stop_reason = value;
    wire::RobotState wire;
    codec::ToWire(expected, wire);
    RobotState restored;
    if (!codec::FromWire(wire, restored) || !Same(expected, restored)) {
      std::cerr << "codec: stop reason " << static_cast<std::uint32_t>(value)
                << " did not round trip\n";
      return false;
    }
  }

  // A state with no joints and no faults is representable.
  RobotState empty = MakeState(kStateSeq);
  empty.feedback.count = 0;
  empty.faults.count = 0;
  wire::RobotState wire_empty;
  codec::ToWire(empty, wire_empty);
  RobotState restored_empty;
  if (!codec::FromWire(wire_empty, restored_empty) || !Same(empty, restored_empty)) {
    std::cerr << "codec: empty snapshot/fault report did not round trip\n";
    return false;
  }

  std::cout << "[dds_codec_test] codec round trips: five messages, 64 joints, "
               "64/256 byte text, high-bit error code 0x"
            << std::hex << kHighBitErrorCode << std::dec
            << ", every enum value\n";
  return true;
}

// ---- codec rejection ------------------------------------------------------

bool RejectInvalidSamples() {
  const JointCommand command = MakeCommand();

  // Counts above the fixed wire capacity are rejected, and the destination is
  // left exactly as it was.
  {
    wire::JointCommand wire_command;
    codec::ToWire(command, wire_command);
    wire_command.joints().count(static_cast<std::uint32_t>(kMaxJoints) + 1);
    JointCommand destination = command;
    const JointCommand before = command;
    if (codec::FromWire(wire_command, destination) || !Same(before, destination)) {
      std::cerr << "codec: a joint vector count above kMaxJoints was accepted\n";
      return false;
    }
  }
  {
    wire::JointCommand wire_command;
    codec::ToWire(command, wire_command);
    wire_command.joints().count(0);
    JointCommand destination = command;
    const JointCommand before = command;
    if (codec::FromWire(wire_command, destination) || !Same(before, destination)) {
      std::cerr << "codec: a zero joint count was accepted\n";
      return false;
    }
  }
  {
    const RobotState state = MakeState(kStateSeq);
    wire::RobotState wire_state;
    codec::ToWire(state, wire_state);
    wire_state.feedback().count(static_cast<std::uint32_t>(kMaxJoints) + 1);
    RobotState destination = state;
    const RobotState before = state;
    if (codec::FromWire(wire_state, destination) || !Same(before, destination)) {
      std::cerr << "codec: a feedback count above kMaxJoints was accepted\n";
      return false;
    }
  }
  {
    const RobotState state = MakeState(kStateSeq);
    wire::RobotState wire_state;
    codec::ToWire(state, wire_state);
    wire_state.faults().count(static_cast<std::uint32_t>(kMaxJoints) + 1);
    RobotState destination = state;
    const RobotState before = state;
    if (codec::FromWire(wire_state, destination) || !Same(before, destination)) {
      std::cerr << "codec: a fault count above kMaxJoints was accepted\n";
      return false;
    }
  }

  // Text that claims more bytes than the wire field holds.
  {
    wire::JointCommand wire_command;
    codec::ToWire(command, wire_command);
    wire_command.layout_id().count(static_cast<std::uint32_t>(kMaxNameBytes) + 1);
    JointCommand destination = command;
    const JointCommand before = command;
    if (codec::FromWire(wire_command, destination) || !Same(before, destination)) {
      std::cerr << "codec: an over-long layout id was accepted\n";
      return false;
    }
  }
  {
    const RobotState state = MakeState(kStateSeq);
    wire::RobotState wire_state;
    codec::ToWire(state, wire_state);
    wire_state.faults().details()[0].message().count(
        static_cast<std::uint32_t>(kMaxReasonBytes) + 1);
    RobotState destination = state;
    const RobotState before = state;
    if (codec::FromWire(wire_state, destination) || !Same(before, destination)) {
      std::cerr << "codec: an over-long fault message was accepted\n";
      return false;
    }
  }

  // Unknown enum values are rejected instead of being cast into the core type.
  {
    wire::ExecutionRequest wire_request;
    codec::ToWire(MakeRequest(Operation::Tracking, kAcquireRequestId),
                  wire_request);
    wire_request.operation(static_cast<wire::Operation>(99));
    ExecutionRequest destination = MakeRequest(Operation::Damping, 1);
    const ExecutionRequest before = destination;
    if (codec::FromWire(wire_request, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown operation was accepted\n";
      return false;
    }
  }
  {
    wire::ExecutionResult wire_result;
    codec::ToWire(MakeResult(ExecutionState::Fault, OutputAction::Damping,
                             ResultStatus::Applied, RejectReason::None),
                  wire_result);
    wire_result.execution_state(static_cast<wire::ExecutionState>(99));
    ExecutionResult destination =
        MakeResult(ExecutionState::Boot, OutputAction::ZeroTorque,
                   ResultStatus::Rejected, RejectReason::StaleRequest);
    const ExecutionResult before = destination;
    if (codec::FromWire(wire_result, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown execution state was accepted\n";
      return false;
    }
  }
  {
    wire::ExecutionResult wire_result;
    codec::ToWire(MakeResult(ExecutionState::Fault, OutputAction::Damping,
                             ResultStatus::Applied, RejectReason::None),
                  wire_result);
    wire_result.output_action(static_cast<wire::OutputAction>(99));
    ExecutionResult destination =
        MakeResult(ExecutionState::Boot, OutputAction::ZeroTorque,
                   ResultStatus::Rejected, RejectReason::StaleRequest);
    const ExecutionResult before = destination;
    if (codec::FromWire(wire_result, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown output action was accepted\n";
      return false;
    }
  }
  {
    wire::ExecutionResult wire_result;
    codec::ToWire(MakeResult(ExecutionState::Fault, OutputAction::Damping,
                             ResultStatus::Applied, RejectReason::None),
                  wire_result);
    wire_result.status(static_cast<wire::ResultStatus>(99));
    ExecutionResult destination =
        MakeResult(ExecutionState::Boot, OutputAction::ZeroTorque,
                   ResultStatus::Rejected, RejectReason::StaleRequest);
    const ExecutionResult before = destination;
    if (codec::FromWire(wire_result, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown result status was accepted\n";
      return false;
    }
  }
  {
    wire::ExecutionResult wire_result;
    codec::ToWire(MakeResult(ExecutionState::Fault, OutputAction::Damping,
                             ResultStatus::Applied, RejectReason::None),
                  wire_result);
    wire_result.reason(static_cast<wire::RejectReason>(99));
    ExecutionResult destination =
        MakeResult(ExecutionState::Boot, OutputAction::ZeroTorque,
                   ResultStatus::Rejected, RejectReason::StaleRequest);
    const ExecutionResult before = destination;
    if (codec::FromWire(wire_result, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown reject reason was accepted\n";
      return false;
    }
  }
  {
    wire::RobotState wire_state;
    codec::ToWire(MakeState(kStateSeq), wire_state);
    wire_state.stop_reason(static_cast<wire::StopReason>(99));
    RobotState destination = MakeState(kFreshStateSeq);
    const RobotState before = destination;
    if (codec::FromWire(wire_state, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown stop reason was accepted\n";
      return false;
    }
  }
  {
    wire::RobotState wire_state;
    codec::ToWire(MakeState(kStateSeq), wire_state);
    wire_state.faults().details()[0].reason(static_cast<wire::FaultReason>(99));
    RobotState destination = MakeState(kFreshStateSeq);
    const RobotState before = destination;
    if (codec::FromWire(wire_state, destination) || !Same(before, destination)) {
      std::cerr << "codec: an unknown fault reason was accepted\n";
      return false;
    }
  }

  std::cout << "[dds_codec_test] codec rejections: counts above capacity, zero "
               "joint count, over-long text, unknown enums\n";
  return true;
}

// ---- endpoint QoS ---------------------------------------------------------

template <typename EndpointQos>
bool CheckEndpointQos(const EndpointQos &qos, const TopicDescriptor &descriptor,
                      const char *role, const std::string &name) {
  const auto durability = qos.template policy<dds::core::policy::Durability>();
  const auto reliability = qos.template policy<dds::core::policy::Reliability>();
  const auto history = qos.template policy<dds::core::policy::History>();
  const auto limits = qos.template policy<dds::core::policy::ResourceLimits>();

  const bool reliable = descriptor.qos.reliability ==
                        irmv::protocol::Reliability::Reliable;
  const std::uint32_t depth = descriptor.qos.history_depth;

  std::cout << "[dds_codec_test] qos " << name << ' ' << role
            << " durability="
            << (durability.kind() == dds::core::policy::DurabilityKind::VOLATILE
                    ? "VOLATILE"
                    : "NOT_VOLATILE")
            << " reliability="
            << (reliability.kind() == dds::core::policy::ReliabilityKind::RELIABLE
                    ? "RELIABLE"
                    : "BEST_EFFORT")
            << " blocking_ms=" << reliability.max_blocking_time().to_millisecs()
            << " history="
            << (history.kind() == dds::core::policy::HistoryKind::KEEP_LAST
                    ? "KEEP_LAST"
                    : "NOT_KEEP_LAST")
            << '/' << history.depth() << " limits=" << limits.max_samples() << '/'
            << limits.max_instances() << '/' << limits.max_samples_per_instance()
            << '\n';

  if (durability.kind() != dds::core::policy::DurabilityKind::VOLATILE) {
    std::cerr << "qos: " << name << ' ' << role << " is not VOLATILE\n";
    return false;
  }
  if (reliable != (reliability.kind() ==
                   dds::core::policy::ReliabilityKind::RELIABLE)) {
    std::cerr << "qos: " << name << ' ' << role
              << " has the wrong reliability kind\n";
    return false;
  }
  if (reliable &&
      reliability.max_blocking_time() !=
          dds::core::Duration::from_millisecs(
              static_cast<int64_t>(descriptor.qos.max_blocking_time_us / 1000))) {
    std::cerr << "qos: " << name << ' ' << role
              << " has the wrong max_blocking_time\n";
    return false;
  }
  if (history.kind() != dds::core::policy::HistoryKind::KEEP_LAST ||
      history.depth() != depth) {
    std::cerr << "qos: " << name << ' ' << role
              << " has the wrong history depth\n";
    return false;
  }
  if (limits.max_samples() != depth || limits.max_instances() != 1 ||
      limits.max_samples_per_instance() != depth) {
    std::cerr << "qos: " << name << ' ' << role
              << " has non-finite or wrong resource limits\n";
    return false;
  }
  return true;
}

template <typename T>
bool CheckTopicQos(dds::pub::Publisher &publisher,
                   dds::sub::Subscriber &subscriber,
                   const TopicDescriptor &descriptor) {
  const std::string name(descriptor.name);
  dds::topic::Topic<T> topic(publisher.participant(), name);
  if (topic.name() != name) {
    std::cerr << "qos: topic name " << topic.name() << " differs from " << name
              << '\n';
    return false;
  }
  dds::pub::DataWriter<T> writer(publisher, topic,
                                 codec::ToWriterQos(descriptor.qos));
  dds::sub::DataReader<T> reader(subscriber, topic,
                                 codec::ToReaderQos(descriptor.qos));
  const bool writer_ok =
      CheckEndpointQos(writer.qos(), descriptor, "writer", name);
  const bool reader_ok =
      CheckEndpointQos(reader.qos(), descriptor, "reader", name);
  return writer_ok && reader_ok;
}

bool CheckFrozenQos(dds::pub::Publisher &publisher,
                    dds::sub::Subscriber &subscriber) {
  return CheckTopicQos<wire::ExecutionRequest>(
             publisher, subscriber, irmv::protocol::kExecutionRequestTopic) &&
         CheckTopicQos<wire::ExecutionResult>(
             publisher, subscriber, irmv::protocol::kExecutionResultTopic) &&
         CheckTopicQos<wire::JointCommand>(
             publisher, subscriber, irmv::protocol::kJointCommandTopic) &&
         CheckTopicQos<wire::SupervisorHeartbeat>(
             publisher, subscriber,
             irmv::protocol::kSupervisorHeartbeatTopic) &&
         CheckTopicQos<wire::RobotState>(
             publisher, subscriber, irmv::protocol::kRobotStateTopic);
}

// ---- peers ----------------------------------------------------------------

template <typename T>
struct Sender {
  Sender(dds::pub::Publisher &publisher, const dds::topic::Topic<T> &topic,
         const TopicDescriptor &descriptor)
      : writer(publisher, topic, codec::ToWriterQos(descriptor.qos)) {}

  void Send(const T &sample) { writer.write(sample); }

  dds::pub::DataWriter<T> writer;
};

// Bounded receive helper: the pinned WaitSet throws on timeout, which is the
// normal "no sample yet" path and must not be mistaken for a failure.
template <typename T>
struct Receiver {
  Receiver(dds::sub::Subscriber &subscriber, const dds::topic::Topic<T> &topic,
           const TopicDescriptor &descriptor)
      : reader(subscriber, topic, codec::ToReaderQos(descriptor.qos)),
        condition(reader, dds::sub::status::DataState::any()) {
    waitset += condition;
  }

  bool Take(int timeout_ms, T *out) {
    try {
      if (waitset.wait(dds::core::Duration::from_millisecs(timeout_ms)).empty()) {
        return false;
      }
    } catch (const dds::core::TimeoutError &) {
      return false;
    }
    auto samples = reader.select()
                       .max_samples(8)
                       .state(dds::sub::status::DataState::any())
                       .take();
    for (const auto &sample : samples) {
      if (sample.info().valid()) {
        *out = sample.data();
        return true;
      }
    }
    return false;
  }

  dds::sub::DataReader<T> reader;
  dds::sub::cond::ReadCondition condition;
  dds::core::cond::WaitSet waitset;
};

struct RunningTransport {
  std::uint32_t domain{0};
  std::unique_ptr<RuntimeConfig> config;
  std::unique_ptr<DdsTransport> transport;
};

std::string InterfaceOverride() {
  const char *value = std::getenv("IRMV_TEST_DDS_INTERFACE");
  return value != nullptr ? std::string(value) : std::string();
}

// Starts a transport on the first usable domain. Every candidate and every
// failure is printed, and an explicit IRMV_TEST_DDS_DOMAIN request is used as
// given instead of being replaced by a fallback.
bool StartTransport(RunningTransport *running) {
  std::uint32_t base = kDefaultDomain;
  std::uint32_t attempts = kDomainAttempts;
  if (const char *override_domain = std::getenv("IRMV_TEST_DDS_DOMAIN")) {
    base = static_cast<std::uint32_t>(std::strtoul(override_domain, nullptr, 10));
    attempts = 1;
  }

  for (std::uint32_t attempt = 0; attempt < attempts; ++attempt) {
    const std::uint32_t domain = base + attempt;
    auto config = std::make_unique<RuntimeConfig>(LoadConfig(kFixtureConfigPath));
    config->dds.domain = domain;
    const std::string interface = InterfaceOverride();
    if (!interface.empty()) {
      config->dds.interface = interface;
    }
    auto transport = std::make_unique<DdsTransport>(*config);
    try {
      transport->Start();
    } catch (const std::exception &error) {
      std::cerr << "[dds_codec_test] domain " << domain
                << " is not usable: " << error.what() << '\n';
      continue;
    }
    running->domain = domain;
    running->config = std::move(config);
    running->transport = std::move(transport);
    return true;
  }
  return false;
}

}  // namespace

int main() {
  CHECK(RoundTripSamples());
  CHECK(RejectInvalidSamples());

  // The configured interface must reach the participant: an interface that does
  // not exist is refused, which can only happen when the configuration is
  // applied to the endpoints CycloneDDS actually uses.
  {
    const char *override_domain = std::getenv("IRMV_TEST_DDS_DOMAIN");
    const std::uint32_t base =
        override_domain != nullptr
            ? static_cast<std::uint32_t>(std::strtoul(override_domain, nullptr, 10))
            : kDefaultDomain;
    RuntimeConfig config = LoadConfig(kFixtureConfigPath);
    config.dds.domain = base + kDomainAttempts + 1;
    config.dds.interface = "irmv-no-such-interface0";
    DdsTransport transport(config);
    bool rejected = false;
    std::string message;
    try {
      transport.Start();
    } catch (const std::exception &error) {
      rejected = true;
      message = error.what();
    }
    std::cout << "[dds_codec_test] missing interface "
              << config.dds.interface << " rejected=" << rejected
              << " error=" << message << '\n';
    CHECK(rejected);
    transport.Stop();
    CHECK(!transport.started());
  }

  RunningTransport running;
  CHECK(StartTransport(&running));
  std::cout << "[dds_codec_test] domain=" << running.domain
            << " interface=" << running.config->dds.interface << '\n';
  DdsTransport &transport = *running.transport;
  CHECK(transport.started());

  // A second participant makes this test a real remote peer of the transport.
  // CycloneDDS caches the domain configuration per domain id, so it inherits
  // the interface the transport bound.
  dds::domain::DomainParticipant participant(running.domain);
  dds::pub::Publisher publisher(participant);
  dds::sub::Subscriber subscriber(participant);

  CHECK(CheckFrozenQos(publisher, subscriber));

  dds::topic::Topic<wire::ExecutionRequest> request_topic(
      participant, std::string(irmv::protocol::kExecutionRequestTopic.name));
  dds::topic::Topic<wire::JointCommand> command_topic(
      participant, std::string(irmv::protocol::kJointCommandTopic.name));
  dds::topic::Topic<wire::SupervisorHeartbeat> heartbeat_topic(
      participant, std::string(irmv::protocol::kSupervisorHeartbeatTopic.name));
  dds::topic::Topic<wire::ExecutionResult> result_topic(
      participant, std::string(irmv::protocol::kExecutionResultTopic.name));
  dds::topic::Topic<wire::RobotState> state_topic(
      participant, std::string(irmv::protocol::kRobotStateTopic.name));

  const TimeUs test_start_us = NowUs();

  // (1) execution_request on the ordinary lane.
  {
    Sender<wire::ExecutionRequest> sender(
        publisher, request_topic, irmv::protocol::kExecutionRequestTopic);
    const ExecutionRequest expected =
        MakeRequest(Operation::AcquireSession, kAcquireRequestId);
    wire::ExecutionRequest wire_request;
    codec::ToWire(expected, wire_request);

    bool seen = false;
    bool stop_lane_traffic = false;
    ExecutionRequest received;
    TimeUs rx_us = 0;
    const bool delivered = RepeatUntil(
        [&sender, &wire_request]() { sender.Send(wire_request); },
        [&]() {
          const TickInput &input = transport.TakeInput();
          stop_lane_traffic = stop_lane_traffic || input.stop_request_count > 0;
          for (std::size_t index = 0; index < input.request_count; ++index) {
            if (input.requests[index].value.request_id == kAcquireRequestId) {
              received = input.requests[index].value;
              rx_us = input.requests[index].rx_us;
              seen = true;
            }
          }
          return seen;
        },
        3'000'000);
    CHECK(delivered);
    CHECK(Same(expected, received));
    CHECK(!stop_lane_traffic);
    // The receive stamp is the host monotonic time of the read, not the
    // sender's stamp.
    CHECK(rx_us != wire_request.first_command().stamp_us());
    CHECK(rx_us >= test_start_us);
    CHECK(rx_us <= NowUs() + 100'000);
    std::cout << "[dds_codec_test] execution_request received on the request "
                 "lane, rx_us="
              << rx_us
              << " sender_stamp_us=" << wire_request.first_command().stamp_us()
              << " stop_lane_empty=" << !stop_lane_traffic << '\n';
  }

  // (2) the stop operations use the other lane.
  {
    Sender<wire::ExecutionRequest> sender(
        publisher, request_topic, irmv::protocol::kExecutionRequestTopic);
    const ExecutionRequest expected = MakeRequest(Operation::Reset, kStopRequestId);
    wire::ExecutionRequest wire_request;
    codec::ToWire(expected, wire_request);

    bool seen = false;
    bool ordinary_lane_traffic = false;
    ExecutionRequest received;
    TimeUs rx_us = 0;
    const bool delivered = RepeatUntil(
        [&sender, &wire_request]() { sender.Send(wire_request); },
        [&]() {
          const TickInput &input = transport.TakeInput();
          for (std::size_t index = 0; index < input.request_count; ++index) {
            if (input.requests[index].value.request_id == kStopRequestId) {
              ordinary_lane_traffic = true;
            }
          }
          for (std::size_t index = 0; index < input.stop_request_count; ++index) {
            if (input.stop_requests[index].value.request_id == kStopRequestId) {
              received = input.stop_requests[index].value;
              rx_us = input.stop_requests[index].rx_us;
              seen = true;
            }
          }
          return seen;
        },
        3'000'000);
    CHECK(delivered);
    CHECK(Same(expected, received));
    CHECK(!ordinary_lane_traffic);
    CHECK(rx_us >= test_start_us);
    CHECK(rx_us <= NowUs() + 100'000);
    std::cout << "[dds_codec_test] reset request received on the stop lane, "
                 "ordinary_lane_empty="
              << !ordinary_lane_traffic << '\n';
  }

  // (3) joint_command.
  {
    Sender<wire::JointCommand> sender(publisher, command_topic,
                                      irmv::protocol::kJointCommandTopic);
    const JointCommand expected = MakeCommand();
    wire::JointCommand wire_command;
    codec::ToWire(expected, wire_command);

    bool seen = false;
    JointCommand received;
    TimeUs rx_us = 0;
    const bool delivered = RepeatUntil(
        [&sender, &wire_command]() { sender.Send(wire_command); },
        [&]() {
          const TickInput &input = transport.TakeInput();
          for (std::size_t index = 0; index < input.command_count; ++index) {
            if (input.commands[index].value.command_seq == kCommandSeq) {
              received = input.commands[index].value;
              rx_us = input.commands[index].rx_us;
              seen = true;
            }
          }
          return seen;
        },
        3'000'000);
    CHECK(delivered);
    CHECK(Same(expected, received));
    CHECK(received.joints.count == kMaxJoints);
    CHECK(rx_us != wire_command.stamp_us());
    CHECK(rx_us >= test_start_us);
    std::cout << "[dds_codec_test] joint_command received with "
              << received.joints.count << " joints, rx_us=" << rx_us
              << " sender_stamp_us=" << wire_command.stamp_us() << '\n';
  }

  // (4) supervisor_heartbeat.
  {
    Sender<wire::SupervisorHeartbeat> sender(
        publisher, heartbeat_topic, irmv::protocol::kSupervisorHeartbeatTopic);
    const SupervisorHeartbeat expected = MakeHeartbeat();
    wire::SupervisorHeartbeat wire_heartbeat;
    codec::ToWire(expected, wire_heartbeat);

    bool seen = false;
    SupervisorHeartbeat received;
    TimeUs rx_us = 0;
    const bool delivered = RepeatUntil(
        [&sender, &wire_heartbeat]() { sender.Send(wire_heartbeat); },
        [&]() {
          const TickInput &input = transport.TakeInput();
          for (std::size_t index = 0; index < input.heartbeat_count; ++index) {
            if (input.heartbeats[index].value.heartbeat_seq ==
                expected.heartbeat_seq) {
              received = input.heartbeats[index].value;
              rx_us = input.heartbeats[index].rx_us;
              seen = true;
            }
          }
          return seen;
        },
        3'000'000);
    CHECK(delivered);
    CHECK(Same(expected, received));
    CHECK(rx_us >= test_start_us);
    std::cout << "[dds_codec_test] supervisor_heartbeat received, rx_us=" << rx_us
              << '\n';
  }

  // (5) execution_result published by the transport.
  {
    Receiver<wire::ExecutionResult> receiver(
        subscriber, result_topic, irmv::protocol::kExecutionResultTopic);
    const ExecutionResult expected = MakeResult(
        ExecutionState::Tracking, OutputAction::Tracking, ResultStatus::Applied,
        RejectReason::None);
    TickResult tick;
    tick.result_count = 1;
    tick.results[0] = expected;
    tick.state = MakeState(kStateSeq);

    wire::ExecutionResult wire_result;
    const bool delivered = RepeatUntil(
        [&transport, &tick]() { transport.Publish(tick); },
        [&]() { return receiver.Take(50, &wire_result); }, 3'000'000);
    CHECK(delivered);
    ExecutionResult restored;
    CHECK(codec::FromWire(wire_result, restored));
    CHECK(Same(expected, restored));
    std::cout << "[dds_codec_test] execution_result published and received, "
                 "request_id="
              << restored.request_id << '\n';
  }

  // A state reader that outlives the publishes below: the replay check needs a
  // peer that has already received the flushed sample.
  Receiver<wire::RobotState> state_receiver(subscriber, state_topic,
                                           irmv::protocol::kRobotStateTopic);

  // (6) robot_state published by the transport.
  {
    const RobotState expected = MakeState(kStateSeq);
    TickResult tick;
    tick.state = expected;

    wire::RobotState wire_state;
    const bool delivered = RepeatUntil(
        [&transport, &tick]() { transport.Publish(tick); },
        [&]() { return state_receiver.Take(50, &wire_state); }, 3'000'000);
    CHECK(delivered);
    RobotState restored;
    CHECK(codec::FromWire(wire_state, restored));
    CHECK(Same(expected, restored));
    CHECK(restored.layout_id.count == kMaxNameBytes);
    std::cout << "[dds_codec_test] robot_state published and received, state_seq="
              << restored.state_seq << " joints=" << restored.feedback.count
              << " text_bytes=" << restored.layout_id.count << '\n';
  }

  // (7) VOLATILE has no replay. The outbound path is flushed first: a marker
  // state is published until the reader that was already matched has it, which
  // proves the transport has written every sample it took before the marker and
  // that nothing is left in its state channel. Only a quiet outbound path lets
  // a silent late reader say anything about durability.
  {
    TickResult flush_tick;
    flush_tick.state = MakeState(kFlushStateSeq);
    wire::RobotState flushed;
    bool flushed_seen = false;
    const bool flushed_ok = RepeatUntil(
        [&transport, &flush_tick]() { transport.Publish(flush_tick); },
        [&]() {
          if (!state_receiver.Take(50, &flushed)) {
            return false;
          }
          flushed_seen = flushed.state_seq() == kFlushStateSeq;
          return flushed_seen;
        },
        3'000'000);
    CHECK(flushed_ok);
    CHECK(flushed_seen);

    Receiver<wire::RobotState> late(subscriber, state_topic,
                                    irmv::protocol::kRobotStateTopic);
    wire::RobotState replayed;
    CHECK(!late.Take(400, &replayed));

    TickResult tick;
    tick.state = MakeState(kFreshStateSeq);
    bool fresh_seen = false;
    const bool delivered = RepeatUntil(
        [&transport, &tick]() { transport.Publish(tick); },
        [&]() {
          if (late.Take(50, &replayed)) {
            fresh_seen = replayed.state_seq() == kFreshStateSeq;
          }
          return fresh_seen;
        },
        3'000'000);
    CHECK(delivered);
    CHECK(fresh_seen);
    std::cout << "[dds_codec_test] volatile no-replay after a flushed publish: "
                 "old sample absent, new sample state_seq="
              << replayed.state_seq() << " received\n";
  }

  // (8) an invalid wire sample is rejected whole and counted: it must never
  // reach the request lane and must never be applied partially.
  {
    Sender<wire::ExecutionRequest> sender(
        publisher, request_topic, irmv::protocol::kExecutionRequestTopic);
    wire::ExecutionRequest wire_request;
    codec::ToWire(MakeRequest(Operation::Tracking, kInvalidRequestId),
                  wire_request);
    wire_request.first_command().joints().count(
        static_cast<std::uint32_t>(kMaxJoints) + 1);

    bool applied = false;
    const bool rejected = RepeatUntil(
        [&sender, &wire_request]() { sender.Send(wire_request); },
        [&]() {
          const TickInput &input = transport.TakeInput();
          for (std::size_t index = 0; index < input.request_count; ++index) {
            if (input.requests[index].value.request_id == kInvalidRequestId) {
              applied = true;
            }
          }
          return transport.inbound_rejected_count() > 0;
        },
        3'000'000);
    CHECK(rejected);
    CHECK(!applied);
    std::cout << "[dds_codec_test] invalid request rejected whole, "
                 "inbound_rejected_count="
              << transport.inbound_rejected_count()
              << " applied=" << applied << '\n';
  }

  // Stop has to return promptly and be idempotent.
  {
    const TimeUs before = NowUs();
    transport.Stop();
    const TimeUs elapsed_us = NowUs() - before;
    CHECK(!transport.started());
    CHECK(elapsed_us < 1'000'000);
    transport.Stop();
    CHECK(!transport.started());
    std::cout << "[dds_codec_test] stop returned in " << elapsed_us << " us\n";
  }

  std::cout << "[dds_codec_test] all five topics verified on domain "
            << running.domain << '\n';
  return 0;
}
