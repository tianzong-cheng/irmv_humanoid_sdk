#include "irmv/transport/codec.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "irmv/protocol/validation.hpp"

namespace irmv::transport::codec {
namespace {

using protocol::kMaxJoints;
using protocol::kMaxNameBytes;
using protocol::kMaxReasonBytes;

// Wire counts are unsigned and can be anything; the core arrays are fixed. A
// count above the capacity is clamped on the way out (the core value is already
// validated) and rejected on the way in.
std::size_t ClampCount(std::uint32_t count, std::size_t capacity) {
  return count > capacity ? capacity : count;
}

// ---- outbound -------------------------------------------------------------

template <typename WireText, typename ProtocolText>
void ToWireText(const ProtocolText &source, WireText &target) {
  const std::size_t capacity = target.bytes().size();
  const std::size_t count = ClampCount(source.count, capacity);
  target.count(static_cast<std::uint32_t>(count));
  target.bytes().fill(0);
  std::copy_n(source.bytes.begin(), count, target.bytes().begin());
}

void ToWireVector(const protocol::JointVector &source,
                  protocol::wire::JointVector &target) {
  const std::size_t count = ClampCount(source.count, kMaxJoints);
  target.count(static_cast<std::uint32_t>(count));
  std::copy_n(source.q.begin(), count, target.q().begin());
  std::copy_n(source.dq.begin(), count, target.dq().begin());
  std::copy_n(source.tau_ff.begin(), count, target.tau_ff().begin());
  std::copy_n(source.kp.begin(), count, target.kp().begin());
  std::copy_n(source.kd.begin(), count, target.kd().begin());
}

void ToWireFeedback(const protocol::JointFeedback &source,
                    protocol::wire::JointFeedback &target) {
  target.valid(source.valid);
  target.connected(source.connected);
  target.q(source.q);
  target.dq(source.dq);
  target.tau_est(source.tau_est);
  target.rx_stamp_us(source.rx_stamp_us);
  target.freq_hz(source.freq_hz);
  target.error_code(source.error_code);
}

void ToWireImu(const protocol::ImuFeedback &source,
               protocol::wire::ImuFeedback &target) {
  target.valid(source.valid);
  target.orientation_rpy(source.orientation_rpy);
  target.angular_velocity(source.angular_velocity);
  target.linear_acceleration(source.linear_acceleration);
  target.rx_stamp_us(source.rx_stamp_us);
}

void ToWireSnapshot(const protocol::FeedbackSnapshot &source,
                    protocol::wire::FeedbackSnapshot &target) {
  const std::size_t count = ClampCount(source.count, kMaxJoints);
  target.snapshot_seq(source.snapshot_seq);
  target.stamp_us(source.stamp_us);
  target.count(static_cast<std::uint32_t>(count));
  for (std::size_t index = 0; index < count; ++index) {
    ToWireFeedback(source.joints[index], target.joints()[index]);
  }
  ToWireImu(source.imu, target.imu());
}

void ToWireFaultDetail(const protocol::FaultDetail &source,
                       protocol::wire::FaultDetail &target) {
  target.reason(static_cast<protocol::wire::FaultReason>(
      static_cast<std::uint32_t>(source.reason)));
  target.joint_index(source.joint_index);
  target.error_code(source.error_code);
  target.observed_value(source.observed_value);
  target.limit(source.limit);
  target.sample_stamp_us(source.sample_stamp_us);
  target.detected_stamp_us(source.detected_stamp_us);
  target.output_action(static_cast<protocol::wire::OutputAction>(
      static_cast<std::uint32_t>(source.output_action)));
  ToWireText(source.message, target.message());
}

void ToWireFaults(const protocol::FaultReport &source,
                  protocol::wire::FaultReport &target) {
  const std::size_t count = ClampCount(source.count, kMaxJoints);
  target.count(static_cast<std::uint32_t>(count));
  for (std::size_t index = 0; index < count; ++index) {
    ToWireFaultDetail(source.details[index], target.details()[index]);
  }
}

// ---- inbound --------------------------------------------------------------

bool ReadText(const protocol::wire::Text64 &source, protocol::Text64 &target) {
  if (source.count() > kMaxNameBytes) {
    return false;
  }
  return target.Assign(std::string_view(source.bytes().data(), source.count()));
}

bool ReadText(const protocol::wire::Text256 &source, protocol::Text256 &target) {
  if (source.count() > kMaxReasonBytes) {
    return false;
  }
  return target.Assign(std::string_view(source.bytes().data(), source.count()));
}

template <typename CoreEnum, typename WireEnum>
bool ReadEnum(WireEnum value, CoreEnum &target) {
  const CoreEnum candidate =
      static_cast<CoreEnum>(static_cast<std::uint32_t>(value));
  if (!protocol::IsValid(candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

bool ReadVector(const protocol::wire::JointVector &source,
                protocol::JointVector &target) {
  const std::uint32_t count = source.count();
  // A command has to name its joints: ValidateCommandShape accepts only the
  // configured, non-zero joint count.
  if (count < 1 || count > kMaxJoints) {
    return false;
  }
  target.count = count;
  std::copy_n(source.q().begin(), count, target.q.begin());
  std::copy_n(source.dq().begin(), count, target.dq.begin());
  std::copy_n(source.tau_ff().begin(), count, target.tau_ff.begin());
  std::copy_n(source.kp().begin(), count, target.kp.begin());
  std::copy_n(source.kd().begin(), count, target.kd.begin());
  return true;
}

void ReadFeedback(const protocol::wire::JointFeedback &source,
                  protocol::JointFeedback &target) {
  target.valid = source.valid();
  target.connected = source.connected();
  target.q = source.q();
  target.dq = source.dq();
  target.tau_est = source.tau_est();
  target.rx_stamp_us = source.rx_stamp_us();
  target.freq_hz = source.freq_hz();
  // A 32 bit error code is copied as it is, never narrowed.
  target.error_code = source.error_code();
}

void ReadImu(const protocol::wire::ImuFeedback &source,
             protocol::ImuFeedback &target) {
  target.valid = source.valid();
  target.orientation_rpy = source.orientation_rpy();
  target.angular_velocity = source.angular_velocity();
  target.linear_acceleration = source.linear_acceleration();
  target.rx_stamp_us = source.rx_stamp_us();
}

bool ReadSnapshot(const protocol::wire::FeedbackSnapshot &source,
                  protocol::FeedbackSnapshot &target) {
  // An empty snapshot is representable; the core classifies a count that does
  // not match the configuration as invalid feedback.
  if (source.count() > kMaxJoints) {
    return false;
  }
  target.snapshot_seq = source.snapshot_seq();
  target.stamp_us = source.stamp_us();
  target.count = source.count();
  for (std::uint32_t index = 0; index < source.count(); ++index) {
    ReadFeedback(source.joints()[index], target.joints[index]);
  }
  ReadImu(source.imu(), target.imu);
  return true;
}

bool ReadFaultDetail(const protocol::wire::FaultDetail &source,
                     protocol::FaultDetail &target) {
  if (!ReadEnum(source.reason(), target.reason)) {
    return false;
  }
  if (!ReadEnum(source.output_action(), target.output_action)) {
    return false;
  }
  if (!ReadText(source.message(), target.message)) {
    return false;
  }
  target.joint_index = source.joint_index();
  target.error_code = source.error_code();
  target.observed_value = source.observed_value();
  target.limit = source.limit();
  target.sample_stamp_us = source.sample_stamp_us();
  target.detected_stamp_us = source.detected_stamp_us();
  return true;
}

bool ReadFaults(const protocol::wire::FaultReport &source,
                protocol::FaultReport &target) {
  // A healthy state carries no faults; the core stops appending at capacity.
  if (source.count() > kMaxJoints) {
    return false;
  }
  target.count = source.count();
  for (std::uint32_t index = 0; index < source.count(); ++index) {
    if (!ReadFaultDetail(source.details()[index], target.details[index])) {
      return false;
    }
  }
  return true;
}

bool ReadCommand(const protocol::wire::JointCommand &source,
                 protocol::JointCommand &target) {
  target.protocol_version = source.protocol_version();
  target.runtime_id = source.runtime_id();
  target.session_id = source.session_id();
  target.producer_id = source.producer_id();
  target.epoch = source.epoch();
  target.command_seq = source.command_seq();
  target.source_state_seq = source.source_state_seq();
  target.stamp_us = source.stamp_us();
  if (!ReadText(source.layout_id(), target.layout_id)) {
    return false;
  }
  return ReadVector(source.joints(), target.joints);
}

bool ReadRequest(const protocol::wire::ExecutionRequest &source,
                 protocol::ExecutionRequest &target) {
  target.protocol_version = source.protocol_version();
  target.supervisor_id = source.supervisor_id();
  target.request_id = source.request_id();
  target.session_id = source.session_id();
  target.runtime_id = source.runtime_id();
  target.expected_epoch = source.expected_epoch();
  target.proposed_epoch = source.proposed_epoch();
  target.producer_id = source.producer_id();
  target.fault_id = source.fault_id();
  if (!ReadEnum(source.operation(), target.operation)) {
    return false;
  }
  return ReadCommand(source.first_command(), target.first_command);
}

bool ReadResult(const protocol::wire::ExecutionResult &source,
                protocol::ExecutionResult &target) {
  target.protocol_version = source.protocol_version();
  target.supervisor_id = source.supervisor_id();
  target.request_id = source.request_id();
  target.session_id = source.session_id();
  target.runtime_id = source.runtime_id();
  if (!ReadEnum(source.execution_state(), target.execution_state)) {
    return false;
  }
  if (!ReadEnum(source.output_action(), target.output_action)) {
    return false;
  }
  if (!ReadEnum(source.status(), target.status)) {
    return false;
  }
  if (!ReadEnum(source.reason(), target.reason)) {
    return false;
  }
  target.producer_id = source.producer_id();
  target.epoch = source.epoch();
  target.fault_id = source.fault_id();
  target.stamp_us = source.stamp_us();
  return true;
}

bool ReadHeartbeat(const protocol::wire::SupervisorHeartbeat &source,
                   protocol::SupervisorHeartbeat &target) {
  target.protocol_version = source.protocol_version();
  target.supervisor_id = source.supervisor_id();
  target.session_id = source.session_id();
  target.runtime_id = source.runtime_id();
  target.heartbeat_seq = source.heartbeat_seq();
  target.stamp_us = source.stamp_us();
  return true;
}

bool ReadState(const protocol::wire::RobotState &source,
               protocol::RobotState &target) {
  target.protocol_version = source.protocol_version();
  target.runtime_id = source.runtime_id();
  target.session_id = source.session_id();
  target.producer_id = source.producer_id();
  target.epoch = source.epoch();
  target.state_seq = source.state_seq();
  target.stamp_us = source.stamp_us();
  if (!ReadEnum(source.execution_state(), target.execution_state)) {
    return false;
  }
  if (!ReadEnum(source.output_action(), target.output_action)) {
    return false;
  }
  if (!ReadEnum(source.stop_reason(), target.stop_reason)) {
    return false;
  }
  target.fault_id = source.fault_id();
  target.last_applied_command_seq = source.last_applied_command_seq();
  target.healthy = source.healthy();
  target.damping_eligible = source.damping_eligible();
  if (!ReadText(source.config_id(), target.config_id)) {
    return false;
  }
  if (!ReadText(source.layout_id(), target.layout_id)) {
    return false;
  }
  if (!ReadSnapshot(source.feedback(), target.feedback)) {
    return false;
  }
  return ReadFaults(source.faults(), target.faults);
}

// ---- endpoint policies ----------------------------------------------------

template <typename Qos>
void ApplyTopicQos(Qos &target, const protocol::TopicQos &source) {
  const std::uint32_t depth = source.history_depth == 0 ? 1 : source.history_depth;
  target << dds::core::policy::Durability::Volatile()
         << dds::core::policy::History::KeepLast(depth)
         << dds::core::policy::ResourceLimits(static_cast<int32_t>(depth), 1,
                                             static_cast<int32_t>(depth));
  if (source.reliability == protocol::Reliability::Reliable) {
    target << dds::core::policy::Reliability::Reliable(
        dds::core::Duration::from_microsecs(
            static_cast<int64_t>(source.max_blocking_time_us)));
  } else {
    // The frozen descriptors leave max_blocking_time at zero for best effort
    // channels; the policy default applies because a zero blocking time is not
    // a meaningful request.
    target << dds::core::policy::Reliability::BestEffort();
  }
}

}  // namespace

void ToWire(const protocol::JointCommand &source,
            protocol::wire::JointCommand &target) {
  target.protocol_version(source.protocol_version);
  target.runtime_id(source.runtime_id);
  target.session_id(source.session_id);
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.command_seq(source.command_seq);
  target.source_state_seq(source.source_state_seq);
  target.stamp_us(source.stamp_us);
  ToWireText(source.layout_id, target.layout_id());
  ToWireVector(source.joints, target.joints());
}

void ToWire(const protocol::ExecutionRequest &source,
            protocol::wire::ExecutionRequest &target) {
  target.protocol_version(source.protocol_version);
  target.supervisor_id(source.supervisor_id);
  target.request_id(source.request_id);
  target.session_id(source.session_id);
  target.runtime_id(source.runtime_id);
  target.expected_epoch(source.expected_epoch);
  target.proposed_epoch(source.proposed_epoch);
  target.producer_id(source.producer_id);
  target.fault_id(source.fault_id);
  target.operation(static_cast<protocol::wire::Operation>(
      static_cast<std::uint32_t>(source.operation)));
  ToWire(source.first_command, target.first_command());
}

void ToWire(const protocol::ExecutionResult &source,
            protocol::wire::ExecutionResult &target) {
  target.protocol_version(source.protocol_version);
  target.supervisor_id(source.supervisor_id);
  target.request_id(source.request_id);
  target.session_id(source.session_id);
  target.runtime_id(source.runtime_id);
  target.execution_state(static_cast<protocol::wire::ExecutionState>(
      static_cast<std::uint32_t>(source.execution_state)));
  target.output_action(static_cast<protocol::wire::OutputAction>(
      static_cast<std::uint32_t>(source.output_action)));
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.fault_id(source.fault_id);
  target.stamp_us(source.stamp_us);
  target.status(static_cast<protocol::wire::ResultStatus>(
      static_cast<std::uint32_t>(source.status)));
  target.reason(static_cast<protocol::wire::RejectReason>(
      static_cast<std::uint32_t>(source.reason)));
}

void ToWire(const protocol::SupervisorHeartbeat &source,
            protocol::wire::SupervisorHeartbeat &target) {
  target.protocol_version(source.protocol_version);
  target.supervisor_id(source.supervisor_id);
  target.session_id(source.session_id);
  target.runtime_id(source.runtime_id);
  target.heartbeat_seq(source.heartbeat_seq);
  target.stamp_us(source.stamp_us);
}

void ToWire(const protocol::RobotState &source,
            protocol::wire::RobotState &target) {
  target.protocol_version(source.protocol_version);
  target.runtime_id(source.runtime_id);
  target.session_id(source.session_id);
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.state_seq(source.state_seq);
  target.stamp_us(source.stamp_us);
  target.execution_state(static_cast<protocol::wire::ExecutionState>(
      static_cast<std::uint32_t>(source.execution_state)));
  target.output_action(static_cast<protocol::wire::OutputAction>(
      static_cast<std::uint32_t>(source.output_action)));
  target.stop_reason(static_cast<protocol::wire::StopReason>(
      static_cast<std::uint32_t>(source.stop_reason)));
  target.fault_id(source.fault_id);
  target.last_applied_command_seq(source.last_applied_command_seq);
  target.healthy(source.healthy);
  target.damping_eligible(source.damping_eligible);
  ToWireText(source.config_id, target.config_id());
  ToWireText(source.layout_id, target.layout_id());
  ToWireSnapshot(source.feedback, target.feedback());
  ToWireFaults(source.faults, target.faults());
}

bool FromWire(const protocol::wire::JointCommand &source,
              protocol::JointCommand &target) {
  protocol::JointCommand candidate;
  if (!ReadCommand(source, candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::ExecutionRequest &source,
              protocol::ExecutionRequest &target) {
  protocol::ExecutionRequest candidate;
  if (!ReadRequest(source, candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::ExecutionResult &source,
              protocol::ExecutionResult &target) {
  protocol::ExecutionResult candidate;
  if (!ReadResult(source, candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::SupervisorHeartbeat &source,
              protocol::SupervisorHeartbeat &target) {
  protocol::SupervisorHeartbeat candidate;
  if (!ReadHeartbeat(source, candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::RobotState &source,
              protocol::RobotState &target) {
  protocol::RobotState candidate;
  if (!ReadState(source, candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

dds::pub::qos::DataWriterQos ToWriterQos(const protocol::TopicQos &qos) {
  dds::pub::qos::DataWriterQos target;
  ApplyTopicQos(target, qos);
  return target;
}

dds::sub::qos::DataReaderQos ToReaderQos(const protocol::TopicQos &qos) {
  dds::sub::qos::DataReaderQos target;
  ApplyTopicQos(target, qos);
  return target;
}

}  // namespace irmv::transport::codec
