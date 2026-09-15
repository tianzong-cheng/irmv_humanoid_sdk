#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace irmv::protocol {

inline constexpr std::uint32_t kProtocolVersion = 1;
inline constexpr std::size_t kMaxJoints = 64;
inline constexpr std::size_t kIdSize = 16;
inline constexpr std::size_t kMaxNameBytes = 64;
inline constexpr std::size_t kMaxReasonBytes = 256;
inline constexpr std::uint32_t kNoJointIndex =
    std::numeric_limits<std::uint32_t>::max();

using Id = std::array<std::uint8_t, kIdSize>;
using TimeUs = std::uint64_t;
using Sequence = std::uint64_t;
inline constexpr Id kNoId{};

template <std::size_t Capacity>
struct BoundedText {
  std::uint32_t count{0};
  std::array<char, Capacity> bytes{};

  bool Assign(std::string_view value) {
    if (value.size() > Capacity) {
      return false;
    }
    std::copy(value.begin(), value.end(), bytes.begin());
    count = static_cast<std::uint32_t>(value.size());
    return true;
  }

  std::string_view View() const {
    return {bytes.data(), std::min<std::size_t>(count, Capacity)};
  }
};

using Text64 = BoundedText<kMaxNameBytes>;
using Text256 = BoundedText<kMaxReasonBytes>;

enum class Reliability : std::uint32_t {
  BestEffort = 0,
  Reliable = 1,
};

enum class Durability : std::uint32_t {
  Volatile = 0,
};

struct TopicQos {
  Reliability reliability{Reliability::BestEffort};
  Durability durability{Durability::Volatile};
  std::uint32_t history_depth{1};
  TimeUs max_blocking_time_us{0};
};

struct TopicDescriptor {
  std::string_view name;
  TopicQos qos;
};

inline constexpr TopicDescriptor kExecutionRequestTopic{
    "irmv/v1/execution_request",
    {Reliability::Reliable, Durability::Volatile, 32, 10'000}};
inline constexpr TopicDescriptor kExecutionResultTopic{
    "irmv/v1/execution_result",
    {Reliability::Reliable, Durability::Volatile, 32, 10'000}};
inline constexpr TopicDescriptor kJointCommandTopic{
    "irmv/v1/joint_command",
    {Reliability::BestEffort, Durability::Volatile, 1, 0}};
inline constexpr TopicDescriptor kSupervisorHeartbeatTopic{
    "irmv/v1/supervisor_heartbeat",
    {Reliability::BestEffort, Durability::Volatile, 8, 0}};
inline constexpr TopicDescriptor kRobotStateTopic{
    "irmv/v1/robot_state",
    {Reliability::BestEffort, Durability::Volatile, 1, 0}};

enum class ExecutionState : std::uint32_t {
  Boot = 0,
  ZeroTorque = 1,
  Damping = 2,
  Tracking = 3,
  Fault = 4,
};

enum class OutputAction : std::uint32_t {
  ZeroTorque = 0,
  Damping = 1,
  Tracking = 2,
};

enum class Operation : std::uint32_t {
  AcquireSession = 0,
  Damping = 1,
  ZeroTorque = 2,
  Tracking = 3,
  Reset = 4,
};

enum class ResultStatus : std::uint32_t {
  Applied = 0,
  Rejected = 1,
};

enum class RejectReason : std::uint32_t {
  None = 0,
  VersionMismatch = 1,
  Malformed = 2,
  WrongInstance = 3,
  WrongSession = 4,
  SessionBusy = 5,
  StaleRequest = 6,
  RequestConflict = 7,
  EpochMismatch = 8,
  InvalidTransition = 9,
  InvalidCommand = 10,
  StaleSource = 11,
  Discontinuity = 12,
  RecoveryNotReady = 13,
  FaultMismatch = 14,
  WriteFailed = 15,
  QueueFull = 16,
};

enum class FaultReason : std::uint32_t {
  None = 0,
  BootFailed = 1,
  FeedbackInvalid = 2,
  DriverError = 3,
  ImuInvalid = 4,
  JointPositionExceeded = 5,
  JointVelocityExceeded = 6,
  CommandInvalid = 7,
  WriteFailed = 8,
  ProtocolExhausted = 9,
};

enum class StopReason : std::uint32_t {
  None = 0,
  Requested = 1,
  CommandTimeout = 2,
  SourceStateTimeout = 3,
  SupervisorTimeout = 4,
};

struct JointVector {
  std::uint32_t count{0};
  std::array<double, kMaxJoints> q{};
  std::array<double, kMaxJoints> dq{};
  std::array<double, kMaxJoints> tau_ff{};
  std::array<double, kMaxJoints> kp{};
  std::array<double, kMaxJoints> kd{};
};

struct JointCommand {
  std::uint32_t protocol_version{kProtocolVersion};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  Sequence epoch{0};
  Sequence command_seq{0};
  Sequence source_state_seq{0};
  TimeUs stamp_us{0};
  Text64 layout_id;
  JointVector joints;
};

struct JointFeedback {
  bool valid{false};
  bool connected{false};
  double q{0.0};
  double dq{0.0};
  double tau_est{0.0};
  TimeUs rx_stamp_us{0};
  double freq_hz{0.0};
  std::uint32_t error_code{0};
};

struct ImuFeedback {
  bool valid{false};
  std::array<double, 3> orientation_rpy{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 3> linear_acceleration{};
  TimeUs rx_stamp_us{0};
};

struct FeedbackSnapshot {
  Sequence snapshot_seq{0};
  TimeUs stamp_us{0};
  std::uint32_t count{0};
  std::array<JointFeedback, kMaxJoints> joints{};
  ImuFeedback imu;
};

struct FaultDetail {
  FaultReason reason{FaultReason::None};
  std::uint32_t joint_index{kNoJointIndex};
  std::uint32_t error_code{0};
  double observed_value{0.0};
  double limit{0.0};
  TimeUs sample_stamp_us{0};
  TimeUs detected_stamp_us{0};
  OutputAction output_action{OutputAction::ZeroTorque};
  Text256 message;
};

struct FaultReport {
  std::uint32_t count{0};
  std::array<FaultDetail, kMaxJoints> details{};
};

struct ExecutionRequest {
  std::uint32_t protocol_version{kProtocolVersion};
  Id supervisor_id{};
  Sequence request_id{0};
  Id session_id{};
  Id runtime_id{};
  Sequence expected_epoch{0};
  Sequence proposed_epoch{0};
  Id producer_id{};
  Sequence fault_id{0};
  Operation operation{Operation::AcquireSession};
  JointCommand first_command;
};

struct ExecutionResult {
  std::uint32_t protocol_version{kProtocolVersion};
  Id supervisor_id{};
  Sequence request_id{0};
  Id session_id{};
  Id runtime_id{};
  ExecutionState execution_state{ExecutionState::Boot};
  OutputAction output_action{OutputAction::ZeroTorque};
  Id producer_id{};
  Sequence epoch{0};
  Sequence fault_id{0};
  TimeUs stamp_us{0};
  ResultStatus status{ResultStatus::Rejected};
  RejectReason reason{RejectReason::None};
};

struct SupervisorHeartbeat {
  std::uint32_t protocol_version{kProtocolVersion};
  Id supervisor_id{};
  Id session_id{};
  Id runtime_id{};
  Sequence heartbeat_seq{0};
  TimeUs stamp_us{0};
};

struct RobotState {
  std::uint32_t protocol_version{kProtocolVersion};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  Sequence epoch{0};
  Sequence state_seq{0};
  TimeUs stamp_us{0};
  ExecutionState execution_state{ExecutionState::Boot};
  OutputAction output_action{OutputAction::ZeroTorque};
  StopReason stop_reason{StopReason::None};
  Sequence fault_id{0};
  Sequence last_applied_command_seq{0};
  bool healthy{false};
  bool damping_eligible{false};
  Text64 config_id;
  Text64 layout_id;
  FeedbackSnapshot feedback;
  FaultReport faults;
};

}  // namespace irmv::protocol
