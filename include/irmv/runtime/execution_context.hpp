#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "irmv/hardware/backend.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/command_gate.hpp"

namespace irmv::runtime {

// Bounded input of exactly one control tick. Every collection is fixed
// capacity: the transport layer drops what does not fit and counts it, so the
// control core never allocates. Stop requests travel in their own lane so that
// ordinary request traffic cannot crowd out a stop.
inline constexpr std::size_t kRequestBatchCapacity = 32;
inline constexpr std::size_t kStopRequestBatchCapacity = 8;
inline constexpr std::size_t kCommandBatchCapacity = 64;
inline constexpr std::size_t kHeartbeatBatchCapacity = 8;

// Bounded output of exactly one control tick. Results and events beyond the
// capacity are dropped and counted; the counters are cumulative over the life
// of the state machine.
inline constexpr std::size_t kResultCapacity = 32;
inline constexpr std::size_t kEventCapacity = 64;

struct TickInput {
  std::array<Received<protocol::ExecutionRequest>, kRequestBatchCapacity>
      requests{};
  std::size_t request_count{0};
  std::array<Received<protocol::ExecutionRequest>, kStopRequestBatchCapacity>
      stop_requests{};
  std::size_t stop_request_count{0};
  std::array<Received<protocol::JointCommand>, kCommandBatchCapacity> commands{};
  std::size_t command_count{0};
  std::array<Received<protocol::SupervisorHeartbeat>, kHeartbeatBatchCapacity>
      heartbeats{};
  std::size_t heartbeat_count{0};
};

// Structural event of one tick: transitions, latches, timeouts, rejections and
// notable actions.
enum class EventCode : std::uint32_t {
  None = 0,
  BootCompleted = 1,
  BootFailed = 2,
  StateChanged = 3,
  SessionAcquired = 4,
  SessionExpired = 5,
  AuthorizationGranted = 6,
  AuthorizationRevoked = 7,
  StopApplied = 8,
  CommandTimeout = 9,
  SourceStateTimeout = 10,
  SupervisorTimeout = 11,
  FaultLatched = 12,
  FaultOutputDegraded = 13,
  RecoveryReady = 14,
  ResetApplied = 15,
  RequestRejected = 16,
  ProtocolExhausted = 17,
};

struct RuntimeEvent {
  protocol::TimeUs stamp_us{0};
  protocol::Sequence tick{0};
  EventCode code{EventCode::None};
  protocol::ExecutionState from{protocol::ExecutionState::Boot};
  protocol::ExecutionState to{protocol::ExecutionState::Boot};
  protocol::RejectReason reject_reason{protocol::RejectReason::None};
  protocol::FaultReason fault_reason{protocol::FaultReason::None};
  protocol::StopReason stop_reason{protocol::StopReason::None};
  protocol::Id producer_id{};
  protocol::Sequence epoch{0};
  protocol::Sequence request_id{0};
};

struct TickResult {
  protocol::RobotState state;
  std::uint32_t result_count{0};
  std::array<protocol::ExecutionResult, kResultCapacity> results{};
  std::uint32_t event_count{0};
  std::array<RuntimeEvent, kEventCapacity> events{};
  hardware::TorqueOutput output;
  bool output_valid{false};
  hardware::WriteResult write_result;
  std::uint64_t dropped_result_count{0};
  std::uint64_t dropped_event_count{0};
};

}  // namespace irmv::runtime
