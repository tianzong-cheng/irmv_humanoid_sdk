#pragma once
#include <array>
#include <variant>

#include "irmv/simulation/clock.hpp"
namespace irmv::simulation {
inline constexpr std::uint16_t kLocalProtocolVersion = 1;
inline constexpr std::size_t kMaxPacketBytes = 65536;
inline constexpr std::size_t kFrameQueueCapacity = 32;
inline constexpr std::size_t kMaxJoints = 64;
inline constexpr std::size_t kMaxPhaseManifestEntries = 64;
using Digest = std::array<std::uint8_t, 32>;
using A3 = std::array<double, 3>;
using A4 = std::array<double, 4>;
using A6 = std::array<double, 6>;
struct Joints {
  std::uint32_t count{0};
  std::array<double, kMaxJoints> values{};
};
enum class Role : std::uint32_t {
  Coordinator = 1,
  Runtime = 2,
  Application = 3
};
enum class MessageKind : std::uint32_t {
  Request = 1,
  Result = 2,
  State = 3,
  Heartbeat = 4,
  Command = 5
};
enum class Disposition : std::uint32_t { Ingress = 1, BusinessConsumed = 2 };
enum class FrameType : std::uint16_t {
  Hello = 1,
  Feedback = 2,
  Torque = 3,
  Manifest = 4,
  Ack = 5,
  ReleaseArm = 6,
  LeaveReady = 7,
  LeaveCommitted = 8,
  Abort = 9,
  AppliedToStep = 10,
  PhaseSeal = 11,
  PhaseAck = 12,
  InventoryReady = 13,
  PublishPermit = 14,
  TickPermit = 15,
  FirstCommandProof = 16
};
// Exact DDS projection plus digest of all sample content; see protocol docs.
struct MessageIdentity {
  MessageKind kind{MessageKind::Request};
  protocol::Id runtime_id{};
  protocol::Id supervisor_id{};
  protocol::Id session_id{};
  protocol::Id producer_id{};
  std::uint64_t request_id{};
  std::uint64_t epoch{};
  std::uint64_t proposed_epoch{};
  std::uint64_t sequence{};
  std::uint64_t source_state_seq{};
  std::uint64_t fault_id{};
  std::uint64_t stamp_us{};
  std::uint32_t operation{};
  std::uint32_t status{};
  std::uint32_t reason{};
  std::uint32_t execution_state{};
  std::uint32_t output_action{};
  Digest content_digest{};
};
struct Hello {
  Role role{Role::Coordinator};
  Digest config_digest{};
  Digest application_digest{};
  Digest engine_digest{};
  protocol::Id runtime_id{};
  std::uint64_t physics_dt_us{1000};
  std::uint64_t control_dt_us{2000};
};
struct Feedback {
  std::uint64_t physics_step{};
  double engine_time_s{};
  Joints q{};
  Joints dq{};
  Joints tau{};
  A4 imu_quaternion{};
  A3 imu_angular_velocity{};
  A3 imu_specific_force{};
  A3 base_position{};
  A6 base_velocity{};
  bool valid{};
  bool imu_valid{};
};
struct Torque {
  std::uint64_t write_id{};
  bool accepted{};
  std::uint64_t state_seq{};
  MessageIdentity command{};
  Joints tau{};
};
// One sample per manifest. This does not assert phase completion.
struct Manifest {
  Role sender{Role::Coordinator};
  MessageIdentity message{};
};
// Only the actual receiving participant may attest this disposition.
struct Ack {
  Role consumer{Role::Coordinator};
  Disposition disposition{Disposition::Ingress};
  Manifest manifest{};
};
struct ReleaseArm {
  MessageIdentity tracking_request{};
  MessageIdentity first_command{};
  MessageIdentity applied_result{};
  MessageIdentity tracking_state{};
};
struct LeaveReady {
  MessageIdentity stop_request{};
  MessageIdentity stop_result{};
  MessageIdentity stopped_state{};
};
struct LeaveCommitted {
  LeaveReady proof{};
  bool authorization_revoked{};
  bool session_closed{};
  bool heartbeat_resolved{};
};
struct Abort {
  std::uint32_t code{};
  protocol::Text256 reason{};
};
// Engine fact, distinct from local Write acceptance and DDS Applied.
struct AppliedToStep {
  std::uint64_t write_id{};
  std::uint64_t physics_step_begin{};
  std::uint64_t physics_step_end{};
};
// Explicit inventory closure, including count=0. Header identifies the phase.
struct PhaseSeal {
  Role sender{Role::Coordinator};
  std::uint32_t manifest_count{0};
  Digest manifest_digest{};
};
// Echo the seal under its original run/step/phase header, never host arrival.
struct PhaseAck {
  Role consumer{Role::Coordinator};
  Disposition disposition{Disposition::Ingress};
  PhaseSeal seal{};
};
// Installation is separate from business completion (PhaseAck).
struct InventoryReady {
  Role consumer{Role::Coordinator};
  PhaseSeal seal{};
};
struct PublishPermit {
  PhaseSeal seal{};
};
// stage 1: runtime normal tick; stage 2: application output computation.
struct TickPermit {
  Role role{Role::Runtime};
  std::uint32_t stage{1};
};
// Auxiliary evidence for a nested command, not another DDS sample/receipt.
struct FirstCommandProof {
  MessageIdentity request{};
  MessageIdentity command{};
};
using Payload =
    std::variant<Hello, Feedback, Torque, Manifest, Ack, ReleaseArm, LeaveReady,
                 LeaveCommitted, Abort, AppliedToStep, PhaseSeal, PhaseAck,
                 InventoryReady, PublishPermit, TickPermit, FirstCommandProof>;
struct Frame {
  StepIdentity identity{};
  std::uint64_t physics_dt_us{1000};
  std::uint64_t control_dt_us{2000};
  Payload payload{Hello{}};
};
// Startup/run consistency is separate from stateless structural decoding.
bool MatchesHello(const Hello&, const Hello&) noexcept;
}  // namespace irmv::simulation
