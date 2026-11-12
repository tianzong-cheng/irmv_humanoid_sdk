#pragma once
#include "irmv/simulation/protocol.hpp"
namespace irmv::simulation {
inline constexpr std::size_t kPhaseCacheCapacity = 32;
// Owned by one control thread. Close binds declaration order, including empty.
struct PhaseManifest {
  StepIdentity identity{};
  std::uint64_t physics_dt_us{1000}, control_dt_us{2000};
  Role sender{Role::Runtime}, consumer{Role::Application};
  Disposition required{Disposition::Ingress};
  std::array<MessageIdentity, kMaxPhaseManifestEntries> messages{};
  std::uint32_t count{0};
  PhaseSeal seal{};
  bool closed{false};
  bool Add(const MessageIdentity&) noexcept;
  bool Close() noexcept;
  // Receiver assembly: reject mixed headers/senders and incomplete/wrong seals.
  bool Add(const Frame&) noexcept;
  bool Close(const Frame&) noexcept;
  bool Valid() const noexcept;
};
struct PhaseReceipt {
  StepIdentity identity{};
  std::uint64_t physics_dt_us{1000}, control_dt_us{2000};
  Role consumer{Role::Application};
  Disposition disposition{Disposition::Ingress};
  MessageIdentity message{};
};
bool SameIdentity(const MessageIdentity&, const MessageIdentity&) noexcept;
bool SamePhase(const StepIdentity&, const StepIdentity&) noexcept;
// Full canonical IDL content; no allocation, padding or inactive tails.
bool Identify(const protocol::ExecutionRequest&, MessageIdentity*) noexcept;
bool Identify(const protocol::ExecutionResult&, MessageIdentity*) noexcept;
bool Identify(const protocol::RobotState&, MessageIdentity*) noexcept;
bool Identify(const protocol::SupervisorHeartbeat&, MessageIdentity*) noexcept;
bool Identify(const protocol::JointCommand&, MessageIdentity*) noexcept;

class Barrier {
 public:
  explicit Barrier(Role local) noexcept : local_(local) {}
  bool Begin(const PhaseManifest&) noexcept;
  bool Observe(const PhaseReceipt&) noexcept;
  bool Complete(const PhaseManifest&) const noexcept;
  // Local consumer only; creates an ACK after every required receipt exists.
  bool MakeAck(const PhaseManifest&, Frame*) noexcept;
  // authenticated_peer is obtained from endpoint role, never ACK self-report.
  bool ObserveAck(const Frame&, Role authenticated_peer) noexcept;

 private:
  struct Phase {
    PhaseManifest manifest{};
    std::array<bool, kMaxPhaseManifestEntries> ingress{}, consumed{};
    bool used{false}, failed{false}, acknowledged{false};
  };
  Role local_;
  std::array<Phase, kPhaseCacheCapacity> phases_{};
  std::size_t next_{0};
};
}  // namespace irmv::simulation
