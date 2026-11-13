#pragma once
#include <atomic>

#include "irmv/simulation/barrier.hpp"
#include "irmv/transport/bounded_channel.hpp"
namespace irmv::simulation {
// Two owners: Control* methods on the single control thread, Worker* on the
// original DDS owner. The borrowed clock must outlive the transport.
class Ingress {
 public:
  Ingress(Role consumer, const LogicalClock& clock) noexcept
      : consumer_(consumer), clock_(clock), barrier_(consumer) {}
  bool ControlBegin(const PhaseManifest&) noexcept;
  // True only after the DDS worker installed this exact current inventory.
  // Separate from ingress/business ACK; caller gates publication on this.
  bool ControlInstalled(const PhaseManifest&) noexcept;
  bool ControlTakeReceipt(PhaseReceipt*) noexcept;
  bool ControlDequeued(const MessageIdentity&) noexcept;
  bool ControlProcessed(const MessageIdentity&) noexcept;
  bool ControlAck(Frame*) noexcept;
  bool Failed() const noexcept { return failed_.load(); }
  void WorkerUpdate() noexcept;
  // 1=new matching sample, 0=identical resend, -1=undeclared/conflict/stale.
  int WorkerCheck(const MessageIdentity&, protocol::TimeUs*) noexcept;
  void WorkerStored(const MessageIdentity&, bool stored) noexcept;

 private:
  struct Installation {
    StepIdentity identity{};
    std::uint64_t physics_dt_us{0}, control_dt_us{0};
    Role consumer{Role::Coordinator};
    Disposition required{Disposition::Ingress};
    PhaseSeal seal{};
    bool Matches(const PhaseManifest&) const noexcept;
  };
  struct Cached {
    PhaseManifest manifest{};
    std::array<bool, kMaxPhaseManifestEntries> seen{};
    bool used{false};
  };
  Role consumer_;
  const LogicalClock& clock_;
  Barrier barrier_;
  PhaseManifest current_{};
  bool begun_{false};
  bool installation_observed_{false};
  Installation installed_{};  // control-owned, never directly written by worker
  transport::BoundedChannel<Installation, 2> installed_notifications_;
  std::array<bool, kMaxPhaseManifestEntries> dequeued_{};
  transport::BoundedChannel<PhaseManifest, 2> installs_;
  transport::BoundedChannel<PhaseReceipt, kMaxPhaseManifestEntries> receipts_;
  std::array<Cached, kPhaseCacheCapacity> cache_{};
  std::size_t next_{0}, active_{0};
  bool active_valid_{false};
  std::atomic<bool> failed_{false};
};
}  // namespace irmv::simulation
