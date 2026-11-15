#pragma once
#include <functional>

#include "irmv/hardware/sim_backend.hpp"
#include "irmv/simulation/barrier.hpp"
#include "irmv/simulation/bootstrap.hpp"
#include "irmv/transport/client_transport.hpp"
#include "irmv/transport/dds_transport.hpp"
namespace irmv::simulation {
struct StepPermit {
  StepIdentity identity{};
  PhaseManifest required{};
  bool stop_control{false};
};
MessageIdentity InactiveCommand(protocol::Id) noexcept;
// Outer-runner coordination only. Never invoked by a backend or pure FSM.
class DriverSession {
 public:
  DriverSession(EndpointConfig, SimulationClock&, transport::DdsTransport*);
  DriverSession(EndpointConfig, SimulationClock&, transport::ClientTransport*);
  void Start();
  bool Poll() noexcept;
  bool Failed() const noexcept;
  bool Queue(const Frame&) noexcept;
  Frame Make(StepIdentity) const noexcept;
  bool Declare(const PhaseManifest&) noexcept;
  bool Processed(const MessageIdentity&) noexcept;
  void BusinessComplete() noexcept { business_complete_ = true; }
  bool TakeTick(StepPermit*) noexcept;
  bool TakeFeedback(FeedbackFrame*) noexcept;
  const PhaseManifest& Inbound() const noexcept { return inbound_; }
  bool AckQueued() const noexcept { return ack_sent_; }
  bool ShutdownAuthorized() const noexcept { return shutdown_authorized_; }
  bool OwnerStopCommitted() const noexcept {
    return shutdown_authorized_ && stop_committed_.write_id != 0 &&
           stop_committed_.state_seq != 0;
  }
  bool RuntimeOnly() const noexcept { return runtime_only_; }
  bool StopCommitted(std::uint64_t write, std::uint64_t state) const noexcept {
    return stop_committed_.write_id == write &&
           stop_committed_.state_seq == state && write != 0;
  }
  bool Leave(const LeaveReady&) noexcept;
  bool Left() const noexcept { return left_; }
  bool Publishing() const noexcept { return publishing_; }
  void OnPublish(std::function<bool()> fn) { publish_ = std::move(fn); }

 private:
  EndpointConfig config_;
  SimulationClock& clock_;
  transport::DdsTransport* lower_{nullptr};
  transport::ClientTransport* upper_{nullptr};
  Endpoint endpoint_;
  std::array<Frame, 140> queued_{};
  std::size_t read_{0}, write_{0};
  PhaseManifest inbound_{}, outbound_{};
  FeedbackFrame feedback_{};
  StepPermit tick_{};
  bool failed_{false}, installing_{false}, installed_{false},
      ready_sent_{false};
  bool tick_pending_{false}, feedback_pending_{false},
      business_complete_{false};
  bool publishing_{false}, ack_sent_{false};
  std::uint64_t next_step_{0};
  protocol::TimeUs deadline_{0}, last_publish_{0};
  std::function<bool()> publish_;
  bool stop_feedback_received_{false};
  bool shutdown_authorized_{false}, runtime_only_{false}, leaving_{false},
      left_{false};
  StepIdentity shutdown_identity_{};
  ShutdownCommitted stop_committed_{};
  LeaveReady leave_{};
};
protocol::TimeUs DriverHostUs() noexcept;
}  // namespace irmv::simulation
