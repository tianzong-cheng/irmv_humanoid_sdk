#pragma once
#include "irmv/application/robot_client.hpp"
#include "irmv/simulation/driver_session.hpp"
namespace irmv::simulation {
// Pure scheduling/ownership gate; completions and logical time are explicit.
class InferencePhase {
 public:
  bool Begin(protocol::TimeUs now, std::uint64_t generation,
             std::uint64_t epoch, protocol::TimeUs period) noexcept {
    if (pending_ || (submitted_ && (now < stamp_ || now - stamp_ < period)))
      return false;
    stamp_ = now;
    generation_ = generation;
    epoch_ = epoch;
    pending_ = submitted_ = true;
    return true;
  }
  bool Matches(protocol::TimeUs stamp, std::uint64_t generation,
               std::uint64_t epoch) const noexcept {
    return pending_ && stamp == stamp_ && generation == generation_ &&
           epoch == epoch_;
  }
  bool Complete(protocol::TimeUs stamp, std::uint64_t generation,
                std::uint64_t epoch) noexcept {
    if (!Matches(stamp, generation, epoch)) return false;
    pending_ = false;
    return true;
  }
  void Cancel() noexcept { pending_ = submitted_ = false; }
  bool pending() const noexcept { return pending_; }

 private:
  protocol::TimeUs stamp_{0};
  std::uint64_t generation_{0}, epoch_{0};
  bool pending_{false}, submitted_{false};
};
class ApplicationDriver final : public application::ClientSink {
 public:
  ApplicationDriver(EndpointConfig, SimulationClock&,
                    transport::ClientTransport&);
  void Start() { session_.Start(); }
  bool TakePermit(StepPermit*) noexcept;
  const transport::ClientEvents& Events() const noexcept { return events_; }
  bool CompletePhase(const PhaseManifest&) noexcept;
  const PhaseManifest& Outputs() const noexcept { return outputs_; }
  bool Processed(const MessageIdentity& id) noexcept {
    return session_.Processed(id);
  }
  bool CancelRelease() noexcept;
  bool Poll() noexcept { return session_.Poll(); }
  // Progress wait and the two descriptors behind it, for a caller that combines
  // simulation progress with another source (the policy response signal or the
  // terminal) in one poll.
  bool WaitForProgress(protocol::TimeUs timeout_us) noexcept {
    return session_.WaitForProgress(timeout_us);
  }
  int WaitFd() const noexcept { return session_.EndpointWaitFd(); }
  int TransportWaitFd() const noexcept { return session_.TransportWaitFd(); }
  void DrainWait() noexcept { session_.DrainWait(); }
  bool OwnerShutdown() const noexcept { return session_.ShutdownAuthorized(); }
  bool OwnerStopCommitted() const noexcept {
    return !Failed() && session_.OwnerStopCommitted();
  }
  bool PrepareLifecycle(bool locomotion, protocol::Sequence first_seq,
                        bool exiting) noexcept;
  bool LeaveComplete() noexcept {
    session_.Poll();
    return session_.Left();
  }
  bool Failed() const noexcept { return failed_ || session_.Failed(); }
  bool PublishRequest(const protocol::ExecutionRequest&) override;
  bool PublishCommand(const protocol::JointCommand&) override;
  bool PublishHeartbeat(const protocol::SupervisorHeartbeat&) override;

 private:
  using Sample =
      std::variant<protocol::ExecutionRequest, protocol::JointCommand,
                   protocol::SupervisorHeartbeat>;
  bool Stage(const Sample&);
  bool Publish();
  DriverSession session_;
  transport::ClientTransport& transport_;
  PhaseManifest outputs_{};
  transport::ClientEvents events_{};
  std::array<Sample, kMaxPhaseManifestEntries> samples_{};
  std::uint32_t count_{0}, received_{0};
  bool permitted_{false}, failed_{false};
  StepPermit permit_{};
  std::array<protocol::ExecutionRequest, 64> requests_{};
  std::size_t request_count_{0};
  ReleaseArm arm_{};
  LeaveReady leave_{};
  bool arm_pending_{false}, leave_pending_{false};
  std::uint64_t armed_request_{0}, cancelled_request_{0};
};
}  // namespace irmv::simulation
