#pragma once
#include "irmv/simulation/driver_session.hpp"
namespace irmv::simulation {
struct TickCompletion {
  const runtime::TickResult& result;
  const protocol::JointCommand* active_target;
};
class RuntimeDriver {
 public:
  RuntimeDriver(EndpointConfig, SimulationClock&, SimulationExchange&,
                transport::DdsTransport&);
  void Start() { session_.Start(); }
  bool TakePermit(StepPermit*) noexcept;
  bool CompleteTick(const TickCompletion&) noexcept;
  bool Processed(const MessageIdentity& id) noexcept {
    return session_.Processed(id);
  }
  bool ShutdownAuthorized() const noexcept {
    return session_.ShutdownAuthorized();
  }
  bool FinishStop() noexcept;
  bool Failed() const noexcept { return session_.Failed(); }

 private:
  DriverSession session_;
  SimulationExchange& exchange_;
  transport::DdsTransport& transport_;
  StepPermit permit_{};
  runtime::TickResult buffered_{};
  bool outstanding_{false};
  std::uint64_t stop_write_{0}, stop_state_{0};
};
}  // namespace irmv::simulation
