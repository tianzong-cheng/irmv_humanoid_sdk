#include "irmv/simulation/application_driver.hpp"
namespace irmv::simulation {
ApplicationDriver::ApplicationDriver(EndpointConfig c, SimulationClock& clock,
                                     transport::ClientTransport& t)
    : session_(std::move(c), clock, &t), transport_(t) {
  session_.OnPublish([this] { return Publish(); });
}
bool ApplicationDriver::Stage(const Sample& s) {
  MessageIdentity id;
  if (!permitted_ ||
      !std::visit([&](const auto& v) { return Identify(v, &id); }, s))
    return false;
  for (std::uint32_t i = 0; i < count_; ++i)
    if (SameIdentity(outputs_.messages[i], id)) return true;
  if (count_ == samples_.size() || !outputs_.Add(id)) {
    failed_ = true;
    return false;
  }
  samples_[count_++] = s;
  return true;
}
bool ApplicationDriver::PublishRequest(const protocol::ExecutionRequest& v) {
  return Stage(v);
}
bool ApplicationDriver::PublishCommand(const protocol::JointCommand& v) {
  return Stage(v);
}
bool ApplicationDriver::PublishHeartbeat(
    const protocol::SupervisorHeartbeat& v) {
  return Stage(v);
}
bool ApplicationDriver::Publish() {
  for (std::uint32_t i = 0; i < count_; ++i) {
    bool ok = std::visit(
        [&](const auto& v) {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_same_v<T, protocol::ExecutionRequest>)
            return transport_.PublishRequest(v);
          else if constexpr (std::is_same_v<T, protocol::JointCommand>)
            return transport_.PublishCommand(v);
          else
            return transport_.PublishHeartbeat(v);
        },
        samples_[i]);
    if (!ok) return false;
  }
  return true;
}
bool ApplicationDriver::TakePermit(StepPermit* p) noexcept {
  if (!session_.Poll() || permitted_) return false;
  StepPermit tick;
  if (session_.TakeTick(&tick)) {
    permit_ = tick;
    events_ = transport::ClientEvents{};
    received_ = 0;
  }
  if (permit_.identity.phase != 1) return false;
  const auto& e = transport_.TakeEvents();
  for (std::size_t i = 0; i < e.result_count; ++i) {
    if (events_.result_count == events_.results.size()) {
      failed_ = true;
      return false;
    }
    events_.results[events_.result_count++] = e.results[i];
    ++received_;
  }
  if (e.has_state) {
    if (events_.has_state) {
      failed_ = true;
      return false;
    }
    events_.has_state = true;
    events_.state = e.state;
    events_.state_rx_us = e.state_rx_us;
    ++received_;
  }
  if (received_ != permit_.required.count) return false;
  outputs_ = PhaseManifest{};
  outputs_.identity = permit_.identity;
  outputs_.identity.phase = 2;
  outputs_.physics_dt_us = permit_.required.physics_dt_us;
  outputs_.control_dt_us = permit_.required.control_dt_us;
  outputs_.sender = Role::Application;
  outputs_.consumer = Role::Runtime;
  outputs_.required = Disposition::Ingress;
  count_ = 0;
  permitted_ = true;
  *p = permit_;
  return true;
}
bool ApplicationDriver::CompletePhase(const PhaseManifest& p) noexcept {
  if (!permitted_ || !SamePhase(p.identity, outputs_.identity) ||
      p.count != count_)
    return false;
  session_.BusinessComplete();
  // ACK must be queued on this same endpoint before phase-2 inventory.
  if (!session_.Poll() || !session_.AckQueued()) return false;
  // Processed reports may still await the worker receipt: caller retries under
  // its host deadline. Poll emits ACK only once the transport can prove all
  // actual consumption. The caller reports every dequeued identity before
  // reaching this operation.
  for (std::uint32_t i = 0; i < count_; ++i) {
    if (auto request = std::get_if<protocol::ExecutionRequest>(&samples_[i]);
        request && request->operation == protocol::Operation::Tracking) {
      FirstCommandProof proof;
      proof.request = outputs_.messages[i];
      if (!Identify(request->first_command, &proof.command)) return false;
      auto f = session_.Make(outputs_.identity);
      f.payload = proof;
      if (!session_.Queue(f)) return false;
    }
  }
  if (!outputs_.Close() || !session_.Declare(outputs_)) return false;
  permitted_ = false;
  permit_.identity.phase = 0;
  return true;
}
}  // namespace irmv::simulation
