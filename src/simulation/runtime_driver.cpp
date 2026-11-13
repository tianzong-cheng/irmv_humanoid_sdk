#include "irmv/simulation/runtime_driver.hpp"
namespace irmv::simulation {
RuntimeDriver::RuntimeDriver(EndpointConfig c, SimulationClock& clock,
                             SimulationExchange& x, transport::DdsTransport& t)
    : session_(std::move(c), clock, &t), exchange_(x), transport_(t) {
  session_.OnPublish([this] {
    transport_.Publish(buffered_);
    return true;
  });
}
bool RuntimeDriver::TakePermit(StepPermit* p) noexcept {
  if (!session_.Poll() || outstanding_) return false;
  if (!session_.TakeTick(&permit_)) return false;
  FeedbackFrame feedback;
  if (!session_.TakeFeedback(&feedback) ||
      !SamePhase(feedback.identity, permit_.identity) ||
      !exchange_.InstallFeedback(feedback))
    return false;
  outstanding_ = true;
  *p = permit_;
  return true;
}
bool RuntimeDriver::CompleteTick(const TickCompletion& c) noexcept {
  if (!outstanding_) return false;
  TorqueFrame torque;
  if (!exchange_.TryTakeTorque(&torque)) return false;
  torque.torque.state_seq = c.result.state.state_seq;
  torque.torque.command = InactiveCommand(c.result.state.runtime_id);
  if (c.active_target && !Identify(*c.active_target, &torque.torque.command))
    return false;
  auto f = session_.Make(permit_.identity);
  f.payload = torque.torque;
  if (!session_.Queue(f)) return false;
  buffered_ = c.result;
  PhaseManifest p;
  p.identity = permit_.identity;
  p.physics_dt_us = f.physics_dt_us;
  p.control_dt_us = f.control_dt_us;
  p.required = Disposition::BusinessConsumed;
  MessageIdentity id;
  if (!Identify(buffered_.state, &id) || !p.Add(id)) return false;
  for (std::uint32_t i = 0; i < buffered_.result_count; ++i)
    if (!Identify(buffered_.results[i], &id) || !p.Add(id)) return false;
  if (!p.Close() || !session_.Declare(p)) return false;
  outstanding_ = false;
  return true;
}
}  // namespace irmv::simulation
