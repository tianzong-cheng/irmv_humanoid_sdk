#include "irmv/simulation/driver_session.hpp"

#include <poll.h>

#include <cerrno>
#include <chrono>
#include <climits>
namespace irmv::simulation {
namespace {
// poll(2) takes milliseconds; round a positive remainder up so a
// sub-millisecond budget never degenerates into a zero-timeout spin.
int PollTimeoutMs(protocol::TimeUs timeout_us) {
  if (timeout_us == 0) return 0;
  const auto ms = (timeout_us + 999) / 1000;
  return ms > static_cast<protocol::TimeUs>(INT_MAX) ? INT_MAX
                                                     : static_cast<int>(ms);
}
}  // namespace
protocol::TimeUs DriverHostUs() noexcept {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
MessageIdentity InactiveCommand(protocol::Id id) noexcept {
  MessageIdentity out{};
  out.kind = MessageKind::Command;
  out.runtime_id = id;
  return out;
}
DriverSession::DriverSession(EndpointConfig c, SimulationClock& clock,
                             transport::DdsTransport* t)
    : config_(std::move(c)), clock_(clock), lower_(t) {}
DriverSession::DriverSession(EndpointConfig c, SimulationClock& clock,
                             transport::ClientTransport* t)
    : config_(std::move(c)), clock_(clock), upper_(t) {}
void DriverSession::Start() {
  deadline_ = DriverHostUs() +
              static_cast<std::uint64_t>(config_.startup_timeout_ms) * 1000;
  endpoint_.Start(config_);
}
bool DriverSession::Failed() const noexcept {
  return failed_ || endpoint_.Failed() ||
         (lower_ ? lower_->SimulationFailed() : upper_->SimulationFailed());
}
bool DriverSession::WaitForProgress(protocol::TimeUs timeout_us) noexcept {
  if (Failed()) return false;
  pollfd descriptors[2] = {{-1, 0, 0}, {-1, 0, 0}};
  nfds_t count = 0;
  const int endpoint_fd = endpoint_.WaitFd();
  if (endpoint_fd >= 0) descriptors[count++] = {endpoint_fd, POLLIN, 0};
  const int transport_fd = TransportWaitFd();
  if (transport_fd >= 0) descriptors[count++] = {transport_fd, POLLIN, 0};
  int ready;
  ready = ::poll(descriptors, count, PollTimeoutMs(timeout_us));
  if (ready < 0 && errno == EINTR) {
    // Do not restart the relative timeout: the caller rechecks its absolute
    // host deadline and stop flags with a budget recomputed from now.
    return true;
  }
  // The signals are level-triggered and carry no payload: clearing them only
  // means the caller's next condition check must look at the channels, which
  // every wait site does before it waits again.
  DrainWait();
  return ready >= 0;
}
Frame DriverSession::Make(StepIdentity id) const noexcept {
  Frame f;
  f.identity = id;
  f.physics_dt_us = config_.local_hello.physics_dt_us;
  f.control_dt_us = config_.local_hello.control_dt_us;
  return f;
}
bool DriverSession::Queue(const Frame& f) noexcept {
  if (write_ - read_ == queued_.size()) {
    failed_ = true;
    return false;
  }
  queued_[write_++ % queued_.size()] = f;
  return true;
}
bool DriverSession::Flush() noexcept {
  while (read_ != write_ && endpoint_.SendAvailable()) {
    if (!endpoint_.TrySend(queued_[read_ % queued_.size()])) {
      failed_ = true;
      return false;
    }
    ++read_;
  }
  return !Failed();
}
bool DriverSession::Declare(const PhaseManifest& p) noexcept {
  if (!p.Valid()) {
    failed_ = true;
    return false;
  }
  outbound_ = p;
  publishing_ = false;
  for (std::uint32_t i = 0; i < p.count; ++i) {
    auto f = Make(p.identity);
    f.payload = Manifest{p.sender, p.messages[i]};
    if (!Queue(f)) return false;
  }
  auto f = Make(p.identity);
  f.payload = p.seal;
  if (!Queue(f)) return false;
  return Flush();
}
bool DriverSession::Processed(const MessageIdentity& id) noexcept {
  return lower_ ? lower_->ReportSimulationProcessed(id)
                : upper_->ReportSimulationProcessed(id);
}
bool DriverSession::TakeTick(StepPermit* p) noexcept {
  if (!tick_pending_ || !p) return false;
  *p = tick_;
  tick_pending_ = false;
  return true;
}
bool DriverSession::TakeFeedback(FeedbackFrame* f) noexcept {
  if (!feedback_pending_ || !f) return false;
  *f = feedback_;
  feedback_pending_ = false;
  return true;
}
bool DriverSession::Leave(const LeaveReady& proof) noexcept {
  if (lower_ || leaving_) return false;
  auto f = Make(inbound_.identity);
  f.identity.phase = 2;
  f.payload = proof;
  if (!Queue(f)) return false;
  leaving_ = true;
  leave_ = proof;
  return Flush();
}
bool DriverSession::Poll() noexcept {
  if (Failed()) return false;
  const auto now = DriverHostUs();
  if (now >= deadline_) {
    failed_ = true;
    return false;
  }
  // A control-side frame becomes visible to the endpoint only through Flush.
  // Every path that queues a frame calls it before returning, so the next
  // waiter cannot park while a locally produced frame is still only queued_.
  if (!Flush()) return false;
  Frame f;
  while (endpoint_.TryTake(&f)) {
    if (auto permit = std::get_if<ShutdownPermit>(&f.payload)) {
      if (shutdown_authorized_ || permit->role != config_.local_hello.role ||
          f.identity.phase != 4 || f.identity.control_step + 1 < next_step_ ||
          f.identity.control_step > next_step_ || !clock_.Commit(f.identity)) {
        failed_ = true;
        return false;
      }
      shutdown_authorized_ = true;
      shutdown_identity_ = f.identity;
      tick_pending_ = feedback_pending_ = publishing_ = installing_ = false;
      read_ = write_;
      deadline_ =
          now + static_cast<std::uint64_t>(config_.io_timeout_ms) * 1000;
      continue;
    }
    if (auto permit = std::get_if<RuntimeOnlyPermit>(&f.payload)) {
      if (!lower_ || runtime_only_ || permit->role != Role::Runtime ||
          f.identity.phase != 2 || f.identity.control_step + 1 != next_step_ ||
          !clock_.Commit(f.identity)) {
        failed_ = true;
        return false;
      }
      runtime_only_ = true;
      publishing_ = installing_ = false;
      continue;
    }
    if (auto committed = std::get_if<ShutdownCommitted>(&f.payload)) {
      if (!shutdown_authorized_ || !SamePhase(f.identity, shutdown_identity_) ||
          !committed->write_id || !committed->state_seq) {
        failed_ = true;
        return false;
      }
      stop_committed_ = *committed;
      continue;
    }
    if (shutdown_authorized_) {
      if (f.identity.control_step <= shutdown_identity_.control_step &&
          f.identity.phase < 4)
        continue;
      if (upper_) {
        failed_ = true;
        return false;
      }
    }
    if (auto committed = std::get_if<LeaveCommitted>(&f.payload)) {
      if (lower_ || !leaving_ || f.identity.phase != 2 ||
          f.identity.control_step + 1 != next_step_ ||
          !SameIdentity(committed->proof.stop_request, leave_.stop_request) ||
          !SameIdentity(committed->proof.stop_result, leave_.stop_result) ||
          !SameIdentity(committed->proof.stopped_state, leave_.stopped_state) ||
          !committed->authorization_revoked || !committed->heartbeat_resolved) {
        failed_ = true;
        return false;
      }
      left_ = true;
      publishing_ = false;
      continue;
    }
    if (std::holds_alternative<Abort>(f.payload)) {
      failed_ = true;
      return false;
    }
    if (auto a = std::get_if<AppliedToStep>(&f.payload)) {
      if (!lower_ || f.identity.phase != 3 ||
          f.identity.control_step + 1 != next_step_ || !a->write_id)
        failed_ = true;
      continue;
    }
    if (auto feedback = std::get_if<Feedback>(&f.payload)) {
      if (shutdown_authorized_ && f.identity.phase == 4 &&
          SamePhase(f.identity, shutdown_identity_)) {
        if (stop_feedback_received_) {
          failed_ = true;
          return false;
        }
        stop_feedback_received_ = true;
        feedback_ = {f.identity, *feedback};
        feedback_pending_ = true;
        tick_ = {f.identity, {}, true};
        tick_pending_ = true;
        continue;
      }
      if (!lower_ || feedback_pending_ || f.identity.phase != 1 ||
          f.identity.control_step != next_step_ || !clock_.Commit(f.identity)) {
        failed_ = true;
        return false;
      }
      feedback_ = {f.identity, *feedback};
      feedback_pending_ = true;
      publishing_ = false;
      deadline_ =
          now + static_cast<std::uint64_t>(config_.io_timeout_ms) * 1000;
    } else if (auto t = std::get_if<TickPermit>(&f.payload)) {
      if (tick_pending_ || t->role != config_.local_hello.role ||
          t->stage != (lower_ ? 1U : 2U) || f.identity.phase != 1 ||
          f.identity.control_step != next_step_ || !clock_.Commit(f.identity)) {
        failed_ = true;
        return false;
      }
      tick_ = {f.identity, inbound_};
      tick_pending_ = true;
      ++next_step_;
      deadline_ =
          now + static_cast<std::uint64_t>(config_.io_timeout_ms) * 1000;
    } else if (std::holds_alternative<Manifest>(f.payload) ||
               std::holds_alternative<PhaseSeal>(f.payload)) {
      const auto expected = lower_ ? 2U : 1U;
      if (f.identity.phase != expected ||
          f.identity.control_step != (lower_ ? next_step_ - 1 : next_step_) ||
          !clock_.Commit(f.identity)) {
        failed_ = true;
        return false;
      }
      if (!installing_) {
        if (upper_) publishing_ = false;
        inbound_ = PhaseManifest{};
        inbound_.identity = f.identity;
        inbound_.physics_dt_us = f.physics_dt_us;
        inbound_.control_dt_us = f.control_dt_us;
        inbound_.sender = lower_ ? Role::Application : Role::Runtime;
        inbound_.consumer = config_.local_hello.role;
        inbound_.required =
            lower_ ? Disposition::Ingress : Disposition::BusinessConsumed;
        installing_ = true;
        installed_ = ready_sent_ = ack_sent_ = business_complete_ = false;
      }
      if (std::holds_alternative<Manifest>(f.payload)) {
        if (!inbound_.Add(f)) {
          failed_ = true;
          return false;
        }
      } else {
        if (!inbound_.Close(f) ||
            (lower_ ? !lower_->BeginSimulationPhase(inbound_)
                    : !upper_->BeginSimulationPhase(inbound_))) {
          failed_ = true;
          return false;
        }
      }
    } else if (auto p = std::get_if<PublishPermit>(&f.payload)) {
      if (!SamePhase(f.identity, outbound_.identity) ||
          p->seal.manifest_count != outbound_.seal.manifest_count ||
          p->seal.manifest_digest != outbound_.seal.manifest_digest ||
          p->seal.sender != outbound_.sender) {
        failed_ = true;
        return false;
      }
      publishing_ = true;
      last_publish_ = 0;
    } else {
      failed_ = true;
      return false;
    }
  }
  if (installing_ && inbound_.closed && !installed_) {
    installed_ = lower_ ? lower_->SimulationPhaseInstalled(inbound_)
                        : upper_->SimulationPhaseInstalled(inbound_);
  }
  if (installed_ && !ready_sent_) {
    f = Make(inbound_.identity);
    f.payload = InventoryReady{inbound_.consumer, inbound_.seal};
    if (!Queue(f)) return false;
    ready_sent_ = true;
  }
  if (ready_sent_ && !ack_sent_ && (lower_ || business_complete_)) {
    if (lower_ ? lower_->TrySimulationAck(&f) : upper_->TrySimulationAck(&f)) {
      auto a = std::get_if<PhaseAck>(&f.payload);
      if (!a || !SamePhase(f.identity, inbound_.identity) ||
          a->seal.manifest_digest != inbound_.seal.manifest_digest) {
        failed_ = true;
        return false;
      }
      if (!Queue(f)) return false;
      ack_sent_ = true;
      installing_ = false;
    }
  }
  // InventoryReady and PhaseAck were created during this call. Send what the
  // endpoint can take now: a peer that waits for them produces no external
  // event, so leaving them queued would park the caller until its host slice.
  if (!Flush()) return false;
  if (publishing_ && now - last_publish_ >= 20000) {
    if (!publish_ || !publish_()) {
      failed_ = true;
      return false;
    }
    last_publish_ = now;
  }
  return !Failed();
}
}  // namespace irmv::simulation
