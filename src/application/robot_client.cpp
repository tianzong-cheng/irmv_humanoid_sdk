#include "irmv/application/robot_client.hpp"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <random>
#include <string_view>

#include "irmv/protocol/validation.hpp"

namespace irmv::application {
namespace {

std::random_device &SupervisorEntropy() {
  static std::random_device device;
  return device;
}

// SplitMix64 finalizer, the same mixing the runtime uses for its own identity:
// cheap, and it spreads every input bit over the result.
std::uint64_t MixBits(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

// The identity used when the system entropy source is unusable, mixed the way
// the runtime mixes its own fallback: the host monotonic clock, the wall clock,
// the process id, a process-local counter and a stack address. It is not
// random, but no two processes share it - a constant would make two such
// processes indistinguishable to the runtime, which identifies the session
// owner by supervisor id alone - and it is never the all-zero id.
protocol::Id FallbackSupervisorId() {
  static std::atomic<std::uint64_t> counter{0};
  const std::uint64_t salt = counter.fetch_add(1, std::memory_order_relaxed);
  protocol::Id id{};
  std::uint64_t state = MixBits(static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  state ^= MixBits(static_cast<std::uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count()));
  state ^= MixBits(static_cast<std::uint64_t>(::getpid()));
  state ^= MixBits(salt);
  state ^= MixBits(reinterpret_cast<std::uintptr_t>(&id));
  for (std::size_t index = 0; index < id.size(); ++index) {
    state = MixBits(state);
    id[index] = static_cast<std::uint8_t>(state & 0xFFU);
  }
  if (id == protocol::kNoId) {
    id[0] = 1;
  }
  return id;
}

// The request id a client starts from, taken from the first eight bytes of its
// supervisor identity, big-endian. The band exists so the exhaustion path stays
// reachable deterministically - a test pins the identity and the band start
// follows from it, which no counter-based start would allow - and so the first
// ids of a session are never a degenerate small range. The value carries no
// further meaning: the runtime clears its request records when it issues a
// session, and a restarted client restarts from the beginning of its own band
// anyway.
protocol::Sequence RequestIdBand(const protocol::Id &supervisor_id) {
  protocol::Sequence base = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    base = (base << 8U) | supervisor_id[index];
  }
  return base;
}

bool SameText(const protocol::Text64 &text, const std::string &value) {
  return text.View() == std::string_view(value);
}

// The witness rule of the frozen confirmation contract: the state published at
// or after an answer has to agree with it on everything the answer claims about
// the authorization.
bool Agrees(const protocol::RobotState &state,
            const protocol::ExecutionResult &result) {
  return state.runtime_id == result.runtime_id &&
         state.session_id == result.session_id &&
         state.execution_state == result.execution_state &&
         state.output_action == result.output_action &&
         state.epoch == result.epoch && state.producer_id == result.producer_id;
}

}  // namespace

protocol::Id MakeSupervisorId() {
  protocol::Id id{};
  bool filled = false;
  try {
    // The identity comes from the system random source, like the runtime's
    // session ids.
    std::random_device &entropy = SupervisorEntropy();
    std::size_t index = 0;
    while (index < id.size()) {
      const std::random_device::result_type value = entropy();
      for (std::size_t byte = 0; byte < sizeof(value) && index < id.size();
           ++byte, ++index) {
        id[index] = static_cast<std::uint8_t>((value >> (8U * byte)) & 0xFFU);
      }
    }
    filled = true;
  } catch (const std::exception &) {
    filled = false;
  }
  if (!filled || id == protocol::kNoId) {
    // An unusable entropy source is not worth failing startup over, but it must
    // not hand out the same identity to every process either.
    id = FallbackSupervisorId();
  }
  // Never the all-zero "none" value, and never a first byte that would leave
  // the request-id band without room: 0x01..0x7f keeps the whole upper half of
  // the sequence space ahead of it.
  id[0] = static_cast<std::uint8_t>((id[0] & 0x7FU) | 0x01U);
  return id;
}

bool IsStopOperation(protocol::Operation operation) {
  return operation == protocol::Operation::Damping ||
         operation == protocol::Operation::ZeroTorque ||
         operation == protocol::Operation::Reset;
}

RobotClient::RobotClient(ClientSink &sink,
                         const configuration::ApplicationConfig &config,
                         protocol::Id supervisor_id)
    : sink_(sink),
      config_(config),
      supervisor_id_(supervisor_id),
      next_request_id_(RequestIdBand(supervisor_id)) {}

bool RobotClient::IdentityValid() const {
  return supervisor_id_ != protocol::kNoId;
}

bool RobotClient::LiveSession() const {
  return IdentityValid() && session_owned_ && !session_lost_;
}

bool RobotClient::SameInstance(const protocol::RobotState &state) const {
  return state.runtime_id != protocol::kNoId &&
         SameText(state.config_id, config_.runtime_config_id()) &&
         SameText(state.layout_id, config_.layout_id());
}

bool RobotClient::StateFresh(protocol::TimeUs now_us) const {
  // The age is judged on the host monotonic clock that stamped the arrival, not
  // on the runtime's publication stamp: a silent link is exactly the case the
  // runtime cannot report about itself.
  return state_known_ && now_us >= latest_state_rx_us_ &&
         now_us - latest_state_rx_us_ <= config_.supervision.state_timeout_us;
}

const RobotClient::StateObservation &RobotClient::HistoryAt(
    std::size_t offset) const {
  const std::size_t oldest =
      (history_next_ + kStateHistoryCapacity - history_count_) %
      kStateHistoryCapacity;
  return history_[(oldest + offset) % kStateHistoryCapacity];
}

void RobotClient::RecordHistory(const protocol::RobotState &state,
                                protocol::TimeUs rx_us) {
  // Assigned field by field: building the observation first would copy the
  // whole state twice for every received sample.
  StateObservation &slot = history_[history_next_];
  slot.state = state;
  slot.rx_us = rx_us;
  history_next_ = (history_next_ + 1) % kStateHistoryCapacity;
  if (history_count_ < kStateHistoryCapacity) {
    ++history_count_;
  }
}

void RobotClient::DropSnapshot() {
  history_count_ = 0;
  history_next_ = 0;
  latest_state_ = protocol::RobotState{};
  latest_state_rx_us_ = 0;
  state_known_ = false;
}

void RobotClient::Resolve(RequestOutcome outcome,
                          protocol::RejectReason reason) {
  // A confirmation adopts the session the result issued or renewed; every other
  // outcome leaves the session as it is, and a link loss clears it separately.
  if (outcome == RequestOutcome::Confirmed && pending_result_valid_) {
    const protocol::Id confirmed_session = pending_result_.session_id;
    if (confirmed_session != session_id_) {
      // A new session, so its lease starts over at the first sequence value.
      heartbeat_seq_ = 0;
      session_id_ = confirmed_session;
    }
    if (pending_result_.epoch > epoch_watermark_) {
      epoch_watermark_ = pending_result_.epoch;
    }
    session_owned_ = true;
    session_lost_ = false;
    runtime_changed_ = false;
  }
  pending_ = false;
  pending_result_ = protocol::ExecutionResult{};
  pending_result_valid_ = false;
  confirmation_refused_ = false;
  pending_sends_ = 0;
  // The newest outcome wins: a session that dies right after a confirmation
  // must not hide the loss behind an outcome no one has read yet.
  report_outcome_ = outcome;
  report_reason_ = reason;
}

void RobotClient::LoseSession() {
  // Nothing was armed: no session and no request could be affected, so there is
  // no loss to report and nothing to withdraw.
  const bool armed =
      IdentityValid() &&
      (session_owned_ || session_id_ != protocol::kNoId || pending_);
  session_owned_ = false;
  session_id_ = protocol::kNoId;
  epoch_watermark_ = 0;
  proposed_epoch_ = 0;
  heartbeat_seq_ = 0;
  if (armed) {
    // Withdraws the pending request as well, so a request that was waiting for
    // an answer can never resolve after the link is gone.
    Resolve(RequestOutcome::Unknown, protocol::RejectReason::None);
    session_lost_ = true;
  }
  DropSnapshot();
}

void RobotClient::ObserveState(const protocol::RobotState &state,
                               protocol::TimeUs rx_us) {
  if (state.protocol_version != protocol::kProtocolVersion) {
    // A packet that does not speak the frozen protocol is no evidence about
    // this client's instance, session or layout.
    return;
  }

  const bool bound = instance_known_ && runtime_id_ == state.runtime_id;
  if (bound && SameInstance(state)) {
    // A state of the instance this client is bound to.
  } else {
    if (instance_known_ && runtime_id_ != state.runtime_id) {
      // Another runtime instance appeared. It is a new process: the old
      // session, the old watermark, the old request and the old snapshot are
      // all worthless, and the change is reported until a new session is
      // confirmed on the new instance.
      runtime_changed_ = true;
      LoseSession();
    } else if (instance_known_ && !SameInstance(state)) {
      // The same instance with another configuration or another layout: it is
      // not the runtime this client was configured for, so nothing bound to it
      // survives either. It is not a restart, so runtime_changed stays as it
      // is.
      LoseSession();
    }
    if (!SameInstance(state)) {
      // Not an instance this client can ever talk to; in particular a foreign
      // layout is never a target layout. Keep the last trusted snapshot away
      // from it.
      instance_known_ = false;
      runtime_id_ = protocol::kNoId;
      DropSnapshot();
      return;
    }
  }

  instance_known_ = true;
  runtime_id_ = state.runtime_id;
  RecordHistory(state, rx_us);
  latest_state_ = state;
  latest_state_rx_us_ = rx_us;
  state_known_ = true;
  if (state.epoch > epoch_watermark_) {
    epoch_watermark_ = state.epoch;
  }

  if (session_owned_ && state.session_id != session_id_) {
    // The runtime revoked the session (a lost lease, a takeover) or replaced
    // it: the client must not command or heartbeat on it any more.
    LoseSession();
  }

  EvaluatePending();
}

bool RobotClient::MatchesPending(const protocol::ExecutionResult &result,
                                 protocol::TimeUs rx_us) const {
  if (!pending_) {
    return false;
  }
  // The answer has to name this request, this supervisor and the instance the
  // request was addressed to.
  if (result.request_id != pending_request_.request_id ||
      result.supervisor_id != supervisor_id_ ||
      result.runtime_id != pending_request_.runtime_id) {
    return false;
  }
  // A result that names no session answers nothing, and a result that names
  // another session does not answer this one. A bootstrap request (no session
  // yet) accepts the session it is granted.
  if (result.session_id == protocol::kNoId) {
    return false;
  }
  if (pending_request_.session_id != protocol::kNoId &&
      result.session_id != pending_request_.session_id) {
    return false;
  }
  // An answer produced before the first send, or arriving before the stamp it
  // carries, cannot answer this request.
  return result.stamp_us >= pending_first_send_us_ && result.stamp_us <= rx_us;
}

void RobotClient::ObserveResult(const protocol::ExecutionResult &result,
                                protocol::TimeUs rx_us) {
  if (result.protocol_version != protocol::kProtocolVersion) {
    return;
  }
  if (!MatchesPending(result, rx_us)) {
    return;
  }
  pending_result_ = result;
  pending_result_valid_ = true;
  if (result.status == protocol::ResultStatus::Applied) {
    EvaluatePending();
    return;
  }
  // A rejection speaks for itself: it needs no state to witness it.
  Resolve(RequestOutcome::Rejected, result.reason);
}

void RobotClient::EvaluatePending() {
  if (!pending_ || !pending_result_valid_ || confirmation_refused_) {
    return;
  }
  if (pending_result_.status != protocol::ResultStatus::Applied) {
    return;
  }
  // The earliest state published at or after the answer. States are kept in
  // arrival order, so this is the first one the runtime produced after the
  // request was handled, whether it was read before or after the answer.
  for (std::size_t offset = 0; offset < history_count_; ++offset) {
    const StateObservation &observation = HistoryAt(offset);
    if (observation.state.stamp_us < pending_result_.stamp_us) {
      continue;
    }
    if (Agrees(observation.state, pending_result_)) {
      Resolve(RequestOutcome::Confirmed, protocol::RejectReason::None);
    } else {
      // The behaviour the answer authorized is already gone: a latched fault, a
      // stop or a newer epoch contradicts it. The refusal is terminal, so a
      // state that arrives later can never revive it; the request then expires
      // at its own deadline.
      confirmation_refused_ = true;
    }
    return;
  }
}

void RobotClient::ResendPending(protocol::TimeUs now_us) {
  // The retry repeats the stored payload byte for byte, so the runtime can
  // recognize it by the request id instead of applying it twice.
  pending_last_send_us_ = now_us;
  sink_.PublishRequest(pending_request_);
  ++pending_sends_;
}

bool RobotClient::Begin(protocol::Operation operation, protocol::TimeUs now_us,
                        protocol::Id producer_id,
                        const protocol::JointCommand *first_command,
                        protocol::Sequence fault_id) {
  if (!IdentityValid() || !protocol::IsValid(operation)) {
    return false;
  }
  const bool stop = IsStopOperation(operation);
  // One ordinary request at a time. A stop may preempt whatever is pending: its
  // larger request id is exactly what the runtime requires to displace the
  // pending one, and the preempted answer can no longer confirm anything.
  if (pending_ && !stop) {
    return false;
  }

  if (operation == protocol::Operation::AcquireSession) {
    // The bootstrap: no live session, and only against an instance the client
    // has actually seen, because the runtime answers any other runtime id as
    // foreign.
    if (session_owned_ || !instance_known_) {
      return false;
    }
  } else if (!LiveSession()) {
    return false;
  }

  if (operation == protocol::Operation::Tracking) {
    if (producer_id == protocol::kNoId || first_command == nullptr) {
      return false;
    }
    if (first_command->joints.count !=
        static_cast<std::uint32_t>(config_.joint_count())) {
      return false;
    }
    if (!SameText(first_command->layout_id, config_.layout_id())) {
      return false;
    }
  }

  // The last sequence value is reserved: an id nothing larger can follow cannot
  // be answered in order by the runtime, so the request is refused instead.
  const protocol::Sequence request_id = next_request_id_;
  if (request_id == std::numeric_limits<protocol::Sequence>::max()) {
    return false;
  }

  protocol::ExecutionRequest request;
  request.protocol_version = protocol::kProtocolVersion;
  request.supervisor_id = supervisor_id_;
  request.request_id = request_id;
  request.operation = operation;
  request.runtime_id = runtime_id_;
  request.session_id = operation == protocol::Operation::AcquireSession
                           ? protocol::kNoId
                           : session_id_;
  request.expected_epoch =
      operation == protocol::Operation::AcquireSession ? 0 : epoch_watermark_;
  request.proposed_epoch = 0;
  request.producer_id = protocol::kNoId;
  request.fault_id = fault_id;
  if (operation == protocol::Operation::Tracking) {
    // A tracking request proposes the next authorization watermark and carries
    // the first frame of the new behaviour stage; its routing identity is the
    // same completion a continuous target gets.
    const protocol::Sequence proposed = next_epoch();
    if (proposed <= epoch_watermark_) {
      return false;
    }
    request.proposed_epoch = proposed;
    request.producer_id = producer_id;
    request.first_command = *first_command;
    request.first_command.protocol_version = protocol::kProtocolVersion;
    request.first_command.runtime_id = runtime_id_;
    request.first_command.session_id = session_id_;
    request.first_command.stamp_us = now_us;
    proposed_epoch_ = proposed;
  }

  next_request_id_ = request_id + 1;
  pending_ = true;
  pending_request_ = request;
  pending_first_send_us_ = now_us;
  pending_last_send_us_ = now_us;
  pending_sends_ = 1;
  pending_result_valid_ = false;
  confirmation_refused_ = false;
  // A new request supersedes an outcome no one has read yet: the supervision
  // loop polls before it begins the next request, so an unread outcome here
  // belongs to a request the caller has already moved on from.
  report_outcome_ = RequestOutcome::None;
  report_reason_ = protocol::RejectReason::None;

  sink_.PublishRequest(pending_request_);
  return true;
}

bool RobotClient::BeginRequest(protocol::Operation operation,
                               protocol::TimeUs now_us,
                               protocol::Id producer_id,
                               const protocol::JointCommand *first_command) {
  return Begin(operation, now_us, producer_id, first_command, 0);
}

bool RobotClient::BeginReset(protocol::Sequence fault_id,
                             protocol::TimeUs now_us) {
  return Begin(protocol::Operation::Reset, now_us, protocol::kNoId, nullptr,
               fault_id);
}

ClientView RobotClient::Poll(protocol::TimeUs now_us) {
  // A state link that goes silent is only visible here: nothing arrives to time
  // out. Losing the lease is losing the session, so it is reported exactly once
  // and withdraws everything the session owned.
  if (session_owned_ && !session_lost_ && !StateFresh(now_us)) {
    LoseSession();
  }

  if (pending_ && report_outcome_ == RequestOutcome::None) {
    // The ages are guarded exactly like the state age: a caller-supplied clock
    // that is not monotonic must not underflow into an immediate timeout or an
    // immediate resend.
    const protocol::TimeUs age =
        now_us >= pending_first_send_us_ ? now_us - pending_first_send_us_ : 0;
    const protocol::TimeUs since_send =
        now_us >= pending_last_send_us_ ? now_us - pending_last_send_us_ : 0;
    if (age >= config_.supervision.request_deadline_us) {
      Resolve(RequestOutcome::TimedOut, protocol::RejectReason::None);
    } else if (since_send >= config_.supervision.request_retry_us) {
      ResendPending(now_us);
    }
  }

  ClientView view;
  view.state = latest_state_;
  view.state_known = state_known_;
  view.state_fresh = StateFresh(now_us);
  view.session_owned = session_owned_;
  view.session_lost = session_lost_;
  view.runtime_changed = runtime_changed_;

  if (report_outcome_ != RequestOutcome::None) {
    view.outcome = report_outcome_;
    view.reject_reason = report_reason_;
    report_outcome_ = RequestOutcome::None;
    report_reason_ = protocol::RejectReason::None;
  } else if (pending_) {
    view.outcome = RequestOutcome::Pending;
  }
  return view;
}

bool RobotClient::SendCommand(protocol::JointCommand *command,
                              protocol::TimeUs now_us) {
  if (command == nullptr || !LiveSession()) {
    return false;
  }
  if (command->joints.count !=
      static_cast<std::uint32_t>(config_.joint_count())) {
    return false;
  }
  if (!SameText(command->layout_id, config_.layout_id())) {
    return false;
  }
  command->protocol_version = protocol::kProtocolVersion;
  command->runtime_id = runtime_id_;
  command->session_id = session_id_;
  command->stamp_us = now_us;
  return sink_.PublishCommand(*command);
}

bool RobotClient::SendHeartbeat(protocol::TimeUs now_us) {
  if (!LiveSession()) {
    return false;
  }
  protocol::SupervisorHeartbeat heartbeat;
  heartbeat.protocol_version = protocol::kProtocolVersion;
  heartbeat.supervisor_id = supervisor_id_;
  heartbeat.session_id = session_id_;
  heartbeat.runtime_id = runtime_id_;
  // Only a strictly increasing sequence refreshes the lease.
  heartbeat.heartbeat_seq = ++heartbeat_seq_;
  heartbeat.stamp_us = now_us;
  if (!sink_.PublishHeartbeat(heartbeat)) {
    // A refusal costs this sample, not the sequence: the next attempt carries a
    // newer value anyway, which is what the lease needs.
    return false;
  }
  return true;
}

protocol::Sequence RobotClient::next_epoch() const {
  // Strictly greater than the watermark the runtime publishes and than anything
  // this client proposed before, so a value is never proposed twice.
  const protocol::Sequence base =
      epoch_watermark_ > proposed_epoch_ ? epoch_watermark_ : proposed_epoch_;
  if (base == std::numeric_limits<protocol::Sequence>::max()) {
    return base;
  }
  return base + 1;
}

const protocol::Id &RobotClient::session_id() const { return session_id_; }

const protocol::Id &RobotClient::runtime_id() const { return runtime_id_; }

bool RobotClient::session_owned() const { return session_owned_; }

bool RobotClient::request_pending() const { return pending_; }

protocol::Sequence RobotClient::pending_request_id() const {
  return pending_ ? pending_request_.request_id : 0;
}

std::size_t RobotClient::retry_count() const { return pending_sends_; }

void RobotClient::AbandonRequest() {
  pending_ = false;
  pending_request_ = protocol::ExecutionRequest{};
  pending_result_ = protocol::ExecutionResult{};
  pending_result_valid_ = false;
  confirmation_refused_ = false;
  pending_sends_ = 0;
  report_outcome_ = RequestOutcome::None;
  report_reason_ = protocol::RejectReason::None;
}

}  // namespace irmv::application
