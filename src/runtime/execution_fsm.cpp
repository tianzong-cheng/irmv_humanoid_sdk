#include "irmv/runtime/execution_fsm.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <random>

#include "irmv/protocol/validation.hpp"

namespace irmv::runtime {
namespace {

// Session identifiers come from the system random source. The device is
// created once per process, outside the tick, so a control tick never opens a
// file; a failure of the random source is reported as a session that is still
// non-zero instead of an exception escaping Tick.
std::random_device &SessionEntropy() {
  static std::random_device device;
  return device;
}

// Unclamped expected torque of one tracking frame on the measured state, used
// by the entry continuity guard.
double RawTrackingTorque(const protocol::JointVector &joints,
                         const protocol::FeedbackSnapshot &feedback,
                         std::size_t index) {
  return joints.kp[index] * (joints.q[index] - feedback.joints[index].q) +
         joints.kd[index] * (joints.dq[index] - feedback.joints[index].dq) +
         joints.tau_ff[index];
}

}  // namespace

ExecutionFsm::ExecutionFsm(const configuration::RuntimeConfig &config,
                           protocol::Id runtime_id)
    : config_(config), runtime_id_(runtime_id), gate_(config) {}

TickResult ExecutionFsm::Tick(protocol::TimeUs now_us,
                              const protocol::FeedbackSnapshot &feedback,
                              const TickInput &input, hardware::Backend &backend) {
  TickResult out;
  TickScope scope;
  scope.now_us = now_us;
  scope.feedback = &feedback;
  scope.out = &out;

  // 1. Tick identity; the initial state is entered on the first tick, which is
  //    also the only moment the boot start can be stamped.
  ++tick_;
  const protocol::Sequence published_seq = ++state_seq_;
  if (!entered_) {
    entered_ = true;
    EnterState(state_, scope);
  }

  // 2. Feedback safety and the published state sequence.
  const bool boot_grace =
      state_ == protocol::ExecutionState::Boot && boot_started_ &&
      now_us - boot_start_us_ <= config_.boot.frequency_grace_us;
  const SafetyReport report =
      CheckFeedback(config_, feedback, now_us, boot_grace);
  scope.report = &report;
  gate_.RecordState(published_seq, now_us);

  // 3. Safety decision first: an observed fault is latched before any request is
  //    considered, so a fault and a motion request in the same tick can never
  //    let the request win.
  if (report.fault && state_ != protocol::ExecutionState::Fault) {
    LatchFault(scope, report.reason, EventCode::FaultLatched,
               DampingEligible(report), report.details);
  }

  // 4. Fault bookkeeping: output degradation latch and recovery window.
  if (state_ == protocol::ExecutionState::Fault) {
    UpdateFaultEligibility(scope);
  }

  // 5. and 6. Stop requests first, then ordinary requests.
  ProcessRequests(scope, input);

  // 6b. Joint commands only feed the gate while a motion epoch is authorized.
  if (state_ == protocol::ExecutionState::Tracking) {
    ProcessCommands(scope, input);
  }

  // 7. Tracking deadlines.
  if (state_ == protocol::ExecutionState::Tracking) {
    ExpireTracking(scope);
  }

  // 8. Supervision lease.
  ProcessHeartbeats(scope, input);

  // 9. Output for the resulting state.
  out.output = ComputeStateOutput(scope);
  out.output_valid = true;

  // 10. One write per tick, confirmed before any applied result is released.
  const hardware::WriteResult write = backend.Write(now_us, out.output);
  out.write_result = write;
  last_write_accepted_ = write.accepted;
  if (!write.accepted) {
    DegradeForWriteFailure(scope, write.error_code);
    const hardware::TorqueOutput fallback = ZeroTorqueOutput();
    output_action_ = protocol::OutputAction::ZeroTorque;
    out.output = fallback;
    // The safe fallback is attempted in the same tick; the failure itself is
    // what the latch reports.
    out.write_result = backend.Write(now_us, fallback);
  }
  CommitPendingResults(scope, write.accepted);

  // 11. Publish the state of this tick.
  PublishState(scope);

  out.dropped_result_count = dropped_results_;
  out.dropped_event_count = dropped_events_;
  return out;
}

void ExecutionFsm::ProcessRequests(TickScope &scope, const TickInput &input) {
  const std::size_t stop_count =
      std::min(input.stop_request_count, kStopRequestBatchCapacity);
  for (std::size_t index = 0; index < stop_count; ++index) {
    HandleRequest(scope, input.stop_requests[index]);
  }
  const std::size_t count =
      std::min(input.request_count, kRequestBatchCapacity);
  for (std::size_t index = 0; index < count; ++index) {
    HandleRequest(scope, input.requests[index]);
  }
}

void ExecutionFsm::ProcessCommands(TickScope &scope, const TickInput &input) {
  const std::size_t count =
      std::min(input.command_count, kCommandBatchCapacity);
  for (std::size_t index = 0; index < count; ++index) {
    const GateResult gate_result = gate_.Accept(input.commands[index],
                                                scope.now_us);
    if (gate_result.accepted()) {
      last_applied_command_seq_ = input.commands[index].value.command_seq;
      continue;
    }
    // Only the authorized owner can produce an invalid owner command; a foreign
    // or stale packet is dropped by the gate without a fault.
    if (gate_result.invalid_owner_command()) {
      LatchFault(scope, protocol::FaultReason::CommandInvalid,
                 EventCode::FaultLatched, DampingEligible(*scope.report),
                 MakeFaultReport(protocol::FaultReason::CommandInvalid,
                                 scope.now_us, 0,
                                 "authorized joint command rejected"));
    }
  }
}

void ExecutionFsm::ProcessHeartbeats(TickScope &scope, const TickInput &input) {
  const std::size_t count =
      std::min(input.heartbeat_count, kHeartbeatBatchCapacity);
  for (std::size_t index = 0; index < count; ++index) {
    const Received<protocol::SupervisorHeartbeat> &packet =
        input.heartbeats[index];
    const protocol::SupervisorHeartbeat &heartbeat = packet.value;
    if (heartbeat.protocol_version != protocol::kProtocolVersion) {
      continue;
    }
    if (heartbeat.runtime_id != runtime_id_) {
      continue;
    }
    if (session_id_ == protocol::kNoId ||
        heartbeat.session_id != session_id_) {
      continue;
    }
    // A peer that only observes the published session id cannot extend a lease
    // it does not own.
    if (heartbeat.supervisor_id != session_supervisor_id_) {
      continue;
    }
    // Only a strictly newer heartbeat refreshes the lease: a repeated or
    // reordered packet must not extend it.
    if (heartbeat_seen_ && heartbeat.heartbeat_seq <= last_heartbeat_seq_) {
      continue;
    }
    heartbeat_seen_ = true;
    last_heartbeat_seq_ = heartbeat.heartbeat_seq;
    last_heartbeat_rx_us_ = packet.rx_us;
  }

  if (session_id_ != protocol::kNoId && !LeaseValid(scope.now_us)) {
    ExpireSession(scope);
  }
}

void ExecutionFsm::ExpireTracking(TickScope &scope) {
  const protocol::StopReason reason = gate_.ExpireReason(scope.now_us);
  if (reason == protocol::StopReason::None) {
    return;
  }
  const protocol::ExecutionState previous = state_;
  if (!RevokeMotion(scope)) {
    return;
  }
  SetState(protocol::ExecutionState::Damping, scope, EventCode::None);
  stop_reason_ = reason;
  EmitEvent(scope,
            reason == protocol::StopReason::CommandTimeout
                ? EventCode::CommandTimeout
                : EventCode::SourceStateTimeout,
            previous, protocol::ExecutionState::Damping,
            protocol::RejectReason::None, protocol::FaultReason::None, reason);
}

void ExecutionFsm::PublishState(TickScope &scope) {
  protocol::RobotState &state = scope.out->state;
  state = protocol::RobotState{};
  state.protocol_version = protocol::kProtocolVersion;
  state.runtime_id = runtime_id_;
  state.session_id = session_id_;
  state.producer_id = producer_id_;
  state.epoch = epoch_;
  state.state_seq = state_seq_;
  state.stamp_us = scope.now_us;
  state.execution_state = state_;
  state.output_action = output_action_;
  state.stop_reason = stop_reason_;
  state.fault_id = fault_id_;
  state.last_applied_command_seq = last_applied_command_seq_;
  state.healthy = !scope.report->fault;
  state.damping_eligible = state_ == protocol::ExecutionState::Fault
                               ? fault_damping_eligible_
                               : scope.report->damping_eligible;
  static_cast<void>(state.config_id.Assign(config_.config_id));
  static_cast<void>(state.layout_id.Assign(config_.layout_id));
  state.feedback = *scope.feedback;
  if (state_ == protocol::ExecutionState::Fault) {
    // A latch without details, such as an incomplete measured snapshot, still
    // reports the reason it latched with.
    state.faults = fault_report_.count > 0
                       ? fault_report_
                       : MakeFaultReport(fault_reason_, scope.now_us, 0,
                                         "latched fault without details");
  } else {
    state.faults = scope.report->details;
  }
}

protocol::ExecutionResult ExecutionFsm::MakeResult(
    const protocol::ExecutionRequest &request,
    protocol::TimeUs now_us) const {
  protocol::ExecutionResult result;
  result.protocol_version = protocol::kProtocolVersion;
  result.supervisor_id = request.supervisor_id;
  result.request_id = request.request_id;
  result.session_id = session_id_;
  result.runtime_id = runtime_id_;
  result.execution_state = state_;
  result.output_action = output_action_;
  result.producer_id = producer_id_;
  result.epoch = epoch_;
  result.fault_id = fault_id_;
  result.stamp_us = now_us;
  return result;
}

void ExecutionFsm::HandleRequest(
    TickScope &scope, const Received<protocol::ExecutionRequest> &packet) {
  const protocol::ExecutionRequest &request = packet.value;
  if (request.protocol_version != protocol::kProtocolVersion) {
    QueueUnrecordedRejection(scope, request,
                             protocol::RejectReason::VersionMismatch);
    return;
  }
  if (request.runtime_id != runtime_id_) {
    QueueUnrecordedRejection(scope, request,
                             protocol::RejectReason::WrongInstance);
    return;
  }
  if (!protocol::IsValid(request.operation)) {
    QueueUnrecordedRejection(scope, request, protocol::RejectReason::Malformed);
    return;
  }
  if (request.operation == protocol::Operation::AcquireSession) {
    HandleAcquireSession(scope, packet);
    return;
  }
  if (session_id_ == protocol::kNoId || request.session_id != session_id_) {
    QueueUnrecordedRejection(scope, request,
                             protocol::RejectReason::WrongSession);
    return;
  }

  // Exactly-once handling: the answers confirmed by earlier ticks, then the
  // answer this tick staged but has not confirmed yet.
  if (request_record_.valid &&
      request.request_id <= request_record_.request_id) {
    if (request.request_id < request_record_.request_id) {
      QueueUnrecordedRejection(scope, request,
                               protocol::RejectReason::StaleRequest);
      return;
    }
    if (PayloadMatches(request, request_record_)) {
      EmitResult(scope, request_record_.result);
      return;
    }
    QueueUnrecordedRejection(scope, request,
                             protocol::RejectReason::RequestConflict);
    return;
  }
  if (staged_record_.valid && request.request_id <= staged_record_.request_id) {
    if (request.request_id < staged_record_.request_id) {
      QueueUnrecordedRejection(scope, request,
                               protocol::RejectReason::StaleRequest);
      return;
    }
    if (PayloadMatches(request, staged_record_)) {
      // The same request twice in one tick: it is answered once, by the answer
      // that is still being confirmed, and consumes no second result.
      return;
    }
    QueueUnrecordedRejection(scope, request,
                             protocol::RejectReason::RequestConflict);
    return;
  }

  switch (request.operation) {
    case protocol::Operation::Damping:
    case protocol::Operation::ZeroTorque:
      HandleStop(scope, packet);
      return;
    case protocol::Operation::Reset:
      HandleReset(scope, packet);
      return;
    case protocol::Operation::Tracking:
      HandleTracking(scope, packet);
      return;
    case protocol::Operation::AcquireSession:
      break;
  }
  QueueUnrecordedRejection(scope, request, protocol::RejectReason::Malformed);
}

void ExecutionFsm::HandleAcquireSession(
    TickScope &scope, const Received<protocol::ExecutionRequest> &packet) {
  const protocol::ExecutionRequest &request = packet.value;
  // The session is issued by the runtime: a bootstrap request carrying one is
  // malformed by contract.
  if (request.session_id != protocol::kNoId) {
    QueueUnrecordedRejection(scope, request, protocol::RejectReason::Malformed);
    return;
  }
  if (session_id_ != protocol::kNoId && LeaseValid(scope.now_us)) {
    // A healthy session is never silently taken over. The supervisor that owns
    // it can repeat its bootstrap request and read the issued session back.
    if (acquire_record_.valid &&
        acquire_record_.supervisor_id == request.supervisor_id &&
        acquire_record_.request_id == request.request_id) {
      if (acquire_record_.answered_tick == tick_) {
        // The same bootstrap packet twice in one batch: it is answered once.
        return;
      }
      EmitResult(scope, acquire_record_.result);
      return;
    }
    QueueUnrecordedRejection(scope, request,
                             protocol::RejectReason::SessionBusy);
    return;
  }
  if (session_id_ != protocol::kNoId) {
    // The lease of the previous session is over: the takeover is completed only
    // after that session stopped commanding torque.
    ExpireSession(scope);
  }
  IssueSession(scope, request);
}

void ExecutionFsm::HandleStop(
    TickScope &scope, const Received<protocol::ExecutionRequest> &packet) {
  const protocol::ExecutionRequest &request = packet.value;
  const protocol::RejectReason allowed =
      CheckStateAllows(state_, request.operation);
  if (allowed != protocol::RejectReason::None) {
    QueueRejected(scope, request, allowed);
    return;
  }
  if (!RevokeMotion(scope)) {
    QueueRejected(scope, request, protocol::RejectReason::EpochMismatch);
    return;
  }
  const protocol::ExecutionState previous = state_;
  const protocol::ExecutionState next =
      request.operation == protocol::Operation::Damping
          ? protocol::ExecutionState::Damping
          : protocol::ExecutionState::ZeroTorque;
  SetState(next, scope, EventCode::None);
  stop_reason_ = protocol::StopReason::Requested;
  EmitEvent(scope, EventCode::StopApplied, previous, next,
            protocol::RejectReason::None, protocol::FaultReason::None,
            protocol::StopReason::Requested, request.request_id);
  QueueApplied(scope, request);
}

void ExecutionFsm::HandleReset(
    TickScope &scope, const Received<protocol::ExecutionRequest> &packet) {
  const protocol::ExecutionRequest &request = packet.value;
  const protocol::RejectReason allowed =
      CheckStateAllows(state_, request.operation);
  if (allowed != protocol::RejectReason::None) {
    QueueRejected(scope, request, allowed);
    return;
  }
  if (request.fault_id != fault_id_) {
    QueueRejected(scope, request, protocol::RejectReason::FaultMismatch);
    return;
  }
  if (!recovery_ready_) {
    QueueRejected(scope, request, protocol::RejectReason::RecoveryNotReady);
    return;
  }
  if (!RevokeMotion(scope)) {
    QueueRejected(scope, request, protocol::RejectReason::EpochMismatch);
    return;
  }
  const protocol::ExecutionState previous = state_;
  fault_id_ = 0;
  fault_reason_ = protocol::FaultReason::None;
  fault_report_ = protocol::FaultReport{};
  fault_damping_eligible_ = false;
  fault_output_degraded_ = false;
  recovery_window_started_ = false;
  recovery_window_start_us_ = 0;
  recovery_ready_ = false;
  stop_reason_ = protocol::StopReason::None;
  SetState(protocol::ExecutionState::ZeroTorque, scope, EventCode::None);
  EmitEvent(scope, EventCode::ResetApplied, previous,
            protocol::ExecutionState::ZeroTorque,
            protocol::RejectReason::None, protocol::FaultReason::None,
            protocol::StopReason::None, request.request_id);
  QueueApplied(scope, request);
}

void ExecutionFsm::HandleTracking(
    TickScope &scope, const Received<protocol::ExecutionRequest> &packet) {
  const protocol::ExecutionRequest &request = packet.value;
  const protocol::RejectReason allowed =
      CheckStateAllows(state_, protocol::Operation::Tracking);
  if (allowed != protocol::RejectReason::None) {
    QueueRejected(scope, request, allowed);
    return;
  }
  if (request.producer_id == protocol::kNoId) {
    QueueRejected(scope, request, protocol::RejectReason::Malformed);
    return;
  }
  // The producer that already owns this epoch must stop before it can hand the
  // motion authorization to a new epoch; the identical replay was answered
  // above from the stored record.
  if (state_ == protocol::ExecutionState::Tracking &&
      request.producer_id == producer_id_ &&
      request.expected_epoch == epoch_) {
    QueueRejected(scope, request, protocol::RejectReason::RequestConflict);
    return;
  }
  if (request.expected_epoch != epoch_ ||
      request.proposed_epoch <= epoch_) {
    QueueRejected(scope, request, protocol::RejectReason::EpochMismatch);
    return;
  }

  const protocol::JointCommand &first = request.first_command;
  const protocol::ValidationResult validation =
      ValidateTrackingCommand(config_, first);
  if (!validation.ok) {
    QueueRejected(scope, request, validation.reason);
    return;
  }
  if (first.runtime_id != runtime_id_ || first.session_id != session_id_ ||
      first.producer_id != request.producer_id ||
      first.epoch != request.proposed_epoch) {
    QueueRejected(scope, request, protocol::RejectReason::Malformed);
    return;
  }
  protocol::TimeUs generation_us = 0;
  if (!gate_.GenerationOf(first.source_state_seq, &generation_us) ||
      scope.now_us - generation_us > config_.timeouts.source_state_us) {
    QueueRejected(scope, request, protocol::RejectReason::StaleSource);
    return;
  }
  if (!CheckContinuity(scope, first)) {
    QueueRejected(scope, request, protocol::RejectReason::Discontinuity);
    return;
  }

  // Every guard passed before the authorization change: commit the producer,
  // the epoch, the gate context and the accepted target atomically.
  gate_.Authorize(runtime_id_, session_id_, request.producer_id,
                  request.proposed_epoch);
  const GateResult gate_result =
      gate_.Accept({first, packet.rx_us}, scope.now_us);
  if (!gate_result.accepted()) {
    // Every gate precondition was validated above, so this is defensive. The
    // motion context is dropped together with the gate: the FSM never names an
    // authorization the gate does not hold.
    ClearMotion(scope);
    QueueRejected(scope, request, gate_result.reason);
    return;
  }
  producer_id_ = request.producer_id;
  epoch_ = request.proposed_epoch;
  last_applied_command_seq_ = first.command_seq;
  stop_reason_ = protocol::StopReason::None;
  const protocol::ExecutionState previous = state_;
  SetState(protocol::ExecutionState::Tracking, scope, EventCode::None);
  EmitEvent(scope, EventCode::AuthorizationGranted, previous,
            protocol::ExecutionState::Tracking, protocol::RejectReason::None,
            protocol::FaultReason::None, protocol::StopReason::None,
            request.request_id);
  QueueApplied(scope, request);
}

void ExecutionFsm::QueueApplied(TickScope &scope,
                                const protocol::ExecutionRequest &request) {
  protocol::ExecutionResult result = MakeResult(request, scope.now_us);
  result.status = protocol::ResultStatus::Applied;
  result.reason = protocol::RejectReason::None;
  StageRequestAnswer(request, result);
  if (pending_count_ >= pending_.size()) {
    ++dropped_results_;
    return;
  }
  pending_[pending_count_++] = PendingResult{result, true, true};
}

void ExecutionFsm::QueueRejected(TickScope &scope,
                                 const protocol::ExecutionRequest &request,
                                 protocol::RejectReason reason) {
  protocol::ExecutionResult result = MakeResult(request, scope.now_us);
  result.status = protocol::ResultStatus::Rejected;
  result.reason = reason;
  StageRequestAnswer(request, result);
  EmitEvent(scope, EventCode::RequestRejected, state_, state_, reason,
            protocol::FaultReason::None, protocol::StopReason::None,
            request.request_id);
  if (pending_count_ >= pending_.size()) {
    ++dropped_results_;
    return;
  }
  pending_[pending_count_++] = PendingResult{result, false, true};
}

void ExecutionFsm::QueueUnrecordedRejection(
    TickScope &scope, const protocol::ExecutionRequest &request,
    protocol::RejectReason reason) {
  protocol::ExecutionResult result = MakeResult(request, scope.now_us);
  result.status = protocol::ResultStatus::Rejected;
  result.reason = reason;
  EmitEvent(scope, EventCode::RequestRejected, state_, state_, reason,
            protocol::FaultReason::None, protocol::StopReason::None,
            request.request_id);
  if (pending_count_ >= pending_.size()) {
    ++dropped_results_;
    return;
  }
  pending_[pending_count_++] = PendingResult{result, false, false};
}

void ExecutionFsm::StageRequestAnswer(
    const protocol::ExecutionRequest &request,
    const protocol::ExecutionResult &result) {
  staged_record_.valid = true;
  staged_record_.operation = request.operation;
  staged_record_.request_id = request.request_id;
  staged_record_.supervisor_id = request.supervisor_id;
  staged_record_.expected_epoch = request.expected_epoch;
  staged_record_.proposed_epoch = request.proposed_epoch;
  staged_record_.producer_id = request.producer_id;
  staged_record_.fault_id = request.fault_id;
  staged_record_.has_command =
      request.operation == protocol::Operation::Tracking;
  if (staged_record_.has_command) {
    staged_record_.first_command = request.first_command;
  }
  staged_record_.result = result;
}

bool ExecutionFsm::PayloadMatches(const protocol::ExecutionRequest &request,
                                 const RequestRecord &record) const {
  if (request.operation != record.operation ||
      request.supervisor_id != record.supervisor_id ||
      request.expected_epoch != record.expected_epoch ||
      request.proposed_epoch != record.proposed_epoch ||
      request.producer_id != record.producer_id ||
      request.fault_id != record.fault_id) {
    return false;
  }
  if (request.operation != protocol::Operation::Tracking) {
    return true;
  }
  if (!record.has_command) {
    return false;
  }
  const protocol::JointCommand &left = request.first_command;
  const protocol::JointCommand &right = record.first_command;
  if (left.protocol_version != right.protocol_version ||
      left.runtime_id != right.runtime_id ||
      left.session_id != right.session_id ||
      left.producer_id != right.producer_id || left.epoch != right.epoch ||
      left.command_seq != right.command_seq ||
      left.source_state_seq != right.source_state_seq ||
      left.layout_id.View() != right.layout_id.View() ||
      left.joints.count != right.joints.count) {
    return false;
  }
  const std::uint32_t count = std::min(left.joints.count, right.joints.count);
  if (count > protocol::kMaxJoints) {
    return false;
  }
  for (std::uint32_t index = 0; index < count; ++index) {
    if (left.joints.q[index] != right.joints.q[index] ||
        left.joints.dq[index] != right.joints.dq[index] ||
        left.joints.tau_ff[index] != right.joints.tau_ff[index] ||
        left.joints.kp[index] != right.joints.kp[index] ||
        left.joints.kd[index] != right.joints.kd[index]) {
      return false;
    }
  }
  return true;
}

void ExecutionFsm::CommitPendingResults(TickScope &scope, bool write_accepted) {
  for (std::size_t index = 0; index < pending_count_; ++index) {
    PendingResult &pending = pending_[index];
    if (pending.applied) {
      if (!write_accepted) {
        pending.result.status = protocol::ResultStatus::Rejected;
        pending.result.reason = protocol::RejectReason::WriteFailed;
        pending.applied = false;
      } else if (!ImplementsAnswer(pending.result)) {
        // The committed output of this tick does not implement the state the
        // request was applied to: an in-tick fault, revocation or fallback
        // superseded it, so the request is not confirmed.
        pending.result.status = protocol::ResultStatus::Rejected;
        pending.result.reason = protocol::RejectReason::InvalidTransition;
        pending.applied = false;
      }
    }
    // Only the confirmed answer of the staged request becomes the record that a
    // later replay repeats; answers that were never recorded leave it alone.
    if (pending.recorded && staged_record_.valid &&
        staged_record_.request_id == pending.result.request_id) {
      staged_record_.result = pending.result;
    }
    EmitResult(scope, pending.result);
  }
  pending_count_ = 0;

  // A provisional answer is never replayable and never survives its tick as
  // Applied: the record is published only here, after the write committed.
  if (staged_record_.valid) {
    request_record_ = staged_record_;
    staged_record_ = RequestRecord{};
  }
}

bool ExecutionFsm::ImplementsAnswer(
    const protocol::ExecutionResult &answer) const {
  return state_ == answer.execution_state &&
         output_action_ == answer.output_action && epoch_ == answer.epoch &&
         producer_id_ == answer.producer_id;
}

void ExecutionFsm::EmitResult(TickScope &scope,
                              const protocol::ExecutionResult &result) {
  if (scope.out->result_count >= kResultCapacity) {
    ++dropped_results_;
    return;
  }
  scope.out->results[scope.out->result_count++] = result;
}

void ExecutionFsm::EmitEvent(TickScope &scope, EventCode code,
                             protocol::ExecutionState from,
                             protocol::ExecutionState to,
                             protocol::RejectReason reject_reason,
                             protocol::FaultReason fault_reason,
                             protocol::StopReason stop_reason,
                             protocol::Sequence request_id) {
  if (scope.out->event_count >= kEventCapacity) {
    ++dropped_events_;
    return;
  }
  RuntimeEvent event;
  event.stamp_us = scope.now_us;
  event.tick = tick_;
  event.code = code;
  event.from = from;
  event.to = to;
  event.reject_reason = reject_reason;
  event.fault_reason = fault_reason;
  event.stop_reason = stop_reason;
  event.producer_id = producer_id_;
  event.epoch = epoch_;
  event.request_id = request_id;
  scope.out->events[scope.out->event_count++] = event;
}

void ExecutionFsm::EnterState(protocol::ExecutionState state,
                              TickScope &scope) {
  switch (state) {
    case protocol::ExecutionState::Boot:
      EnterBoot(scope);
      return;
    case protocol::ExecutionState::ZeroTorque:
      EnterZeroTorque(scope);
      return;
    case protocol::ExecutionState::Damping:
      EnterDamping(scope);
      return;
    case protocol::ExecutionState::Tracking:
      EnterTracking(scope);
      return;
    case protocol::ExecutionState::Fault:
      EnterFault(scope);
      return;
  }
}

void ExecutionFsm::LeaveState(protocol::ExecutionState state,
                              TickScope &scope) {
  switch (state) {
    case protocol::ExecutionState::Boot:
      LeaveBoot(scope);
      return;
    case protocol::ExecutionState::ZeroTorque:
      LeaveZeroTorque(scope);
      return;
    case protocol::ExecutionState::Damping:
      LeaveDamping(scope);
      return;
    case protocol::ExecutionState::Tracking:
      LeaveTracking(scope);
      return;
    case protocol::ExecutionState::Fault:
      LeaveFault(scope);
      return;
  }
}

void ExecutionFsm::SetState(protocol::ExecutionState next, TickScope &scope,
                            EventCode code) {
  if (next == state_) {
    return;
  }
  const protocol::ExecutionState previous = state_;
  LeaveState(previous, scope);
  state_ = next;
  EnterState(next, scope);
  if (code != EventCode::None) {
    EmitEvent(scope, code, previous, next);
  }
}

hardware::TorqueOutput ExecutionFsm::ComputeStateOutput(TickScope &scope) {
  const protocol::ExecutionState before = state_;
  hardware::TorqueOutput output;
  switch (state_) {
    case protocol::ExecutionState::Boot:
      output = StepBoot(scope);
      break;
    case protocol::ExecutionState::ZeroTorque:
      output = StepZeroTorque(scope);
      break;
    case protocol::ExecutionState::Damping:
      output = StepDamping(scope);
      break;
    case protocol::ExecutionState::Tracking:
      output = StepTracking(scope);
      break;
    case protocol::ExecutionState::Fault:
      output = StepFault(scope);
      break;
  }
  if (state_ == protocol::ExecutionState::Fault &&
      before != protocol::ExecutionState::Fault) {
    // The step latched a fault: the output follows the fault rules instead.
    output = StepFault(scope);
  }
  return output;
}

hardware::TorqueOutput ExecutionFsm::ZeroTorqueOutput() const {
  hardware::TorqueOutput output;
  output.count = static_cast<std::uint32_t>(config_.joint_count());
  return output;
}

protocol::RejectReason ExecutionFsm::CheckStateAllows(
    protocol::ExecutionState state, protocol::Operation operation) const {
  switch (state) {
    case protocol::ExecutionState::Boot:
      return BootAllows(operation);
    case protocol::ExecutionState::ZeroTorque:
      return ZeroTorqueAllows(operation);
    case protocol::ExecutionState::Damping:
      return DampingAllows(operation);
    case protocol::ExecutionState::Tracking:
      return TrackingAllows(operation);
    case protocol::ExecutionState::Fault:
      return FaultAllows(operation);
  }
  return protocol::RejectReason::InvalidTransition;
}

void ExecutionFsm::ClearMotion(TickScope &scope) {
  const bool authorized = gate_.authorized();
  gate_.Revoke();
  producer_id_ = protocol::kNoId;
  if (authorized) {
    EmitEvent(scope, EventCode::AuthorizationRevoked, state_, state_);
  }
}

bool ExecutionFsm::RevokeMotion(TickScope &scope) {
  if (!AdvanceWatermark(scope)) {
    return false;
  }
  ClearMotion(scope);
  return true;
}

bool ExecutionFsm::AdvanceWatermark(TickScope &scope) {
  if (epoch_ == std::numeric_limits<protocol::Sequence>::max()) {
    LatchFault(scope, protocol::FaultReason::ProtocolExhausted,
               EventCode::ProtocolExhausted, false,
               MakeFaultReport(protocol::FaultReason::ProtocolExhausted,
                               scope.now_us, 0,
                               "authorization epoch watermark exhausted"));
    return false;
  }
  ++epoch_;
  return true;
}

protocol::Id ExecutionFsm::GenerateSessionId() {
  protocol::Id id{};
  try {
    std::random_device &entropy = SessionEntropy();
    for (std::size_t attempt = 0; attempt < 4; ++attempt) {
      std::size_t index = 0;
      while (index < id.size()) {
        const std::random_device::result_type value = entropy();
        for (std::size_t byte = 0;
             byte < sizeof(value) && index < id.size(); ++byte, ++index) {
          id[index] =
              static_cast<std::uint8_t>((value >> (8 * byte)) & 0xFFu);
        }
      }
      if (id != protocol::kNoId) {
        return id;
      }
    }
  } catch (const std::exception &) {
    // The session identity is mandatory: fall through to a non-zero value
    // instead of letting an entropy failure escape the control tick.
  }
  id = protocol::kNoId;
  id[protocol::kIdSize - 1] = 1;
  return id;
}

bool ExecutionFsm::LeaseValid(protocol::TimeUs now_us) const {
  return session_id_ != protocol::kNoId &&
         now_us - last_heartbeat_rx_us_ <= config_.timeouts.supervisor_us;
}

void ExecutionFsm::IssueSession(
    TickScope &scope, const protocol::ExecutionRequest &request) {
  session_id_ = GenerateSessionId();
  session_supervisor_id_ = request.supervisor_id;
  last_heartbeat_rx_us_ = scope.now_us;
  last_heartbeat_seq_ = 0;
  heartbeat_seen_ = false;
  request_record_ = RequestRecord{};
  protocol::ExecutionResult result = MakeResult(request, scope.now_us);
  result.status = protocol::ResultStatus::Applied;
  result.reason = protocol::RejectReason::None;
  acquire_record_.valid = true;
  acquire_record_.supervisor_id = request.supervisor_id;
  acquire_record_.request_id = request.request_id;
  acquire_record_.answered_tick = tick_;
  acquire_record_.result = result;
  // Issuing a session commands no torque, so its answer is complete the moment
  // the session exists: it is released without waiting for the write
  // confirmation of this tick, which is what keeps a lost handshake reply
  // retryable even when the send path is failing.
  EmitResult(scope, result);
  EmitEvent(scope, EventCode::SessionAcquired, state_, state_,
            protocol::RejectReason::None, protocol::FaultReason::None,
            protocol::StopReason::None, request.request_id);
}

void ExecutionFsm::ExpireSession(TickScope &scope) {
  if (session_id_ == protocol::kNoId) {
    return;
  }
  session_id_ = protocol::kNoId;
  session_supervisor_id_ = protocol::kNoId;
  last_heartbeat_rx_us_ = 0;
  last_heartbeat_seq_ = 0;
  heartbeat_seen_ = false;
  acquire_record_ = AcquireRecord{};
  request_record_ = RequestRecord{};
  EmitEvent(scope, EventCode::SessionExpired, state_, state_);
  if (!RevokeMotion(scope)) {
    return;
  }
  if (state_ == protocol::ExecutionState::Tracking) {
    const protocol::ExecutionState previous = state_;
    SetState(protocol::ExecutionState::Damping, scope, EventCode::None);
    stop_reason_ = protocol::StopReason::SupervisorTimeout;
    EmitEvent(scope, EventCode::SupervisorTimeout, previous,
              protocol::ExecutionState::Damping,
              protocol::RejectReason::None, protocol::FaultReason::None,
              protocol::StopReason::SupervisorTimeout);
    return;
  }
  // Damping, ZeroTorque and a latched Fault stay where they are.
  stop_reason_ = state_ == protocol::ExecutionState::Fault
                     ? stop_reason_
                     : protocol::StopReason::SupervisorTimeout;
}

bool ExecutionFsm::CheckContinuity(TickScope &scope,
                                  const protocol::JointCommand &command) const {
  const protocol::FeedbackSnapshot &feedback = *scope.feedback;
  const std::size_t joint_count = config_.joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints ||
      feedback.count != joint_count ||
      command.joints.count != joint_count) {
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    if (!(std::fabs(command.joints.q[index] - feedback.joints[index].q) <=
          config_.continuity.position_delta[index])) {
      return false;
    }
    if (!(std::fabs(command.joints.dq[index] - feedback.joints[index].dq) <=
          config_.continuity.velocity_delta[index])) {
      return false;
    }
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double expected_new =
        RawTrackingTorque(command.joints, feedback, index);
    const double expected_old = ExpectedCurrentTorque(feedback, index);
    if (!(std::fabs(expected_new - expected_old) <=
          config_.continuity.torque_delta[index])) {
      return false;
    }
  }
  return true;
}

double ExecutionFsm::ExpectedCurrentTorque(
    const protocol::FeedbackSnapshot &feedback, std::size_t index) const {
  if (state_ == protocol::ExecutionState::Tracking && gate_.has_target()) {
    return RawTrackingTorque(gate_.current_target().joints, feedback, index);
  }
  if (state_ == protocol::ExecutionState::Damping) {
    return -config_.damping_kd[index] * feedback.joints[index].dq;
  }
  return 0.0;
}

}  // namespace irmv::runtime
