#include "irmv/application/behavior_fsm.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace irmv::application {

namespace {

// The one producer id the initialization trajectory and the Ready hold are
// published under. It is a documented constant rather than a value derived from
// the supervisor identity: the frames of one behaviour stage have to be
// recognizable across runs, and the runtime authorizes a producer by the
// session and the epoch it was granted, never by the id alone.
//
// "irmv.init.holder" is exactly the sixteen bytes of an Id.
protocol::Id BehaviorProducerId() {
  constexpr char kName[] = "irmv.init.holder";
  protocol::Id id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(kName[index]);
  }
  return id;
}

// The reason of the first fault detail a state carries; None when it carries
// none.
protocol::FaultReason FaultReasonOf(const protocol::RobotState &state) {
  const std::uint32_t count = std::min<std::uint32_t>(
      state.faults.count, static_cast<std::uint32_t>(protocol::kMaxJoints));
  for (std::uint32_t index = 0; index < count; ++index) {
    if (state.faults.details[index].reason != protocol::FaultReason::None) {
      return state.faults.details[index].reason;
    }
  }
  return protocol::FaultReason::None;
}

// The behaviours whose execution context is a tracking stream.
bool TrackingBehavior(BehaviorState state) {
  return state == BehaviorState::Initializing ||
         state == BehaviorState::Ready || state == BehaviorState::Locomotion;
}

// The instance, the joint layout and the configuration of a state are the ones
// this application was configured for. The client already refuses a snapshot
// that does not match, so this is the second, explicit check the transition
// guard owes the contract.
bool SameDigests(const configuration::ApplicationConfig &config,
                 const protocol::RobotState &state) {
  return state.config_id.View() == config.runtime_config_id() &&
         state.layout_id.View() == config.layout_id();
}

}  // namespace

BehaviorFsm::BehaviorFsm(const configuration::ApplicationConfig &config,
                         RobotClient &client, LocomotionSource source)
    : config_(config),
      client_(client),
      producer_id_(BehaviorProducerId()),
      locomotion_source_(source),
      locomotion_candidate_ttl_us_(
          source == LocomotionSource::External
              ? config.external_producer.prepare_timeout_us
              : config.policy.inference_timeout_us) {}

BehaviorStep BehaviorFsm::Step(protocol::TimeUs now_us, InputEvent event,
                               const ClientView &view) {
  BehaviorStep out;
  // The lower layer's facts, as the last trustworthy state reported them. A
  // state that is known but no longer current is not a present fact: the loop
  // must print Unknown rather than the last remembered enumeration.
  out.runtime_state_known = view.state_known && view.state_fresh;
  if (view.state_known) {
    out.runtime_state = view.state.execution_state;
    out.runtime_output = view.state.output_action;
    out.runtime_stop_reason = view.state.stop_reason;
    out.runtime_fault_reason = FaultReasonOf(view.state);
  }

  // The outcome of a view is edge triggered and the loop may hand one poll to
  // several Steps (one per input event, then a final one): only the first Step
  // to see a given poll may act on its outcome, or a Step would apply an answer
  // to the request the previous Step had just begun. The watermark is
  // monotonic, so a view that is presented again after a newer poll was seen
  // counts as consumed instead of re-enabling an old outcome. The sharp edge:
  // a hand-built ClientView keeps the default poll_seq of 0 and is therefore
  // treated as already consumed.
  const RequestOutcome outcome =
      view.poll_seq > consumed_poll_seq_ ? view.outcome : RequestOutcome::None;
  consumed_poll_seq_ = std::max(consumed_poll_seq_, view.poll_seq);

  // Phase 1 - the fresh lower state and every fault it proves. It runs first
  // because nothing may be decided against a state that is already wrong, and
  // because the faults it latches are what the later phases must respect.
  FreshStateAndFaults(now_us, view);

  // Phase 2 - the stops that do not wait for a confirmation: the operator's
  // emergency stop and the exit. Both displace whatever is pending instead of
  // queueing behind it.
  if (event == InputEvent::EmergencyStop) {
    EmergencyStop(now_us, view);
  }
  if (event == InputEvent::Exit) {
    RequestExit(now_us);
  }
  if (exit_requested_) {
    ExitStep(now_us, view, outcome);
  }

  // Phase 3 - the answer to the pending request, then the ordinary requests and
  // the initialization targets. Once the exit started it owns the iteration:
  // nothing is generated and nothing new is asked for.
  if (!exit_requested_) {
    ConsumeOutcome(now_us, view, outcome);
    // A confirmation whose behaviour the state had not witnessed waits here,
    // before anything new is asked for: the request phase of this iteration is
    // the first one that can see the state that shows it.
    ApplyHeldConfirmation(now_us, view);
    OrdinaryRequests(now_us, event, view, &out);
    // EmitTarget itself refuses while a stop of ours is in flight: the stop may
    // have been begun by an earlier Step of this very iteration, and the target
    // frame that would leave now belongs to the authorization the stop is
    // withdrawing.
    EmitTarget(now_us, view, &out);
  }

  // Phase 4 - the supervision heartbeat, and only after every check above:
  // publishing it is the statement that this iteration ran its checks to the
  // end. Standby, Damping and Fault keep sending it, which is permission to
  // supervise, never permission to move.
  if (client_.session_owned() && HeartbeatDue(now_us)) {
    out.send_heartbeat = true;
    last_heartbeat_us_ = now_us;
    have_heartbeat_ = true;
  }

  out.state = state_;
  out.fault_cause = fault_cause_;
  // Filled last: the reason of a rejection this very iteration consumed is the
  // one to display.
  out.last_reject_reason = last_reject_reason_;
  out.exit_ready = exit_ready_;
  out.exit_stop_confirmed = exit_stop_confirmed_;
  out.locomotion_armed = locomotion_armed_;
  return out;
}

void BehaviorFsm::FreshStateAndFaults(protocol::TimeUs now_us,
                                      const ClientView &view) {
  if (view.state_known) {
    saw_state_ = true;
  }
  const bool fresh = view.state_known && view.state_fresh;

  if (view.runtime_changed) {
    // Another runtime instance: the session, the watermark, the move and the
    // pending request are already worthless in the client, and no request of
    // ours can be addressed to the new instance before it is taken over.
    LatchFault(FaultCause::RuntimeRestarted, now_us, false);
    return;
  }
  if (fresh && view.state.execution_state == protocol::ExecutionState::Fault) {
    // The runtime latched a fault of its own. Its fault identity is kept for
    // the explicit Reset, and its own output rule stays in charge of the
    // machine: the upper asks for nothing that the runtime would only refuse.
    lower_fault_id_ = view.state.fault_id;
    LatchFault(FaultCause::RuntimeFault, now_us, false);
    return;
  }
  if (saw_state_ && !fresh) {
    // Nothing arrived inside the state deadline. Only an unreachable link
    // explains a silent one, and the FSM must never keep presenting the last
    // remembered state as the runtime's present fact. A stop is attempted, but
    // it can never be reported as confirmed.
    LatchFault(FaultCause::RuntimeUnreachable, now_us, true);
    return;
  }
  if (view.session_lost) {
    // The session was withdrawn under a link that still delivers: a lost lease
    // or a takeover. Nothing this FSM commands is authorized any more.
    LatchFault(FaultCause::RuntimeUnreachable, now_us, true);
    return;
  }
  if (!fresh) {
    // Nothing trustworthy has been observed yet: Starting waits for an
    // instance instead of calling the absent link unreachable.
    return;
  }
  if (view.state.execution_state == protocol::ExecutionState::Tracking &&
      state_ != BehaviorState::Starting && !ExpectedTracking(view.state)) {
    // A tracking context nobody here authorized. An unexplained difference is a
    // fault, never a state to compare enumerators on.
    //
    // Starting is the exception: a runtime that was left tracking by a previous
    // application is exactly the takeover this phase exists for, and the
    // takeover stop of the Starting guard is what ends it. Faulting there would
    // dead-end the documented restart scenario.
    LatchFault(FaultCause::UnexpectedContext, now_us, true);
    return;
  }
  if (TrackingBehavior(state_) && !StopPending() &&
      (view.state.execution_state == protocol::ExecutionState::Damping ||
       view.state.execution_state == protocol::ExecutionState::ZeroTorque)) {
    // The lower layer left tracking on its own: its target deadline expired, or
    // the authorization was revoked. A legal, unlatched combination, and the
    // movement is over - the upper cancels the move, gives up on the answer it
    // was waiting for (so nothing keeps resending an authorization the runtime
    // has withdrawn) and shows the lower layer's stop reason instead of
    // claiming Ready. A latched upper fault is not affected: its state is not a
    // tracking behaviour, and the states a move can actually fall back to are
    // the two stops - a lower Boot is not one of them and is never displayed as
    // Damping.
    //
    // The behaviour follows the lower stop state, because the two are not
    // interchangeable: the runtime refuses Tracking from a lower ZeroTorque
    // (InvalidTransition), so a fallback folded into Damping would make the
    // operator's next Initialize build a request that is guaranteed to be
    // rejected. Standby is the upper state whose lower mapping is ZeroTorque,
    // and Damping the one whose lower mapping is Damping.
    CancelInitialization();
    // The authorization is over, and that includes the locomotion source: the
    // frames of a context the runtime no longer holds belong to nothing, and an
    // external producer must be told to stop instead of being left publishing
    // into a withdrawn epoch.
    CancelLocomotion();
    client_.AbandonRequest();
    pending_ = Pending::None;
    state_ = view.state.execution_state == protocol::ExecutionState::ZeroTorque
                 ? BehaviorState::Standby
                 : BehaviorState::Damping;
  }
}

void BehaviorFsm::LatchFault(FaultCause cause, protocol::TimeUs now_us,
                             bool request_stop) {
  // A fault withdraws whatever authorization this behaviour held, the
  // locomotion source included: no frame of it may be published afterwards.
  CancelLocomotion();
  // The first cause wins: a latch is reported once, and a later observation
  // never replaces the identity of what went wrong first.
  const bool first = fault_cause_ == FaultCause::None;
  if (first) {
    // Nothing in flight survives a *new* latch: the move is cancelled and a
    // late answer to a withdrawn request can never revive a behaviour. A latch
    // already in place leaves the requests the latched behaviour itself makes
    // alone: the Reset that leaves it and the handover that reconnects it are
    // this FSM's own decisions, and abandoning them on every iteration a fresh
    // fault state is observed would drop their confirmations - the runtime
    // keeps reporting the same fault until it is reset, so that branch is
    // observed again and again.
    CancelInitialization();
    fault_cause_ = cause;
  }
  // Every latch, new or repeated, invalidates a confirmation that was waiting
  // for a state: the behaviour it would take is not the latched one.
  DropHeldConfirmation();
  state_ = BehaviorState::Fault;

  if (!request_stop) {
    if (first) {
      client_.AbandonRequest();
      pending_ = Pending::None;
    }
    return;
  }
  if (pending_ == Pending::Acquire) {
    // A handover on its way is a reconnection, not a motion request: dropping
    // it would only make this FSM ask for it again on the next iteration.
    return;
  }
  if (StopPending()) {
    // A stop of ours is already on the wire - Damping, ZeroTorque, or the Reset
    // the operator asked for under a latched fault. The runtime answers it by
    // request id and re-asking on every iteration would be a storm nothing
    // needs; for a Reset it would also displace the request the operator made
    // with a Damping the runtime could only refuse.
    return;
  }
  client_.AbandonRequest();
  pending_ = Pending::None;
  BeginDampingStop(now_us);
}

bool BehaviorFsm::BeginDampingStop(protocol::TimeUs now_us) {
  return BeginStop(protocol::Operation::Damping, Pending::Damping, now_us);
}

bool BehaviorFsm::BeginZeroTorqueStop(protocol::TimeUs now_us) {
  return BeginStop(protocol::Operation::ZeroTorque, Pending::ZeroTorque,
                   now_us);
}

bool BehaviorFsm::BeginStop(protocol::Operation operation, Pending kind,
                            protocol::TimeUs now_us) {
  // A stop is the withdrawal of every authorization this behaviour held; the
  // frames of the locomotion source are part of it.
  CancelLocomotion();
  if (client_.request_pending() && pending_operation_ == operation) {
    // A stop of the same operation is already on the wire and owns the
    // decision: the client resends the identical payload inside the deadline.
    // Beginning it again would publish a new request id with a fresh deadline
    // on every key repeat - and a stop that is never answered would then never
    // reach its own timeout.
    return false;
  }
  if (!client_.BeginRequest(operation, now_us)) {
    // Without a live session, or with a request the client cannot address, no
    // stop leaves and none may be claimed.
    return false;
  }
  pending_ = kind;
  pending_operation_ = operation;
  pending_producer_ = protocol::kNoId;
  pending_epoch_ = 0;
  return true;
}

bool BehaviorFsm::BeginAcquire(protocol::TimeUs now_us) {
  if (!client_.BeginRequest(protocol::Operation::AcquireSession, now_us)) {
    return false;
  }
  pending_ = Pending::Acquire;
  pending_operation_ = protocol::Operation::AcquireSession;
  pending_producer_ = protocol::kNoId;
  pending_epoch_ = 0;
  have_acquire_ = true;
  last_acquire_us_ = now_us;
  return true;
}

bool BehaviorFsm::AcquireNeeded(protocol::TimeUs now_us,
                                const ClientView &view) const {
  if (!view.state_known || client_.session_owned() ||
      client_.request_pending()) {
    return false;
  }
  // A handover the runtime refused is retried on the request retry cadence, not
  // on every iteration: a busy session would otherwise be flooded with
  // identical requests.
  if (!have_acquire_) {
    return true;
  }
  return now_us >= last_acquire_us_ &&
         now_us - last_acquire_us_ >= config_.supervision.request_retry_us;
}

void BehaviorFsm::CancelInitialization() {
  // The generator latches its own progress and offers no cancel, so cancelling
  // is rebuilding the (allocation-free) storage in place: a later Start then
  // begins a new trajectory instead of being refused by the old one.
  initialization_.reset();
  tracking_active_ = false;
  tracking_producer_ = protocol::kNoId;
  tracking_epoch_ = 0;
  have_target_ = false;
}

bool BehaviorFsm::StopPending() const {
  return pending_ == Pending::Damping || pending_ == Pending::ZeroTorque ||
         pending_ == Pending::Reset;
}

bool BehaviorFsm::StopInFlight() const {
  // Both halves are durable across the Steps of one iteration: the client still
  // holds the request, and the operation the FSM began last is a stop. Only a
  // stop uses the transport's stop lane, and only while the client has not
  // resolved it may the FSM think a stop is under way.
  return client_.request_pending() && IsStopOperation(pending_operation_);
}

bool BehaviorFsm::ExpectedTracking(const protocol::RobotState &state) const {
  if (TrackingBehavior(state_) && tracking_active_ &&
      state.producer_id == tracking_producer_ &&
      state.epoch == tracking_epoch_) {
    // The confirmed initialization/hold context, which Initializing and Ready
    // share.
    return true;
  }
  // The context the tracking request still on its way proposes.
  if (pending_ == Pending::Tracking &&
      state.producer_id == pending_producer_ && state.epoch == pending_epoch_) {
    return true;
  }
  // The context that was live until that request was proposed: the runtime
  // keeps publishing it until it applies the request, and a state that crossed
  // the request on the wire is the context this behaviour itself asked for a
  // moment ago - not an authorization nobody asked for.
  return have_previous_tracking_ &&
         state.producer_id == previous_tracking_producer_ &&
         state.epoch == previous_tracking_epoch_;
}

void BehaviorFsm::RememberConfirmedTracking() {
  have_previous_tracking_ = tracking_active_;
  previous_tracking_producer_ = tracking_producer_;
  previous_tracking_epoch_ = tracking_epoch_;
}

void BehaviorFsm::DropPreviousTracking() {
  have_previous_tracking_ = false;
  previous_tracking_producer_ = protocol::kNoId;
  previous_tracking_epoch_ = 0;
}

void BehaviorFsm::EmergencyStop(protocol::TimeUs now_us,
                                const ClientView &view) {
  CancelInitialization();
  if (fault_cause_ == FaultCause::None) {
    fault_cause_ = FaultCause::OperatorEmergencyStop;
  }
  state_ = BehaviorState::Fault;
  // The pending request is preempted rather than left to answer into the latch:
  // the stop carries the larger request id that displaces it.
  client_.AbandonRequest();
  pending_ = Pending::None;
  DropHeldConfirmation();
  const bool lower_faulted =
      view.state_known && view.state_fresh &&
      view.state.execution_state == protocol::ExecutionState::Fault;
  if (!lower_faulted) {
    // A runtime whose own fault output rule is already in charge keeps it: a
    // Damping request would only be refused, because a latched fault is left
    // through Reset alone.
    BeginDampingStop(now_us);
  }
}

void BehaviorFsm::RequestExit(protocol::TimeUs now_us) {
  if (exit_requested_) {
    return;
  }
  exit_requested_ = true;
  exit_deadline_us_ = now_us + config_.supervision.request_deadline_us;
  // The exit gives up on whatever was pending: nothing needs the answer of a
  // request the process is leaving behind. It does not clear the tracking
  // context: the lower layer may still report Tracking while the Damping stop
  // this exit asks for is on the wire, and that context is one this FSM
  // authorized - dropping it here would read our own authorization as an
  // unexplained one on the next iteration. Generation is already over: the
  // exit owns the iteration and skips the target phase.
  client_.AbandonRequest();
  pending_ = Pending::None;
  DropHeldConfirmation();
}

void BehaviorFsm::ExitStep(protocol::TimeUs now_us, const ClientView &view,
                           RequestOutcome outcome) {
  const bool fresh = view.state_known && view.state_fresh;

  // A runtime already in Fault keeps its own output rule: there is no Damping
  // to ask for, and the exit only has to report that it proved nothing.
  if (fresh && view.state.execution_state == protocol::ExecutionState::Fault) {
    client_.AbandonRequest();
    pending_ = Pending::None;
    exit_ready_ = true;
    exit_stop_confirmed_ = false;
    return;
  }
  if (outcome == RequestOutcome::Confirmed) {
    const bool stopped =
        pending_ == Pending::Damping && fresh &&
        view.state.execution_state == protocol::ExecutionState::Damping;
    pending_ = Pending::None;
    if (stopped) {
      // The only exit that may be called a damping stop: a fresh state that
      // shows it.
      exit_ready_ = true;
      exit_stop_confirmed_ = true;
      return;
    }
  } else if (outcome == RequestOutcome::Rejected ||
             outcome == RequestOutcome::TimedOut ||
             outcome == RequestOutcome::Unknown) {
    // The stop cannot be confirmed any more. The exit stays bounded and honest:
    // the loop reports Unknown instead of a stop that never happened.
    pending_ = Pending::None;
    exit_ready_ = true;
    exit_stop_confirmed_ = false;
    return;
  }
  // The damping stop is asked for once, and never before the lower layer is
  // known not to be in a fault of its own. The `!exit_ready_` gate matters
  // because the loop may step the same view again after the exit resolved: a
  // terminal resolution is terminal, and re-asking there would publish a fresh
  // request with a new id for a stop the exit no longer waits for.
  if (!exit_ready_ && pending_ == Pending::None && !client_.request_pending()) {
    BeginDampingStop(now_us);
  }
  if (now_us >= exit_deadline_us_) {
    exit_ready_ = true;
    exit_stop_confirmed_ = false;
  }
}

void BehaviorFsm::ConsumeOutcome(protocol::TimeUs now_us,
                                 const ClientView &view,
                                 RequestOutcome outcome) {
  switch (outcome) {
    case RequestOutcome::None:
    case RequestOutcome::Pending:
      return;
    case RequestOutcome::Rejected:
      // The lower layer refused the request, so its invariant still holds: the
      // behaviour is kept and only the reason is recorded for the display.
      last_reject_reason_ = view.reject_reason;
      if (pending_ == Pending::Tracking) {
        // Nothing was authorized, so no frame of that move may keep moving.
        CancelInitialization();
      }
      pending_ = Pending::None;
      return;
    case RequestOutcome::TimedOut:
      // An unconfirmed request is not a request that was never applied: the
      // upper faults and asks for the safest output it can still ask for.
      pending_ = Pending::None;
      LatchFault(FaultCause::RequestTimeout, now_us, true);
      return;
    case RequestOutcome::Unknown:
      // The link went away under a session that looked alive; the client only
      // reports this when it can notice it.
      pending_ = Pending::None;
      LatchFault(FaultCause::RuntimeUnreachable, now_us, true);
      return;
    case RequestOutcome::Confirmed:
      break;
  }

  if (pending_ == Pending::None) {
    return;
  }
  const Pending pending = pending_;
  pending_ = Pending::None;
  if (!(view.state_known && view.state_fresh)) {
    // The request is answered and the client confirmed it; only this Step has
    // no current state to witness the behaviour it authorizes. Dropping it here
    // would lose a transition the runtime really applied, so it waits for the
    // state that shows it - inside the request's own bound.
    HoldConfirmation(pending, now_us);
    return;
  }
  if (ApplyConfirmation(pending, view.state) == Confirmation::Deferred) {
    HoldConfirmation(pending, now_us);
  }
}

BehaviorFsm::Confirmation BehaviorFsm::ApplyConfirmation(
    Pending pending, const protocol::RobotState &state) {
  switch (pending) {
    case Pending::None:
      return Confirmation::NotApplicable;
    case Pending::Acquire:
      // The handover is confirmed by the Starting guard, which also wants the
      // fresh ZeroTorque the runtime reaches on its own.
      return Confirmation::NotApplicable;
    case Pending::Damping:
      if (fault_cause_ != FaultCause::None) {
        // A latch owns the behaviour: the stop it owes the machine is already
        // on its way, and a confirmation cannot unlock anything.
        return Confirmation::Applied;
      }
      if (state.execution_state != protocol::ExecutionState::Damping) {
        return Confirmation::Deferred;
      }
      CancelInitialization();
      state_ = BehaviorState::Damping;
      return Confirmation::Applied;
    case Pending::ZeroTorque:
      if (fault_cause_ != FaultCause::None) {
        return Confirmation::Applied;
      }
      if (state.execution_state != protocol::ExecutionState::ZeroTorque) {
        return Confirmation::Deferred;
      }
      CancelInitialization();
      state_ = BehaviorState::Standby;
      return Confirmation::Applied;
    case Pending::Tracking:
      if (fault_cause_ != FaultCause::None) {
        return Confirmation::Applied;
      }
      if (state.execution_state != protocol::ExecutionState::Tracking) {
        return Confirmation::Deferred;
      }
      if (state.producer_id != tracking_producer_ ||
          state.epoch != tracking_epoch_) {
        // The context the answer authorized is already gone: nothing was
        // started, and no frame may follow it.
        CancelInitialization();
        CancelLocomotion();
        return Confirmation::Applied;
      }
      if (pending_locomotion_) {
        // The authorized context is the locomotion source. Its first frame
        // travelled inside the request; it stays the current target until the
        // loop offers the next one, so the machine is never left without a
        // frame it just authorized.
        pending_locomotion_ = false;
        state_ = BehaviorState::Locomotion;
        // The context that was live until this request is replaced: nothing may
        // publish it any more, so its memory goes with the confirmation.
        DropPreviousTracking();
        return Confirmation::Applied;
      }
      state_ = BehaviorState::Initializing;
      DropPreviousTracking();
      return Confirmation::Applied;
    case Pending::Reset:
      if (state.execution_state != protocol::ExecutionState::ZeroTorque) {
        return Confirmation::Deferred;
      }
      // The only way out of the latch, and only into Standby: the old
      // trajectory, epoch and policy state are not restarted.
      CancelInitialization();
      fault_cause_ = FaultCause::None;
      lower_fault_id_ = 0;
      state_ = BehaviorState::Standby;
      return Confirmation::Applied;
  }
  return Confirmation::NotApplicable;
}

void BehaviorFsm::HoldConfirmation(Pending pending, protocol::TimeUs now_us) {
  if (pending == Pending::None) {
    return;
  }
  // The bound is the request's own final deadline: a transition the state never
  // witnessed must not survive longer than the request that proposed it.
  held_ = pending;
  held_deadline_us_ = now_us + config_.supervision.request_deadline_us;
}

void BehaviorFsm::ApplyHeldConfirmation(protocol::TimeUs now_us,
                                        const ClientView &view) {
  if (held_ == Pending::None) {
    return;
  }
  if (now_us >= held_deadline_us_ || client_.request_pending()) {
    // The bound lapsed, or a newer request now owns the decision this held
    // confirmation would take.
    DropHeldConfirmation();
    return;
  }
  if (!(view.state_known && view.state_fresh)) {
    // Nothing current to judge it against: it keeps waiting for its deadline.
    return;
  }
  if (ApplyConfirmation(held_, view.state) != Confirmation::Deferred) {
    held_ = Pending::None;
  }
}

void BehaviorFsm::DropHeldConfirmation() {
  held_ = Pending::None;
  held_deadline_us_ = 0;
}

void BehaviorFsm::OrdinaryRequests(protocol::TimeUs now_us, InputEvent event,
                                   const ClientView &view, BehaviorStep *out) {
  const bool fresh = view.state_known && view.state_fresh;
  switch (state_) {
    case BehaviorState::Starting: {
      const bool ours =
          fresh && client_.session_owned() && SameDigests(config_, view.state);
      if (ours &&
          view.state.execution_state == protocol::ExecutionState::ZeroTorque) {
        // The instance, the layout, the configuration and the session are ours
        // and the lower layer is at zero torque: the only way out of Starting.
        // Nothing else is a reason to show a behaviour.
        state_ = BehaviorState::Standby;
        break;
      }
      if (ours &&
          (view.state.execution_state == protocol::ExecutionState::Damping ||
           view.state.execution_state == protocol::ExecutionState::Tracking)) {
        // The takeover stop: the runtime kept damping after a previous
        // application left, or was left tracking by one. ZeroTorque is what
        // takes it over, and a fresh ZeroTorque is the only state that promotes
        // the behaviour - a left-over tracking state is never shown as Ready or
        // Locomotion, and the stop is asked for once, not once per iteration.
        BeginZeroTorqueStop(now_us);
        break;
      }
      // A lower Boot, or a session not ours yet, keeps waiting: Starting sends
      // no motion request of its own.
      if (AcquireNeeded(now_us, view)) {
        BeginAcquire(now_us);
      }
      break;
    }
    case BehaviorState::Standby:
      if (event == InputEvent::Damping) {
        BeginDampingStop(now_us);
      }
      // A ZeroTorque key here is a repeated key of the current behaviour.
      break;
    case BehaviorState::Damping:
      if (event == InputEvent::Initialize) {
        TryStartInitialization(now_us, view, out);
      } else if (event == InputEvent::ZeroTorque) {
        BeginZeroTorqueStop(now_us);
      }
      break;
    case BehaviorState::Initializing:
    case BehaviorState::Ready:
    case BehaviorState::Locomotion:
      if (event == InputEvent::Damping) {
        BeginDampingStop(now_us);
      } else if (event == InputEvent::ZeroTorque) {
        BeginZeroTorqueStop(now_us);
      } else if (event == InputEvent::StartLocomotion) {
        // The operator asks for the locomotion behaviour. It is accepted only
        // from Ready - a move in flight is not interrupted by it, and a running
        // locomotion is not restarted - and it only *arms* the source: the
        // request follows once a first frame produced from a fresh state is
        // available, which is what keeps the authorization and its first frame
        // atomic.
        if (state_ == BehaviorState::Ready) {
          locomotion_armed_ = true;
        }
      }
      // An Initialize key is refused here: nothing returns to the
      // initialization pose from a move, and a repeated key must not restart
      // the move either.
      if (state_ == BehaviorState::Ready && locomotion_armed_) {
        // Arming and proposing are separate: the operator's key arms the source
        // and the first frame produced from a fresh state is what turns it into
        // a request. Until then the Ready hold keeps the machine where it is.
        TryStartLocomotion(now_us, view);
      }
      break;
    case BehaviorState::Fault:
      if (event == InputEvent::Reset) {
        TryReset(now_us, view);
      }
      // A latched fault with no session has to reconnect before its Reset can
      // be addressed at all; the handover is the only request it sends.
      if (AcquireNeeded(now_us, view)) {
        BeginAcquire(now_us);
      }
      break;
  }
}

void BehaviorFsm::TryStartInitialization(protocol::TimeUs now_us,
                                         const ClientView &view,
                                         BehaviorStep *out) {
  // One request at a time: a move whose answer is still on its way holds the
  // decision, and asking again would only replace it with a second proposal.
  if (client_.request_pending()) {
    return;
  }
  if (!(view.state_known && view.state_fresh)) {
    // The guard wants a fresh, usable state; what an unusable one means has
    // already been decided by the state deadline's own rule.
    return;
  }
  if (view.state.execution_state != protocol::ExecutionState::Damping) {
    // The mandated path into a move is Standby -> Damping -> Initializing, and
    // the runtime refuses Tracking from a lower ZeroTorque (InvalidTransition):
    // asking from any other lower state would build a request the runtime is
    // guaranteed to reject. The fallback above maps a lower ZeroTorque to
    // Standby, so this is the structural guard behind that mapping.
    return;
  }
  // Neither a rejected request nor a cancelled move is restarted: the generator
  // is rebuilt so that Start admits a new trajectory. The context this FSM has
  // confirmed until now is remembered first: the runtime keeps publishing it
  // until it applies this request, and a state of it that crosses the request on
  // the wire is not an authorization nobody asked for.
  RememberConfirmedTracking();
  CancelInitialization();
  initialization_.emplace(config_);
  const protocol::Sequence epoch = client_.next_epoch();
  protocol::JointCommand first{};
  if (!initialization_->Start(view.state, epoch, producer_id_, now_us,
                              &first)) {
    // A plan the runtime would refuse: stay in Damping, ask for nothing, latch
    // nothing, and let the operator raise the duration or the deadline.
    CancelInitialization();
    out->initialization_refused = true;
    return;
  }
  if (first.epoch != epoch) {
    // The generator and the client disagree about the authorization: sending
    // frames for an epoch no request proposed would only draw a rejection.
    CancelInitialization();
    return;
  }
  if (!client_.BeginRequest(protocol::Operation::Tracking, now_us, producer_id_,
                            &first)) {
    CancelInitialization();
    return;
  }
  if (client_.pending_proposed_epoch() != epoch) {
    // The request on the wire proposes another authorization than the one the
    // generator's frames carry (the client proposes from its own watermark).
    // Frames and request would disagree, so the move is cancelled rather than
    // sent for an epoch the request never asked for.
    client_.AbandonRequest();
    CancelInitialization();
    return;
  }
  // The request proposes exactly the epoch the trajectory was started with, so
  // the first frame and the authorization agree.
  tracking_active_ = true;
  tracking_producer_ = producer_id_;
  tracking_epoch_ = epoch;
  pending_ = Pending::Tracking;
  pending_operation_ = protocol::Operation::Tracking;
  pending_producer_ = producer_id_;
  pending_epoch_ = epoch;
  // The first frame travelled inside the request: the next target is due one
  // target period later.
  have_target_ = true;
  last_target_us_ = now_us;
}

void BehaviorFsm::TryReset(protocol::TimeUs now_us, const ClientView &view) {
  if (fault_cause_ == FaultCause::None) {
    // Only a latched fault is left through Reset.
    return;
  }
  if (!(view.state_known && view.state_fresh) || !client_.session_owned() ||
      client_.request_pending()) {
    // A link that is unknown, or an instance and session not re-established,
    // changes nothing: a Reset is a request addressed to the runtime.
    return;
  }
  const protocol::RobotState &state = view.state;
  if (state.execution_state == protocol::ExecutionState::Fault) {
    if (!state.healthy) {
      // The runtime's recovery window has not opened: it would refuse, and the
      // operator has to ask again once it reports healthy ticks.
      return;
    }
    if (client_.BeginReset(lower_fault_id_, now_us)) {
      pending_ = Pending::Reset;
      pending_operation_ = protocol::Operation::Reset;
      pending_producer_ = protocol::kNoId;
      pending_epoch_ = 0;
    }
    return;
  }
  if (!state.healthy) {
    return;
  }
  // Not in Fault: the runtime is asked for ZeroTorque explicitly. Being in
  // Damping already is not Standby.
  if (BeginZeroTorqueStop(now_us)) {
    pending_ = Pending::Reset;
  }
}

void BehaviorFsm::EmitTarget(protocol::TimeUs now_us, const ClientView &view,
                             BehaviorStep *out) {
  if (state_ == BehaviorState::Locomotion) {
    if (locomotion_source_ == LocomotionSource::External) {
      // The external producer publishes the frames of the context it prepared
      // itself, on the execution topic and under the producer identity the
      // runtime granted. This behaviour republishing the first frame - which is
      // all it would ever have, because no other frame of that source reaches
      // this process - would interleave a second sequence into one stream, and
      // the runtime's command gate is right to refuse that as a discontinuity.
      // The authorization has exactly one writer, and it is not this process.
      return;
    }
    // The locomotion source is the generator, and this behaviour only decides
    // when its newest frame leaves. A frame is published only while the source
    // is still fresh - a stop, a fault or a late result withdraws it - and only
    // while it belongs to the producer and epoch this behaviour authorized.
    if (!have_locomotion_target_ || !locomotion_armed_ || StopInFlight()) {
      return;
    }
    if (now_us - locomotion_target_us_ > config_.policy.inference_timeout_us) {
      return;
    }
    if (locomotion_target_.producer_id != tracking_producer_ ||
        locomotion_target_.epoch != tracking_epoch_) {
      return;
    }
    if (!TargetDue(now_us)) {
      return;
    }
    protocol::JointCommand frame = locomotion_target_;
    // Each published frame carries a new sequence: the runtime's command
    // deadline is refreshed by a strictly increasing sequence, and republishing
    // a frame under its old sequence would not refresh anything.
    frame.command_seq = ++locomotion_seq_;
    last_target_us_ = now_us;
    have_target_ = true;
    out->send_command = true;
    out->command = std::move(frame);
    return;
  }
  if (state_ != BehaviorState::Initializing && state_ != BehaviorState::Ready) {
    return;
  }
  if (StopInFlight()) {
    // A stop of ours is on the wire and still unanswered (or still waiting for
    // the fresh state that confirms it). That is the durable fact, and it holds
    // for every Step of the pending window: a target frame published now would
    // be part of the authorization the stop is withdrawing, and - because the
    // loop hands one poll to several Steps - gating on this iteration's local
    // "a stop was just begun" flag would leave the final Step of the same
    // iteration free to publish exactly that frame.
    return;
  }
  if (!initialization_ || !(view.state_known && view.state_fresh)) {
    return;
  }
  if (!TargetDue(now_us)) {
    return;
  }
  protocol::JointCommand frame{};
  const InitializationStatus status =
      state_ == BehaviorState::Ready
          ? initialization_->Hold(view.state, now_us, &frame)
          : initialization_->Step(view.state, now_us, &frame);
  if (status == InitializationStatus::Failed) {
    if (initialization_->failed()) {
      // The move cannot continue - its deadline passed or a sample it cannot
      // use arrived: the upper faults and asks for damping. A sample the
      // generator merely could not use does not latch anything.
      LatchFault(FaultCause::InitializationFailed, now_us, true);
    }
    return;
  }
  last_target_us_ = now_us;
  have_target_ = true;
  out->send_command = true;
  out->command = std::move(frame);
  if (status == InitializationStatus::Ready &&
      state_ == BehaviorState::Initializing) {
    // The task-3 completion condition is the only thing that makes Ready, and
    // the same generator keeps holding: same gains, same producer, same epoch,
    // a new sequence and the fresh state's sequence.
    state_ = BehaviorState::Ready;
  }
}

bool BehaviorFsm::TargetDue(protocol::TimeUs now_us) const {
  if (!have_target_) {
    return true;
  }
  // A caller clock that is not monotonic must not underflow into a flood of
  // targets.
  return now_us >= last_target_us_ &&
         now_us - last_target_us_ >= config_.supervision.target_period_us;
}

bool BehaviorFsm::HeartbeatDue(protocol::TimeUs now_us) const {
  if (!have_heartbeat_) {
    // The first iteration with a live session refreshes the lease at once.
    return true;
  }
  return now_us >= last_heartbeat_us_ &&
         now_us - last_heartbeat_us_ >= config_.supervision.heartbeat_period_us;
}

void BehaviorFsm::OfferLocomotionCandidate(const protocol::JointCommand &frame,
                                           protocol::TimeUs now_us) {
  // The freshest candidate wins: an older one describes a state that has
  // already been superseded, and the request must carry the newest first frame
  // the source produced.
  locomotion_candidate_ = frame;
  locomotion_candidate_us_ = now_us;
  have_locomotion_candidate_ = true;
}

void BehaviorFsm::OfferLocomotionTarget(const protocol::JointCommand &frame,
                                        protocol::TimeUs now_us) {
  locomotion_target_ = frame;
  locomotion_target_us_ = now_us;
  have_locomotion_target_ = true;
}

void BehaviorFsm::LatchLocalPolicyFault(protocol::TimeUs now_us) {
  LatchFault(FaultCause::LocalPolicyFailed, now_us, true);
}

protocol::Id BehaviorFsm::locomotion_producer() const {
  return pending_locomotion_ || state_ == BehaviorState::Locomotion
             ? tracking_producer_
             : protocol::kNoId;
}

protocol::Sequence BehaviorFsm::locomotion_epoch() const {
  return pending_locomotion_ || state_ == BehaviorState::Locomotion
             ? tracking_epoch_
             : 0;
}

protocol::Sequence BehaviorFsm::locomotion_first_command_seq() const {
  return pending_locomotion_ || state_ == BehaviorState::Locomotion
             ? locomotion_first_seq_
             : 0;
}

void BehaviorFsm::CancelLocomotion() {
  have_locomotion_candidate_ = false;
  have_locomotion_target_ = false;
  locomotion_candidate_us_ = 0;
  locomotion_target_us_ = 0;
  locomotion_armed_ = false;
  locomotion_seq_ = 0;
  locomotion_first_seq_ = 0;
  pending_locomotion_ = false;
  // The context this behaviour held is over, so nothing may still be read as
  // its predecessor: the memory of the replaced context dies with it.
  DropPreviousTracking();
}

void BehaviorFsm::TryStartLocomotion(protocol::TimeUs now_us,
                                     const ClientView &view) {
  if (!have_locomotion_candidate_) {
    return;
  }
  if (now_us - locomotion_candidate_us_ > locomotion_candidate_ttl_us_) {
    // The first frame is older than the source's own deadline: it is the first
    // frame of nothing any more. The supervision loop latches the fault;
    // proposing it here would authorize a frame no state still supports.
    return;
  }
  if (!(view.state_known && view.state_fresh) || client_.request_pending() ||
      StopInFlight()) {
    return;
  }
  const protocol::Sequence epoch = client_.next_epoch();
  protocol::JointCommand first = locomotion_candidate_;
  // The context that is live until this request is applied - the initialization
  // hold of Ready, or the context an earlier request already replaced - is
  // remembered first, for the same reason a move's request remembers it: the
  // runtime keeps publishing it while the request is on its way.
  RememberConfirmedTracking();
  // The source owns the identity the runtime authorizes: the local model is
  // this application, while an external producer is the process whose frames
  // the runtime will gate. A candidate that names no producer is this
  // application's own, and the epoch is the one the request proposes, so the
  // frame and the request that carries it can never disagree.
  const protocol::Id source_producer =
      first.producer_id != protocol::kNoId ? first.producer_id : producer_id_;
  first.producer_id = source_producer;
  first.epoch = epoch;
  first.command_seq = ++locomotion_seq_;
  if (!client_.BeginRequest(protocol::Operation::Tracking, now_us,
                            source_producer, &first)) {
    return;
  }
  if (client_.pending_proposed_epoch() != epoch) {
    // The client proposes from its own watermark and the request on the wire
    // asks for another authorization than the frame carries: Nothing that would
    // be sent after it could match, so the request is abandoned instead.
    client_.AbandonRequest();
    return;
  }
  tracking_active_ = true;
  tracking_producer_ = source_producer;
  tracking_epoch_ = epoch;
  pending_ = Pending::Tracking;
  pending_operation_ = protocol::Operation::Tracking;
  pending_producer_ = source_producer;
  pending_epoch_ = epoch;
  pending_locomotion_ = true;
  locomotion_first_seq_ = first.command_seq;
  // The first frame travelled inside the request and stays the current target
  // until the loop offers the next one, so the authorization is never left
  // without the frame it just adopted.
  locomotion_target_ = first;
  locomotion_target_us_ = locomotion_candidate_us_;
  have_locomotion_target_ = true;
  have_locomotion_candidate_ = false;
  // The first frame is what the request carried: the next target is due one
  // target period later, exactly like the initialization's first frame.
  have_target_ = true;
  last_target_us_ = now_us;
}

}  // namespace irmv::application
