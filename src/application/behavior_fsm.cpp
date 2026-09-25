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
                         RobotClient &client)
    : config_(config), client_(client), producer_id_(BehaviorProducerId()) {}

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
    ExitStep(now_us, view, &out);
  }

  // Phase 3 - the answer to the pending request, then the ordinary requests and
  // the initialization targets. Once the exit started it owns the iteration:
  // nothing is generated and nothing new is asked for.
  if (!exit_requested_) {
    ConsumeOutcome(now_us, view, &out);
    OrdinaryRequests(now_us, event, view, &out);
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
      !ExpectedTracking(view.state)) {
    // A tracking context nobody here authorized. An unexplained difference is a
    // fault, never a state to compare enumerators on.
    LatchFault(FaultCause::UnexpectedContext, now_us, true);
    return;
  }
  if (TrackingBehavior(state_) &&
      view.state.execution_state != protocol::ExecutionState::Tracking &&
      !StopPending()) {
    // The lower layer left tracking on its own: its target deadline expired, or
    // the authorization was revoked. A legal, unlatched combination, and the
    // movement is over - the upper cancels the move, gives up on the answer it
    // was waiting for (so nothing keeps resending an authorization the runtime
    // has withdrawn) and shows the lower layer's stop reason instead of
    // claiming Ready. A latched upper fault is not affected: its state is not a
    // tracking behaviour.
    CancelInitialization();
    client_.AbandonRequest();
    pending_ = Pending::None;
    state_ = BehaviorState::Damping;
  }
}

void BehaviorFsm::LatchFault(FaultCause cause, protocol::TimeUs now_us,
                             bool request_stop) {
  // Nothing in flight survives a latch: the move is cancelled, and a late
  // answer to a withdrawn request can never revive a behaviour.
  CancelInitialization();
  if (fault_cause_ == FaultCause::None) {
    // The first cause wins: a latch is reported once, and a later observation
    // never replaces the identity of what went wrong first.
    fault_cause_ = cause;
  }
  state_ = BehaviorState::Fault;

  if (!request_stop) {
    client_.AbandonRequest();
    pending_ = Pending::None;
    return;
  }
  if (pending_ == Pending::Acquire) {
    // A handover on its way is a reconnection, not a motion request: dropping
    // it would only make this FSM ask for it again on the next iteration.
    return;
  }
  if (StopPending()) {
    // A stop of ours is already on the wire. The runtime answers it by request
    // id, and re-asking on every iteration would be a storm nothing needs.
    return;
  }
  client_.AbandonRequest();
  pending_ = Pending::None;
  BeginStop(protocol::Operation::Damping, now_us);
}

bool BehaviorFsm::BeginStop(protocol::Operation operation,
                            protocol::TimeUs now_us) {
  if (operation != protocol::Operation::Damping &&
      operation != protocol::Operation::ZeroTorque) {
    return false;
  }
  if (!client_.BeginRequest(operation, now_us)) {
    // Without a live session, or with a request the client cannot address, no
    // stop leaves and none may be claimed.
    return false;
  }
  pending_ = operation == protocol::Operation::Damping ? Pending::Damping
                                                       : Pending::ZeroTorque;
  pending_producer_ = protocol::kNoId;
  pending_epoch_ = 0;
  return true;
}

bool BehaviorFsm::BeginAcquire(protocol::TimeUs now_us) {
  if (!client_.BeginRequest(protocol::Operation::AcquireSession, now_us)) {
    return false;
  }
  pending_ = Pending::Acquire;
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

bool BehaviorFsm::ExpectedTracking(const protocol::RobotState &state) const {
  if (TrackingBehavior(state_) && tracking_active_ &&
      state.producer_id == tracking_producer_ &&
      state.epoch == tracking_epoch_) {
    // The confirmed initialization/hold context, which Initializing and Ready
    // share.
    return true;
  }
  // The context the tracking request still on its way proposes.
  return pending_ == Pending::Tracking &&
         state.producer_id == pending_producer_ &&
         state.epoch == pending_epoch_;
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
  const bool lower_faulted =
      view.state_known && view.state_fresh &&
      view.state.execution_state == protocol::ExecutionState::Fault;
  if (!lower_faulted) {
    // A runtime whose own fault output rule is already in charge keeps it: a
    // Damping request would only be refused, because a latched fault is left
    // through Reset alone.
    BeginStop(protocol::Operation::Damping, now_us);
  }
}

void BehaviorFsm::RequestExit(protocol::TimeUs now_us) {
  if (exit_requested_) {
    return;
  }
  exit_requested_ = true;
  exit_deadline_us_ = now_us + config_.supervision.request_deadline_us;
  // The exit stops generating and gives up on whatever was pending: nothing
  // needs the answer of a request the process is leaving behind.
  CancelInitialization();
  client_.AbandonRequest();
  pending_ = Pending::None;
}

void BehaviorFsm::ExitStep(protocol::TimeUs now_us, const ClientView &view,
                           BehaviorStep *out) {
  static_cast<void>(out);
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
  if (view.outcome == RequestOutcome::Confirmed) {
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
  } else if (view.outcome == RequestOutcome::Rejected ||
             view.outcome == RequestOutcome::TimedOut ||
             view.outcome == RequestOutcome::Unknown) {
    // The stop cannot be confirmed any more. The exit stays bounded and honest:
    // the loop reports Unknown instead of a stop that never happened.
    pending_ = Pending::None;
    exit_ready_ = true;
    exit_stop_confirmed_ = false;
    return;
  }
  // The damping stop is asked for once, and never before the lower layer is
  // known not to be in a fault of its own.
  if (pending_ == Pending::None && !client_.request_pending()) {
    BeginStop(protocol::Operation::Damping, now_us);
  }
  if (now_us >= exit_deadline_us_) {
    exit_ready_ = true;
    exit_stop_confirmed_ = false;
  }
}

void BehaviorFsm::ConsumeOutcome(protocol::TimeUs now_us,
                                 const ClientView &view, BehaviorStep *out) {
  static_cast<void>(out);
  switch (view.outcome) {
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
    // The client confirms only against a state, so a confirmation without a
    // current one is a caller that did not poll. Nothing is applied.
    return;
  }
  const protocol::RobotState &state = view.state;
  switch (pending) {
    case Pending::None:
      return;
    case Pending::Acquire:
      // The handover is confirmed by the Starting guard, which also wants the
      // fresh ZeroTorque the runtime reaches on its own.
      return;
    case Pending::Damping:
      if (fault_cause_ == FaultCause::None &&
          state.execution_state == protocol::ExecutionState::Damping) {
        CancelInitialization();
        state_ = BehaviorState::Damping;
      }
      return;
    case Pending::ZeroTorque:
      if (fault_cause_ == FaultCause::None &&
          state.execution_state == protocol::ExecutionState::ZeroTorque) {
        CancelInitialization();
        state_ = BehaviorState::Standby;
      }
      return;
    case Pending::Tracking:
      if (fault_cause_ == FaultCause::None &&
          state.execution_state == protocol::ExecutionState::Tracking &&
          state.producer_id == tracking_producer_ &&
          state.epoch == tracking_epoch_) {
        state_ = BehaviorState::Initializing;
      } else {
        // The context the answer authorized is already gone: nothing was
        // started, and no frame may follow it.
        CancelInitialization();
      }
      return;
    case Pending::Reset:
      if (state.execution_state == protocol::ExecutionState::ZeroTorque) {
        // The only way out of the latch, and only into Standby: the old
        // trajectory, epoch and policy state are not restarted.
        CancelInitialization();
        fault_cause_ = FaultCause::None;
        lower_fault_id_ = 0;
        state_ = BehaviorState::Standby;
      }
      return;
  }
}

void BehaviorFsm::OrdinaryRequests(protocol::TimeUs now_us, InputEvent event,
                                   const ClientView &view, BehaviorStep *out) {
  const bool fresh = view.state_known && view.state_fresh;
  switch (state_) {
    case BehaviorState::Starting:
      if (fresh && client_.session_owned() &&
          view.state.execution_state == protocol::ExecutionState::ZeroTorque &&
          SameDigests(config_, view.state)) {
        // The instance, the layout and the configuration match, the session is
        // ours and the lower layer is at zero torque: the only way out of
        // Starting. Nothing else is a reason to show a behaviour.
        state_ = BehaviorState::Standby;
        break;
      }
      if (AcquireNeeded(now_us, view)) {
        BeginAcquire(now_us);
      }
      break;
    case BehaviorState::Standby:
      if (event == InputEvent::Damping) {
        BeginStop(protocol::Operation::Damping, now_us);
      }
      // A ZeroTorque key here is a repeated key of the current behaviour.
      break;
    case BehaviorState::Damping:
      if (event == InputEvent::Initialize) {
        TryStartInitialization(now_us, view, out);
      } else if (event == InputEvent::ZeroTorque) {
        BeginStop(protocol::Operation::ZeroTorque, now_us);
      }
      break;
    case BehaviorState::Initializing:
    case BehaviorState::Ready:
    case BehaviorState::Locomotion:
      // Locomotion is unreachable in this phase, but its stop rules are the
      // rules of every behaviour that holds an authorization.
      if (event == InputEvent::Damping) {
        BeginStop(protocol::Operation::Damping, now_us);
      } else if (event == InputEvent::ZeroTorque) {
        BeginStop(protocol::Operation::ZeroTorque, now_us);
      }
      // An Initialize key is refused here: nothing returns to the
      // initialization pose from a move, and a repeated key must not restart
      // the move either.
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
  // Neither a rejected request nor a cancelled move is restarted: the generator
  // is rebuilt so that Start admits a new trajectory.
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
  // The request proposes exactly the epoch the trajectory was started with, so
  // the first frame and the authorization agree.
  tracking_active_ = true;
  tracking_producer_ = producer_id_;
  tracking_epoch_ = epoch;
  pending_ = Pending::Tracking;
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
  if (BeginStop(protocol::Operation::ZeroTorque, now_us)) {
    pending_ = Pending::Reset;
  }
}

void BehaviorFsm::EmitTarget(protocol::TimeUs now_us, const ClientView &view,
                             BehaviorStep *out) {
  if (state_ != BehaviorState::Initializing && state_ != BehaviorState::Ready) {
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

bool BehaviorFsm::StartLocomotion() {
  // This phase ships no locomotion target generator, so there is no execution
  // context to propose, nothing to prepare and nothing to change: the call
  // reports unavailable and the behaviour stays where it is, Ready included.
  return false;
}

}  // namespace irmv::application
