// The behaviour FSM and the serial supervision order.
//
// Every case runs on explicit simulated times: the harness drives a real
// RobotClient over a recording ClientSink, the runtime's answers and states are
// injected by hand exactly where the lower layer would produce them, and the
// FSM is stepped with the view a Poll produced. No clock is read, nothing
// sleeps, no thread and no DDS takes part.
//
// The assertions are as explicit about the latch and about the absence of a
// change as they are about the positive transitions: a behaviour only moves on
// a matching confirmation, and a latched fault is never unlocked by a stop, a
// timeout, a heartbeat or a damping observation.

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

#include "irmv/application/behavior_fsm.hpp"
#include "irmv/application/initialization.hpp"
#include "irmv/application/robot_client.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::BehaviorFsm;
using irmv::application::BehaviorState;
using irmv::application::BehaviorStep;
using irmv::application::ClientSink;
using irmv::application::ClientView;
using irmv::application::FaultCause;
using irmv::application::InputEvent;
using irmv::application::RequestOutcome;
using irmv::application::RobotClient;
using irmv::configuration::ApplicationConfig;
using irmv::configuration::LoadApplicationConfig;
using irmv::protocol::ExecutionRequest;
using irmv::protocol::ExecutionResult;
using irmv::protocol::ExecutionState;
using irmv::protocol::FaultReason;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::kNoId;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::Operation;
using irmv::protocol::OutputAction;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;

// Simulated instants. The harness never reads a clock: everything below is
// derived from these and from the configuration's periods.
constexpr TimeUs kStartUs = 1'000'000;
constexpr TimeUs kTickUs = 1'000;    // one supervision iteration
constexpr TimeUs kAnswerUs = 2'000;  // an answer lands this long after a Step
constexpr TimeUs kStateAgeUs = 500;  // the runtime's own publication age

Id MakeId(std::uint8_t seed) {
  Id id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

const Id kRuntimeId = MakeId(0x20);
const Id kGrantedSession = MakeId(0x40);

// The position the runtime reports while the cases do not move it: away from
// the configured target, inside its position limits and inside the continuity
// and velocity bounds of the first frame.
constexpr double kMeasuredPosition = -0.5;

// Records every payload that leaves the loop: the requests the FSM decided on,
// the frames and the heartbeats the client published with them.
class RecordingSink : public ClientSink {
 public:
  bool PublishRequest(const ExecutionRequest &request) override {
    requests.push_back(request);
    return true;
  }

  bool PublishCommand(const JointCommand &command) override {
    commands.push_back(command);
    return true;
  }

  bool PublishHeartbeat(
      const irmv::protocol::SupervisorHeartbeat &heartbeat) override {
    heartbeats.push_back(heartbeat);
    return true;
  }

  std::vector<ExecutionRequest> requests;
  std::vector<JointCommand> commands;
  std::vector<irmv::protocol::SupervisorHeartbeat> heartbeats;
};

// The runtime's present facts, as the harness reports them. A case mutates this
// to move the runtime the way the lower layer would.
struct LowerState {
  ExecutionState state{ExecutionState::ZeroTorque};
  OutputAction output{OutputAction::ZeroTorque};
  Sequence epoch{0};
  Id producer{};
  Sequence fault_id{0};
  FaultReason fault_reason{FaultReason::None};
  bool healthy{true};
  double q{kMeasuredPosition};
  double dq{0.0};
  StopReason stop_reason{StopReason::None};
};

// A whole application: the configuration, the recording sink, a real client
// over it, the FSM, and the runtime the cases drive.
class Harness {
 public:
  Harness()
      : config_(LoadApplicationConfig("config/sdk.json")),
        client_(sink_, config_, MakeId(0x10)),
        fsm_(config_, client_) {}

  const ApplicationConfig &config() const { return config_; }
  RobotClient &client() { return client_; }
  BehaviorFsm &fsm() { return fsm_; }
  RecordingSink &sink() { return sink_; }
  TimeUs now_us() const { return now_; }
  // The state the runtime published last, as the FSM saw it.
  const RobotState &published() const { return published_; }
  const Id &granted_session() const { return session_id_; }
  const ExecutionRequest *last_request() const {
    return sink_.requests.empty() ? nullptr : &sink_.requests.back();
  }
  const ExecutionRequest *last_request_of(Operation operation) const {
    for (std::size_t index = sink_.requests.size(); index > 0; --index) {
      if (sink_.requests[index - 1].operation == operation) {
        return &sink_.requests[index - 1];
      }
    }
    return nullptr;
  }

  // The runtime's present state and whether the next iterations observe it. A
  // silent link is exactly the case a Stop is not published in.
  LowerState lower;
  bool publish{true};

  // One supervision iteration, plus the four lines the application loop owns:
  // advance the clock, hand the client what the transport received, poll once,
  // step, and send exactly what the step asked for.
  BehaviorStep Step(InputEvent event = InputEvent::None,
                    TimeUs advance = kTickUs) {
    return StepView(PollOnce(advance), event);
  }

  // Advances the clock, hands the client the runtime's state and polls once.
  // The loop does this once per iteration and then runs a Step per input event
  // on the view it got, which is why a case can reuse the view.
  ClientView PollOnce(TimeUs advance = kTickUs) {
    now_ += advance;
    if (publish) {
      published_ = State(now_ - kStateAgeUs);
      client_.ObserveState(published_, now_);
    }
    return client_.Poll(now_);
  }

  // One Step against a view the caller already polled, plus the sends the loop
  // owns.
  BehaviorStep StepView(const ClientView &view,
                        InputEvent event = InputEvent::None) {
    BehaviorStep step = fsm_.Step(now_, event, view);
    if (step.send_command) {
      client_.SendCommand(&step.command, now_);
    }
    if (step.send_heartbeat) {
      client_.SendHeartbeat(now_);
    }
    return step;
  }

  ClientView Poll() { return client_.Poll(now_); }

  // One state of the runtime's present facts, stamped by the runtime itself.
  RobotState State(TimeUs stamp_us) {
    RobotState state{};
    state.protocol_version = kProtocolVersion;
    state.runtime_id = kRuntimeId;
    state.session_id = session_id_;
    state.producer_id = lower.producer;
    state.epoch = lower.epoch;
    state.state_seq = ++state_seq_;
    state.stamp_us = stamp_us;
    state.execution_state = lower.state;
    state.output_action = lower.output;
    state.stop_reason = lower.stop_reason;
    state.fault_id = lower.fault_id;
    state.healthy = lower.healthy;
    state.damping_eligible = lower.output == OutputAction::Damping;
    state.config_id.Assign(config_.runtime_config_id());
    state.layout_id.Assign(config_.layout_id());
    // Deliberately not the state sequence: a target must carry the sequence the
    // runtime records for the state, never the feedback snapshot's.
    state.feedback.snapshot_seq = state.state_seq + 5'000;
    state.feedback.stamp_us = stamp_us;
    state.feedback.count = static_cast<std::uint32_t>(config_.joint_count());
    for (std::size_t index = 0; index < config_.joint_count(); ++index) {
      irmv::protocol::JointFeedback &joint = state.feedback.joints[index];
      joint.valid = true;
      joint.connected = true;
      joint.q = lower.q;
      joint.dq = lower.dq;
      joint.rx_stamp_us = stamp_us;
      joint.freq_hz = 500.0;
    }
    if (lower.fault_reason != FaultReason::None) {
      state.faults.count = 1;
      state.faults.details[0].reason = lower.fault_reason;
      state.faults.details[0].output_action = lower.output;
      state.faults.details[0].detected_stamp_us = stamp_us;
    }
    return state;
  }

  // Puts the runtime into the tracking context the last request proposed, the
  // way a runtime that applied a Tracking request reports it.
  void EnterTracking() {
    const ExecutionRequest &request = sink_.requests.back();
    tracking_producer_value = request.producer_id;
    tracking_epoch_value = request.proposed_epoch;
    lower.state = ExecutionState::Tracking;
    lower.output = OutputAction::Tracking;
    lower.producer = request.producer_id;
    lower.epoch = request.proposed_epoch;
  }

  const Id &tracking_producer() const { return tracking_producer_value; }
  Sequence tracking_epoch() const { return tracking_epoch_value; }

  // The result the runtime would answer the last request with, built from the
  // runtime's present state.
  ExecutionResult Answered(const ExecutionRequest &request, ResultStatus status,
                           RejectReason reason) {
    ExecutionResult result{};
    result.protocol_version = kProtocolVersion;
    result.supervisor_id = request.supervisor_id;
    result.request_id = request.request_id;
    result.session_id =
        request.session_id == kNoId ? session_id_ : request.session_id;
    result.runtime_id = request.runtime_id;
    result.execution_state = lower.state;
    result.output_action = lower.output;
    result.producer_id = lower.producer;
    result.epoch = lower.epoch;
    result.fault_id = lower.fault_id;
    result.stamp_us = now_ + kAnswerUs;
    result.status = status;
    result.reason = reason;
    return result;
  }

  // Feeds only the Applied result of the last request: the confirmation still
  // needs a state published after it.
  bool AnswerResultOnly() {
    if (sink_.requests.empty()) {
      return false;
    }
    const ExecutionRequest request = sink_.requests.back();
    if (request.operation == Operation::AcquireSession) {
      session_id_ = kGrantedSession;
    }
    const ExecutionResult result =
        Answered(request, ResultStatus::Applied, RejectReason::None);
    client_.ObserveResult(result, result.stamp_us + 1);
    now_ = result.stamp_us + kTickUs;
    return true;
  }

  // Answers the last request with its Applied result and a state published
  // after it that agrees with it, so the client's next Poll confirms it.
  bool Answer() {
    if (!AnswerResultOnly()) {
      return false;
    }
    published_ = State(now_ - kStateAgeUs);
    client_.ObserveState(published_, now_);
    return true;
  }

  // Answers the last request with a rejection: no state witnesses it.
  bool Reject(RejectReason reason) {
    if (sink_.requests.empty()) {
      return false;
    }
    const ExecutionRequest request = sink_.requests.back();
    const ExecutionResult result =
        Answered(request, ResultStatus::Rejected, reason);
    client_.ObserveResult(result, result.stamp_us + 1);
    now_ = result.stamp_us + kTickUs;
    return true;
  }

 private:
  ApplicationConfig config_;
  RecordingSink sink_;
  RobotClient client_;
  BehaviorFsm fsm_;
  TimeUs now_{kStartUs};
  Id session_id_{kNoId};
  Sequence state_seq_{0};
  RobotState published_{};
  Id tracking_producer_value{};
  Sequence tracking_epoch_value{0};
};

// Power-on to Standby: nothing is requested before an instance this client can
// talk to has been observed, the session handover is confirmed, and only a
// fresh ZeroTorque promotes the behaviour.
int ToStandby(Harness &h) {
  h.publish = false;
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Starting);
  CHECK(h.sink().requests.empty());
  CHECK(!step.send_command);
  CHECK(!step.send_heartbeat);
  CHECK(!step.runtime_state_known);

  // The matching instance appears: the FSM asks for the session handover, and
  // the handover alone does not promote it.
  h.publish = true;
  step = h.Step();
  CHECK(step.state == BehaviorState::Starting);
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::AcquireSession);
  CHECK(request->session_id == kNoId);
  CHECK(request->runtime_id == kRuntimeId);
  CHECK(request->producer_id == kNoId);
  CHECK(request->expected_epoch == 0);
  step = h.Step();
  CHECK(step.state == BehaviorState::Starting);
  CHECK(h.sink().requests.size() ==
        1);  // the client retries, the FSM re-asks no

  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  CHECK(step.runtime_state_known);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(!step.send_command);
  // A live session now refreshes the lease.
  CHECK(step.send_heartbeat);
  CHECK(h.sink().heartbeats.size() == 1);
  return 0;
}

// Standby to Damping, only through the matching confirmation.
int ToDamping(Harness &h) {
  BehaviorStep step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Standby);
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Damping);
  CHECK(request->session_id == h.granted_session());
  CHECK(request->producer_id == kNoId);
  CHECK(!step.send_command);
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.runtime_state == ExecutionState::Damping);
  return 0;
}

// Damping to Initializing, only through the matching confirmation, and with the
// admitted first frame travelling inside the request.
int ToInitializing(Harness &h) {
  BehaviorStep step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Damping);
  CHECK(!step.initialization_refused);
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Tracking);
  CHECK(request->producer_id != kNoId);
  CHECK(request->proposed_epoch > request->expected_epoch);
  // The authorization the request proposes is exactly the one the generator's
  // frames carry.
  CHECK(h.client().pending_proposed_epoch() == request->proposed_epoch);
  const JointCommand &frame = request->first_command;
  CHECK(frame.joints.count ==
        static_cast<std::uint32_t>(h.config().joint_count()));
  CHECK(frame.layout_id.View() == h.config().layout_id());
  CHECK(frame.producer_id == request->producer_id);
  CHECK(frame.epoch == request->proposed_epoch);
  CHECK(frame.source_state_seq == h.published().state_seq);
  for (std::size_t index = 0; index < h.config().joint_count(); ++index) {
    CHECK(frame.joints.q[index] == h.lower.q);
    CHECK(frame.joints.dq[index] == 0.0);
    CHECK(frame.joints.tau_ff[index] == 0.0);
    CHECK(frame.joints.kp[index] == h.config().initialization.gains.kp[index]);
    CHECK(frame.joints.kd[index] == h.config().initialization.gains.kd[index]);
  }
  h.EnterTracking();
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Initializing);
  CHECK(step.runtime_state == ExecutionState::Tracking);
  return 0;
}

// Standby's Damping and Initializing to a completed move: the ramp runs, the
// joints reach the target and hold it for the stable window.
int ToReady(Harness &h) {
  CHECK(ToDamping(h) == 0);
  CHECK(ToInitializing(h) == 0);
  const TimeUs period = h.config().supervision.target_period_us;
  const TimeUs duration = h.config().initialization.trajectory.duration_us;
  const TimeUs window = h.config().initialization.trajectory.stable_window_us;
  const int limit = static_cast<int>(duration / period + window / period) + 8;
  for (int index = 0; index < limit; ++index) {
    if (static_cast<TimeUs>(index) * period >= duration) {
      // The joints arrive and hold the target.
      h.lower.q = h.config().initialization.target_q[0];
    }
    const BehaviorStep step = h.Step(InputEvent::None, period);
    if (step.state == BehaviorState::Ready) {
      return 0;
    }
    CHECK(step.state == BehaviorState::Initializing);
  }
  CHECK(false);
  return 1;
}

// The power-on handover, the promotion guard and a foreign instance.
int StartingHandover() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  // The handover is the only request a Starting FSM makes: no motion request is
  // sent before it is confirmed.
  CHECK(h.sink().requests.size() == 1);

  // A session is not enough: the promotion also wants the lower layer at zero
  // torque, and a runtime that it finds damping is taken over with an explicit
  // ZeroTorque stop first.
  {
    Harness takeover;
    takeover.lower.state = ExecutionState::Damping;
    takeover.lower.output = OutputAction::Damping;
    BehaviorStep step = takeover.Step();
    CHECK(step.state == BehaviorState::Starting);
    const ExecutionRequest *request = takeover.last_request();
    CHECK(request != nullptr);
    if (request == nullptr) {
      return 1;
    }
    CHECK(request->operation == Operation::AcquireSession);

    // The handover is answered, and the FSM then asks for ZeroTorque rather
    // than waiting for a stop it never asked for.
    CHECK(takeover.Answer());
    const std::size_t requests = takeover.sink().requests.size();
    step = takeover.Step();
    CHECK(step.state == BehaviorState::Starting);
    CHECK(step.runtime_state == ExecutionState::Damping);
    CHECK(step.runtime_state_known);
    CHECK(!step.send_command);
    const ExecutionRequest *stop = takeover.last_request();
    CHECK(stop != nullptr);
    if (stop == nullptr) {
      return 1;
    }
    CHECK(stop->operation == Operation::ZeroTorque);
    CHECK(stop->session_id == takeover.granted_session());
    CHECK(takeover.sink().requests.size() == requests + 1);
    // The stop is asked for once, not once per iteration.
    step = takeover.Step(InputEvent::None, 3 * kTickUs);
    CHECK(step.state == BehaviorState::Starting);
    CHECK(takeover.sink().requests.size() == requests + 1);

    takeover.lower.state = ExecutionState::ZeroTorque;
    takeover.lower.output = OutputAction::ZeroTorque;
    CHECK(takeover.Answer());
    step = takeover.Step();
    CHECK(step.state == BehaviorState::Standby);
    CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  }

  // The same takeover from a runtime a previous application left tracking: the
  // behaviour never shows Ready or Locomotion on the way.
  {
    Harness left_tracking;
    left_tracking.lower.state = ExecutionState::Tracking;
    left_tracking.lower.output = OutputAction::Tracking;
    left_tracking.lower.producer = MakeId(0x77);
    left_tracking.lower.epoch = 9;
    BehaviorStep step = left_tracking.Step();
    CHECK(step.state == BehaviorState::Starting);
    const ExecutionRequest *request = left_tracking.last_request();
    CHECK(request != nullptr);
    if (request == nullptr) {
      return 1;
    }
    CHECK(request->operation == Operation::AcquireSession);
    CHECK(left_tracking.Answer());
    step = left_tracking.Step();
    CHECK(step.state == BehaviorState::Starting);
    CHECK(step.runtime_state == ExecutionState::Tracking);
    const ExecutionRequest *stop = left_tracking.last_request();
    CHECK(stop != nullptr);
    if (stop == nullptr) {
      return 1;
    }
    CHECK(stop->operation == Operation::ZeroTorque);

    left_tracking.lower.state = ExecutionState::ZeroTorque;
    left_tracking.lower.output = OutputAction::ZeroTorque;
    left_tracking.lower.producer = kNoId;
    left_tracking.lower.epoch = 0;
    CHECK(left_tracking.Answer());
    step = left_tracking.Step();
    CHECK(step.state == BehaviorState::Standby);
    CHECK(step.fault_cause == FaultCause::None);
  }

  // A runtime still booting is not a takeover: the behaviour keeps waiting and
  // asks for nothing but the handover.
  {
    Harness booting;
    booting.lower.state = ExecutionState::Boot;
    booting.lower.output = OutputAction::ZeroTorque;
    BehaviorStep step = booting.Step();
    CHECK(step.state == BehaviorState::Starting);
    CHECK(booting.Answer());
    const std::size_t requests = booting.sink().requests.size();
    step = booting.Step();
    CHECK(step.state == BehaviorState::Starting);
    CHECK(step.runtime_state == ExecutionState::Boot);
    CHECK(step.runtime_state_known);
    CHECK(!step.send_command);
    CHECK(booting.sink().requests.size() == requests);
    // and it is never displayed as a stop the upper layer did not ask for.
    step = booting.Step(InputEvent::None, 2 * kTickUs);
    CHECK(step.state == BehaviorState::Starting);
    CHECK(step.runtime_state == ExecutionState::Boot);
  }

  // A state whose configuration is not ours is never this client's target, so
  // the handover cannot even begin.
  {
    Harness foreign;
    foreign.publish = false;
    RobotState other = foreign.State(foreign.now_us() - kStateAgeUs);
    CHECK(other.config_id.Assign("someone-elses-config"));
    foreign.client().ObserveState(other, foreign.now_us());
    BehaviorStep step = foreign.Step();
    CHECK(step.state == BehaviorState::Starting);
    CHECK(!step.runtime_state_known);
    CHECK(foreign.sink().requests.empty());
    CHECK(!step.send_command);
  }

  // A link that goes silent is unreachable, in Starting exactly as anywhere
  // else: the state deadline is the rule.
  {
    Harness silent;
    CHECK(ToStandby(silent) == 0);
    silent.publish = false;
    const BehaviorStep step = silent.Step(
        InputEvent::None, silent.config().supervision.state_timeout_us + 1);
    CHECK(step.state == BehaviorState::Fault);
    CHECK(step.fault_cause == FaultCause::RuntimeUnreachable);
  }
  return 0;
}

// Standby and Damping only move on a matching confirmation, and the global
// ZeroTorque stop leaves through Standby.
int StopConfirmations() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  BehaviorStep step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Standby);
  const ExecutionRequest *damping = h.last_request();
  CHECK(damping != nullptr);
  if (damping == nullptr) {
    return 1;
  }
  CHECK(damping->operation == Operation::Damping);
  CHECK(damping->session_id == h.granted_session());
  CHECK(damping->runtime_id == kRuntimeId);
  CHECK(!step.send_command);

  // The Applied result alone is not a confirmation: the behaviour waits for the
  // state the runtime publishes after it.
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.AnswerResultOnly());
  CHECK(h.Poll().outcome == RequestOutcome::Pending);
  CHECK(h.fsm().state() == BehaviorState::Standby);
  step = h.Step();
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.runtime_state == ExecutionState::Damping);
  CHECK(step.runtime_output == OutputAction::Damping);
  CHECK(step.fault_cause == FaultCause::None);

  // A repeated Damping key in Damping is a no-op.
  const std::size_t requests = h.sink().requests.size();
  step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Damping);
  CHECK(h.sink().requests.size() == requests);

  // The global explicit ZeroTorque stop leaves from any non-fault behaviour and
  // lands in Standby, never in Damping.
  step = h.Step(InputEvent::ZeroTorque);
  CHECK(step.state == BehaviorState::Damping);  // not before the confirmation
  const ExecutionRequest *zero = h.last_request();
  CHECK(zero != nullptr);
  if (zero == nullptr) {
    return 1;
  }
  CHECK(zero->operation == Operation::ZeroTorque);
  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  CHECK(step.fault_cause == FaultCause::None);

  // A repeated ZeroTorque key in Standby is a no-op.
  const std::size_t after = h.sink().requests.size();
  step = h.Step(InputEvent::ZeroTorque);
  CHECK(step.state == BehaviorState::Standby);
  CHECK(h.sink().requests.size() == after);
  return 0;
}

// The confirmation the client accepted can be older than the newest state: the
// FSM re-checks that the state it is about to act on shows the behaviour it
// asked for, so an answer the runtime has already overtaken leaves the
// behaviour where it was.
int ConfirmationOvertakenByTheState() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  BehaviorStep step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Standby);

  // The runtime applies the damping and publishes the witness...
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.Answer());
  // ... and it is somewhere else again before the loop reads the answer.
  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  const RobotState moved = h.State(h.now_us() + kTickUs);
  h.client().ObserveState(moved, h.now_us() + kTickUs);

  step = h.Step(InputEvent::None, 2 * kTickUs);
  // The request was answered and resolved; what the FSM refuses to do is act on
  // a state that does not show the behaviour.
  CHECK(!h.client().request_pending());
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  CHECK(step.runtime_state_known);
  return 0;
}

// The loop polls once per iteration and then runs a Step per input event on
// that one view. The outcome is edge triggered, so only the first Step to see a
// poll may act on it - otherwise the second Step applies an answer to the
// request the first one had just begun, and a live stop is left unanswered.
int OneViewManySteps() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  // The runtime applies the damping the operator asked for.
  BehaviorStep step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Standby);
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.Answer());

  // The operator's next key is already queued when the loop polls.
  const ClientView view = h.PollOnce();
  const std::size_t requests = h.sink().requests.size();
  step = h.StepView(view, InputEvent::ZeroTorque);
  CHECK(step.state == BehaviorState::Damping);  // the answer was applied
  CHECK(h.sink().requests.size() == requests + 1);
  const ExecutionRequest *stop = h.last_request();
  CHECK(stop != nullptr);
  if (stop == nullptr) {
    return 1;
  }
  CHECK(stop->operation == Operation::ZeroTorque);
  const Sequence stop_id = stop->request_id;
  const std::size_t commands = h.sink().commands.size();
  const std::size_t heartbeats = h.sink().heartbeats.size();

  // The second Step on the same view: the answer is already consumed, so it may
  // not be applied to the stop the first Step just began, nothing new may be
  // asked for, and nothing may be published twice.
  step = h.StepView(view);
  CHECK(step.state == BehaviorState::Damping);
  CHECK(h.sink().requests.size() == requests + 1);
  CHECK(h.sink().commands.size() == commands);
  CHECK(h.sink().heartbeats.size() == heartbeats);
  CHECK(h.client().request_pending());
  CHECK(h.client().pending_request_id() == stop_id);

  // The stop is still the live request and still confirms.
  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(!h.client().request_pending());

  // A target frame is not published twice either.
  {
    Harness held;
    CHECK(ToStandby(held) == 0);
    CHECK(ToReady(held) == 0);
    const std::size_t held_commands = held.sink().commands.size();
    const ClientView one =
        held.PollOnce(held.config().supervision.target_period_us);
    step = held.StepView(one);
    CHECK(step.state == BehaviorState::Ready);
    CHECK(step.send_command);
    CHECK(held.sink().commands.size() == held_commands + 1);
    step = held.StepView(one);
    CHECK(step.state == BehaviorState::Ready);
    CHECK(!step.send_command);
    CHECK(held.sink().commands.size() == held_commands + 1);
  }
  return 0;
}

// The loop shape the suppression exists for: one poll, the operator's Damping
// event Step, and then the final None Step of the very same iteration. A stop
// of ours is in flight in both, so neither may publish a target frame - the
// frame the final Step would otherwise emit belongs to the authorization the
// stop is withdrawing. Gating on "this Step began a stop" left exactly that
// hole.
int StopInFlightSuppressesTarget() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToReady(h) == 0);
  const TimeUs period = h.config().supervision.target_period_us;

  // The poll advances a full target period, so a hold frame is due.
  const ClientView view = h.PollOnce(period);
  const std::size_t commands = h.sink().commands.size();

  BehaviorStep step = h.StepView(view, InputEvent::Damping);
  CHECK(step.state == BehaviorState::Ready);  // only a confirmation moves it
  CHECK(!step.send_command);
  const ExecutionRequest *stop = h.last_request();
  CHECK(stop != nullptr);
  if (stop == nullptr) {
    return 1;
  }
  CHECK(stop->operation == Operation::Damping);
  CHECK(h.client().request_pending());
  CHECK(h.client().pending_request_id() == stop->request_id);

  // The same view, the same instant, the final Step of the iteration: the
  // target is due but the stop is still unresolved, so nothing is published.
  step = h.StepView(view);
  CHECK(step.state == BehaviorState::Ready);
  CHECK(!step.send_command);
  CHECK(h.sink().commands.size() == commands);
  CHECK(h.client().request_pending());
  CHECK(h.client().pending_request_id() == stop->request_id);
  return 0;
}

// A repeated identical stop key never re-asks: the request on the wire owns the
// decision and the client owns its bounded resend, so the deadline of an
// unanswered stop is never restarted by the key.
int RepeatedStopKeyDoesNotReask() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  const std::size_t before = h.sink().requests.size();  // the handover
  BehaviorStep step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Standby);
  const ExecutionRequest *begun = h.last_request();
  CHECK(begun != nullptr);
  if (begun == nullptr) {
    return 1;
  }
  const Sequence id = begun->request_id;

  // Four more presses inside the deadline: one request, resent by the client.
  const TimeUs spacing = h.config().supervision.request_deadline_us / 5;
  for (int index = 0; index < 4; ++index) {
    step = h.Step(InputEvent::Damping, spacing);
    CHECK(step.state == BehaviorState::Standby);
  }
  CHECK(h.sink().requests.size() > before + 1);
  for (std::size_t index = before; index < h.sink().requests.size(); ++index) {
    CHECK(h.sink().requests[index].request_id == id);
  }
  CHECK(h.client().retry_count() >= 2);

  // An emergency stop is a different decision and still displaces it.
  step = h.Step(InputEvent::EmergencyStop);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::OperatorEmergencyStop);
  const ExecutionRequest *stop = h.last_request();
  CHECK(stop != nullptr);
  if (stop == nullptr) {
    return 1;
  }
  CHECK(stop->operation == Operation::Damping);
  CHECK(stop->request_id > id);

  // The repeated keys did not restart the deadline either: a stop that is never
  // answered still reaches the latch at its own final instant.
  {
    Harness unanswered;
    CHECK(ToStandby(unanswered) == 0);
    step = unanswered.Step(InputEvent::Damping);
    CHECK(step.state == BehaviorState::Standby);
    for (int index = 0; index < 4; ++index) {
      step = unanswered.Step(InputEvent::Damping, spacing);
      CHECK(step.state == BehaviorState::Standby);
    }
    step = unanswered.Step(InputEvent::None, spacing);
    CHECK(step.state == BehaviorState::Fault);
    CHECK(step.fault_cause == FaultCause::RequestTimeout);
    const ExecutionRequest *after = unanswered.last_request();
    CHECK(after != nullptr);
    if (after == nullptr) {
      return 1;
    }
    CHECK(after->operation == Operation::Damping);
    CHECK(after->request_id > id);
  }
  return 0;
}

// A rejected request keeps the behaviour whose lower invariant still holds, and
// records the reason for the display.
int RejectedRequestKeepsBehaviour() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);

  const BehaviorStep start = h.Step(InputEvent::Initialize);
  CHECK(start.state == BehaviorState::Damping);
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Tracking);

  CHECK(h.Reject(RejectReason::Discontinuity));
  const BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.last_reject_reason == RejectReason::Discontinuity);
  CHECK(!step.send_command);
  // A refused authorization leaves no frames behind.
  CHECK(h.sink().commands.empty());
  // ... and does not block the next explicit attempt.
  const std::size_t requests = h.sink().requests.size();
  const BehaviorStep again = h.Step(InputEvent::Initialize);
  CHECK(again.state == BehaviorState::Damping);
  CHECK(h.sink().requests.size() == requests + 1);
  CHECK(h.last_request() != nullptr &&
        h.last_request()->operation == Operation::Tracking);
  return 0;
}

// A request that is never answered is not "never applied": at its final
// deadline the upper behaviour faults and asks for a stop with a larger id.
int RequestTimeoutFaults() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  const BehaviorStep begun = h.Step(InputEvent::Damping);
  CHECK(begun.state == BehaviorState::Standby);
  const ExecutionRequest *pending = h.last_request();
  CHECK(pending != nullptr);
  if (pending == nullptr) {
    return 1;
  }
  CHECK(pending->operation == Operation::Damping);

  const BehaviorStep step =
      h.Step(InputEvent::None, h.config().supervision.request_deadline_us);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RequestTimeout);
  const ExecutionRequest *stop = h.last_request();
  CHECK(stop != nullptr);
  if (stop == nullptr) {
    return 1;
  }
  CHECK(stop->operation == Operation::Damping);
  CHECK(stop->request_id > pending->request_id);
  CHECK(!step.send_command);
  return 0;
}

// The admission of an initialization plan: a plan the runtime would refuse is
// refused here, with no request, no latch and the refusal displayed.
int InitializationAdmission() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  // The initialization is only ever started from Damping.
  const std::size_t requests = h.sink().requests.size();
  BehaviorStep step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Standby);
  CHECK(h.sink().requests.size() == requests);

  CHECK(ToDamping(h) == 0);

  // A measured origin outside the configured position limits is a plan the
  // runtime would refuse: the FSM stays in Damping, asks for nothing and
  // latches nothing.
  h.lower.q = 2.5;
  const std::size_t before = h.sink().requests.size();
  step = h.Step(InputEvent::Initialize);
  CHECK(step.initialization_refused);
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(h.sink().requests.size() == before);
  CHECK(!step.send_command);
  CHECK(h.sink().commands.empty());
  // The refusal is that step only.
  h.lower.q = kMeasuredPosition;
  step = h.Step();
  CHECK(!step.initialization_refused);
  CHECK(step.state == BehaviorState::Damping);

  // The runnable plan is admitted, and the request carries the frame the guard
  // approved.
  CHECK(ToInitializing(h) == 0);
  return 0;
}

// The completion condition, the Ready hold period, and a repeated Initialize
// during the move.
int CompletionAndReadyHold() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);
  CHECK(ToInitializing(h) == 0);

  const ApplicationConfig &config = h.config();
  const TimeUs period = config.supervision.target_period_us;
  const TimeUs duration = config.initialization.trajectory.duration_us;
  const TimeUs window = config.initialization.trajectory.stable_window_us;
  const Sequence epoch = h.tracking_epoch();
  const Id producer = h.tracking_producer();

  // A repeated Initialize while the move runs is a no-op: no new request, and
  // the frame sequence continues instead of restarting.
  const std::size_t requests = h.sink().requests.size();
  const ExecutionRequest *tracking = h.last_request_of(Operation::Tracking);
  CHECK(tracking != nullptr);
  if (tracking == nullptr) {
    return 1;
  }
  const Sequence last_seq = tracking->first_command.command_seq;
  BehaviorStep step = h.Step(InputEvent::Initialize, period);
  CHECK(step.state == BehaviorState::Initializing);
  CHECK(h.sink().requests.size() == requests);
  CHECK(step.send_command);
  CHECK(step.command.command_seq == last_seq + 1);
  CHECK(h.sink().commands.size() == 1);

  // The ramp keeps running: the FSM is Initializing and emits one frame per
  // target period, each with the fresh state's sequence and the confirmed
  // context.
  const std::size_t ramp_start = h.sink().commands.size();
  for (int index = 0; index < 24; ++index) {
    step = h.Step(InputEvent::None, period);
    CHECK(step.state == BehaviorState::Initializing);
    CHECK(step.send_command);
    CHECK(step.command.epoch == epoch);
    CHECK(step.command.producer_id == producer);
    CHECK(step.command.source_state_seq == h.published().state_seq);
  }
  CHECK(h.sink().commands.size() == ramp_start + 24);
  // The ramp interpolates: a frame in the middle of the move is strictly
  // between the measured origin and the target, and the last one has arrived.
  const double mid = h.sink().commands[ramp_start + 5].joints.q[0];
  CHECK(mid > kMeasuredPosition);
  CHECK(mid < config.initialization.target_q[0]);
  CHECK(h.sink().commands.back().joints.q[0] ==
        config.initialization.target_q[0]);
  CHECK(h.sink().commands.back().joints.dq[0] == 0.0);

  // The end of the ramp is not the completion condition: with the joints still
  // away from the target the move keeps running.
  step = h.Step(InputEvent::None, period);
  CHECK(step.state == BehaviorState::Initializing);

  // The joints arrive and hold the target: the stable window is what completes
  // the move.
  h.lower.q = config.initialization.target_q[0];
  const int window_calls = static_cast<int>(window / period);
  for (int index = 0; index < window_calls; ++index) {
    step = h.Step(InputEvent::None, period);
    CHECK(step.state == BehaviorState::Initializing);
  }
  step = h.Step(InputEvent::None, period);
  CHECK(step.state == BehaviorState::Ready);

  // Ready holds the target at the configured period: a new frame with a new
  // sequence and the fresh state's sequence, and nothing inside the period.
  const Sequence ready_seq = step.command.command_seq;
  step = h.Step(InputEvent::None, period / 2);
  CHECK(step.state == BehaviorState::Ready);
  CHECK(!step.send_command);
  const std::size_t held = h.sink().commands.size();
  step = h.Step(InputEvent::None, period);
  CHECK(step.state == BehaviorState::Ready);
  CHECK(step.send_command);
  CHECK(h.sink().commands.size() == held + 1);
  CHECK(step.command.command_seq > ready_seq);
  CHECK(step.command.source_state_seq == h.published().state_seq);
  CHECK(step.command.epoch == epoch);
  CHECK(step.command.producer_id == producer);
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    CHECK(step.command.joints.q[index] ==
          config.initialization.target_q[index]);
    CHECK(step.command.joints.dq[index] == 0.0);
    CHECK(step.command.joints.tau_ff[index] == 0.0);
    CHECK(step.command.joints.kp[index] ==
          config.initialization.gains.kp[index]);
    CHECK(step.command.joints.kd[index] ==
          config.initialization.gains.kd[index]);
  }
  // A sample the generator cannot use latches nothing: Ready holds again as
  // soon as a usable one arrives, without a fault and without a frame.
  {
    h.publish = false;
    const TimeUs stale_stamp =
        h.now_us() - h.config().runtime.timeouts.source_state_us - kTickUs;
    const RobotState stale = h.State(stale_stamp);
    h.client().ObserveState(stale, h.now_us());
    step = h.Step(InputEvent::None, period);
    CHECK(step.state == BehaviorState::Ready);
    CHECK(step.fault_cause == FaultCause::None);
    CHECK(!step.send_command);
    CHECK(step.runtime_state_known);  // the link itself is current
    h.publish = true;
    step = h.Step(InputEvent::None, period);
    CHECK(step.state == BehaviorState::Ready);
    CHECK(step.send_command);
    CHECK(step.command.command_seq > ready_seq);
  }

  return 0;
}

// Ordinary stops out of a motion behaviour: an ordinary Damping stop lands in
// Damping and cancels the move, the global ZeroTorque stop lands in Standby,
// and neither restarts the old trajectory.
int OrdinaryStopsFromMotion() {
  // (a) A stop from Initializing confirms to Damping and cancels the move.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    CHECK(ToDamping(h) == 0);
    CHECK(ToInitializing(h) == 0);
    const Sequence old_epoch = h.tracking_epoch();

    BehaviorStep step =
        h.Step(InputEvent::Damping, h.config().supervision.target_period_us);
    CHECK(step.state ==
          BehaviorState::Initializing);  // only a confirmation moves it
    // The iteration that begins the stop emits no target frame, even though a
    // full period has passed: a stop of ours is in flight until the client
    // resolves it, and that durable fact is what suppresses the target.
    CHECK(!step.send_command);
    const ExecutionRequest *request = h.last_request();
    CHECK(request != nullptr);
    if (request == nullptr) {
      return 1;
    }
    CHECK(request->operation == Operation::Damping);
    h.lower.state = ExecutionState::Damping;
    h.lower.output = OutputAction::Damping;
    CHECK(h.Answer());
    step = h.Step();
    CHECK(step.state == BehaviorState::Damping);
    CHECK(!step.send_command);

    // The cancelled move is not restarted, and a new move is a new
    // authorization, not the old epoch.
    const std::size_t commands = h.sink().commands.size();
    step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
    CHECK(step.state == BehaviorState::Damping);
    CHECK(!step.send_command);
    CHECK(h.sink().commands.size() == commands);
    step = h.Step(InputEvent::Initialize);
    CHECK(step.state == BehaviorState::Damping);
    const ExecutionRequest *fresh = h.last_request();
    CHECK(fresh != nullptr);
    if (fresh == nullptr) {
      return 1;
    }
    CHECK(fresh->operation == Operation::Tracking);
    CHECK(fresh->proposed_epoch > old_epoch);
  }

  // (b) The global ZeroTorque stop from Ready goes to Standby and cancels the
  // move as well.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    CHECK(ToReady(h) == 0);
    BehaviorStep step = h.Step(InputEvent::ZeroTorque);
    CHECK(step.state == BehaviorState::Ready);
    const ExecutionRequest *request = h.last_request();
    CHECK(request != nullptr);
    if (request == nullptr) {
      return 1;
    }
    CHECK(request->operation == Operation::ZeroTorque);
    h.lower.state = ExecutionState::ZeroTorque;
    h.lower.output = OutputAction::ZeroTorque;
    CHECK(h.Answer());
    step = h.Step();
    CHECK(step.state == BehaviorState::Standby);
    CHECK(step.runtime_state == ExecutionState::ZeroTorque);
    CHECK(!step.send_command);
    // No hold frame survives the cancel.
    const std::size_t commands = h.sink().commands.size();
    step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
    CHECK(step.state == BehaviorState::Standby);
    CHECK(h.sink().commands.size() == commands);
  }
  return 0;
}

// The emergency stop: it preempts what is on the wire, latches the operator
// cause, and nothing short of a confirmed Reset unlocks it.
int EmergencyStopLatches() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);

  // An initialization request is on the wire when the operator hits space.
  BehaviorStep step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Damping);
  const ExecutionRequest *tracking_request = h.last_request();
  CHECK(tracking_request != nullptr);
  if (tracking_request == nullptr) {
    return 1;
  }
  const ExecutionRequest tracking = *tracking_request;
  CHECK(tracking.operation == Operation::Tracking);

  step = h.Step(InputEvent::EmergencyStop);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::OperatorEmergencyStop);
  CHECK(!step.send_command);
  const ExecutionRequest *stop = h.last_request();
  CHECK(stop != nullptr);
  if (stop == nullptr) {
    return 1;
  }
  CHECK(stop->operation == Operation::Damping);
  // It displaces the pending request instead of queueing behind it.
  CHECK(stop->request_id > tracking.request_id);
  CHECK(stop->runtime_id == kRuntimeId);
  CHECK(stop->session_id == h.granted_session());

  // Releasing the key is a no-op.
  step = h.Step(InputEvent::None);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::OperatorEmergencyStop);

  // A damping that arrives (and is confirmed) does not unlock the latch, and
  // the lower layer's real output is what is displayed.
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  // The runtime applies the Damping stop we asked for, so its own state names
  // that reason.
  h.lower.stop_reason = StopReason::Requested;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  // The two display facts of the status line at once: the latched upper cause
  // (`cause=`) and the lower layer's own reason for its stop (`stop=`).
  CHECK(step.fault_cause == FaultCause::OperatorEmergencyStop);
  CHECK(step.runtime_stop_reason == StopReason::Requested);
  CHECK(step.runtime_state == ExecutionState::Damping);
  CHECK(step.runtime_output == OutputAction::Damping);
  CHECK(!step.send_command);

  // The runtime is not in Fault, so the explicit Reset asks for ZeroTorque and
  // only a confirmation returns to Standby. A link that reports unhealthy ticks
  // is not a cleared local error: the Reset sends nothing.
  h.lower.healthy = false;
  const std::size_t before_reset = h.sink().requests.size();
  step = h.Step(InputEvent::Reset);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::OperatorEmergencyStop);
  CHECK(h.sink().requests.size() == before_reset);
  h.lower.healthy = true;
  step = h.Step(InputEvent::Reset);
  const ExecutionRequest *zero = h.last_request();
  CHECK(zero != nullptr);
  if (zero == nullptr) {
    return 1;
  }
  CHECK(zero->operation == Operation::ZeroTorque);
  CHECK(step.state == BehaviorState::Fault);
  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);

  // Nothing restarts the old trajectory: Standby does not act on Initialize,
  // and the next move is a new epoch.
  const std::size_t requests = h.sink().requests.size();
  step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Standby);
  CHECK(h.sink().requests.size() == requests);
  CHECK(!step.send_command);
  CHECK(h.sink().commands.empty());

  CHECK(ToDamping(h) == 0);
  step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Damping);
  const ExecutionRequest *moved = h.last_request();
  CHECK(moved != nullptr);
  if (moved == nullptr) {
    return 1;
  }
  CHECK(moved->operation == Operation::Tracking);
  CHECK(moved->proposed_epoch > tracking.proposed_epoch);
  return 0;
}

// The lower layer's own fault: the upper latches the cause, keeps displaying
// the runtime's real output rule, and leaves only through an explicit Reset
// that the runtime's recovery window accepts.
int RuntimeFaultRecovery() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  h.lower.state = ExecutionState::Fault;
  h.lower.output = OutputAction::Damping;  // its own fault output rule
  h.lower.fault_id = 42;
  h.lower.fault_reason = FaultReason::DriverError;
  h.lower.healthy = false;
  const std::size_t requests = h.sink().requests.size();
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeFault);
  CHECK(step.runtime_state == ExecutionState::Fault);
  CHECK(step.runtime_output == OutputAction::Damping);
  CHECK(step.runtime_fault_reason == FaultReason::DriverError);
  CHECK(step.runtime_state_known);
  CHECK(!step.send_command);
  // A latched lower fault is left through Reset alone: the upper asks for
  // nothing that the runtime would only refuse.
  CHECK(h.sink().requests.size() == requests);

  // The latch survives a heartbeat period, a stop key and a Reset that the
  // recovery window is not ready for.
  step = h.Step(InputEvent::None, h.config().supervision.heartbeat_period_us);
  CHECK(step.state == BehaviorState::Fault);
  step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(h.sink().requests.size() == requests);
  step = h.Step(InputEvent::ZeroTorque);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(h.sink().requests.size() == requests);
  step = h.Step(InputEvent::Reset);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(h.sink().requests.size() == requests);

  // The runtime reports a healthy tick: the Reset now carries the latched fault
  // id, and only its confirmation leaves Fault.
  h.lower.healthy = true;
  step = h.Step(InputEvent::Reset);
  const ExecutionRequest *reset = h.last_request();
  CHECK(reset != nullptr);
  if (reset == nullptr) {
    return 1;
  }
  CHECK(reset->operation == Operation::Reset);
  CHECK(reset->fault_id == 42);
  CHECK(reset->session_id == h.granted_session());
  CHECK(step.state == BehaviorState::Fault);

  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  h.lower.fault_id = 0;
  h.lower.fault_reason = FaultReason::None;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  CHECK(step.runtime_fault_reason == FaultReason::None);
  CHECK(!step.send_command);
  // Ready is not restored by the reset either, and no locomotion
  // authorization survives it.
  CHECK(h.fsm().locomotion_epoch() == 0);
  CHECK(h.fsm().locomotion_producer() == kNoId);
  CHECK(h.fsm().state() == BehaviorState::Standby);
  return 0;
}

// A tracking context nobody here authorized is a fault, not a state to compare
// enumerators on.
int UnexpectedTracking() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);

  h.lower.state = ExecutionState::Tracking;
  h.lower.output = OutputAction::Tracking;
  h.lower.producer = MakeId(0x77);
  h.lower.epoch = 9;
  const BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::UnexpectedContext);
  CHECK(step.runtime_state == ExecutionState::Tracking);
  CHECK(!step.send_command);
  // The stop it owes the machine, on the live session.
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Damping);
  CHECK(request->session_id == h.granted_session());
  return 0;
}

// The lower layer leaving tracking on its own is a legal, unlatched
// combination: the upper revokes the movement without faulting and without
// restarting it.
int LowerFellBackToDamping() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);
  CHECK(ToInitializing(h) == 0);

  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  h.lower.producer = kNoId;
  h.lower.epoch = 0;
  h.lower.stop_reason = StopReason::CommandTimeout;
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.runtime_stop_reason == StopReason::CommandTimeout);
  CHECK(!step.send_command);
  // The move and any answer it was waiting for are gone with it; the session
  // itself is untouched, because the link never broke.
  CHECK(!h.client().request_pending());
  CHECK(h.client().session_owned());

  // No automatic restart and no automatic Ready.
  const std::size_t requests = h.sink().requests.size();
  for (int index = 0; index < 5; ++index) {
    step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
    CHECK(step.state == BehaviorState::Damping);
    CHECK(!step.send_command);
  }
  CHECK(h.sink().requests.size() == requests);
  // A lower state that is neither a stop nor the tracking context is not turned
  // into one: a Boot state leaves the behaviour where it is instead of
  // displaying a Damping stop nobody asked for.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    CHECK(ToDamping(h) == 0);
    CHECK(ToInitializing(h) == 0);
    h.lower.state = ExecutionState::Boot;
    h.lower.output = OutputAction::ZeroTorque;
    h.lower.producer = kNoId;
    h.lower.epoch = 0;
    const BehaviorStep step = h.Step();
    CHECK(step.state == BehaviorState::Initializing);
    CHECK(step.fault_cause == FaultCause::None);
    CHECK(step.runtime_state == ExecutionState::Boot);
  }

  // A stop key in Damping is still a no-op, and no initialization runs.
  step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Damping);
  CHECK(h.sink().requests.size() == requests);
  return 0;
}

// The lower layer falling back to ZeroTorque is the other half of the same
// rule, and it is *not* folded into Damping: the runtime refuses Tracking from
// a lower ZeroTorque, so an upper Damping there would make the operator's next
// Initialize build a request that is guaranteed to be rejected. The upper
// follows the lower stop state - ZeroTorque maps to Standby - and only the
// documented Standby -> Damping -> Initializing path starts a move.
int LowerFellBackToZeroTorque() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);
  CHECK(ToInitializing(h) == 0);

  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  h.lower.producer = kNoId;
  h.lower.epoch = 0;
  h.lower.stop_reason = StopReason::SupervisorTimeout;
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.fault_cause == FaultCause::None);
  // The lower stop reason is surfaced for the display, not hidden behind the
  // upper behaviour.
  CHECK(step.runtime_stop_reason == StopReason::SupervisorTimeout);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  CHECK(!step.send_command);
  CHECK(!h.client().request_pending());
  CHECK(h.client().session_owned());

  // No automatic restart and no automatic Ready.
  const std::size_t requests = h.sink().requests.size();
  for (int index = 0; index < 5; ++index) {
    step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
    CHECK(step.state == BehaviorState::Standby);
    CHECK(!step.send_command);
  }
  CHECK(h.sink().requests.size() == requests);

  // `i` in Standby is a local no-op: nothing may ask for Tracking from a lower
  // ZeroTorque, and no request leaves for it.
  step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Standby);
  CHECK(!step.initialization_refused);
  CHECK(h.sink().requests.size() == requests);
  CHECK(!h.client().request_pending());

  // The documented path still works: `d` first, then `i`.
  step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Standby);
  const ExecutionRequest *damping = h.last_request();
  CHECK(damping != nullptr);
  if (damping == nullptr) {
    return 1;
  }
  CHECK(damping->operation == Operation::Damping);
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.runtime_state == ExecutionState::Damping);

  step = h.Step(InputEvent::Initialize);
  CHECK(step.state == BehaviorState::Damping);  // only a confirmation moves it
  const ExecutionRequest *tracking = h.last_request();
  CHECK(tracking != nullptr);
  if (tracking == nullptr) {
    return 1;
  }
  CHECK(tracking->operation == Operation::Tracking);
  CHECK(tracking->proposed_epoch > tracking->expected_epoch);
  return 0;
}

// Another runtime instance is a restart: the session, the epoch and the pending
// request are worthless, and the upper latches the cause.
int RuntimeRestartedLatches() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);

  const Id replaced_runtime = MakeId(0x60);
  h.publish = false;
  RobotState replaced = h.State(h.now_us());
  replaced.runtime_id = replaced_runtime;
  replaced.session_id = kNoId;
  h.client().ObserveState(replaced, h.now_us());
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeRestarted);
  CHECK(!step.send_command);
  CHECK(h.fsm().state() == BehaviorState::Fault);

  // A message may stay on the wire only after a new session is taken over on
  // the new instance: what the FSM asks for is the handover, never a motion
  // context.
  step = h.Step(InputEvent::None, h.config().supervision.request_retry_us + 1);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeRestarted);
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::AcquireSession);
  CHECK(request->runtime_id == replaced_runtime);
  CHECK(request->session_id == kNoId);
  return 0;
}

// A move that never reaches its target is never Ready: at its deadline the
// upper faults and asks for damping.
int InitializationDeadlineFaults() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToDamping(h) == 0);
  CHECK(ToInitializing(h) == 0);

  const TimeUs period = h.config().supervision.target_period_us;
  const TimeUs deadline = h.config().initialization.trajectory.deadline_us;
  const int limit = static_cast<int>(deadline / period) + 4;
  BehaviorStep step{};
  for (int index = 0; index < limit; ++index) {
    step = h.Step(InputEvent::None, period);
    if (step.state == BehaviorState::Fault) {
      break;
    }
  }
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::InitializationFailed);
  CHECK(!step.send_command);
  // The fault owes the machine the stop it can still ask for.
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Damping);
  return 0;
}

// An emergency stop while the runtime already reports a fault of its own: the
// first latched cause is kept, nothing is asked of a runtime that could only
// refuse, and the supervision keeps its lease alive.
int EmergencyStopInRuntimeFault() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  h.lower.state = ExecutionState::Fault;
  h.lower.output = OutputAction::ZeroTorque;
  h.lower.fault_id = 11;
  h.lower.fault_reason = FaultReason::JointVelocityExceeded;
  h.lower.healthy = false;
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeFault);

  const std::size_t requests = h.sink().requests.size();
  const std::size_t heartbeats = h.sink().heartbeats.size();
  step = h.Step(InputEvent::EmergencyStop);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeFault);  // the first cause wins
  CHECK(step.runtime_fault_reason == FaultReason::JointVelocityExceeded);
  CHECK(step.runtime_output == OutputAction::ZeroTorque);
  CHECK(!step.send_command);
  // The runtime's own output rule owns the machine: no Damping of ours leaves,
  // because a latched fault is left through Reset alone.
  CHECK(h.sink().requests.size() == requests);

  // The supervision loop is still alive after the latch: the lease keeps being
  // refreshed, which is permission to supervise and not permission to move.
  for (int index = 0; index < 3; ++index) {
    step = h.Step(InputEvent::None, h.config().supervision.heartbeat_period_us);
    CHECK(step.state == BehaviorState::Fault);
    CHECK(step.fault_cause == FaultCause::RuntimeFault);
    CHECK(step.send_heartbeat);
    CHECK(!step.send_command);
  }
  CHECK(h.sink().heartbeats.size() >= heartbeats + 3);
  CHECK(h.sink().requests.size() == requests);
  return 0;
}

// The session the runtime withdrew is a lost link, not a new normal: the upper
// latches it and asks for nothing it cannot address. The case deliberately
// re-establishes a known state after the loss (the client drops its snapshot
// when it loses the session, and re-observing the revoked state makes it
// current again), so the FSM takes the `view.session_lost` branch and not the
// stale-state one, which would latch the same cause for another reason.
int SessionLossLatches() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  // The runtime answers a state of its own that no longer names our session.
  h.publish = false;
  RobotState revoked = h.State(h.now_us());
  revoked.session_id = MakeId(0x50);
  h.client().ObserveState(revoked, h.now_us());
  CHECK(!h.client().session_owned());
  // A second observation of the same instance: it is a known, fresh state that
  // carries the loss, exactly the combination the branch under test wants.
  revoked = h.State(h.now_us() + 1);
  revoked.session_id = MakeId(0x50);
  h.client().ObserveState(revoked, h.now_us());

  // The loss reports its own Unknown outcome exactly once. A first poll
  // consumes that outcome, so the view the Step below acts on can reach the
  // latch only through view.session_lost - the branch this case is named for.
  const ClientView loss = h.Poll();
  CHECK(loss.outcome == RequestOutcome::Unknown);
  const ClientView view = h.Poll();
  CHECK(view.outcome != RequestOutcome::Unknown);
  CHECK(view.state_known);
  CHECK(view.state_fresh);
  CHECK(view.session_lost);
  CHECK(!view.session_owned);

  const std::size_t requests = h.sink().requests.size();
  const BehaviorStep step = h.StepView(view);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeUnreachable);
  CHECK(!step.send_command);
  CHECK(!step.send_heartbeat);
  // Nothing can leave without a session, so no stop is claimed either.
  CHECK(h.sink().requests.size() == requests);
  return 0;
}

// The state deadline: the upper layer faults, displays Unknown, and never
// claims a stop it cannot confirm.
int StateTimeoutFaults() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  const TimeUs timeout = h.config().supervision.state_timeout_us;
  h.publish = false;
  BehaviorStep step = h.Step(InputEvent::None, timeout + 1);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeUnreachable);
  CHECK(!step.runtime_state_known);
  CHECK(!step.send_command);
  CHECK(!step.send_heartbeat);  // the lease is gone with the session

  // Nothing can leave without a session, so no stop is claimed either.
  const std::size_t requests = h.sink().requests.size();
  step = h.Step(InputEvent::None, 10 * kTickUs);
  CHECK(step.state == BehaviorState::Fault);
  CHECK(!step.runtime_state_known);
  CHECK(h.sink().requests.size() == requests);
  CHECK(!step.send_command);
  return 0;
}

// Orderly exit: a confirmed damping stop proves it; a silent link or a runtime
// already in Fault is bounded and proves nothing.
int ExitSequence() {
  // (a) The damping stop is confirmed by a fresh state.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    CHECK(ToReady(h) == 0);
    h.fsm().RequestExit(h.now_us());
    BehaviorStep step = h.Step();
    CHECK(!step.exit_ready);
    const ExecutionRequest *request = h.last_request();
    CHECK(request != nullptr);
    if (request == nullptr) {
      return 1;
    }
    CHECK(request->operation == Operation::Damping);
    CHECK(!step.send_command);  // target generation stopped
    h.lower.state = ExecutionState::Damping;
    h.lower.output = OutputAction::Damping;
    CHECK(h.Answer());
    const ClientView view = h.PollOnce();
    const std::size_t settled = h.sink().requests.size();
    step = h.StepView(view);
    CHECK(step.exit_ready);
    CHECK(step.exit_stop_confirmed);
    // A second Step on the same view, after the exit resolved terminally: the
    // resolution is terminal, and no fresh Damping request may leave.
    step = h.StepView(view);
    CHECK(step.exit_ready);
    CHECK(step.exit_stop_confirmed);
    CHECK(h.sink().requests.size() == settled);
    CHECK(!h.client().request_pending());
  }

  // (b) A silent link is bounded by the deadline and proves nothing.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    CHECK(ToReady(h) == 0);
    h.publish = false;
    h.fsm().RequestExit(h.now_us());
    const BehaviorStep step = h.Step(
        InputEvent::None, h.config().supervision.request_deadline_us + 1);
    CHECK(step.exit_ready);
    CHECK(!step.exit_stop_confirmed);
    CHECK(!step.runtime_state_known);  // the loop prints Unknown
    CHECK(!step.send_command);
  }

  // (c) A runtime already in Fault keeps its own output rule: nothing is turned
  // into damping and the exit proves nothing.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    CHECK(ToReady(h) == 0);
    h.lower.state = ExecutionState::Fault;
    h.lower.output = OutputAction::ZeroTorque;
    h.lower.fault_id = 5;
    h.lower.healthy = false;
    CHECK(h.Step().state == BehaviorState::Fault);
    const std::size_t requests = h.sink().requests.size();
    h.fsm().RequestExit(h.now_us());
    const BehaviorStep step = h.Step();
    CHECK(step.exit_ready);
    CHECK(!step.exit_stop_confirmed);
    CHECK(step.runtime_state == ExecutionState::Fault);
    CHECK(!step.send_command);
    CHECK(h.sink().requests.size() == requests);
  }

  // (d) An exit before any session can be taken over asks for nothing, so the
  // deadline is the only thing that can end it - and it ends unproven.
  {
    Harness h;
    BehaviorStep step = h.Step();
    CHECK(step.state == BehaviorState::Starting);
    CHECK(h.sink().requests.size() == 1);  // the handover, still unanswered
    h.fsm().RequestExit(h.now_us());
    step = h.Step(InputEvent::None,
                  h.config().supervision.request_deadline_us - 1);
    CHECK(!step.exit_ready);
    step = h.Step(InputEvent::None, 2);
    CHECK(step.exit_ready);
    CHECK(!step.exit_stop_confirmed);
    CHECK(step.runtime_state_known);  // a live link is not a confirmed stop
    CHECK(!step.send_command);
  }
  return 0;
}

// One complete frame of the local policy, as the supervision loop would build
// it: every joint of the layout, the identity of the authorization it belongs
// to and the state sequence it was computed from.
JointCommand LocomotionFrame(const Harness &h, double q, Sequence epoch,
                             Id producer) {
  JointCommand frame{};
  frame.layout_id.Assign(h.config().layout_id());
  frame.producer_id = producer;
  frame.epoch = epoch;
  frame.command_seq = 0;
  frame.source_state_seq = h.published().state_seq;
  frame.joints.count = static_cast<std::uint32_t>(h.config().joint_count());
  for (std::size_t index = 0; index < h.config().joint_count(); ++index) {
    frame.joints.q[index] = q;
    frame.joints.dq[index] = 0.0;
    frame.joints.tau_ff[index] = 0.0;
    frame.joints.kp[index] = 10.0;
    frame.joints.kd[index] = 1.0;
  }
  return frame;
}

// The local locomotion source: the key arms it, the first frame the loop offers
// becomes the first frame of the Tracking request, the confirmed context enters
// Locomotion, and only frames of that very authorization are published
// afterwards.
int LocomotionRequest() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToReady(h) == 0);
  const std::size_t requests = h.sink().requests.size();

  // The key only arms the source: without a first frame there is nothing to
  // propose, and Ready keeps holding.
  BehaviorStep step = h.Step(InputEvent::StartLocomotion);
  CHECK(step.state == BehaviorState::Ready);
  CHECK(step.locomotion_armed);
  CHECK(h.sink().requests.size() == requests);
  CHECK(h.fsm().locomotion_epoch() == 0);

  // A frame offered before the operator asked changes nothing either: the
  // behaviour is not armed.
  Harness idle;
  CHECK(ToStandby(idle) == 0);
  CHECK(ToReady(idle) == 0);
  idle.fsm().OfferLocomotionCandidate(LocomotionFrame(idle, 0.1, 0, kNoId),
                                      idle.now_us());
  step = idle.Step();
  CHECK(step.state == BehaviorState::Ready);
  CHECK(!step.locomotion_armed);

  // The first frame arrives while armed: the behaviour proposes Tracking
  // carrying it, with the epoch the request proposes and the first frame
  // sharing it.
  h.fsm().OfferLocomotionCandidate(LocomotionFrame(h, 0.25, 0, kNoId),
                                   h.now_us());
  step = h.Step();
  CHECK(h.sink().requests.size() == requests + 1);
  const ExecutionRequest *request = h.last_request();
  CHECK(request->operation == Operation::Tracking);
  CHECK(request->producer_id != kNoId);
  CHECK(request->proposed_epoch > 0);
  CHECK(request->first_command.epoch == request->proposed_epoch);
  CHECK(request->first_command.producer_id == request->producer_id);
  CHECK(request->first_command.command_seq == 1);
  CHECK(request->first_command.joints.q[0] == 0.25);
  // The answer has not been confirmed yet, so the behaviour is still Ready.
  CHECK(step.state == BehaviorState::Ready);
  CHECK(h.fsm().locomotion_epoch() == request->proposed_epoch);

  // The runtime applied it and reports the authorized context.
  h.EnterTracking();
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Locomotion);
  CHECK(step.locomotion_armed);
  CHECK(step.fault_cause == FaultCause::None);

  // A frame of the authorized source is published once the target period
  // passed, and it carries a new sequence every time.
  const Sequence epoch = h.fsm().locomotion_epoch();
  const Id producer = h.fsm().locomotion_producer();
  h.fsm().OfferLocomotionTarget(LocomotionFrame(h, 0.5, epoch, producer),
                                h.now_us());
  step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
  CHECK(step.state == BehaviorState::Locomotion);
  CHECK(step.send_command);
  CHECK(step.command.joints.q[0] == 0.5);
  CHECK(step.command.epoch == epoch);
  CHECK(step.command.producer_id == producer);
  const Sequence published_seq = step.command.command_seq;

  // A frame that belongs to another authorization is never published: a result
  // of a withdrawn epoch cannot drive the machine.
  h.fsm().OfferLocomotionTarget(LocomotionFrame(h, 0.75, epoch + 1, producer),
                                h.now_us());
  step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
  CHECK(!step.send_command);

  h.fsm().OfferLocomotionTarget(LocomotionFrame(h, 0.75, epoch, kNoId),
                                h.now_us());
  step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
  CHECK(!step.send_command);

  // The same frame of the right authorization is published, and republishing it
  // never reuses the sequence of the previous one.
  h.fsm().OfferLocomotionTarget(LocomotionFrame(h, 0.75, epoch, producer),
                                h.now_us());
  step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
  CHECK(step.send_command);
  CHECK(step.command.joints.q[0] == 0.75);
  CHECK(step.command.command_seq > published_seq);

  // The operator stops: the source is disarmed at once, the stop is on the
  // wire, and no further frame of it is published.
  step = h.Step(InputEvent::Damping, h.config().supervision.target_period_us);
  CHECK(!step.locomotion_armed);
  CHECK(h.last_request_of(Operation::Damping) != nullptr);
  step = h.Step(InputEvent::None, h.config().supervision.target_period_us);
  CHECK(!step.send_command);

  // An inference the loop could not complete latches the upper fault and asks
  // for damping.
  Harness failing;
  CHECK(ToStandby(failing) == 0);
  CHECK(ToReady(failing) == 0);
  failing.fsm().OfferLocomotionCandidate(
      LocomotionFrame(failing, 0.25, 0, kNoId), failing.now_us());
  static_cast<void>(failing.Step(InputEvent::StartLocomotion));
  failing.EnterTracking();
  CHECK(failing.Answer());
  BehaviorStep faulted = failing.Step();
  CHECK(faulted.state == BehaviorState::Locomotion);
  failing.fsm().LatchLocalPolicyFault(failing.now_us());
  faulted = failing.Step();
  CHECK(faulted.state == BehaviorState::Fault);
  CHECK(faulted.fault_cause == FaultCause::LocalPolicyFailed);
  CHECK(!faulted.locomotion_armed);
  CHECK(failing.last_request_of(Operation::Damping) != nullptr);
  CHECK(failing.fsm().locomotion_epoch() == 0);

  // A first frame that is older than the local source's deadline is not
  // proposed: it is the first frame of nothing any more.
  Harness stale;
  CHECK(ToStandby(stale) == 0);
  CHECK(ToReady(stale) == 0);
  const std::size_t stale_requests = stale.sink().requests.size();
  static_cast<void>(stale.Step(InputEvent::StartLocomotion));
  stale.fsm().OfferLocomotionCandidate(LocomotionFrame(stale, 0.25, 0, kNoId),
                                       stale.now_us());
  step = stale.Step(InputEvent::None,
                    stale.config().policy.inference_timeout_us + 1);
  CHECK(stale.sink().requests.size() == stale_requests);
  CHECK(stale.fsm().state() == BehaviorState::Ready);
  return 0;
}

// The ordered exit from a tracking behaviour while the lower layer still
// reports our own Tracking: the context is one this FSM authorized and the
// Damping stop it asked for is still on the wire, so the state is expected and
// never a fault. The exit is only confirmed once a fresh state shows Damping.
int ExitFromTrackingKeepsTheContext() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToReady(h) == 0);

  BehaviorStep step = h.Step(InputEvent::Exit);
  CHECK(step.state == BehaviorState::Ready);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(!step.exit_ready);
  CHECK(!step.send_command);  // target generation stopped
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Damping);

  // Several iterations still read the lower layer as Tracking: it is our own
  // producer and epoch, the stop has not landed yet, and none of these Steps
  // may latch anything.
  for (int index = 0; index < 8; ++index) {
    step = h.Step();
    CHECK(step.state == BehaviorState::Ready);
    CHECK(step.fault_cause == FaultCause::None);
    CHECK(step.runtime_state == ExecutionState::Tracking);
    CHECK(!step.exit_ready);
  }

  // The stop lands: the exit completes and is confirmed by the fresh state.
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.exit_ready);
  CHECK(step.exit_stop_confirmed);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.runtime_state == ExecutionState::Damping);
  return 0;
}

// The same context while an ordinary stop is on the wire: a Damping key from
// Ready is not a fault either, and the Damping the runtime confirms is what
// moves the behaviour.
int StopFromMotionKeepsTheContext() {
  Harness h;
  CHECK(ToStandby(h) == 0);
  CHECK(ToReady(h) == 0);

  BehaviorStep step = h.Step(InputEvent::Damping);
  CHECK(step.state == BehaviorState::Ready);
  CHECK(step.fault_cause == FaultCause::None);
  const ExecutionRequest *request = h.last_request();
  CHECK(request != nullptr);
  if (request == nullptr) {
    return 1;
  }
  CHECK(request->operation == Operation::Damping);
  for (int index = 0; index < 8; ++index) {
    step = h.Step();
    CHECK(step.state == BehaviorState::Ready);
    CHECK(step.fault_cause == FaultCause::None);
    CHECK(step.runtime_state == ExecutionState::Tracking);
  }
  h.lower.state = ExecutionState::Damping;
  h.lower.output = OutputAction::Damping;
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Damping);
  CHECK(step.fault_cause == FaultCause::None);
  return 0;
}

// A confirmed Reset whose fresh ZeroTorque state is only read in a later poll:
// the transition is held until the state witnesses it instead of being dropped
// by the Step that happened to poll first. One Reset is enough.
int ResetConfirmationWaitsForTheState() {
  Harness h;
  CHECK(ToStandby(h) == 0);

  h.lower.state = ExecutionState::Fault;
  h.lower.output = OutputAction::Damping;  // the runtime's own fault output
  h.lower.fault_id = 42;
  h.lower.fault_reason = FaultReason::DriverError;
  BehaviorStep step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeFault);

  step = h.Step(InputEvent::Reset);
  const ExecutionRequest *reset = h.last_request();
  CHECK(reset != nullptr);
  if (reset == nullptr) {
    return 1;
  }
  CHECK(reset->operation == Operation::Reset);
  CHECK(reset->fault_id == 42);
  CHECK(step.state == BehaviorState::Fault);

  // The confirmation is read against the facts published before the runtime
  // applied the Reset: the transition cannot be taken yet and must not be lost.
  CHECK(h.Answer());
  step = h.Step();
  CHECK(step.state == BehaviorState::Fault);
  CHECK(step.fault_cause == FaultCause::RuntimeFault);

  // The runtime now reports the ZeroTorque the Reset reached.
  h.lower.state = ExecutionState::ZeroTorque;
  h.lower.output = OutputAction::ZeroTorque;
  h.lower.fault_id = 0;
  h.lower.fault_reason = FaultReason::None;
  step = h.Step();
  CHECK(step.state == BehaviorState::Standby);
  CHECK(step.fault_cause == FaultCause::None);
  CHECK(step.runtime_state == ExecutionState::ZeroTorque);
  return 0;
}

// The held confirmation is not a queue: the emergency stop and a session loss
// own the behaviour, and a later fresh state must not promote it behind them.
int HeldConfirmationIsDropped() {
  // (a) The emergency stop.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    h.lower.state = ExecutionState::Fault;
    h.lower.output = OutputAction::Damping;
    h.lower.fault_id = 42;
    h.lower.fault_reason = FaultReason::DriverError;
    CHECK(h.Step().state == BehaviorState::Fault);
    CHECK(h.Step(InputEvent::Reset).state == BehaviorState::Fault);
    CHECK(h.Answer());
    CHECK(h.Step().state == BehaviorState::Fault);  // the confirmation is held

    BehaviorStep step = h.Step(InputEvent::EmergencyStop);
    CHECK(step.state == BehaviorState::Fault);
    // The first cause was the runtime's own fault; the emergency stop latches
    // the behaviour without replacing it.
    CHECK(step.fault_cause == FaultCause::RuntimeFault);
    h.lower.state = ExecutionState::ZeroTorque;
    h.lower.output = OutputAction::ZeroTorque;
    h.lower.fault_id = 0;
    h.lower.fault_reason = FaultReason::None;
    step = h.Step(InputEvent::None, h.config().supervision.heartbeat_period_us);
    CHECK(step.state == BehaviorState::Fault);
    CHECK(step.fault_cause == FaultCause::RuntimeFault);
  }

  // (b) The session loss: the runtime reports a state of its own that no longer
  // names our session, which withdraws the lease and latches the upper layer.
  {
    Harness h;
    CHECK(ToStandby(h) == 0);
    h.lower.state = ExecutionState::Fault;
    h.lower.output = OutputAction::Damping;
    h.lower.fault_id = 42;
    h.lower.fault_reason = FaultReason::DriverError;
    CHECK(h.Step().state == BehaviorState::Fault);
    CHECK(h.Step(InputEvent::Reset).state == BehaviorState::Fault);
    CHECK(h.Answer());
    CHECK(h.Step().state == BehaviorState::Fault);  // the confirmation is held

    h.publish = false;
    RobotState revoked = h.State(h.now_us());
    revoked.session_id = MakeId(0x50);
    h.client().ObserveState(revoked, h.now_us());
    revoked = h.State(h.now_us() + 1);
    revoked.session_id = MakeId(0x50);
    h.client().ObserveState(revoked, h.now_us());
    CHECK(h.Poll().outcome == RequestOutcome::Unknown);
    const ClientView view = h.Poll();
    BehaviorStep step = h.StepView(view);
    CHECK(step.state == BehaviorState::Fault);
    // The first cause was the runtime's fault: the loss latches the behaviour
    // without replacing the identity of what went wrong first.
    CHECK(step.fault_cause == FaultCause::RuntimeFault);

    // A fresh state of the runtime's own ZeroTorque must not unlock Standby.
    h.publish = true;
    h.lower.state = ExecutionState::ZeroTorque;
    h.lower.output = OutputAction::ZeroTorque;
    h.lower.fault_id = 0;
    h.lower.fault_reason = FaultReason::None;
    step = h.Step();
    CHECK(step.state == BehaviorState::Fault);
  }
  return 0;
}

}  // namespace

int main() {
  int failures = 0;
  failures += StartingHandover();
  failures += StopConfirmations();
  failures += ConfirmationOvertakenByTheState();
  failures += OneViewManySteps();
  failures += StopInFlightSuppressesTarget();
  failures += RepeatedStopKeyDoesNotReask();
  failures += RejectedRequestKeepsBehaviour();
  failures += RequestTimeoutFaults();
  failures += InitializationAdmission();
  failures += CompletionAndReadyHold();
  failures += OrdinaryStopsFromMotion();
  failures += EmergencyStopLatches();
  failures += RuntimeFaultRecovery();
  failures += UnexpectedTracking();
  failures += EmergencyStopInRuntimeFault();
  failures += SessionLossLatches();
  failures += LowerFellBackToDamping();
  failures += LowerFellBackToZeroTorque();
  failures += RuntimeRestartedLatches();
  failures += InitializationDeadlineFaults();
  failures += StateTimeoutFaults();
  failures += ExitSequence();
  failures += ExitFromTrackingKeepsTheContext();
  failures += StopFromMotionKeepsTheContext();
  failures += ResetConfirmationWaitsForTheState();
  failures += HeldConfirmationIsDropped();
  failures += LocomotionRequest();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
