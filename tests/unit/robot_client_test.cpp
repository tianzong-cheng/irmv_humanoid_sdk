// Unit contract of the official supervisor client
// (irmv::application::RobotClient).
//
// Every case runs on explicit simulated times and a recording ClientSink, so no
// clock, no thread and no DDS is involved: the client is a pure function of the
// states, the results and the times it is handed.
#include "irmv/application/robot_client.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::ClientSink;
using irmv::application::ClientView;
using irmv::application::IsStopOperation;
using irmv::application::RequestOutcome;
using irmv::application::RobotClient;
using irmv::configuration::ApplicationConfig;
using irmv::protocol::ExecutionRequest;
using irmv::protocol::ExecutionResult;
using irmv::protocol::ExecutionState;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::Operation;
using irmv::protocol::OutputAction;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;

constexpr std::string_view kConfigId = "config-digest";
constexpr std::string_view kLayoutId = "layout-digest";
constexpr TimeUs kRetryUs = 50'000;
constexpr TimeUs kDeadlineUs = 1'000'000;
constexpr TimeUs kStateTimeoutUs = 150'000;

ApplicationConfig MakeConfig() {
  ApplicationConfig config;
  config.runtime.joint_names = {"test_left", "test_right"};
  config.runtime.config_id = std::string(kConfigId);
  config.runtime.layout_id = std::string(kLayoutId);
  config.supervision.target_period_us = 20'000;
  config.supervision.heartbeat_period_us = 50'000;
  config.supervision.request_retry_us = kRetryUs;
  config.supervision.request_deadline_us = kDeadlineUs;
  config.supervision.state_timeout_us = kStateTimeoutUs;
  return config;
}

Id MakeId(std::uint8_t seed) {
  Id id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

// A supervisor identity whose request-id band starts at `base`: the band is the
// leading eight bytes big-endian, which is how a test drives the exhaustion
// path without ever sending two to the power of sixty-four requests.
Id BandId(std::uint64_t base) {
  Id id{};
  for (std::size_t index = 0; index < 8; ++index) {
    id[index] = static_cast<std::uint8_t>(base >> (56U - 8U * index));
  }
  return id;
}

RobotState MakeState(const Id &runtime_id, const Id &session_id, Sequence epoch,
                     Sequence state_seq, TimeUs stamp_us, ExecutionState state,
                     OutputAction action,
                     const Id &producer_id = irmv::protocol::kNoId,
                     Sequence fault_id = 0) {
  RobotState value;
  value.protocol_version = irmv::protocol::kProtocolVersion;
  value.runtime_id = runtime_id;
  value.session_id = session_id;
  value.producer_id = producer_id;
  value.epoch = epoch;
  value.state_seq = state_seq;
  value.stamp_us = stamp_us;
  value.execution_state = state;
  value.output_action = action;
  value.stop_reason = irmv::protocol::StopReason::None;
  value.fault_id = fault_id;
  value.healthy = true;
  value.config_id.Assign(kConfigId);
  value.layout_id.Assign(kLayoutId);
  return value;
}

ExecutionResult MakeResult(const Id &supervisor_id, const Id &runtime_id,
                           const Id &session_id, Sequence request_id,
                           TimeUs stamp_us, ResultStatus status,
                           RejectReason reason, Sequence epoch,
                           ExecutionState state, OutputAction action,
                           const Id &producer_id = irmv::protocol::kNoId) {
  ExecutionResult result;
  result.protocol_version = irmv::protocol::kProtocolVersion;
  result.supervisor_id = supervisor_id;
  result.request_id = request_id;
  result.session_id = session_id;
  result.runtime_id = runtime_id;
  result.execution_state = state;
  result.output_action = action;
  result.producer_id = producer_id;
  result.epoch = epoch;
  result.fault_id = 0;
  result.stamp_us = stamp_us;
  result.status = status;
  result.reason = reason;
  return result;
}

JointCommand MakeCommand(const ApplicationConfig &config) {
  JointCommand command;
  command.protocol_version = irmv::protocol::kProtocolVersion;
  command.producer_id = MakeId(0x33);
  command.epoch = 1;
  command.command_seq = 5;
  command.source_state_seq = 42;
  command.stamp_us = 10;
  command.layout_id.Assign(config.layout_id());
  command.joints.count = static_cast<std::uint32_t>(config.joint_count());
  return command;
}

// The exact payload a retry has to repeat: the header of the request and, for a
// Tracking request, its first frame as well.
bool SameCommand(const JointCommand &left, const JointCommand &right) {
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
  for (std::size_t index = 0; index < irmv::protocol::kMaxJoints; ++index) {
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

bool SameRequest(const ExecutionRequest &left, const ExecutionRequest &right) {
  if (left.operation != right.operation ||
      left.supervisor_id != right.supervisor_id ||
      left.request_id != right.request_id ||
      left.runtime_id != right.runtime_id ||
      left.session_id != right.session_id ||
      left.expected_epoch != right.expected_epoch ||
      left.proposed_epoch != right.proposed_epoch ||
      left.producer_id != right.producer_id ||
      left.fault_id != right.fault_id) {
    return false;
  }
  if (left.operation != Operation::Tracking) {
    return true;
  }
  return SameCommand(left.first_command, right.first_command);
}

// Records every payload the client hands to a transport, and can refuse a
// publish to prove the client keeps its pending request when the transport
// cannot take it.
class RecordingSink : public ClientSink {
 public:
  bool PublishRequest(const ExecutionRequest &request) override {
    requests.push_back(request);
    return request_accepted;
  }

  bool PublishCommand(const JointCommand &command) override {
    commands.push_back(command);
    return command_accepted;
  }

  bool PublishHeartbeat(
      const irmv::protocol::SupervisorHeartbeat &heartbeat) override {
    heartbeats.push_back(heartbeat);
    return heartbeat_accepted;
  }

  bool request_accepted{true};
  bool command_accepted{true};
  bool heartbeat_accepted{true};
  std::vector<ExecutionRequest> requests;
  std::vector<JointCommand> commands;
  std::vector<irmv::protocol::SupervisorHeartbeat> heartbeats;
};

}  // namespace

int main() {
  const ApplicationConfig config = MakeConfig();
  const Id runtime_id = MakeId(0x10);
  const Id session_id = MakeId(0x20);
  const Id supervisor_id = MakeId(0x30);
  const Id producer_id = MakeId(0x44);

  // The frozen stop lane: Damping, ZeroTorque and Reset may preempt a pending
  // request, every other operation may not.
  CHECK(IsStopOperation(Operation::Damping));
  CHECK(IsStopOperation(Operation::ZeroTorque));
  CHECK(IsStopOperation(Operation::Reset));
  CHECK(!IsStopOperation(Operation::AcquireSession));
  CHECK(!IsStopOperation(Operation::Tracking));

  // Feeds one matching state of instance `runtime_id`.
  const auto observe_state = [&](RobotClient &client, TimeUs stamp_us) {
    client.ObserveState(
        MakeState(runtime_id, irmv::protocol::kNoId, 0, 1, stamp_us,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        stamp_us + 20);
  };

  // Acquires a session: send, resend inside the deadline, then confirm with the
  // state published with the answer.
  const auto acquire = [&](RecordingSink &sink, RobotClient &client,
                           TimeUs start_us, Sequence *request_id) -> int {
    CHECK(client.BeginRequest(Operation::AcquireSession, start_us));
    CHECK(client.request_pending());
    CHECK(sink.requests.size() == 1);
    const Sequence id = sink.requests.back().request_id;
    CHECK(sink.requests.back().session_id == irmv::protocol::kNoId);
    CHECK(client.Poll(start_us).outcome == RequestOutcome::Pending);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, start_us + 100,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        start_us + 150);
    CHECK(client.Poll(start_us + 160).outcome == RequestOutcome::Pending);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 100, start_us + 120,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        start_us + 200);
    CHECK(client.Poll(start_us + 210).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_owned());
    if (request_id != nullptr) {
      *request_id = id;
    }
    return 0;
  };

  // ---- bootstrap, no early confirmation, nothing before the session --------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    JointCommand command = MakeCommand(config);

    CHECK(client.session_id() == irmv::protocol::kNoId);
    CHECK(client.runtime_id() == irmv::protocol::kNoId);
    CHECK(!client.session_owned());
    CHECK(!client.request_pending());
    CHECK(client.next_epoch() == 1);
    CHECK(!client.SendHeartbeat(100));
    CHECK(!client.SendCommand(&command, 100));
    CHECK(!client.BeginRequest(Operation::Damping, 100));
    CHECK(!client.BeginRequest(Operation::AcquireSession, 100));
    CHECK(sink.requests.empty());
    CHECK(sink.heartbeats.empty());
    CHECK(sink.commands.empty());

    const ClientView unknown = client.Poll(100);
    CHECK(unknown.outcome == RequestOutcome::None);
    CHECK(!unknown.state_known);
    CHECK(!unknown.state_fresh);
    CHECK(!unknown.session_owned && !unknown.session_lost);
    CHECK(!unknown.runtime_changed);

    observe_state(client, 900);
    const ClientView ready = client.Poll(1'000);
    CHECK(ready.outcome == RequestOutcome::None);
    CHECK(ready.state_known && ready.state_fresh);
    CHECK(!ready.session_owned && !ready.session_lost);
    CHECK(client.runtime_id() == runtime_id);
    CHECK(!client.BeginRequest(Operation::ZeroTorque, 1'000));

    CHECK(acquire(sink, client, 1'100, nullptr) == 0);
    CHECK(client.session_id() == session_id);
    CHECK(client.runtime_id() == runtime_id);
    CHECK(!client.request_pending());
    // Edge triggered: the confirmed outcome is reported once.
    CHECK(client.Poll(1'400).outcome == RequestOutcome::None);

    // Only now may a heartbeat or a target leave.
    CHECK(client.SendHeartbeat(1'500));
    CHECK(sink.heartbeats.size() == 1);
    CHECK(sink.heartbeats.back().supervisor_id == supervisor_id);
    CHECK(sink.heartbeats.back().session_id == session_id);
    CHECK(sink.heartbeats.back().runtime_id == runtime_id);
    CHECK(sink.heartbeats.back().heartbeat_seq == 1);
    CHECK(sink.heartbeats.back().stamp_us == 1'500);
    CHECK(client.SendHeartbeat(1'600));
    CHECK(sink.heartbeats.back().heartbeat_seq == 2);

    CHECK(client.SendCommand(&command, 1'700));
    CHECK(sink.commands.size() == 1);
    CHECK(command.protocol_version == irmv::protocol::kProtocolVersion);
    CHECK(command.runtime_id == runtime_id);
    CHECK(command.session_id == session_id);
    CHECK(command.stamp_us == 1'700);
    // The generator keeps its own fields.
    CHECK(command.producer_id == MakeId(0x33));
    CHECK(command.epoch == 1 && command.command_seq == 5);
    CHECK(command.source_state_seq == 42);

    // A frame of another layout or of another joint count is refused whole.
    JointCommand foreign_layout = MakeCommand(config);
    CHECK(foreign_layout.layout_id.Assign("foreign-layout"));
    CHECK(!client.SendCommand(&foreign_layout, 1'800));
    JointCommand short_frame = MakeCommand(config);
    short_frame.joints.count = 1;
    CHECK(!client.SendCommand(&short_frame, 1'800));
    CHECK(sink.commands.size() == 1);
  }

  // ---- identical payload, identical id, bounded resends -------------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 2'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 2'100));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.retry_count() == 1);
    CHECK(client.Poll(2'100).outcome == RequestOutcome::Pending);
    CHECK(client.Poll(2'100 + kRetryUs - 1).outcome == RequestOutcome::Pending);
    CHECK(sink.requests.size() == 1);

    CHECK(client.Poll(2'100 + kRetryUs).outcome == RequestOutcome::Pending);
    CHECK(sink.requests.size() == 2);
    CHECK(client.retry_count() == 2);
    CHECK(SameRequest(sink.requests[0], sink.requests[1]));
    CHECK(sink.requests[1].request_id == id);

    CHECK(client.Poll(2'100 + 2 * kRetryUs).outcome == RequestOutcome::Pending);
    CHECK(sink.requests.size() == 3);
    CHECK(client.retry_count() == 3);
    CHECK(SameRequest(sink.requests[1], sink.requests[2]));

    // A different payload while one is pending is refused; only a stop may
    // take over (below).
    CHECK(!client.BeginRequest(Operation::AcquireSession, 2'150));
    JointCommand first_frame = MakeCommand(config);
    CHECK(!client.BeginRequest(Operation::Tracking, 2'150, producer_id,
                               &first_frame));
    CHECK(sink.requests.size() == 3);
    CHECK(client.request_pending() && client.pending_request_id() == id);

    // The answer of the very first send still confirms the retried request.
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 2'100 + 150,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        2'100 + 200);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 101, 2'100 + 180,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        2'100 + 250);
    CHECK(client.Poll(2'100 + 260).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_id() == session_id);
  }

  // ---- a transport that refuses the first send keeps the request pending ---
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 2'900);
    sink.request_accepted = false;
    CHECK(client.BeginRequest(Operation::AcquireSession, 3'000));
    CHECK(sink.requests.size() == 1);
    CHECK(client.request_pending());
    sink.request_accepted = true;
    CHECK(client.Poll(3'000 + kRetryUs).outcome == RequestOutcome::Pending);
    CHECK(sink.requests.size() == 2);
    CHECK(SameRequest(sink.requests[0], sink.requests[1]));
  }

  // ---- state first and result first both confirm --------------------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 4'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 4'100));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(4'100).outcome == RequestOutcome::Pending);
    // The state is observed before the result that it witnesses.
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 102, 4'120,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        4'180);
    CHECK(client.Poll(4'190).outcome == RequestOutcome::Pending);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 4'110,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        4'200);
    CHECK(client.Poll(4'210).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_owned() && client.session_id() == session_id);
  }
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 5'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 5'100));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(5'100).outcome == RequestOutcome::Pending);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 5'110,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        5'150);
    CHECK(client.Poll(5'160).outcome == RequestOutcome::Pending);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 103, 5'120,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        5'200);
    CHECK(client.Poll(5'210).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_owned() && client.session_id() == session_id);
  }

  // ---- a state older than the answer never witnesses it -------------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 6'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 6'100));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(6'100).outcome == RequestOutcome::Pending);
    // A state the runtime published before the answer was produced.
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 104, 6'090,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        6'150);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 6'200,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        6'250);
    CHECK(client.Poll(6'260).outcome == RequestOutcome::Pending);
    CHECK(!client.session_owned());
    // A result stamped before the first send cannot answer it either.
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 6'050,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        6'270);
    CHECK(client.Poll(6'280).outcome == RequestOutcome::Pending);
    // The state published after the answer completes it.
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 105, 6'210,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        6'300);
    CHECK(client.Poll(6'310).outcome == RequestOutcome::Confirmed);
  }

  // ---- a rejection resolves the pending request with its reason -----------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 7'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 7'100));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(7'100).outcome == RequestOutcome::Pending);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 7'120,
                   ResultStatus::Rejected, RejectReason::SessionBusy, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        7'160);
    const ClientView rejected = client.Poll(7'170);
    CHECK(rejected.outcome == RequestOutcome::Rejected);
    CHECK(rejected.reject_reason == RejectReason::SessionBusy);
    CHECK(!rejected.session_owned);
    CHECK(!client.session_owned() &&
          client.session_id() == irmv::protocol::kNoId);
    CHECK(!client.request_pending());
    // Edge triggered, and nothing revives the request afterwards.
    CHECK(client.Poll(7'180).outcome == RequestOutcome::None);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 106, 7'130,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        7'200);
    CHECK(client.Poll(7'210).outcome == RequestOutcome::None);
    CHECK(!client.session_owned());
  }

  // ---- a late Applied answer after a fault confirms nothing ---------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 8'000);
    CHECK(acquire(sink, client, 8'100, nullptr) == 0);
    JointCommand first_frame = MakeCommand(config);
    CHECK(client.BeginRequest(Operation::Tracking, 8'400, producer_id,
                              &first_frame));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(8'400).outcome == RequestOutcome::Pending);
    // The runtime applied the authorization, and then latched a fault before
    // publishing the agreeing state.
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 8'500,
                   ResultStatus::Applied, RejectReason::None, 1,
                   ExecutionState::Tracking, OutputAction::Tracking,
                   producer_id),
        8'550);
    CHECK(client.Poll(8'560).outcome == RequestOutcome::Pending);
    client.ObserveState(
        MakeState(runtime_id, session_id, 2, 300, 8'520, ExecutionState::Fault,
                  OutputAction::ZeroTorque, irmv::protocol::kNoId, 9),
        8'600);
    CHECK(client.Poll(8'610).outcome == RequestOutcome::Pending);
    // The refused confirmation leaves the session itself alone.
    CHECK(client.session_owned() && client.session_id() == session_id);
    // Even a later state that would agree cannot revive the answer.
    client.ObserveState(MakeState(runtime_id, session_id, 1, 301, 8'700,
                                  ExecutionState::Tracking,
                                  OutputAction::Tracking, producer_id),
                        8'750);
    CHECK(client.Poll(8'760).outcome == RequestOutcome::Pending);
    // Fresh states keep the link alive while the refused request runs to its
    // own deadline.
    client.ObserveState(
        MakeState(runtime_id, session_id, 2, 302, 1'008'000,
                  ExecutionState::Fault, OutputAction::ZeroTorque,
                  irmv::protocol::kNoId, 9),
        1'008'200);
    CHECK(client.Poll(8'400 + kDeadlineUs - 1).outcome ==
          RequestOutcome::Pending);
    CHECK(client.Poll(8'400 + kDeadlineUs).outcome == RequestOutcome::TimedOut);
    CHECK(!client.request_pending());
    CHECK(client.Poll(8'400 + kDeadlineUs + 1).outcome == RequestOutcome::None);
  }

  // ---- a stop preempts a pending request ---------------------------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 9'000);
    Sequence acquire_id = 0;
    CHECK(acquire(sink, client, 9'100, &acquire_id) == 0);

    JointCommand first_frame = MakeCommand(config);
    CHECK(client.BeginRequest(Operation::Tracking, 9'400, producer_id,
                              &first_frame));
    CHECK(sink.requests.size() == 2);
    const Sequence ordinary_id = sink.requests.back().request_id;
    CHECK(ordinary_id > acquire_id);
    CHECK(sink.requests.back().proposed_epoch > 0);
    CHECK(client.Poll(9'400).outcome == RequestOutcome::Pending);

    CHECK(client.BeginRequest(Operation::ZeroTorque, 9'420));
    CHECK(sink.requests.size() == 3);
    const Sequence stop_id = sink.requests.back().request_id;
    CHECK(stop_id > ordinary_id);
    CHECK(sink.requests.back().operation == Operation::ZeroTorque);
    CHECK(client.pending_request_id() == stop_id);
    CHECK(client.retry_count() == 1);
    CHECK(client.Poll(9'430).outcome == RequestOutcome::Pending);

    // A late Applied answer of the preempted request resolves nothing, even
    // with an agreeing state behind it.
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, ordinary_id, 9'500,
                   ResultStatus::Applied, RejectReason::None, 1,
                   ExecutionState::Tracking, OutputAction::Tracking,
                   producer_id),
        9'550);
    client.ObserveState(MakeState(runtime_id, session_id, 1, 400, 9'520,
                                  ExecutionState::Tracking,
                                  OutputAction::Tracking, producer_id),
                        9'600);
    CHECK(client.Poll(9'610).outcome == RequestOutcome::Pending);

    // The stop's own answer confirms it, and the epoch advances with it.
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, stop_id, 9'700,
                   ResultStatus::Applied, RejectReason::None, 2,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        9'750);
    client.ObserveState(
        MakeState(runtime_id, session_id, 2, 401, 9'720,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        9'800);
    CHECK(client.Poll(9'810).outcome == RequestOutcome::Confirmed);
    CHECK(client.next_epoch() == 3);
  }

  // ---- sequence exhaustion leaves the session as it is -------------------
  {
    RecordingSink sink;
    // The band starts at the value before the reserved last sequence value.
    const Id band_id = BandId(0xFFFFFFFFFFFFFFFEULL);
    RobotClient client(sink, config, band_id);
    observe_state(client, 20'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 20'100));
    CHECK(sink.requests.size() == 1);
    const Sequence last_id = sink.requests.back().request_id;
    CHECK(last_id == 0xFFFFFFFFFFFFFFFEULL);
    client.ObserveResult(
        MakeResult(band_id, runtime_id, session_id, last_id, 20'150,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        20'180);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 500, 20'160,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        20'200);
    CHECK(client.Poll(20'210).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_owned());

    JointCommand first_frame = MakeCommand(config);
    CHECK(!client.BeginRequest(Operation::ZeroTorque, 20'300));
    CHECK(!client.BeginRequest(Operation::Tracking, 20'300, producer_id,
                               &first_frame));
    CHECK(sink.requests.size() == 1);
    CHECK(!client.request_pending());
    CHECK(client.session_owned() && client.session_id() == session_id);
    // Still live after the refusal.
    CHECK(client.SendHeartbeat(20'400));
    CHECK(sink.heartbeats.size() == 1);
  }

  // ---- a changed runtime id invalidates the old session ------------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 30'000);
    CHECK(acquire(sink, client, 30'100, nullptr) == 0);
    CHECK(client.session_id() == session_id);

    const Id new_runtime = MakeId(0x60);
    client.ObserveState(
        MakeState(new_runtime, irmv::protocol::kNoId, 0, 600, 30'600,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        30'650);
    const ClientView changed = client.Poll(30'700);
    CHECK(changed.outcome == RequestOutcome::Unknown);
    CHECK(changed.runtime_changed);
    CHECK(changed.session_lost);
    CHECK(!changed.session_owned);
    CHECK(changed.state_known && changed.state_fresh);
    CHECK(changed.state.runtime_id == new_runtime);
    CHECK(client.session_id() == irmv::protocol::kNoId);
    CHECK(client.runtime_id() == new_runtime);
    // Nothing may leave on the old session.
    JointCommand command = MakeCommand(config);
    CHECK(!client.SendHeartbeat(30'710));
    CHECK(!client.SendCommand(&command, 30'710));
    CHECK(!client.BeginRequest(Operation::Damping, 30'720));
    CHECK(sink.heartbeats.empty() && sink.commands.empty());
    CHECK(client.Poll(30'730).outcome == RequestOutcome::None);
    CHECK(changed.runtime_changed && client.Poll(30'740).runtime_changed);

    // A new session on the new instance is a fresh identity.
    const Id new_session = MakeId(0x70);
    CHECK(client.BeginRequest(Operation::AcquireSession, 30'800));
    const Sequence id = sink.requests.back().request_id;
    CHECK(sink.requests.size() == 2);
    client.ObserveResult(
        MakeResult(supervisor_id, new_runtime, new_session, id, 30'850,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        30'900);
    client.ObserveState(
        MakeState(new_runtime, new_session, 0, 601, 30'860,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        30'950);
    const ClientView acquired = client.Poll(30'960);
    CHECK(acquired.outcome == RequestOutcome::Confirmed);
    CHECK(acquired.session_owned && !acquired.session_lost);
    CHECK(!acquired.runtime_changed);
    CHECK(client.session_id() == new_session);
    CHECK(client.session_id() != session_id);
  }

  // ---- a foreign layout or configuration is never a target layout ---------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 40'000);
    CHECK(client.runtime_id() == runtime_id);

    RobotState foreign_layout =
        MakeState(runtime_id, irmv::protocol::kNoId, 0, 700, 40'100,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque);
    CHECK(foreign_layout.layout_id.Assign("foreign-layout"));
    client.ObserveState(foreign_layout, 40'150);
    const ClientView layout_view = client.Poll(40'200);
    CHECK(layout_view.outcome == RequestOutcome::None);
    CHECK(!layout_view.state_known);
    CHECK(client.runtime_id() == irmv::protocol::kNoId);
    CHECK(!client.BeginRequest(Operation::AcquireSession, 40'300));
    CHECK(sink.requests.empty());

    RobotState foreign_config =
        MakeState(runtime_id, irmv::protocol::kNoId, 0, 701, 40'400,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque);
    CHECK(foreign_config.config_id.Assign("foreign-config"));
    client.ObserveState(foreign_config, 40'450);
    CHECK(!client.Poll(40'500).state_known);
    CHECK(client.runtime_id() == irmv::protocol::kNoId);
    CHECK(!client.BeginRequest(Operation::AcquireSession, 40'600));

    // An empty identity is not an instance either.
    RobotState no_instance =
        MakeState(irmv::protocol::kNoId, irmv::protocol::kNoId, 0, 702, 40'700,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque);
    client.ObserveState(no_instance, 40'750);
    CHECK(!client.Poll(40'800).state_known);
    CHECK(!client.BeginRequest(Operation::AcquireSession, 40'900));

    // The configuration's own layout brings the instance back.
    observe_state(client, 41'000);
    CHECK(client.runtime_id() == runtime_id);
    CHECK(client.BeginRequest(Operation::AcquireSession, 41'100));
    CHECK(sink.requests.size() == 1);
  }
  {
    // A layout change while a session is owned is a lost session.
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 42'000);
    CHECK(acquire(sink, client, 42'100, nullptr) == 0);
    RobotState changed_layout =
        MakeState(runtime_id, session_id, 0, 800, 42'600,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque);
    CHECK(changed_layout.layout_id.Assign("foreign-layout"));
    client.ObserveState(changed_layout, 42'650);
    const ClientView view = client.Poll(42'700);
    CHECK(view.outcome == RequestOutcome::Unknown);
    CHECK(view.session_lost && !view.session_owned);
    CHECK(client.session_id() == irmv::protocol::kNoId);
  }

  // ---- a silent state link loses the session, once ------------------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 50'000);
    CHECK(acquire(sink, client, 50'100, nullptr) == 0);
    // The state that witnessed the answer is the last one received.
    const TimeUs witness_rx_us = 50'300;

    // Inside the timeout the link is still live.
    CHECK(client.Poll(witness_rx_us + kStateTimeoutUs).outcome ==
          RequestOutcome::None);
    CHECK(client.session_owned());

    const ClientView lost = client.Poll(witness_rx_us + kStateTimeoutUs + 1);
    CHECK(lost.outcome == RequestOutcome::Unknown);
    CHECK(lost.session_lost && !lost.session_owned);
    CHECK(!lost.state_known && !lost.state_fresh);
    CHECK(!lost.runtime_changed);
    CHECK(client.session_id() == irmv::protocol::kNoId);
    CHECK(client.runtime_id() == runtime_id);
    CHECK(client.next_epoch() == 1);
    JointCommand command = MakeCommand(config);
    CHECK(!client.SendHeartbeat(witness_rx_us + kStateTimeoutUs + 2));
    CHECK(!client.SendCommand(&command, witness_rx_us + kStateTimeoutUs + 2));
    // Reported once, but the loss stays visible until a new session.
    const ClientView again = client.Poll(witness_rx_us + kStateTimeoutUs + 3);
    CHECK(again.outcome == RequestOutcome::None);
    CHECK(again.session_lost && !again.session_owned);
  }
  {
    // A pending request is dropped by the link loss, not reported as its own
    // timeout.
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 60'000);
    CHECK(acquire(sink, client, 60'100, nullptr) == 0);
    // The last state was received at 60'300.
    JointCommand first_frame = MakeCommand(config);
    CHECK(client.BeginRequest(Operation::Tracking, 60'400, producer_id,
                              &first_frame));
    CHECK(client.Poll(60'400).outcome == RequestOutcome::Pending);
    CHECK(client.request_pending());
    const ClientView lost = client.Poll(60'320 + kStateTimeoutUs + 1);
    CHECK(lost.outcome == RequestOutcome::Unknown);
    CHECK(!client.request_pending());
    CHECK(client.session_id() == irmv::protocol::kNoId);
  }

  // ---- the pending request is abandoned on the exit path -----------------
  {
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 70'000);
    CHECK(client.BeginRequest(Operation::AcquireSession, 70'100));
    CHECK(client.Poll(70'100).outcome == RequestOutcome::Pending);
    client.AbandonRequest();
    CHECK(!client.request_pending());
    CHECK(client.Poll(70'110).outcome == RequestOutcome::None);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id,
                   sink.requests.back().request_id, 70'120,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        70'160);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 900, 70'130,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        70'200);
    CHECK(client.Poll(70'210).outcome == RequestOutcome::None);
    CHECK(!client.session_owned());
  }

  // ---- the witness survives until the window is full ---------------------
  {
    // The witness of an answer is the first state published at or after it, so
    // it has to stay in the history while seven later states arrive: the depth
    // the header promises is exactly the last eight observations.
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    observe_state(client, 80'000);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 901, 80'120,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        80'150);
    CHECK(client.BeginRequest(Operation::AcquireSession, 80'100));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(80'100).outcome == RequestOutcome::Pending);
    // The seven later states deliberately contradict the answer (they carry the
    // next watermark), so the scan can only confirm while the real witness at
    // 80'120 is still retained: with a history one shorter than the promised
    // eight the oldest entry would be one of these and the request would be
    // refused instead of confirmed.
    for (Sequence index = 1; index <= 7; ++index) {
      const TimeUs stamp = 80'120 + index * 10;
      client.ObserveState(
          MakeState(runtime_id, session_id, 1, 901 + index, stamp,
                    ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
          stamp + 20);
    }
    CHECK(client.Poll(80'300).outcome == RequestOutcome::Pending);
    // The answer arrives last; the witness that preceded it still confirms it.
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 80'120,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        80'320);
    CHECK(client.Poll(80'330).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_id() == session_id);
  }

  // ---- a rolled-over window still resolves a later request ---------------
  {
    // Ten observations roll the eight-deep window over completely while the
    // instance, the session and the configuration stay the same: a request
    // that starts afterwards must still be confirmed, so the rollover cannot
    // leave the history in a state that refuses everything after it.
    RecordingSink sink;
    RobotClient client(sink, config, supervisor_id);
    for (Sequence index = 0; index < 10; ++index) {
      const TimeUs stamp = 90'000 + index * 10;
      client.ObserveState(
          MakeState(runtime_id, session_id, 0, 950 + index, stamp,
                    ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
          stamp + 5);
    }
    CHECK(client.runtime_id() == runtime_id);
    CHECK(client.BeginRequest(Operation::AcquireSession, 90'200));
    const Sequence id = sink.requests.back().request_id;
    CHECK(client.Poll(90'200).outcome == RequestOutcome::Pending);
    client.ObserveResult(
        MakeResult(supervisor_id, runtime_id, session_id, id, 90'220,
                   ResultStatus::Applied, RejectReason::None, 0,
                   ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        90'250);
    CHECK(client.Poll(90'260).outcome == RequestOutcome::Pending);
    client.ObserveState(
        MakeState(runtime_id, session_id, 0, 960, 90'230,
                  ExecutionState::ZeroTorque, OutputAction::ZeroTorque),
        90'280);
    CHECK(client.Poll(90'290).outcome == RequestOutcome::Confirmed);
    CHECK(client.session_owned());
  }

  return 0;
}
