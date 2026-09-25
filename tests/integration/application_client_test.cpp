// Real-DDS contract of the official client against a standalone robot_runtime.
//
// Usage: application_client_test <path to robot_runtime> <source directory>
//
// Every scenario spawns a real frozen runtime (a fake deployment) on a private
// DDS domain and drives it through the production ClientTransport, feeding the
// events it returns into the production RobotClient by hand so the arrival
// order of the answer and the state is deterministic. Every wait is bounded,
// every process is cleaned up on both the success and the failure path, and no
// check claims anything about a motor: Applied is a host-path fact only.
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "irmv/application/robot_client.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"
#include "irmv/transport/client_transport.hpp"
#include "irmv/transport/dds_transport.hpp"

namespace {

namespace fs = std::filesystem;
using nlohmann::json;

using irmv::application::ClientSink;
using irmv::application::ClientView;
using irmv::application::RequestOutcome;
using irmv::application::RobotClient;
using irmv::configuration::ApplicationConfig;
using irmv::configuration::ConfigOverrides;
using irmv::configuration::LoadApplicationConfig;
using irmv::protocol::ExecutionRequest;
using irmv::protocol::ExecutionResult;
using irmv::protocol::ExecutionState;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::Operation;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::transport::ClientEvents;
using irmv::transport::ClientTransport;

constexpr int kExitOk = 0;
constexpr int kExitFailure = 1;
constexpr int kExitUsage = 2;

// The configuration on disk carries this domain; the command line override must
// win on both sides, which is what makes the two configuration digests meet.
constexpr std::uint32_t kConfigDomain = 7;

const Id kProducerId = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x33};

TimeUs NowUs() {
  return static_cast<TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

void SleepMs(int milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

const char *OutcomeName(RequestOutcome outcome) {
  switch (outcome) {
    case RequestOutcome::None:
      return "None";
    case RequestOutcome::Pending:
      return "Pending";
    case RequestOutcome::Confirmed:
      return "Confirmed";
    case RequestOutcome::Rejected:
      return "Rejected";
    case RequestOutcome::TimedOut:
      return "TimedOut";
    case RequestOutcome::Unknown:
      return "Unknown";
  }
  return "?";
}

const char *ReasonName(RejectReason reason) {
  switch (reason) {
    case RejectReason::None:
      return "None";
    case RejectReason::VersionMismatch:
      return "VersionMismatch";
    case RejectReason::Malformed:
      return "Malformed";
    case RejectReason::WrongInstance:
      return "WrongInstance";
    case RejectReason::WrongSession:
      return "WrongSession";
    case RejectReason::SessionBusy:
      return "SessionBusy";
    case RejectReason::StaleRequest:
      return "StaleRequest";
    case RejectReason::RequestConflict:
      return "RequestConflict";
    case RejectReason::EpochMismatch:
      return "EpochMismatch";
    case RejectReason::InvalidTransition:
      return "InvalidTransition";
    case RejectReason::InvalidCommand:
      return "InvalidCommand";
    case RejectReason::StaleSource:
      return "StaleSource";
    case RejectReason::Discontinuity:
      return "Discontinuity";
    case RejectReason::RecoveryNotReady:
      return "RecoveryNotReady";
    case RejectReason::FaultMismatch:
      return "FaultMismatch";
    case RejectReason::WriteFailed:
      return "WriteFailed";
    case RejectReason::QueueFull:
      return "QueueFull";
  }
  return "?";
}

std::string Number(std::uint64_t value) { return std::to_string(value); }

// The request-id band a supervisor identity carries: its first eight bytes,
// big-endian. The client derives its sequence start the same way, so this pins
// the property the identity has to have rather than the client's arithmetic.
Sequence BandStart(const Id &supervisor_id) {
  Sequence base = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    base = (base << 8U) | supervisor_id[index];
  }
  return base;
}

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

// A retry has to repeat the whole payload: the header and, for Tracking, the
// first frame as well.
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

// ---- process control ------------------------------------------------------

// Removes the temporary workspace on every path out of main, including the
// early returns that happen before any transport exists.
class WorkspaceGuard {
 public:
  explicit WorkspaceGuard(fs::path path) : path_(std::move(path)) {}

  ~WorkspaceGuard() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }

  WorkspaceGuard(const WorkspaceGuard &) = delete;
  WorkspaceGuard &operator=(const WorkspaceGuard &) = delete;

 private:
  fs::path path_;
};

// One robot_runtime child with a log file and a bounded lifetime. The
// destructor kills whatever is still running, so every failure path is clean.
class RuntimeProcess {
 public:
  RuntimeProcess(const fs::path &binary, const std::vector<std::string> &args,
                 const fs::path &log_path)
      : log_path_(log_path) {
    const int log_fd =
        ::open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    std::vector<std::string> owned;
    owned.push_back(binary.string());
    for (const std::string &argument : args) {
      owned.push_back(argument);
    }
    std::vector<char *> argv;
    argv.reserve(owned.size() + 1);
    for (std::string &argument : owned) {
      argv.push_back(argument.data());
    }
    argv.push_back(nullptr);

    pid_ = ::fork();
    if (pid_ == 0) {
      if (log_fd >= 0) {
        ::dup2(log_fd, STDOUT_FILENO);
        ::dup2(log_fd, STDERR_FILENO);
      }
      ::execv(argv[0], argv.data());
      ::_exit(127);
    }
    if (log_fd >= 0) {
      ::close(log_fd);
    }
  }

  ~RuntimeProcess() {
    if (!Stop(2000)) {
      std::cerr << "runtime " << pid_ << " could not be stopped\n";
    }
  }

  RuntimeProcess(const RuntimeProcess &) = delete;
  RuntimeProcess &operator=(const RuntimeProcess &) = delete;

  // True while the child has neither exited nor been reaped.
  bool Running() {
    if (pid_ <= 0 || reaped_) {
      return false;
    }
    int status = 0;
    const pid_t result = ::waitpid(pid_, &status, WNOHANG);
    if (result == 0) {
      return true;
    }
    reaped_ = true;
    exit_code_ = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return false;
  }

  // SIGTERM, then a bounded wait, then SIGKILL: the runtime has its own
  // shutdown and gets the chance to take it.
  bool Stop(int timeout_ms) {
    if (!Running()) {
      return true;
    }
    ::kill(pid_, SIGTERM);
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!Running()) {
        return true;
      }
      SleepMs(5);
    }
    ::kill(pid_, SIGKILL);
    const std::chrono::steady_clock::time_point kill_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < kill_deadline) {
      if (!Running()) {
        break;
      }
      SleepMs(5);
    }
    return !Running();
  }

  std::string LogTail(std::size_t bytes) const {
    std::ifstream input(log_path_, std::ios::binary);
    if (!input) {
      return {};
    }
    input.seekg(0, std::ios::end);
    const std::streamoff size = input.tellg();
    const std::streamoff start = size > static_cast<std::streamoff>(bytes)
                                     ? size - static_cast<std::streamoff>(bytes)
                                     : 0;
    input.seekg(start);
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
  }

 private:
  fs::path log_path_;
  pid_t pid_{-1};
  bool reaped_{false};
  int exit_code_{-1};
};

// ---- the scenario fixture -------------------------------------------------

class RecordingSink : public ClientSink {
 public:
  explicit RecordingSink(ClientTransport &transport) : transport_(transport) {}

  bool PublishRequest(const ExecutionRequest &request) override {
    requests.push_back(request);
    return transport_.PublishRequest(request);
  }

  bool PublishCommand(const JointCommand &command) override {
    commands.push_back(command);
    return transport_.PublishCommand(command);
  }

  bool PublishHeartbeat(
      const irmv::protocol::SupervisorHeartbeat &heartbeat) override {
    heartbeats.push_back(heartbeat);
    return transport_.PublishHeartbeat(heartbeat);
  }

  std::vector<ExecutionRequest> requests;
  std::vector<JointCommand> commands;
  std::vector<irmv::protocol::SupervisorHeartbeat> heartbeats;

 private:
  ClientTransport &transport_;
};

struct Fixture {
  const ApplicationConfig *config{nullptr};
  ClientTransport *transport{nullptr};
  RecordingSink *sink{nullptr};
  RobotClient *client{nullptr};
  TimeUs next_heartbeat_us{0};
  std::uint64_t checks{0};
  std::uint64_t failures{0};
};

bool Check(Fixture &fx, bool condition, std::string_view name,
           std::string detail = {}) {
  ++fx.checks;
  if (condition) {
    return true;
  }
  ++fx.failures;
  std::cerr << "FAIL: " << name;
  if (!detail.empty()) {
    std::cerr << " (" << detail << ")";
  }
  std::cerr << "\n";
  return false;
}

std::uint32_t JointCount(const ApplicationConfig &config) {
  return static_cast<std::uint32_t>(config.joint_count());
}

// The answer of the request under observation, held back in the state-first
// order until a state published at or after it has been fed.
struct PendingAnswer {
  Sequence request_id{0};
  bool found{false};
  bool fed{false};
  ExecutionResult result{};
  TimeUs rx_us{0};
};

// Refreshes the lease of a live session on the supervision period. Only the
// supervision loop sends heartbeats, and this is that loop.
void PumpHeartbeat(Fixture &fx) {
  if (!fx.client->session_owned()) {
    return;
  }
  const TimeUs now = NowUs();
  if (fx.next_heartbeat_us == 0 || now >= fx.next_heartbeat_us) {
    fx.client->SendHeartbeat(now);
    fx.next_heartbeat_us = now + fx.config->supervision.heartbeat_period_us;
  }
}

// Feeds one drained batch into the client. In the result-first order the
// results of the batch are fed before its state; in the state-first order the
// answer of the request under observation is held back until a state published
// at or after it has been fed first.
void FeedBatch(Fixture &fx, const ClientEvents &events, PendingAnswer *answer,
               bool result_first) {
  for (std::size_t index = 0; index < events.result_count; ++index) {
    const auto &received = events.results[index];
    const bool is_answer =
        answer != nullptr && received.value.request_id == answer->request_id;
    if (is_answer) {
      // The newest answer of the observed request; a retry replays the same
      // one.
      answer->found = true;
      answer->result = received.value;
      answer->rx_us = received.rx_us;
      if (!result_first || answer->fed) {
        continue;
      }
    }
    fx.client->ObserveResult(received.value, received.rx_us);
    if (is_answer) {
      answer->fed = true;
    }
  }
  if (events.has_state) {
    fx.client->ObserveState(events.state, events.state_rx_us);
  }
  if (!result_first && answer != nullptr && answer->found && !answer->fed &&
      events.has_state && events.state.stamp_us >= answer->result.stamp_us) {
    // The witness arrived before the answer it belongs to.
    fx.client->ObserveResult(answer->result, answer->rx_us);
    answer->fed = true;
  }
}

// One supervision iteration: take the events, feed them and refresh the lease.
ClientView Step(Fixture &fx, PendingAnswer *answer, bool result_first) {
  const ClientView view = fx.client->Poll(NowUs());
  FeedBatch(fx, fx.transport->TakeEvents(), answer, result_first);
  PumpHeartbeat(fx);
  return view;
}

// Pumps the transport into the client until the pending request is confirmed.
bool ConfirmPending(Fixture &fx, bool result_first, int timeout_ms,
                    std::string_view name) {
  PendingAnswer answer;
  answer.request_id = fx.client->pending_request_id();
  const TimeUs deadline = NowUs() + static_cast<TimeUs>(timeout_ms) * 1000;
  while (NowUs() < deadline) {
    const ClientView view = Step(fx, &answer, result_first);
    if (view.outcome == RequestOutcome::Confirmed) {
      return true;
    }
    if (view.outcome != RequestOutcome::None &&
        view.outcome != RequestOutcome::Pending) {
      return Check(fx, false, name,
                   std::string("outcome ") + OutcomeName(view.outcome) + "/" +
                       ReasonName(view.reject_reason));
    }
    SleepMs(1);
  }
  return Check(fx, false, name, "not confirmed inside the bound");
}

// Waits for a state the client accepts as its instance, settled out of Boot: a
// stop requested during the boot grace would be refused as an invalid
// transition, which is the runtime's own rule and not the client's.
bool WaitForInstance(Fixture &fx, int timeout_ms, std::string_view name) {
  const TimeUs deadline = NowUs() + static_cast<TimeUs>(timeout_ms) * 1000;
  while (NowUs() < deadline) {
    const ClientView view = Step(fx, nullptr, true);
    if (view.state_known &&
        view.state.execution_state == ExecutionState::ZeroTorque) {
      return Check(fx, true, name, "instance observed");
    }
    SleepMs(1);
  }
  return Check(fx, false, name, "no matching settled state inside the bound");
}

bool AcquireSession(Fixture &fx, bool result_first, int timeout_ms,
                    std::string_view name, Id *session_id) {
  const std::size_t before = fx.sink->requests.size();
  if (!Check(fx, fx.client->BeginRequest(Operation::AcquireSession, NowUs()),
             std::string(name) + ": begin")) {
    return false;
  }
  if (!Check(fx, fx.sink->requests.size() == before + 1,
             std::string(name) + ": one bootstrap request left")) {
    return false;
  }
  const ExecutionRequest &request = fx.sink->requests.back();
  Check(fx,
        request.session_id == irmv::protocol::kNoId &&
            request.expected_epoch == 0 && request.proposed_epoch == 0 &&
            request.producer_id == irmv::protocol::kNoId &&
            request.fault_id == 0,
        std::string(name) + ": bootstrap header");
  if (!ConfirmPending(fx, result_first, timeout_ms, name)) {
    return false;
  }
  *session_id = fx.client->session_id();
  return Check(
      fx, *session_id != irmv::protocol::kNoId && fx.client->session_owned(),
      std::string(name) + ": session owned");
}

// ---- scenarios ------------------------------------------------------------

// Scenario 1: the bootstrap is confirmed by an answer that arrived before the
// state that witnesses it, and nothing may leave before it is confirmed.
bool ScenarioAcquireResultFirst(Fixture &fx, Id *session_id) {
  JointCommand command;
  const std::size_t heartbeats = fx.sink->heartbeats.size();
  const std::size_t commands = fx.sink->commands.size();
  Check(fx, !fx.client->SendHeartbeat(NowUs()),
        "acquire: no heartbeat before the session is acquired");
  Check(fx, !fx.client->SendCommand(&command, NowUs()),
        "acquire: no target before the session is acquired");
  Check(fx,
        fx.sink->heartbeats.size() == heartbeats &&
            fx.sink->commands.size() == commands,
        "acquire: nothing left the client before the session");
  Check(fx, !fx.client->session_owned(),
        "acquire: the session is not owned before it is confirmed");
  return AcquireSession(fx, true, 5000, "acquire (result first)", session_id);
}

// Scenario 3: the answer of one request is deliberately not fed until after the
// retry period, so the client resends it identically and the runtime answers
// the recorded result instead of applying the request twice.
bool ScenarioRetry(Fixture &fx, Id *session_id) {
  const std::uint64_t failures_before = fx.failures;
  const TimeUs start = NowUs();
  if (!Check(fx, fx.client->BeginRequest(Operation::Damping, start),
             "retry: begin stop")) {
    return false;
  }
  const Sequence request_id = fx.client->pending_request_id();
  const std::size_t sends = fx.sink->requests.size();
  if (!Check(fx, fx.sink->requests.back().request_id == request_id,
             "retry: the stop left once")) {
    return false;
  }
  const ExecutionRequest first_send = fx.sink->requests.back();
  Check(fx, fx.client->retry_count() == 1, "retry: one send so far");
  Check(fx, fx.client->Poll(NowUs()).outcome == RequestOutcome::Pending,
        "retry: pending");
  Check(fx, fx.sink->requests.size() == sends, "retry: no early resend");

  const int retry_ms =
      static_cast<int>(fx.config->supervision.request_retry_us / 1000);
  SleepMs(retry_ms + 20);
  const ClientView after_retry = fx.client->Poll(NowUs());
  Check(fx, after_retry.outcome == RequestOutcome::Pending,
        "retry: still pending after the retry period");
  Check(fx, fx.sink->requests.size() == sends + 1,
        "retry: resent after the retry period",
        Number(fx.sink->requests.size()));
  if (fx.sink->requests.size() == sends + 1) {
    Check(fx, SameRequest(first_send, fx.sink->requests.back()),
          "retry: the resend repeats the payload byte for byte");
    Check(fx, fx.sink->requests.back().request_id == request_id,
          "retry: the resend repeats the request id");
  }
  Check(fx, fx.client->retry_count() == 2, "retry: two sends of the same id");

  // Both answers now travel: the answer of the first send and the replay of the
  // recorded one. Identical stamps and epochs are what proves the runtime
  // applied the stop once and replayed the answer afterwards.
  std::vector<ExecutionResult> answers;
  bool confirmed = false;
  const TimeUs deadline = NowUs() + 5'000'000;
  while (NowUs() < deadline) {
    const ClientView view = fx.client->Poll(NowUs());
    if (view.outcome == RequestOutcome::Confirmed) {
      confirmed = true;
    } else if (view.outcome == RequestOutcome::Rejected ||
               view.outcome == RequestOutcome::TimedOut ||
               view.outcome == RequestOutcome::Unknown) {
      Check(fx, false, "retry: the stop was refused or the link was lost",
            std::string(OutcomeName(view.outcome)) + "/" +
                ReasonName(view.reject_reason));
      break;
    }
    const ClientEvents &events = fx.transport->TakeEvents();
    for (std::size_t index = 0; index < events.result_count; ++index) {
      const auto &received = events.results[index];
      if (received.value.request_id == request_id) {
        answers.push_back(received.value);
      }
      fx.client->ObserveResult(received.value, received.rx_us);
    }
    if (events.has_state) {
      fx.client->ObserveState(events.state, events.state_rx_us);
    }
    PumpHeartbeat(fx);
    if (confirmed && answers.size() >= 2) {
      break;
    }
    SleepMs(1);
  }
  Check(fx, confirmed, "retry: the stop was confirmed");
  Check(fx, answers.size() == 2, "retry: two answers of the same request id",
        Number(answers.size()));
  if (answers.size() == 2) {
    Check(fx,
          answers[0].status == ResultStatus::Applied &&
              answers[1].status == ResultStatus::Applied,
          "retry: the stop was applied");
    Check(fx,
          answers[0].stamp_us == answers[1].stamp_us &&
              answers[0].epoch == answers[1].epoch,
          "retry: the replay repeats the recorded answer, so the stop was "
          "applied once");
    Check(fx, fx.client->next_epoch() == answers[0].epoch + 1,
          "retry: the watermark advanced exactly once");
  }
  *session_id = fx.client->session_id();
  return fx.failures == failures_before;
}

// Scenario 6: a stop preempts a pending ordinary request with a strictly larger
// request id and takes the transport's own stop lane; the late answer of the
// preempted request confirms nothing, and the stop's own answer does.
bool ScenarioStopPreemption(Fixture &fx, Id *session_id) {
  const std::uint64_t failures_before = fx.failures;
  const ClientView state = fx.client->Poll(NowUs());
  if (!Check(fx, state.state_known && state.state_fresh,
             "stop priority: a fresh state to build the first frame from")) {
    return false;
  }
  const ApplicationConfig &config = *fx.config;
  if (!Check(fx,
             state.state.feedback.count == JointCount(config) &&
                 state.state.execution_state == ExecutionState::Damping,
             "stop priority: the machine damps with a full feedback frame")) {
    return false;
  }

  // The first frame of the authorization holds the measured position and
  // velocity, so the continuity bounds of the runtime accept it.
  JointCommand frame;
  frame.protocol_version = irmv::protocol::kProtocolVersion;
  frame.producer_id = kProducerId;
  frame.epoch = fx.client->next_epoch();
  frame.command_seq = 1;
  frame.source_state_seq = state.state.state_seq;
  frame.layout_id.Assign(config.layout_id());
  frame.joints.count = JointCount(config);
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    frame.joints.q[index] = state.state.feedback.joints[index].q;
    frame.joints.dq[index] = state.state.feedback.joints[index].dq;
  }

  const Id live_session = fx.client->session_id();
  const std::size_t sends = fx.sink->requests.size();
  const std::uint64_t stop_lane = fx.transport->stop_lane_publish_count();
  if (!Check(fx,
             fx.client->BeginRequest(Operation::Tracking, NowUs(), kProducerId,
                                     &frame),
             "stop priority: begin the ordinary request")) {
    return false;
  }
  const Sequence ordinary_id = fx.client->pending_request_id();
  Check(fx, fx.sink->requests.size() == sends + 1,
        "stop priority: the ordinary request left");
  Check(fx, fx.transport->stop_lane_publish_count() == stop_lane,
        "stop priority: an ordinary request does not use the stop lane");
  Check(fx, fx.client->Poll(NowUs()).outcome == RequestOutcome::Pending,
        "stop priority: the ordinary request is pending");

  // The runtime handles the ordinary request first, so its answer is a real
  // applied one; it is held back instead of being fed, which is what makes it
  // the late answer of a preempted request.
  ExecutionResult ordinary_answer;
  TimeUs ordinary_rx_us = 0;
  bool have_ordinary_answer = false;
  const TimeUs answer_deadline = NowUs() + 2'000'000;
  while (!have_ordinary_answer && NowUs() < answer_deadline) {
    fx.client->Poll(NowUs());
    const ClientEvents &events = fx.transport->TakeEvents();
    for (std::size_t index = 0; index < events.result_count; ++index) {
      const auto &received = events.results[index];
      if (received.value.request_id == ordinary_id) {
        ordinary_answer = received.value;
        ordinary_rx_us = received.rx_us;
        have_ordinary_answer = true;
        continue;
      }
      fx.client->ObserveResult(received.value, received.rx_us);
    }
    if (events.has_state) {
      fx.client->ObserveState(events.state, events.state_rx_us);
    }
    PumpHeartbeat(fx);
    SleepMs(1);
  }
  if (!Check(fx, have_ordinary_answer,
             "stop priority: the ordinary request was answered") ||
      !Check(fx, ordinary_answer.status == ResultStatus::Applied,
             "stop priority: the ordinary request was applied")) {
    return false;
  }

  // The stop takes over with a larger request id and its own lane.
  Check(fx, fx.client->BeginRequest(Operation::ZeroTorque, NowUs()),
        "stop priority: begin the stop");
  Check(fx, fx.client->pending_request_id() > ordinary_id,
        "stop priority: the stop carries a larger request id",
        Number(fx.client->pending_request_id()) + " > " + Number(ordinary_id));
  Check(fx, fx.client->retry_count() == 1, "stop priority: the stop left once");
  Check(fx, fx.sink->requests.size() == sends + 2,
        "stop priority: the stop left after the ordinary request");
  Check(fx, fx.transport->stop_lane_publish_count() == stop_lane + 1,
        "stop priority: the stop took the transport's stop lane");

  // The held answer of the preempted request arrives now: it must not resolve
  // anything, with or without a state that would agree with it.
  fx.client->ObserveResult(ordinary_answer, ordinary_rx_us);
  Check(fx, fx.client->Poll(NowUs()).outcome == RequestOutcome::Pending,
        "stop priority: the late answer of the preempted request confirms "
        "nothing");
  Check(fx, fx.client->request_pending(),
        "stop priority: the stop is still the pending request");

  // Every answer is fed in arrival order; the stop's own answer confirms it.
  PendingAnswer answer;
  answer.request_id = fx.client->pending_request_id();
  const TimeUs deadline = NowUs() + 5'000'000;
  while (NowUs() < deadline) {
    const ClientView view = Step(fx, &answer, true);
    if (view.outcome == RequestOutcome::Confirmed) {
      Check(fx, fx.client->session_id() == live_session,
            "stop priority: confirmed on the live session");
      Check(fx, !fx.client->request_pending(),
            "stop priority: no request is pending after the confirmation");
      *session_id = fx.client->session_id();
      return fx.failures == failures_before;
    }
    if (view.outcome != RequestOutcome::None &&
        view.outcome != RequestOutcome::Pending) {
      Check(fx, false, "stop priority",
            std::string("outcome ") + OutcomeName(view.outcome) + "/" +
                ReasonName(view.reject_reason));
      return false;
    }
    SleepMs(1);
  }
  return Check(fx, false, "stop priority", "the stop was not confirmed");
}

// Scenario 5: a consumer that stops taking events for longer than the target
// period loses the superseded states (the lane keeps the newest) but no result
// and no sample that mattered: the lane keeps the answers in order, and the
// request that was answered while the consumer was slow still confirms.
bool ScenarioSlowConsumer(Fixture &fx) {
  const std::uint64_t failures_before = fx.failures;
  const Id runtime_id = fx.client->runtime_id();
  const int target_ms = static_cast<int>(
      std::max<TimeUs>(fx.config->supervision.target_period_us / 1000, 1));
  const std::uint64_t states_before = fx.transport->dropped_state_count();
  const std::uint64_t rejected_before = fx.transport->inbound_rejected_count();
  const std::uint64_t read_failures_before =
      fx.transport->inbound_read_failure_count();

  // Drain whatever is left and remember the newest state before the pause.
  const ClientView before = Step(fx, nullptr, true);
  const TimeUs stamp_before = before.state.stamp_us;
  Check(fx, fx.client->session_owned(),
        "slow consumer: the session is live before the pause");

  // The request whose answer the slow consumer will not read: it is sent now,
  // so its answer arrives while nothing is taken and the confirmation below
  // proves that answer survived the pause.
  if (!Check(fx, fx.client->BeginRequest(Operation::Damping, NowUs()),
             "slow consumer: begin a stop before the pause")) {
    return false;
  }
  const Sequence client_id = fx.client->pending_request_id();
  Check(fx, fx.client->retry_count() == 1,
        "slow consumer: the stop left once before the pause");

  // Two further requests of a session the runtime does not own: they are
  // answered as WrongSession without touching the live session's request
  // record, so the three answers are three distinguishable requests that the
  // result lane has to keep in order.
  const auto raw_request = [&](Operation operation, Sequence request_id) {
    ExecutionRequest request;
    request.protocol_version = irmv::protocol::kProtocolVersion;
    request.supervisor_id = irmv::protocol::kNoId;
    request.request_id = request_id;
    request.session_id = irmv::protocol::kNoId;
    request.runtime_id = runtime_id;
    request.operation = operation;
    return request;
  };
  const Sequence first_id = 0x5F5F0001ULL;
  const Sequence second_id = 0x5F5F0002ULL;
  Check(fx,
        fx.transport->PublishRequest(raw_request(Operation::Damping, first_id)),
        "slow consumer: the first foreign request left");
  SleepMs(target_ms);
  Check(fx,
        fx.transport->PublishRequest(
            raw_request(Operation::ZeroTorque, second_id)),
        "slow consumer: the second foreign request left");

  // Nothing is taken for ten target periods: the states are superseded (which
  // the counter reports) while the three answers wait in their queue.
  SleepMs(target_ms * 10);

  const ClientEvents &events = fx.transport->TakeEvents();
  Check(fx, events.has_state, "slow consumer: a state was delivered");
  Check(fx, events.state.stamp_us >= stamp_before,
        "slow consumer: the newest state was kept, not an old one");
  Check(fx, fx.transport->dropped_state_count() > states_before,
        "slow consumer: the superseded states were counted",
        Number(fx.transport->dropped_state_count()));
  Check(fx, events.result_count == 3, "slow consumer: every answer arrived",
        Number(events.result_count));
  if (events.result_count == 3) {
    Check(fx,
          events.results[0].value.request_id == client_id &&
              events.results[1].value.request_id == first_id &&
              events.results[2].value.request_id == second_id,
          "slow consumer: the result lane is FIFO");
    Check(fx, events.results[0].value.status == ResultStatus::Applied,
          "slow consumer: the stop was applied while nothing was taken");
    Check(fx,
          events.results[1].value.status == ResultStatus::Rejected &&
              events.results[2].value.status == ResultStatus::Rejected &&
              events.results[1].value.reason == RejectReason::WrongSession &&
              events.results[2].value.reason == RejectReason::WrongSession,
          "slow consumer: the runtime refused the foreign sessions");
  }
  Check(fx,
        fx.transport->inbound_rejected_count() == rejected_before &&
            fx.transport->inbound_read_failure_count() == read_failures_before,
        "slow consumer: nothing was silently lost or rejected");
  Check(fx,
        fx.transport->dropped_request_count() == 0 &&
            fx.transport->dropped_result_count() == 0 &&
            fx.transport->dropped_command_count() == 0 &&
            fx.transport->dropped_heartbeat_count() == 0,
        "slow consumer: no lane overflowed");

  // The fresh state is fed with the answers, so the confirmation happens on the
  // state published after the pause and the session survives it.
  FeedBatch(fx, events, nullptr, true);
  if (!ConfirmPending(fx, true, 5000,
                      "slow consumer: the request answered during the pause")) {
    return false;
  }
  Check(fx, fx.client->session_owned(),
        "slow consumer: the session survived the pause");
  return fx.failures == failures_before;
}

// Scenario 4a: the runtime the client is bound to leaves. Nothing arrives any
// more, so the state timeout is what the supervision thread has to notice: the
// session, the epoch, the pending request and the snapshot are withdrawn and
// the loss is reported once.
bool ScenarioRuntimeLost(Fixture &fx, RuntimeProcess &runtime,
                         const Id &old_runtime, const Id &old_session) {
  const std::uint64_t failures_before = fx.failures;
  Check(fx, old_session != irmv::protocol::kNoId,
        "runtime change: a session was live before the restart");
  Check(fx, fx.client->runtime_id() == old_runtime,
        "runtime change: the client is bound to the first instance");
  Check(fx, fx.client->SendHeartbeat(NowUs()),
        "runtime change: the lease could be refreshed before the restart");

  Check(fx, runtime.Stop(5000), "runtime change: the first instance left");
  bool reported = false;
  const TimeUs deadline = NowUs() + 5'000'000;
  while (NowUs() < deadline) {
    const ClientView view = Step(fx, nullptr, true);
    if (view.outcome == RequestOutcome::Unknown) {
      reported = true;
      Check(fx, view.session_lost && !view.session_owned,
            "runtime change: the session is lost with the silent link");
      break;
    }
    SleepMs(2);
  }
  if (!Check(fx, reported,
             "runtime change: the silent link is reported once")) {
    return false;
  }
  Check(fx, fx.client->session_id() == irmv::protocol::kNoId,
        "runtime change: the old session is not reused");
  Check(fx, !fx.client->SendHeartbeat(NowUs()),
        "runtime change: no heartbeat without a session");
  JointCommand command;
  Check(fx, !fx.client->SendCommand(&command, NowUs()),
        "runtime change: no target on the old session");
  Check(fx, !fx.client->BeginRequest(Operation::Damping, NowUs()),
        "runtime change: no stop on the old session");
  return fx.failures == failures_before;
}

// Scenario 4b: a fresh runtime process appears. It is a different instance, so
// the client reports the change and keeps refusing the old session until a new
// one is confirmed.
bool ScenarioRuntimeChanged(Fixture &fx, const Id &old_runtime) {
  const std::uint64_t failures_before = fx.failures;
  bool changed = false;
  const TimeUs deadline = NowUs() + 15'000'000;
  while (NowUs() < deadline) {
    const ClientView view = Step(fx, nullptr, true);
    if (view.state_known && view.state.runtime_id != old_runtime) {
      changed = true;
      Check(fx, view.runtime_changed,
            "runtime change: the new instance is reported as a change");
      Check(fx, view.session_lost && !view.session_owned,
            "runtime change: the old session stays dead");
      break;
    }
    SleepMs(2);
  }
  if (!Check(fx, changed, "runtime change: the new instance was observed")) {
    return false;
  }
  Check(fx,
        fx.client->runtime_id() != old_runtime &&
            fx.client->runtime_id() != irmv::protocol::kNoId,
        "runtime change: the client is bound to the new instance");
  Check(fx, fx.client->session_id() == irmv::protocol::kNoId,
        "runtime change: no old session survived the restart");
  JointCommand command;
  Check(fx, !fx.client->SendCommand(&command, NowUs()),
        "runtime change: no target on the old session");
  Check(fx, !fx.client->SendHeartbeat(NowUs()),
        "runtime change: no heartbeat on the old session");
  Check(fx, !fx.client->BeginRequest(Operation::Damping, NowUs()),
        "runtime change: no stop on the old session");
  return fx.failures == failures_before;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: application_client_test <robot_runtime> <source "
                 "directory>\n";
    return kExitUsage;
  }
  const fs::path binary = fs::absolute(argv[1]);
  const fs::path source = fs::absolute(argv[2]);
  if (!fs::is_regular_file(binary)) {
    std::cerr << "FAIL: robot_runtime " << binary << " does not exist\n";
    return kExitUsage;
  }
  const fs::path scenario = source / "tests/support/fake_scenario.json";
  if (!fs::is_regular_file(scenario)) {
    std::cerr << "FAIL: fake scenario " << scenario << " does not exist\n";
    return kExitUsage;
  }

  // 20..209, derived from the process id so parallel runs stay apart. The file
  // on disk carries another domain, so the override is what both processes
  // share.
  const std::uint32_t domain =
      20 + (static_cast<std::uint32_t>(::getpid()) % 190);
  const fs::path workspace =
      fs::temp_directory_path() /
      ("irmv-application-client-" + std::to_string(::getpid()));
  const WorkspaceGuard guard(workspace);
  std::error_code ignored;
  fs::remove_all(workspace, ignored);
  fs::create_directories(workspace / "logs", ignored);

  // The runtime section is written with the on-disk domain replaced and the
  // telemetry redirected into the workspace, so the run never touches the
  // source tree. A source configuration that does not parse is reported like
  // any other failure instead of letting the parser abort the process.
  const fs::path config_path = workspace / "config.json";
  try {
    std::ifstream input(source / "config/sdk.json");
    json document;
    input >> document;
    document["dds"]["domain"] = kConfigDomain;
    document["telemetry"]["directory"] = (workspace / "logs").string();
    std::ofstream output(config_path);
    output << document.dump(2);
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << (source / "config/sdk.json")
              << " did not parse: " << error.what() << "\n";
    return kExitFailure;
  }

  ApplicationConfig config;
  try {
    config = LoadApplicationConfig(config_path, ConfigOverrides{domain});
  } catch (const std::exception &error) {
    std::cerr << "FAIL: the application configuration did not load: "
              << error.what() << "\n";
    return kExitFailure;
  }

  const std::vector<std::string> runtime_args = {
      "--config",        config_path.string(),
      "--domain",        std::to_string(domain),
      "--fake-scenario", scenario.string(),
  };

  Fixture fx;
  fx.config = &config;

  // The identity is never the reserved empty id, it differs between two calls
  // of one process, and the request-id band it carries leaves room below the
  // reserved last sequence value. These are properties of the identity, not
  // proof of the entropy-fallback path: the entropy source normally works, so
  // the same three checks pass on an implementation whose fallback is constant.
  const Id first_identity = irmv::application::MakeSupervisorId();
  const Id second_identity = irmv::application::MakeSupervisorId();
  Check(fx,
        first_identity != irmv::protocol::kNoId &&
            second_identity != irmv::protocol::kNoId,
        "identity: MakeSupervisorId is never the empty id");
  Check(fx, first_identity != second_identity,
        "identity: two identities of one process differ");
  const Sequence band = BandStart(first_identity);
  Check(fx, band != 0 && band < std::numeric_limits<Sequence>::max(),
        "identity: the request-id band starts below the reserved maximum",
        Number(band));

  // The DDS-free client cannot include the transport's lane predicate, so this
  // is where the two definitions are pinned to each other over the whole frozen
  // operation range: every value the protocol accepts, not a list this test
  // wrote down by hand.
  for (std::uint32_t raw = 0; raw <= 64; ++raw) {
    const Operation operation = static_cast<Operation>(raw);
    if (!irmv::protocol::IsValid(operation)) {
      continue;
    }
    Check(fx,
          irmv::application::IsStopOperation(operation) ==
              irmv::transport::IsStopLaneOperation(operation),
          "stop lane: the application predicate is the transport's lane",
          "operation " + std::to_string(raw));
  }

  bool ok = true;
  Id session_id = irmv::protocol::kNoId;
  Id old_runtime = irmv::protocol::kNoId;
  {
    ClientTransport transport(config);
    const Id supervisor_id = irmv::application::MakeSupervisorId();
    try {
      transport.Start();
    } catch (const std::exception &error) {
      std::cerr << "FAIL: the client transport did not start: " << error.what()
                << "\n";
      return kExitFailure;
    }
    ok = Check(fx, transport.started(), "transport: started") && ok;

    RecordingSink sink(transport);
    RobotClient client(sink, config, supervisor_id);
    fx.transport = &transport;
    fx.sink = &sink;
    fx.client = &client;

    {
      RuntimeProcess runtime(binary, runtime_args, workspace / "runtime_a.log");
      ok = ok &&
           Check(fx, runtime.Running(), "runtime: the first instance started");
      ok = ok && WaitForInstance(fx, 15000, "bootstrap: the instance is known");
      old_runtime = fx.client->runtime_id();
      ok = ok && ScenarioAcquireResultFirst(fx, &session_id);
      ok = ok && ScenarioRetry(fx, &session_id);
      ok = ok && ScenarioStopPreemption(fx, &session_id);
      ok = ok && ScenarioSlowConsumer(fx);
      if (ok) {
        ok = ScenarioRuntimeLost(fx, runtime, old_runtime, session_id);
      }
    }
    if (ok) {
      RuntimeProcess second(binary, runtime_args, workspace / "runtime_b.log");
      ok =
          Check(fx, second.Running(), "runtime: the second instance started") &&
          ok;
      // The same client, on the instance that appeared: the change is reported
      // first, then a bootstrap with the empty session, a fresh epoch and this
      // time a state before the answer.
      ok = ok && ScenarioRuntimeChanged(fx, old_runtime);
      Id new_session = irmv::protocol::kNoId;
      ok = ok &&
           WaitForInstance(fx, 15000, "restart: the new instance is known");
      ok = ok &&
           AcquireSession(fx, false, 5000,
                          "acquire (state first, new instance)", &new_session);
      ok = Check(fx,
                 new_session != irmv::protocol::kNoId &&
                     new_session != session_id,
                 "restart: the new session is not the old one") &&
           ok;
      ok = Check(fx, fx.client->runtime_id() != old_runtime,
                 "restart: the client talks to the new instance") &&
           ok;
      ok = Check(fx, fx.client->SendHeartbeat(NowUs()),
                 "restart: the new session can refresh its lease") &&
           ok;
    }

    transport.Stop();
    ok = Check(fx, !transport.started(), "transport: stopped") && ok;
  }

  const bool failed = fx.failures != 0 || !ok;
  if (failed) {
    std::cerr << "application_client_test: " << fx.failures << " of "
              << fx.checks << " checks failed\n";
    for (const char *name : {"runtime_a.log", "runtime_b.log"}) {
      std::ifstream log(workspace / name, std::ios::binary);
      if (!log) {
        continue;
      }
      std::string contents{std::istreambuf_iterator<char>(log),
                           std::istreambuf_iterator<char>()};
      const std::size_t tail = 4000;
      if (contents.size() > tail) {
        contents.erase(0, contents.size() - tail);
      }
      std::cerr << "--- " << name << " (tail) ---\n" << contents << "\n";
    }
  } else {
    std::cout << "application_client_test: all " << fx.checks
              << " checks passed (domain " << domain << ")\n";
  }
  return failed ? kExitFailure : kExitOk;
}
