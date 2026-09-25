#include "irmv/application/application_loop.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <optional>
#include <thread>

#include "irmv/application/behavior_fsm.hpp"
#include "irmv/application/input_router.hpp"
#include "irmv/application/robot_client.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/transport/client_transport.hpp"

namespace irmv::application {

namespace {

using Clock = std::chrono::steady_clock;

// How often the status line is repeated while nothing changed. A harness waits
// for a behaviour, a reject reason or a pending operation, so the line has to
// reappear without flooding the pipe.
constexpr protocol::TimeUs kStatusIntervalUs = 500000;
// Bytes read from the terminal in one iteration, and how many reads one
// iteration performs: bounded so a stream nobody drains cannot starve the
// supervision checks, generous enough for a burst of keystrokes.
constexpr std::size_t kReadChunk = 64;
constexpr int kReadAttempts = 64;

// ---------------------------------------------------------------------------
// Names of the enumerations of the status line
// ---------------------------------------------------------------------------

const char *BehaviorName(BehaviorState state) {
  switch (state) {
    case BehaviorState::Starting:
      return "Starting";
    case BehaviorState::Standby:
      return "Standby";
    case BehaviorState::Damping:
      return "Damping";
    case BehaviorState::Initializing:
      return "Initializing";
    case BehaviorState::Ready:
      return "Ready";
    case BehaviorState::Locomotion:
      return "Locomotion";
    case BehaviorState::Fault:
      return "Fault";
  }
  return "Unknown";
}

const char *StateName(protocol::ExecutionState state) {
  switch (state) {
    case protocol::ExecutionState::Boot:
      return "Boot";
    case protocol::ExecutionState::ZeroTorque:
      return "ZeroTorque";
    case protocol::ExecutionState::Damping:
      return "Damping";
    case protocol::ExecutionState::Tracking:
      return "Tracking";
    case protocol::ExecutionState::Fault:
      return "Fault";
  }
  return "Unknown";
}

const char *OutputName(protocol::OutputAction action) {
  switch (action) {
    case protocol::OutputAction::ZeroTorque:
      return "ZeroTorque";
    case protocol::OutputAction::Damping:
      return "Damping";
    case protocol::OutputAction::Tracking:
      return "Tracking";
  }
  return "Unknown";
}

const char *OperationName(protocol::Operation operation) {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      return "AcquireSession";
    case protocol::Operation::Damping:
      return "Damping";
    case protocol::Operation::ZeroTorque:
      return "ZeroTorque";
    case protocol::Operation::Tracking:
      return "Tracking";
    case protocol::Operation::Reset:
      return "Reset";
  }
  return "-";
}

// The upper layer's own latch. "-" while it holds none, so the display never
// reports a cause for a behaviour that is not Faulted.
const char *FaultCauseName(FaultCause cause) {
  switch (cause) {
    case FaultCause::None:
      return "-";
    case FaultCause::OperatorEmergencyStop:
      return "OperatorEmergencyStop";
    case FaultCause::RuntimeFault:
      return "RuntimeFault";
    case FaultCause::RuntimeUnreachable:
      return "RuntimeUnreachable";
    case FaultCause::RuntimeRestarted:
      return "RuntimeRestarted";
    case FaultCause::InitializationFailed:
      return "InitializationFailed";
    case FaultCause::RequestTimeout:
      return "RequestTimeout";
    case FaultCause::UnexpectedContext:
      return "UnexpectedContext";
  }
  return "-";
}

// The lower layer's own reason for leaving tracking, as the last trustworthy
// state reported it. "-" means it reported none (or is not known), never that a
// stop happened.
const char *StopReasonName(protocol::StopReason reason) {
  switch (reason) {
    case protocol::StopReason::None:
      return "-";
    case protocol::StopReason::Requested:
      return "Requested";
    case protocol::StopReason::CommandTimeout:
      return "CommandTimeout";
    case protocol::StopReason::SourceStateTimeout:
      return "SourceStateTimeout";
    case protocol::StopReason::SupervisorTimeout:
      return "SupervisorTimeout";
  }
  return "-";
}

const char *RejectName(protocol::RejectReason reason) {
  switch (reason) {
    case protocol::RejectReason::None:
      return "-";
    case protocol::RejectReason::VersionMismatch:
      return "VersionMismatch";
    case protocol::RejectReason::Malformed:
      return "Malformed";
    case protocol::RejectReason::WrongInstance:
      return "WrongInstance";
    case protocol::RejectReason::WrongSession:
      return "WrongSession";
    case protocol::RejectReason::SessionBusy:
      return "SessionBusy";
    case protocol::RejectReason::StaleRequest:
      return "StaleRequest";
    case protocol::RejectReason::RequestConflict:
      return "RequestConflict";
    case protocol::RejectReason::EpochMismatch:
      return "EpochMismatch";
    case protocol::RejectReason::InvalidTransition:
      return "InvalidTransition";
    case protocol::RejectReason::InvalidCommand:
      return "InvalidCommand";
    case protocol::RejectReason::StaleSource:
      return "StaleSource";
    case protocol::RejectReason::Discontinuity:
      return "Discontinuity";
    case protocol::RejectReason::RecoveryNotReady:
      return "RecoveryNotReady";
    case protocol::RejectReason::FaultMismatch:
      return "FaultMismatch";
    case protocol::RejectReason::WriteFailed:
      return "WriteFailed";
    case protocol::RejectReason::QueueFull:
      return "QueueFull";
  }
  return "-";
}

// ---------------------------------------------------------------------------
// Clock and pacing
// ---------------------------------------------------------------------------

protocol::TimeUs MonotonicUs() {
  const auto elapsed = Clock::now().time_since_epoch();
  return static_cast<protocol::TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

// The loop period: the shortest period the behaviour asks for, so neither a
// target nor a heartbeat can be late because the iteration was waiting for the
// other one. The configuration guarantees positive periods; the guard keeps a
// hand-written one from turning the loop into a spin.
protocol::TimeUs LoopPeriodUs(const configuration::ApplicationConfig &config) {
  const protocol::TimeUs target = config.supervision.target_period_us;
  const protocol::TimeUs heartbeat = config.supervision.heartbeat_period_us;
  const protocol::TimeUs period = target < heartbeat ? target : heartbeat;
  return period == 0 ? 1 : period;
}

// The terminal wait of one iteration, clamped: the wait is the pacing of the
// loop, so it is never longer than a second even for a configurably long
// period, which then simply means more idle iterations.
int PollTimeoutMs(protocol::TimeUs period_us) {
  const protocol::TimeUs milliseconds = (period_us + 999) / 1000;
  if (milliseconds < 1) {
    return 1;
  }
  if (milliseconds > 1000) {
    return 1000;
  }
  return static_cast<int>(milliseconds);
}

// Reads everything the terminal has, bounded per iteration. Returns true when
// the input device is lost, which is an EOF or a terminal error and asks for
// the ordered exit - never for an emergency stop and never for a claim that the
// hardware stopped.
//
// Precondition: STDIN_FILENO is already O_NONBLOCK (main's terminal guard set
// it). That flag is what makes "nothing to read" return EAGAIN instead of
// waiting for a byte, so this read, the pacing poll at the end of the iteration
// and the supervision heartbeat stay independent of the typing hand.
bool ReadTerminal(InputRouter *router, std::array<char, kReadChunk> *buffer,
                  protocol::TimeUs now_us) {
  for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
    const ssize_t count = ::read(STDIN_FILENO, buffer->data(), buffer->size());
    if (count > 0) {
      for (ssize_t index = 0; index < count; ++index) {
        const char key = (*buffer)[static_cast<std::size_t>(index)];
        // A terminal byte stream carries presses; a release of a one-shot key
        // is the absence of a byte, which InputRouter's repeat guard already
        // treats as one event per press.
        static_cast<void>(router->OnKey(key, true, now_us));
      }
      continue;
    }
    if (count == 0) {
      router->OnInputLost();
      return true;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    router->OnInputLost();
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Transport adapter
// ---------------------------------------------------------------------------

// The three forwards of ClientSink to the transport, plus the one display fact
// the client does not expose: the operation of the request that was handed over
// last. irmv_transport must not depend on irmv_application, so ClientTransport
// cannot implement ClientSink itself, and the application executable is the
// only place that knows both.
class SinkAdapter final : public ClientSink {
 public:
  explicit SinkAdapter(transport::ClientTransport &transport)
      : transport_(transport) {}

  bool PublishRequest(const protocol::ExecutionRequest &request) override {
    // Recorded before the lane decides: the display shows the operation of the
    // request the client is waiting on, whether or not the transport could
    // queue that copy of it.
    last_operation_ = request.operation;
    have_operation_ = true;
    return transport_.PublishRequest(request);
  }

  bool PublishCommand(const protocol::JointCommand &command) override {
    return transport_.PublishCommand(command);
  }

  bool PublishHeartbeat(
      const protocol::SupervisorHeartbeat &heartbeat) override {
    return transport_.PublishHeartbeat(heartbeat);
  }

  const char *PendingOperation() const {
    return have_operation_ ? OperationName(last_operation_) : "-";
  }

 private:
  transport::ClientTransport &transport_;
  protocol::Operation last_operation_{protocol::Operation::AcquireSession};
  bool have_operation_{false};
};

// ---------------------------------------------------------------------------
// Status line
// ---------------------------------------------------------------------------

// One rendering of the status line. Every field is a name of a frozen
// enumeration, so the line is compared field by field and nothing on this path
// allocates.
struct StatusFields {
  const char *behavior{"Unknown"};
  const char *runtime{"Unknown"};
  const char *output{"Unknown"};
  const char *session{"none"};
  const char *pending{"-"};
  const char *reject{"-"};
  const char *fresh{"no"};
  const char *init{"-"};
  // The upper layer's latched cause and the lower layer's own stop reason:
  // without them an operator cannot tell an operator emergency stop from an
  // unreachable runtime, or a lower command timeout from a supervisor timeout.
  const char *cause{"-"};
  const char *stop{"-"};
};

bool SameFields(const StatusFields &left, const StatusFields &right) {
  return std::strcmp(left.behavior, right.behavior) == 0 &&
         std::strcmp(left.runtime, right.runtime) == 0 &&
         std::strcmp(left.output, right.output) == 0 &&
         std::strcmp(left.session, right.session) == 0 &&
         std::strcmp(left.pending, right.pending) == 0 &&
         std::strcmp(left.reject, right.reject) == 0 &&
         std::strcmp(left.fresh, right.fresh) == 0 &&
         std::strcmp(left.init, right.init) == 0 &&
         std::strcmp(left.cause, right.cause) == 0 &&
         std::strcmp(left.stop, right.stop) == 0;
}

void PrintStatus(const StatusFields &fields) {
  // Sized for the longest names of every field with room to spare; a line that
  // did not fit would be truncated, which the length clamp below turns into a
  // shorter line rather than an unterminated one.
  std::array<char, 320> line{};
  const int written = std::snprintf(
      line.data(), line.size(),
      "[application] behavior=%s runtime=%s output=%s session=%s pending=%s "
      "reject=%s fresh=%s init=%s cause=%s stop=%s\n",
      fields.behavior, fields.runtime, fields.output, fields.session,
      fields.pending, fields.reject, fields.fresh, fields.init, fields.cause,
      fields.stop);
  if (written <= 0) {
    return;
  }
  const std::size_t length =
      std::min<std::size_t>(static_cast<std::size_t>(written), line.size() - 1);
  static_cast<void>(std::fwrite(line.data(), 1, length, stdout));
  // A harness reads the pipe while the process runs: nothing may sit in a
  // buffer until the exit.
  static_cast<void>(std::fflush(stdout));
}

const char *SessionField(const RobotClient &client, const ClientView &view) {
  if (view.session_owned) {
    return "owned";
  }
  if (view.session_lost || client.session_id() != protocol::kNoId) {
    return "lost";
  }
  return "none";
}

// The pending request of the display: the operation of the request the client
// is waiting on, and nothing while none is pending.
void PendingField(const RobotClient &client, const SinkAdapter &sink,
                  StatusFields *fields) {
  fields->pending = client.request_pending() ? sink.PendingOperation() : "-";
}

// ---------------------------------------------------------------------------
// Startup and the supervision loop
// ---------------------------------------------------------------------------

int LoadConfiguration(const ApplicationOptions &options,
                      configuration::ApplicationConfig *config) {
  configuration::ConfigOverrides overrides;
  if (options.dds_domain) {
    overrides.dds_domain = *options.dds_domain;
  }
  try {
    *config =
        configuration::LoadApplicationConfig(options.config_path, overrides);
  } catch (const std::exception &error) {
    // No claim about hardware, a device or the lower process: the invocation,
    // the file or a field of it was refused, and the process leaves before the
    // transport - the first thing that touches DDS - is constructed.
    std::fprintf(stderr, "application: %s\n", error.what());
    return kExitStartupError;
  }
  return kExitOk;
}

}  // namespace

int RunApplication(const ApplicationOptions &options,
                   const std::atomic<bool> &stop_requested) {
  configuration::ApplicationConfig config;
  const int load_error = LoadConfiguration(options, &config);
  if (load_error != kExitOk) {
    return load_error;
  }

  // The transport is the first thing of this process that touches DDS, and
  // after this point every decision of the loop is a decision about a live
  // link.
  transport::ClientTransport transport(config);
  try {
    transport.Start();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "application: transport start failed: %s\n",
                 error.what());
    return kExitStartupError;
  }

  // The identity, the client, the behaviour and the input router, in that
  // order: the client is the only writer of the protocol, the behaviour is the
  // only decision maker, and the router is the only reader of the keyboard.
  SinkAdapter sink(transport);
  RobotClient client(sink, config, MakeSupervisorId());
  BehaviorFsm fsm(config, client);
  InputRouter router(config);

  const protocol::TimeUs period_us = LoopPeriodUs(config);
  const int poll_timeout_ms = PollTimeoutMs(period_us);
  std::array<char, kReadChunk> bytes{};

  StatusFields last{};
  bool have_status = false;
  protocol::TimeUs next_status_us = 0;
  bool input_lost = false;

  while (true) {
    // One monotonic clock per iteration, taken once and passed everywhere. The
    // loop itself reads no other clock, and the runtime's stamps are only ever
    // compared against a host receive stamp.
    const protocol::TimeUs now = MonotonicUs();

    // (1) The terminal. A read that would block is not the loop's pacing: the
    // poll at the end of the iteration is, so the terminal can never stall the
    // heartbeat.
    if (!input_lost) {
      input_lost = ReadTerminal(&router, &bytes, now);
    }
    router.Tick(now);

    // (2) Everything the transport thread received since the last iteration.
    const transport::ClientEvents &events = transport.TakeEvents();
    for (std::size_t index = 0; index < events.result_count; ++index) {
      client.ObserveResult(events.results[index].value,
                           events.results[index].rx_us);
    }
    if (events.has_state) {
      client.ObserveState(events.state, events.state_rx_us);
    }

    // (3) Exactly one poll per iteration: the outcome it reports is edge
    // triggered, and the behaviour consumes the serial of this poll for its
    // own outcome handling.
    const ClientView view = client.Poll(now);

    // The signal is not a keystroke and never a control path of its own: it
    // asks for the same ordered exit the exit key does.
    if (stop_requested.load(std::memory_order_relaxed)) {
      fsm.RequestExit(now);
    }

    // (4) One Step per input event, then one final Step that does the periodic
    // work, and (5) the sends each Step asked for. The sends happen per Step
    // because a Step's target belongs to the iteration that produced it.
    BehaviorStep behavior{};
    bool initialization_refused = false;
    for (;;) {
      const InputEvent event = router.TakeEvent();
      behavior = fsm.Step(now, event, view);
      if (behavior.initialization_refused) {
        initialization_refused = true;
      }
      if (behavior.send_command) {
        static_cast<void>(client.SendCommand(&behavior.command, now));
      }
      if (behavior.send_heartbeat) {
        static_cast<void>(client.SendHeartbeat(now));
      }
      if (behavior.exit_ready || event == InputEvent::None) {
        break;
      }
    }

    // (6) The display: on every change, and at most every status interval
    // otherwise.
    StatusFields fields;
    fields.behavior = BehaviorName(behavior.state);
    fields.runtime = behavior.runtime_state_known
                         ? StateName(behavior.runtime_state)
                         : "Unknown";
    fields.output = behavior.runtime_state_known
                        ? OutputName(behavior.runtime_output)
                        : "Unknown";
    fields.session = SessionField(client, view);
    PendingField(client, sink, &fields);
    fields.reject = RejectName(behavior.last_reject_reason);
    fields.fresh = view.state_fresh ? "yes" : "no";
    if (initialization_refused) {
      fields.init = "refused";
    } else if (behavior.state == BehaviorState::Initializing ||
               behavior.state == BehaviorState::Ready) {
      fields.init = "ok";
    }
    // The latched cause is the upper layer's own statement and stays visible
    // while it holds; the lower stop reason is a present fact of the state, so
    // it is shown only while a current state is known.
    fields.cause = FaultCauseName(behavior.fault_cause);
    fields.stop = behavior.runtime_state_known
                      ? StopReasonName(behavior.runtime_stop_reason)
                      : "-";
    if (!have_status || !SameFields(fields, last) || now >= next_status_us) {
      PrintStatus(fields);
      last = fields;
      have_status = true;
      next_status_us = now + kStatusIntervalUs;
    }

    // (7) The exit. `exit_ready` is the FSM's own statement that the sequence
    // it was asked for is over: either a fresh lower state shows the damping
    // stop, or the sequence reached its own deadline and proved nothing.
    if (behavior.exit_ready) {
      const char *lower = behavior.runtime_state_known
                              ? StateName(behavior.runtime_state)
                              : "Unknown";
      int exit_code = kExitStopUnknown;
      if (behavior.exit_stop_confirmed) {
        std::printf("application: exit stop=confirmed state=%s\n", lower);
        exit_code = kExitOk;
      } else {
        std::printf("application: exit stop=unknown lower=%s\n", lower);
      }
      static_cast<void>(std::fflush(stdout));
      // Nothing of this process keeps running behind the exit line: the
      // transport thread is joined here, and the runtime is not touched - its
      // own watchdog owns the link this process leaves behind.
      transport.Stop();
      return exit_code;
    }

    // The wait that paces the iteration, and the terminal wait with it. Once
    // the input device is gone the descriptor reports it forever, so the
    // pacing becomes a plain sleep instead of a spin.
    if (input_lost) {
      std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    } else {
      struct pollfd descriptor {};
      descriptor.fd = STDIN_FILENO;
      descriptor.events = POLLIN;
      // An interrupted wait and every other result just end the wait: the next
      // iteration re-reads the clock and re-runs its checks, which is also what
      // keeps a signalled exit bounded.
      static_cast<void>(::poll(&descriptor, 1, poll_timeout_ms));
    }
  }
}

}  // namespace irmv::application
