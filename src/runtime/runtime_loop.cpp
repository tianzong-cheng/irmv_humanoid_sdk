#include "irmv/runtime/runtime_loop.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <utility>

#include "irmv/runtime/execution_fsm.hpp"
#include "irmv/telemetry/logger.hpp"
#include "irmv/transport/dds_transport.hpp"

namespace irmv::runtime {
namespace {

using Clock = std::chrono::steady_clock;

// Period of the bounded status line and of the shutdown summary. Both carry the
// overrun accounting, so no tick ever logs at control frequency.
constexpr protocol::TimeUs kStatusIntervalUs = 1'000'000;

// Bounded retry cadence of the self-issued ZeroTorque stop request: a rejected
// request is retried with a newer request id, never at control frequency.
constexpr protocol::TimeUs kStopRetryIntervalUs = 100'000;

// Largest IMU stamp offset a scenario may declare: one hour either way.
constexpr std::int64_t kMaxStampOffsetUs = 3'600'000'000LL;

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------

// Set by the SIGINT/SIGTERM handler and read by the control loop. The handler
// does nothing else: no logging, no allocation, no backend call. std::atomic
// is required to be lock free for that to hold, which the assertion below pins.
std::atomic<bool> g_stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "the stop flag must be usable from a signal handler");

extern "C" void HandleStopSignal(int signal_number) {
  static_cast<void>(signal_number);
  g_stop_requested.store(true, std::memory_order_relaxed);
}

bool InstallStopHandlers() {
  return std::signal(SIGINT, HandleStopSignal) != SIG_ERR &&
         std::signal(SIGTERM, HandleStopSignal) != SIG_ERR;
}

// ---------------------------------------------------------------------------
// Text rendering
//
// Bounded and allocation free: every helper writes into a caller buffer. The
// per-tick event mapping renders no number at all; only the startup line, the
// bounded status line and the shutdown summary render one, at most twice a
// second.
// ---------------------------------------------------------------------------

std::string_view UintText(std::uint64_t value, char *buffer) {
  char digits[20];
  std::size_t length = 0;
  do {
    digits[length++] = static_cast<char>('0' + (value % 10));
    value /= 10;
  } while (value != 0);
  for (std::size_t index = 0; index < length; ++index) {
    buffer[index] = digits[length - 1 - index];
  }
  return {buffer, length};
}

std::string_view HexId(const protocol::Id &id, char *buffer) {
  constexpr char kHex[] = "0123456789abcdef";
  for (std::size_t index = 0; index < id.size(); ++index) {
    buffer[2 * index] = kHex[(id[index] >> 4) & 0x0f];
    buffer[2 * index + 1] = kHex[id[index] & 0x0f];
  }
  return {buffer, 2 * id.size()};
}

std::string_view DoubleText(double value, char *buffer, std::size_t capacity) {
  const int written = std::snprintf(buffer, capacity, "%.6g", value);
  if (written <= 0) {
    return {"0", 1};
  }
  const std::size_t length =
      std::min<std::size_t>(static_cast<std::size_t>(written), capacity - 1);
  return {buffer, length};
}

// ---------------------------------------------------------------------------
// Names of the protocol enumerations, as they appear in the log
// ---------------------------------------------------------------------------

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

const char *OutputActionName(protocol::OutputAction action) {
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

const char *RejectReasonName(protocol::RejectReason reason) {
  switch (reason) {
    case protocol::RejectReason::None:
      return "None";
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
  return "Unknown";
}

const char *FaultReasonName(protocol::FaultReason reason) {
  switch (reason) {
    case protocol::FaultReason::None:
      return "None";
    case protocol::FaultReason::BootFailed:
      return "BootFailed";
    case protocol::FaultReason::FeedbackInvalid:
      return "FeedbackInvalid";
    case protocol::FaultReason::DriverError:
      return "DriverError";
    case protocol::FaultReason::ImuInvalid:
      return "ImuInvalid";
    case protocol::FaultReason::JointPositionExceeded:
      return "JointPositionExceeded";
    case protocol::FaultReason::JointVelocityExceeded:
      return "JointVelocityExceeded";
    case protocol::FaultReason::CommandInvalid:
      return "CommandInvalid";
    case protocol::FaultReason::WriteFailed:
      return "WriteFailed";
    case protocol::FaultReason::ProtocolExhausted:
      return "ProtocolExhausted";
  }
  return "Unknown";
}

const char *StopReasonName(protocol::StopReason reason) {
  switch (reason) {
    case protocol::StopReason::None:
      return "None";
    case protocol::StopReason::Requested:
      return "Requested";
    case protocol::StopReason::CommandTimeout:
      return "CommandTimeout";
    case protocol::StopReason::SourceStateTimeout:
      return "SourceStateTimeout";
    case protocol::StopReason::SupervisorTimeout:
      return "SupervisorTimeout";
  }
  return "Unknown";
}

const char *EventCodeName(EventCode code) {
  switch (code) {
    case EventCode::None:
      return "None";
    case EventCode::BootCompleted:
      return "BootCompleted";
    case EventCode::BootFailed:
      return "BootFailed";
    case EventCode::StateChanged:
      return "StateChanged";
    case EventCode::SessionAcquired:
      return "SessionAcquired";
    case EventCode::SessionExpired:
      return "SessionExpired";
    case EventCode::AuthorizationGranted:
      return "AuthorizationGranted";
    case EventCode::AuthorizationRevoked:
      return "AuthorizationRevoked";
    case EventCode::StopApplied:
      return "StopApplied";
    case EventCode::CommandTimeout:
      return "CommandTimeout";
    case EventCode::SourceStateTimeout:
      return "SourceStateTimeout";
    case EventCode::SupervisorTimeout:
      return "SupervisorTimeout";
    case EventCode::FaultLatched:
      return "FaultLatched";
    case EventCode::FaultOutputDegraded:
      return "FaultOutputDegraded";
    case EventCode::RecoveryReady:
      return "RecoveryReady";
    case EventCode::ResetApplied:
      return "ResetApplied";
    case EventCode::RequestRejected:
      return "RequestRejected";
    case EventCode::ProtocolExhausted:
      return "ProtocolExhausted";
  }
  return "Unknown";
}

// Severity of one tick event: a latched fault is an error, something a
// supervisor or a device failed to deliver is a warning, everything else is
// informational.
configuration::LogLevel EventLevel(EventCode code) {
  switch (code) {
    case EventCode::FaultLatched:
    case EventCode::FaultOutputDegraded:
    case EventCode::BootFailed:
    case EventCode::ProtocolExhausted:
      return configuration::LogLevel::Error;
    case EventCode::RequestRejected:
    case EventCode::CommandTimeout:
    case EventCode::SourceStateTimeout:
    case EventCode::SupervisorTimeout:
      return configuration::LogLevel::Warning;
    default:
      return configuration::LogLevel::Info;
  }
}

// Events that report the output action the tick committed.
bool ReportsOutputAction(EventCode code) {
  return code == EventCode::FaultLatched ||
         code == EventCode::FaultOutputDegraded ||
         code == EventCode::BootFailed ||
         code == EventCode::ProtocolExhausted;
}

// ---------------------------------------------------------------------------
// Log records
// ---------------------------------------------------------------------------

// One event under construction. Every string is copied into the fixed capacity
// of the logger's own event, so building a record neither allocates nor
// formats a line: the logger worker renders it.
class LogRecord {
 public:
  LogRecord(configuration::LogLevel level, protocol::TimeUs stamp_us,
            protocol::Sequence tick, std::string_view component,
            std::string_view message) {
    event_.level = level;
    event_.stamp_us = stamp_us;
    event_.tick = tick;
    static_cast<void>(event_.component.Assign(component));
    static_cast<void>(event_.message.Assign(message));
  }

  void SetEventCode(std::uint32_t code) { event_.event_code = code; }

  void Field(std::string_view name, std::string_view value) {
    if (event_.field_count >= telemetry::kMaxLogFields) {
      return;
    }
    telemetry::LogField &field = event_.fields[event_.field_count];
    if (!field.name.Assign(name) || !field.value.Assign(value)) {
      return;
    }
    ++event_.field_count;
  }

  void UintField(std::string_view name, std::uint64_t value) {
    char buffer[24];
    Field(name, UintText(value, buffer));
  }

  const telemetry::LogEvent &event() const { return event_; }

 private:
  telemetry::LogEvent event_{};
};

void LogStartup(telemetry::Logger &logger,
                const configuration::RuntimeConfig &config,
                const protocol::Id &runtime_id,
                protocol::ExecutionState state, protocol::TimeUs stamp_us) {
  LogRecord record(configuration::LogLevel::Info, stamp_us, 0, "runtime",
                   "runtime started");
  record.Field("backend", config.backend);
  char number[32];
  record.Field("control_hz", DoubleText(config.control_hz, number, 32));
  record.UintField("period_us", config.control_period_us);
  record.UintField("domain", config.dds.domain);
  record.Field("config_id", config.config_id);
  record.Field("layout_id", config.layout_id);
  char identity[33];
  record.Field("runtime_id", HexId(runtime_id, identity));
  record.Field("state", StateName(state));
  logger.TryLog(record.event());
}

// Maps one tick result into the log. Events are structural and rare, so the
// whole batch is logged; the fields carry the protocol enumerations the tick
// reported, never a rendered number.
void LogTickEvents(telemetry::Logger &logger, const TickResult &result) {
  const std::uint32_t count =
      std::min<std::uint32_t>(result.event_count, kEventCapacity);
  for (std::uint32_t index = 0; index < count; ++index) {
    const RuntimeEvent &event = result.events[index];
    LogRecord record(EventLevel(event.code), event.stamp_us, event.tick,
                     "runtime", EventCodeName(event.code));
    record.SetEventCode(static_cast<std::uint32_t>(event.code));
    record.Field("from", StateName(event.from));
    record.Field("to", StateName(event.to));
    if (event.fault_reason != protocol::FaultReason::None) {
      record.Field("fault", FaultReasonName(event.fault_reason));
    }
    if (event.reject_reason != protocol::RejectReason::None) {
      record.Field("reason", RejectReasonName(event.reject_reason));
    }
    if (event.stop_reason != protocol::StopReason::None) {
      record.Field("stop", StopReasonName(event.stop_reason));
    }
    if (ReportsOutputAction(event.code)) {
      record.Field("action", OutputActionName(result.state.output_action));
    }
    logger.TryLog(record.event());
  }
}

struct LoopCounters {
  std::uint64_t ticks{0};
  std::uint64_t overruns{0};
  protocol::TimeUs worst_overrun_us{0};
  std::uint64_t fsm_dropped{0};
};

std::uint64_t TransportDropped(const transport::DdsTransport &transport) {
  return transport.dropped_result_count() + transport.dropped_state_count() +
         transport.dropped_request_count() + transport.dropped_command_count() +
         transport.dropped_heartbeat_count() +
         transport.inbound_rejected_count();
}

// Bounded status line and the shutdown summary: the current state, and what the
// loop dropped, delayed or failed to log.
void LogSummary(telemetry::Logger &logger, std::string_view message,
                const ExecutionFsm &fsm, const LoopCounters &counters,
                const transport::DdsTransport &transport,
                std::uint64_t log_dropped, protocol::TimeUs stamp_us) {
  LogRecord record(configuration::LogLevel::Info, stamp_us, fsm.tick(),
                   "runtime", message);
  record.Field("state", StateName(fsm.execution_state()));
  record.Field("action", OutputActionName(fsm.output_action()));
  record.UintField("ticks", counters.ticks);
  record.UintField("overruns", counters.overruns);
  record.UintField("worst_overrun_us", counters.worst_overrun_us);
  record.UintField("log_dropped", log_dropped);
  record.UintField("fsm_dropped", counters.fsm_dropped);
  record.UintField("transport_dropped", TransportDropped(transport));
  logger.TryLog(record.event());
}

// Final line on stderr: the logger is already stopped here, so the numbers are
// final. Nothing about a clean stop is claimed before this point.
void ReportCleanStop(const telemetry::Logger &logger, const LoopCounters &counters,
                     const transport::DdsTransport &transport) {
  std::fprintf(stderr,
               "robot_runtime: clean shutdown: state=ZeroTorque ticks=%llu "
               "overruns=%llu worst_overrun_us=%llu written=%llu undrained=%llu "
               "dropped=%llu filtered=%llu errors=%llu transport_dropped=%llu\n",
               static_cast<unsigned long long>(counters.ticks),
               static_cast<unsigned long long>(counters.overruns),
               static_cast<unsigned long long>(counters.worst_overrun_us),
               static_cast<unsigned long long>(logger.Written()),
               static_cast<unsigned long long>(logger.Undrained()),
               static_cast<unsigned long long>(logger.Dropped()),
               static_cast<unsigned long long>(logger.Filtered()),
               static_cast<unsigned long long>(logger.error_count()),
               static_cast<unsigned long long>(TransportDropped(transport)));
}

// ---------------------------------------------------------------------------
// Runtime identity
// ---------------------------------------------------------------------------

struct RuntimeIdentity {
  protocol::Id id{};
  // True when the system entropy source was unusable and the identity is only
  // process unique instead of random.
  bool entropy_fallback{false};
};

std::uint64_t MixBits(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

RuntimeIdentity MakeRuntimeId() {
  RuntimeIdentity identity;
  try {
    std::random_device entropy;
    for (std::size_t index = 0; index < identity.id.size(); ++index) {
      identity.id[index] = static_cast<std::uint8_t>(entropy() & 0xFFu);
    }
  } catch (const std::exception &) {
    identity.entropy_fallback = true;
  }
  if (identity.id != protocol::kNoId) {
    return identity;
  }

  // The protocol reserves the all-zero identity and the runtime identity is
  // mandatory, so an unusable entropy source falls back to a value mixed from
  // the two host clocks, the process id and a stack address. It is not random,
  // but it is unique per process and never all-zero.
  identity.entropy_fallback = true;
  std::uint64_t state = MixBits(MonotonicUs());
  state ^= MixBits(static_cast<std::uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count()));
  state ^= MixBits(static_cast<std::uint64_t>(::getpid()));
  state ^= MixBits(reinterpret_cast<std::uintptr_t>(&identity));
  for (std::size_t index = 0; index < identity.id.size(); ++index) {
    state = MixBits(state);
    identity.id[index] = static_cast<std::uint8_t>(state & 0xFFu);
  }
  if (identity.id == protocol::kNoId) {
    identity.id[0] = 1;
  }
  return identity;
}

// ---------------------------------------------------------------------------
// Scenario loading
// ---------------------------------------------------------------------------

using Json = nlohmann::json;

[[noreturn]] void ScenarioFail(std::string field_path, std::string message) {
  throw ScenarioError(std::move(field_path), std::move(message));
}

std::string Join(const std::string &path, std::string_view key) {
  if (path.empty()) {
    return std::string(key);
  }
  return path + "." + std::string(key);
}

const Json *FindMember(const Json &object, std::string_view key) {
  const auto found = object.find(key);
  return found == object.end() ? nullptr : &(*found);
}

const Json &RequiredMember(const Json &object, std::string_view key,
                           const std::string &path) {
  const Json *member = FindMember(object, key);
  if (member == nullptr) {
    ScenarioFail(Join(path, key), "is missing");
  }
  return *member;
}

void RejectUnknownMembers(const Json &object, const std::string &path,
                          std::initializer_list<std::string_view> allowed) {
  for (auto member = object.begin(); member != object.end(); ++member) {
    bool known = false;
    for (const std::string_view name : allowed) {
      if (member.key() == name) {
        known = true;
        break;
      }
    }
    if (!known) {
      ScenarioFail(Join(path, member.key()), "is not a known field");
    }
  }
}

bool BoolValue(const Json &value, const std::string &path) {
  if (!value.is_boolean()) {
    ScenarioFail(path, "must be a boolean");
  }
  return value.get<bool>();
}

// Finite number: an overflow, a NaN or an infinity would poison the state
// machine's comparisons, so it is refused at load time.
double NumberValue(const Json &value, const std::string &path) {
  if (!value.is_number()) {
    ScenarioFail(path, "must be a number");
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    ScenarioFail(path, "must be finite");
  }
  return number;
}

std::uint64_t CountValue(const Json &value, const std::string &path,
                         std::uint64_t maximum) {
  std::uint64_t result = 0;
  if (value.is_number_unsigned()) {
    result = value.get<std::uint64_t>();
  } else if (value.is_number_integer()) {
    const std::int64_t signed_value = value.get<std::int64_t>();
    if (signed_value < 0) {
      ScenarioFail(path, "must not be negative");
    }
    result = static_cast<std::uint64_t>(signed_value);
  } else {
    ScenarioFail(path, "must be an integer");
  }
  if (result > maximum) {
    ScenarioFail(path, "is outside the supported range");
  }
  return result;
}

std::int64_t OffsetValue(const Json &value, const std::string &path,
                         std::int64_t limit) {
  if (value.is_number_unsigned()) {
    const std::uint64_t magnitude = value.get<std::uint64_t>();
    if (magnitude > static_cast<std::uint64_t>(limit)) {
      ScenarioFail(path, "is outside the supported range");
    }
    return static_cast<std::int64_t>(magnitude);
  }
  if (!value.is_number_integer()) {
    ScenarioFail(path, "must be an integer");
  }
  const std::int64_t result = value.get<std::int64_t>();
  if (result < -limit || result > limit) {
    ScenarioFail(path, "is outside the supported range");
  }
  return result;
}

ScenarioFeedbackEntry ParseFeedback(const Json &value, const std::string &path,
                                    std::uint32_t *effective_count) {
  if (!value.is_object()) {
    ScenarioFail(path, "must be an object");
  }
  RejectUnknownMembers(value, path, {"count", "joints", "imu"});
  ScenarioFeedbackEntry feedback;
  if (const Json *count = FindMember(value, "count")) {
    feedback.count = static_cast<std::uint32_t>(
        CountValue(*count, Join(path, "count"), protocol::kMaxJoints));
    *effective_count = *feedback.count;
  }
  if (const Json *joints = FindMember(value, "joints")) {
    const std::string joints_path = Join(path, "joints");
    if (!joints->is_array()) {
      ScenarioFail(joints_path, "must be an array");
    }
    if (joints->size() > protocol::kMaxJoints) {
      ScenarioFail(joints_path, "must hold at most 64 joints");
    }
    feedback.joints.reserve(joints->size());
    for (std::size_t index = 0; index < joints->size(); ++index) {
      const std::string joint_path =
          joints_path + "[" + std::to_string(index) + "]";
      const Json &joint = (*joints)[index];
      if (!joint.is_object()) {
        ScenarioFail(joint_path, "must be an object");
      }
      RejectUnknownMembers(joint, joint_path,
                           {"q", "dq", "tau_est", "valid", "connected",
                            "freq_hz", "error_code"});
      if (index >= *effective_count) {
        ScenarioFail(joint_path,
                     "cannot be applied: it addresses a joint outside the "
                     "snapshot count");
      }
      ScenarioJointOverride parsed;
      if (const Json *q = FindMember(joint, "q")) {
        parsed.q = NumberValue(*q, Join(joint_path, "q"));
      }
      if (const Json *dq = FindMember(joint, "dq")) {
        parsed.dq = NumberValue(*dq, Join(joint_path, "dq"));
      }
      if (const Json *tau = FindMember(joint, "tau_est")) {
        parsed.tau_est = NumberValue(*tau, Join(joint_path, "tau_est"));
      }
      if (const Json *valid = FindMember(joint, "valid")) {
        parsed.valid = BoolValue(*valid, Join(joint_path, "valid"));
      }
      if (const Json *connected = FindMember(joint, "connected")) {
        parsed.connected = BoolValue(*connected, Join(joint_path, "connected"));
      }
      if (const Json *frequency = FindMember(joint, "freq_hz")) {
        const std::string frequency_path = Join(joint_path, "freq_hz");
        const double value_hz = NumberValue(*frequency, frequency_path);
        if (value_hz < 0.0) {
          ScenarioFail(frequency_path, "must not be negative");
        }
        parsed.freq_hz = value_hz;
      }
      if (const Json *error = FindMember(joint, "error_code")) {
        parsed.error_code = static_cast<std::uint32_t>(CountValue(
            *error, Join(joint_path, "error_code"),
            std::numeric_limits<std::uint32_t>::max()));
      }
      feedback.joints.push_back(parsed);
    }
  }
  if (const Json *imu = FindMember(value, "imu")) {
    const std::string imu_path = Join(path, "imu");
    if (!imu->is_object()) {
      ScenarioFail(imu_path, "must be an object");
    }
    RejectUnknownMembers(*imu, imu_path, {"valid", "rx_stamp_us"});
    ScenarioImuOverride parsed;
    if (const Json *valid = FindMember(*imu, "valid")) {
      parsed.valid = BoolValue(*valid, Join(imu_path, "valid"));
    }
    if (const Json *stamp = FindMember(*imu, "rx_stamp_us")) {
      parsed.rx_stamp_offset_us =
          OffsetValue(*stamp, Join(imu_path, "rx_stamp_us"), kMaxStampOffsetUs);
    }
    feedback.imu = parsed;
  }
  return feedback;
}

}  // namespace

ScenarioError::ScenarioError(std::string field_path, std::string message)
    : std::runtime_error(field_path + ": " + message),
      field_path_(std::move(field_path)) {}

FakeScenario LoadFakeScenario(const std::filesystem::path &path,
                              const configuration::RuntimeConfig &config) {
  std::ifstream input(path);
  if (!input) {
    ScenarioFail("$", "cannot open scenario file " + path.string());
  }
  Json document;
  try {
    input >> document;
  } catch (const Json::exception &error) {
    ScenarioFail("$", std::string("invalid JSON: ") + error.what());
  }
  if (!document.is_object()) {
    ScenarioFail("$", "must be an object");
  }
  RejectUnknownMembers(document, "", {"$comment", "entries"});
  const Json &entries = RequiredMember(document, "entries", "");
  if (!entries.is_array()) {
    ScenarioFail("entries", "must be an array");
  }

  FakeScenario scenario;
  scenario.entries.reserve(entries.size());
  auto effective_count =
      static_cast<std::uint32_t>(config.joint_count());
  bool has_previous = false;
  protocol::TimeUs previous_elapsed = 0;
  for (std::size_t index = 0; index < entries.size(); ++index) {
    const std::string entry_path = "entries[" + std::to_string(index) + "]";
    const Json &entry = entries[index];
    if (!entry.is_object()) {
      ScenarioFail(entry_path, "must be an object");
    }
    RejectUnknownMembers(entry, entry_path,
                         {"elapsed_us", "paused", "feedback", "write_result"});
    ScenarioEntry parsed;
    parsed.elapsed_us = CountValue(
        RequiredMember(entry, "elapsed_us", entry_path),
        Join(entry_path, "elapsed_us"),
        std::numeric_limits<protocol::TimeUs>::max());
    if (has_previous && parsed.elapsed_us <= previous_elapsed) {
      ScenarioFail(Join(entry_path, "elapsed_us"), "must be strictly increasing");
    }
    has_previous = true;
    previous_elapsed = parsed.elapsed_us;
    if (const Json *paused = FindMember(entry, "paused")) {
      parsed.paused = BoolValue(*paused, Join(entry_path, "paused"));
    }
    if (const Json *feedback = FindMember(entry, "feedback")) {
      parsed.feedback =
          ParseFeedback(*feedback, Join(entry_path, "feedback"),
                        &effective_count);
    }
    if (const Json *write = FindMember(entry, "write_result")) {
      const std::string write_path = Join(entry_path, "write_result");
      if (!write->is_object()) {
        ScenarioFail(write_path, "must be an object");
      }
      RejectUnknownMembers(*write, write_path, {"accepted", "error_code"});
      if (FindMember(*write, "error_code") != nullptr) {
        // FakeBackend::SetWriteAccepted is the only write control the fake
        // backend exposes, and it reports its own rejection code.
        ScenarioFail(Join(write_path, "error_code"),
                     "is not supported: the fake backend reports its own "
                     "rejection code");
      }
      parsed.write_accepted =
          BoolValue(RequiredMember(*write, "accepted", write_path),
                    Join(write_path, "accepted"));
    }
    scenario.entries.push_back(std::move(parsed));
  }
  return scenario;
}

// ---------------------------------------------------------------------------
// Scenario application
// ---------------------------------------------------------------------------

ScenarioApplier::ScenarioApplier(FakeScenario scenario,
                                 const configuration::RuntimeConfig &config)
    : scenario_(std::move(scenario)) {
  snapshot_.count = static_cast<std::uint32_t>(config.joint_count());
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    snapshot_.joints[index].valid = true;
    snapshot_.joints[index].connected = true;
    snapshot_.joints[index].freq_hz = config.control_hz;
  }
  snapshot_.imu.valid = false;
}

void ScenarioApplier::ApplyDue(protocol::TimeUs elapsed_us,
                               protocol::TimeUs now_us,
                               hardware::FakeBackend &backend) {
  while (applied_entries_ < scenario_.entries.size() &&
         scenario_.entries[applied_entries_].elapsed_us <= elapsed_us) {
    const ScenarioEntry &entry = scenario_.entries[applied_entries_];
    paused_ = entry.paused;
    if (entry.write_accepted) {
      backend.SetWriteAccepted(*entry.write_accepted);
    }
    if (entry.feedback) {
      ApplyFeedback(*entry.feedback);
    }
    ++applied_entries_;
  }
  RefreshStamps(now_us);
  backend.Inject(snapshot_);
}

void ScenarioApplier::ApplyFeedback(const ScenarioFeedbackEntry &entry) {
  if (entry.count) {
    snapshot_.count = *entry.count;
  }
  for (std::size_t index = 0; index < entry.joints.size(); ++index) {
    const ScenarioJointOverride &source = entry.joints[index];
    protocol::JointFeedback &target = snapshot_.joints[index];
    if (source.q) {
      target.q = *source.q;
    }
    if (source.dq) {
      target.dq = *source.dq;
    }
    if (source.tau_est) {
      target.tau_est = *source.tau_est;
    }
    if (source.valid) {
      target.valid = *source.valid;
    }
    if (source.connected) {
      target.connected = *source.connected;
    }
    if (source.freq_hz) {
      target.freq_hz = *source.freq_hz;
    }
    if (source.error_code) {
      target.error_code = *source.error_code;
    }
  }
  if (entry.imu) {
    if (entry.imu->valid) {
      snapshot_.imu.valid = *entry.imu->valid;
    }
    if (entry.imu->rx_stamp_offset_us) {
      imu_stamp_offset_us_ = *entry.imu->rx_stamp_offset_us;
    }
  }
}

void ScenarioApplier::RefreshStamps(protocol::TimeUs now_us) {
  // A paused entry holds the previous sample time, which is what lets an
  // injected sample age past the configured feedback timeout.
  if (paused_) {
    return;
  }
  snapshot_.stamp_us = now_us;
  ++snapshot_.snapshot_seq;
  const std::uint32_t count =
      std::min<std::uint32_t>(snapshot_.count, protocol::kMaxJoints);
  for (std::uint32_t index = 0; index < count; ++index) {
    snapshot_.joints[index].rx_stamp_us = now_us;
  }
  if (imu_stamp_offset_us_ >= 0) {
    snapshot_.imu.rx_stamp_us =
        now_us + static_cast<protocol::TimeUs>(imu_stamp_offset_us_);
  } else {
    const auto age = static_cast<protocol::TimeUs>(-imu_stamp_offset_us_);
    snapshot_.imu.rx_stamp_us = age > now_us ? 0 : now_us - age;
  }
}

// ---------------------------------------------------------------------------
// Shutdown policy
// ---------------------------------------------------------------------------

namespace {

class ShutdownPlanner {
 public:
  // Returns the input of this tick. A stop signal while motion may still be
  // authorized adds the runtime's own ZeroTorque stop request to the stop lane
  // of this tick; every other tick passes the transport input through.
  const TickInput *PlanStop(const TickInput &inbound, const ExecutionFsm &fsm,
                            protocol::TimeUs now_us) {
    const protocol::ExecutionState state = fsm.execution_state();
    if (state != protocol::ExecutionState::Damping &&
        state != protocol::ExecutionState::Tracking) {
      return &inbound;
    }
    if (fsm.session_id() == protocol::kNoId) {
      // Damping holds without a session only when a supervisor stopped
      // tracking and its lease expired, or when it stopped from Tracking. The
      // state machine answers every non-bootstrap request of a session-less
      // machine with WrongSession, so no local request can leave this state:
      // the runtime reports it and keeps commanding damping torque, which is
      // exactly what "never report a stop that did not happen" requires.
      Report(Notice::MotionWithoutSession);
      return &inbound;
    }
    if (now_us < next_attempt_us_) {
      return &inbound;
    }
    if (!stop_input_) {
      stop_input_ = std::make_unique<TickInput>();
    }
    // The only copy of an inbound batch this loop makes, and only while a stop
    // request is due: at most once every kStopRetryIntervalUs.
    *stop_input_ = inbound;
    AppendStopRequest(stop_input_.get(), fsm, now_us);
    next_attempt_us_ = now_us + kStopRetryIntervalUs;
    // Requesting the stop and telling the operator that the runtime still waits
    // for the confirmation belong together: the request alone is not a stop.
    Report(Notice::Motion);
    return stop_input_.get();
  }

  // True only for a stop that really happened: ZeroTorque with the torque write
  // of this tick accepted. Every other state is reported and the loop runs on.
  bool MayExit(const TickResult &result, const ExecutionFsm &fsm) {
    switch (fsm.execution_state()) {
      case protocol::ExecutionState::ZeroTorque:
        if (result.write_result.accepted) {
          return true;
        }
        Report(Notice::WriteRefused);
        return false;
      case protocol::ExecutionState::Fault:
        Report(Notice::Fault);
        return false;
      case protocol::ExecutionState::Boot:
        Report(Notice::Boot);
        return false;
      case protocol::ExecutionState::Damping:
      case protocol::ExecutionState::Tracking:
        Report(Notice::Motion);
        return false;
    }
    return false;
  }

 private:
  enum class Notice { None, Boot, Motion, MotionWithoutSession, Fault, WriteRefused };

  // One report per situation: the state can change while the loop waits, but a
  // status that repeats at control frequency is never printed on stderr.
  void Report(Notice notice) {
    if (notice == last_notice_) {
      return;
    }
    last_notice_ = notice;
    switch (notice) {
      case Notice::None:
        return;
      case Notice::Boot:
        std::fprintf(stderr,
                     "robot_runtime: exit requested during the startup "
                     "self-check; waiting for ZeroTorque\n");
        return;
      case Notice::Motion:
        std::fprintf(stderr,
                     "robot_runtime: exit requested while motion may be "
                     "authorized; ZeroTorque stop requested, an explicit "
                     "ZeroTorque confirmation is still required\n");
        return;
      case Notice::MotionWithoutSession:
        std::fprintf(stderr,
                     "robot_runtime: exit requested while damping holds "
                     "without a supervision session; no request path is "
                     "available, an explicit ZeroTorque confirmation is still "
                     "required\n");
        return;
      case Notice::Fault:
        std::fprintf(stderr,
                     "robot_runtime: exit requested with a latched fault; a "
                     "Reset is required, the fault is not cleared\n");
        return;
      case Notice::WriteRefused:
        std::fprintf(stderr,
                     "robot_runtime: exit requested but the torque write of "
                     "this tick was not accepted; not exiting\n");
        return;
    }
  }

  // The runtime is not a supervisor: no supervisor identity is involved, and
  // the request carries the current session and a request id taken from the
  // tick counter, which is strictly increasing and therefore newer than the
  // answer the session already recorded. A supervisor that keeps commanding
  // below that id is answered as stale until it advances past it; the
  // alternative - guessing a foreign sequence - cannot be made correct here.
  static void AppendStopRequest(TickInput *input, const ExecutionFsm &fsm,
                                protocol::TimeUs now_us) {
    protocol::ExecutionRequest request;
    request.protocol_version = protocol::kProtocolVersion;
    request.supervisor_id = protocol::kNoId;
    request.request_id = fsm.tick() + 1;
    request.session_id = fsm.session_id();
    request.runtime_id = fsm.runtime_id();
    request.expected_epoch = fsm.epoch();
    request.proposed_epoch = 0;
    request.producer_id = fsm.producer_id();
    request.fault_id = fsm.fault_id();
    request.operation = protocol::Operation::ZeroTorque;
    input->stop_requests[0] = {request, now_us};
    input->stop_request_count = 1;
  }

  Notice last_notice_{Notice::None};
  protocol::TimeUs next_attempt_us_{0};
  std::unique_ptr<TickInput> stop_input_;
};

}  // namespace

protocol::TimeUs MonotonicUs() {
  const auto elapsed = Clock::now().time_since_epoch();
  return static_cast<protocol::TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

const char *UsageLine() {
  return "usage: robot_runtime --config PATH [--domain N] "
         "[--fake-scenario PATH]";
}

CommandLine ParseCommandLine(int argc, char **argv) {
  CommandLine result;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    const bool has_value = index + 1 < argc;
    if (argument == "--config") {
      if (!has_value) {
        result.error = "--config requires a path";
        return result;
      }
      result.options.config_path = argv[++index];
      continue;
    }
    if (argument == "--fake-scenario") {
      if (!has_value) {
        result.error = "--fake-scenario requires a path";
        return result;
      }
      result.options.fake_scenario_path = argv[++index];
      continue;
    }
    if (argument == "--domain") {
      if (!has_value) {
        result.error = "--domain requires a value";
        return result;
      }
      const std::string value(argv[++index]);
      std::size_t consumed = 0;
      std::uint64_t domain = 0;
      try {
        domain = std::stoull(value, &consumed);
      } catch (const std::exception &) {
        result.error = "--domain must be an integer, got \"" + value + "\"";
        return result;
      }
      if (consumed != value.size() || domain > kMaxDdsDomain) {
        result.error = "--domain must be between 0 and " +
                       std::to_string(kMaxDdsDomain) + ", got \"" + value + "\"";
        return result;
      }
      result.options.dds_domain = static_cast<std::uint32_t>(domain);
      continue;
    }
    result.error = "unknown argument \"" + std::string(argument) + "\"";
    return result;
  }

  if (result.options.config_path.empty()) {
    result.error = "--config is required";
    return result;
  }
  result.ok = true;
  result.exit_code = kExitOk;
  return result;
}

int RunStandaloneRuntime(const RuntimeOptions &options) {
  configuration::RuntimeConfig config;
  try {
    config = configuration::LoadConfig(options.config_path);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "robot_runtime: configuration error: %s\n",
                 error.what());
    return kExitStartupError;
  }
  // The configuration parser already refuses every backend but the fake one, so
  // this is the second, explicit gate: nothing touches a device or DDS before
  // it passed, and `--fake-scenario` therefore can only reach a fake backend.
  if (config.backend != "fake") {
    std::fprintf(stderr,
                 "robot_runtime: backend \"%s\" is not supported; only the "
                 "fake backend can be started\n",
                 config.backend.c_str());
    return kExitStartupError;
  }
  if (options.dds_domain) {
    config.dds.domain = *options.dds_domain;
  }

  std::optional<FakeScenario> scenario;
  if (options.fake_scenario_path) {
    try {
      scenario = LoadFakeScenario(*options.fake_scenario_path, config);
    } catch (const std::exception &error) {
      std::fprintf(stderr, "robot_runtime: scenario error: %s\n", error.what());
      return kExitStartupError;
    }
  }

  const RuntimeIdentity identity = MakeRuntimeId();
  hardware::FakeBackend backend(config);
  backend.Initialize();

  ExecutionFsm fsm(config, identity.id);
  telemetry::Logger logger(config.telemetry);
  if (!logger.Start()) {
    std::fprintf(stderr, "robot_runtime: the logger did not start\n");
    backend.Close();
    return kExitStartupError;
  }
  if (!InstallStopHandlers()) {
    std::fprintf(stderr, "robot_runtime: cannot install the stop handlers\n");
    logger.Stop();
    backend.Close();
    return kExitStartupError;
  }

  transport::DdsTransport transport(config);
  try {
    transport.Start();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "robot_runtime: transport start failed: %s\n",
                 error.what());
    logger.Stop();
    backend.Close();
    return kExitStartupError;
  }

  ScenarioApplier applier(scenario.value_or(FakeScenario{}), config);
  const protocol::TimeUs start_us = MonotonicUs();
  LogStartup(logger, config, identity.id, fsm.execution_state(), start_us);
  if (identity.entropy_fallback) {
    LogRecord record(configuration::LogLevel::Warning, start_us, 0, "runtime",
                     "system entropy unusable; runtime id is only process "
                     "unique");
    char rendered[33];
    record.Field("runtime_id", HexId(identity.id, rendered));
    logger.TryLog(record.event());
  }

  const protocol::TimeUs period_us =
      config.control_period_us == 0 ? 1 : config.control_period_us;
  LoopCounters counters;
  protocol::TimeUs next_status_us = start_us + kStatusIntervalUs;
  auto deadline = Clock::now();
  ShutdownPlanner shutdown;

  while (true) {
    const protocol::TimeUs now = MonotonicUs();
    // Due scenario entries are applied before the read of this tick, so the
    // snapshot the state machine sees is the one the scenario describes at this
    // instant - including the entry due at process start.
    applier.ApplyDue(now - start_us, now, backend);

    const bool stop_requested =
        g_stop_requested.load(std::memory_order_relaxed);
    const TickInput *input = &transport.TakeInput();
    if (stop_requested) {
      input = shutdown.PlanStop(*input, fsm, now);
    }
    const protocol::FeedbackSnapshot feedback = backend.Read(now);
    const TickResult result = fsm.Tick(now, feedback, *input, backend);
    transport.Publish(result);
    ++counters.ticks;
    counters.fsm_dropped =
        result.dropped_result_count + result.dropped_event_count;
    LogTickEvents(logger, result);

    if (stop_requested && shutdown.MayExit(result, fsm)) {
      LogSummary(logger, "shutdown", fsm, counters, transport, logger.Dropped(),
                 now);
      transport.Stop();
      logger.Stop();
      ReportCleanStop(logger, counters, transport);
      backend.Close();
      return kExitOk;
    }

    if (now >= next_status_us) {
      LogSummary(logger, "status", fsm, counters, transport, logger.Dropped(),
                 now);
      // Anchored on the current time: a delayed loop reports late instead of
      // printing a burst of catch-up lines.
      next_status_us = now + kStatusIntervalUs;
    }

    // Absolute deadline, advanced by exactly one period: the phase never
    // accumulates drift and no tick busy-waits. A tick that ran past its
    // deadline is counted and reported; one that ran a whole period late
    // re-anchors the schedule instead of starting a catch-up burst.
    deadline += std::chrono::microseconds(period_us);
    const Clock::time_point work_end = Clock::now();
    if (work_end < deadline) {
      std::this_thread::sleep_until(deadline);
      continue;
    }
    const auto late = std::chrono::duration_cast<std::chrono::microseconds>(
                          work_end - deadline)
                          .count();
    ++counters.overruns;
    counters.worst_overrun_us =
        std::max(counters.worst_overrun_us, static_cast<protocol::TimeUs>(late));
    if (late >= static_cast<std::chrono::microseconds::rep>(period_us)) {
      deadline = work_end;
    }
  }
}

}  // namespace irmv::runtime
