#include "irmv/runtime/runtime_loop.hpp"

#include <unistd.h>

#include <algorithm>
#include <array>
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
#include <utility>

#include "irmv/hardware/backend_factory.hpp"
#include "irmv/runtime/execution_fsm.hpp"
#include "irmv/telemetry/diagnostic_write.hpp"
#include "irmv/telemetry/logger.hpp"
#include "irmv/telemetry/recorder.hpp"
#include "irmv/transport/bounded_channel.hpp"
#include "irmv/transport/dds_transport.hpp"

namespace irmv::runtime {
namespace {

using Clock = std::chrono::steady_clock;

// Every diagnostic of the control thread, of the trace worker and of the exit
// path goes through the bounded writer. stderr may be a pipe nobody drains -
// the logger can even be left detached inside a sink parked on it - and neither
// the control loop nor the shutdown may wait for a reader that never comes. The
// band is a hard bound: whatever does not fit is dropped, and a truncated line
// is never reported as written.
void StderrLine(std::string_view text) {
  static_cast<void>(telemetry::WriteDiagnosticBounded(::fileno(stderr), text));
}

// Same, for diagnostics with arguments. The line is rendered on the stack and
// bounded like every other one.
template <typename... Args>
void StderrFormat(const char *format, Args... args) {
  std::array<char, 512> line{};
  const int length = std::snprintf(line.data(), line.size(), format, args...);
  if (length <= 0) {
    return;
  }
  const std::size_t size =
      std::min<std::size_t>(static_cast<std::size_t>(length), line.size() - 1);
  StderrLine(std::string_view(line.data(), size));
}

// Period of the bounded status line and of the shutdown summary. Both carry the
// overrun accounting, so no tick ever logs at control frequency.
constexpr protocol::TimeUs kStatusIntervalUs = 1'000'000;

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
         code == EventCode::BootFailed || code == EventCode::ProtocolExhausted;
}

// ---------------------------------------------------------------------------
// Test-only output trace
// ---------------------------------------------------------------------------
//
// One JSON-lines record per torque write of the fake backend, built from what
// really happened: the backend's own output history and the state the tick
// published. Nothing in the trace is recomputed or interpolated, so a tick
// whose record went missing stays a hole in the recorded tick sequence and the
// consumer must fail the acceptance instead of repairing it. A tick whose write
// was refused and whose safe fallback was attempted carries two records - the
// same tick twice - and the consumer reads the last one as the effective write.
//
// The control thread only copies a fixed-size record into a bounded channel;
// one writer thread owns the file, so a slow or blocked file system can never
// delay a control tick. A full channel drops the record and counts it, which is
// the only honest report of a trace that is not complete.

inline constexpr std::size_t kTraceChannelCapacity = 64;
// Writes the control thread drains from the backend history per call. A tick
// makes one write, or two when a refused write is followed by the safe
// fallback, so the buffer stays small and the loop repeats the drain.
inline constexpr std::size_t kTraceDrainBatch = 8;
// Largest line a record renders to: eight joint arrays of 64 values, each
// printed round-trip exact, plus the scalars. A record that does not fit is
// refused and counted instead of being truncated into a wrong measurement.
inline constexpr std::size_t kTraceLineCapacity = 24 * 1024;
// Writer idle poll. Long enough to cost nothing, short enough that a record
// reaches the file well inside the next control period.
inline constexpr std::uint32_t kTraceIdleSleepUs = 250;

struct OutputTraceRecord {
  protocol::Sequence tick{0};
  protocol::TimeUs stamp_us{0};
  protocol::ExecutionState execution_state{protocol::ExecutionState::Boot};
  protocol::OutputAction output_action{protocol::OutputAction::ZeroTorque};
  protocol::Id runtime_id{};
  protocol::Id session_id{};
  protocol::Id producer_id{};
  protocol::Sequence epoch{0};
  protocol::Sequence fault_id{0};
  protocol::Sequence last_applied_command_seq{0};
  bool write_accepted{false};
  bool has_target{false};
  protocol::TimeUs target_rx_us{0};
  protocol::Sequence target_command_seq{0};
  protocol::Sequence target_epoch{0};
  protocol::Sequence target_source_state_seq{0};
  protocol::Id target_producer_id{};
  std::uint32_t joint_count{0};
  std::array<double, protocol::kMaxJoints> target_q{};
  std::array<double, protocol::kMaxJoints> target_dq{};
  std::array<double, protocol::kMaxJoints> target_kp{};
  std::array<double, protocol::kMaxJoints> target_kd{};
  std::array<double, protocol::kMaxJoints> target_tau_ff{};
  std::array<double, protocol::kMaxJoints> measured_q{};
  std::array<double, protocol::kMaxJoints> measured_dq{};
  std::array<double, protocol::kMaxJoints> torque{};
};

// Allocation-free JSON writer over a caller buffer. Every append is checked:
// overflow is reported once and the record is refused by the caller, so a
// truncated line never looks like a measurement.
class JsonLine {
 public:
  JsonLine(char *buffer, std::size_t capacity)
      : buffer_(buffer), capacity_(capacity) {
    if (capacity_ > 0) {
      buffer_[0] = '\0';
    }
  }

  void Field(std::string_view name, std::uint64_t value) {
    if (Key(name)) {
      AppendUint(value);
    }
  }

  void Field(std::string_view name, bool value) {
    if (Key(name)) {
      Append(value ? "true" : "false");
    }
  }

  void Field(std::string_view name, double value) {
    if (!Key(name)) {
      return;
    }
    char rendered[40];
    const int written =
        std::snprintf(rendered, sizeof(rendered), "%.17g", value);
    if (written <= 0) {
      Fail();
      return;
    }
    Append(std::string_view(
        rendered, std::min<std::size_t>(static_cast<std::size_t>(written),
                                        sizeof(rendered) - 1)));
  }

  void Field(std::string_view name, const char *text) {
    Field(name, std::string_view(text));
  }

  void Field(std::string_view name, std::string_view text) {
    if (!Key(name)) {
      return;
    }
    Append("\"");
    for (const char byte : text) {
      if (byte == '"' || byte == '\\') {
        Append("\\");
        Append(std::string_view(&byte, 1));
      } else if (static_cast<unsigned char>(byte) < 0x20) {
        Append("?");
      } else {
        Append(std::string_view(&byte, 1));
      }
    }
    Append("\"");
  }

  // Identifiers cross the trace as their lowercase hex form, the same
  // rendering the log uses.
  void IdField(std::string_view name, const protocol::Id &id) {
    if (!Key(name)) {
      return;
    }
    constexpr char kHex[] = "0123456789abcdef";
    char text[2 * protocol::kIdSize];
    for (std::size_t index = 0; index < id.size(); ++index) {
      text[2 * index] = kHex[(id[index] >> 4) & 0x0f];
      text[2 * index + 1] = kHex[id[index] & 0x0f];
    }
    Append("\"");
    Append(std::string_view(text, sizeof(text)));
    Append("\"");
  }

  void ArrayField(std::string_view name, const double *values,
                  std::uint32_t count) {
    if (!Key(name)) {
      return;
    }
    Append("[");
    for (std::uint32_t index = 0; index < count; ++index) {
      if (index != 0) {
        Append(",");
      }
      char rendered[40];
      const int written =
          std::snprintf(rendered, sizeof(rendered), "%.17g", values[index]);
      if (written <= 0) {
        Fail();
        return;
      }
      Append(std::string_view(
          rendered, std::min<std::size_t>(static_cast<std::size_t>(written),
                                          sizeof(rendered) - 1)));
    }
    Append("]");
  }

  // Root object of one record. Begin() opens it and resets the field list of
  // the root scope so the first key does not inherit a comma.
  void Begin() {
    if (overflow_) {
      return;
    }
    Append("{");
    has_field_[nested_] = false;
  }

  void End() { Append("}"); }

  void OpenObject(std::string_view name) {
    if (!Key(name)) {
      return;
    }
    Append("{");
    if (nested_ > 0) {
      Fail();
      return;
    }
    ++nested_;
    has_field_[nested_] = false;
  }

  void CloseObject() {
    if (nested_ == 0) {
      Fail();
      return;
    }
    Append("}");
    --nested_;
    has_field_[nested_] = true;
  }

  bool ok() const { return !overflow_; }

  std::string_view line() const { return {buffer_, size_}; }

 private:
  bool Key(std::string_view name) {
    if (overflow_) {
      return false;
    }
    if (has_field_[nested_]) {
      Append(",");
    }
    Append("\"");
    Append(name);
    Append("\":");
    return !overflow_;
  }

  void Fail() { overflow_ = true; }

  void Append(std::string_view text) {
    if (overflow_ || size_ + text.size() + 1 > capacity_) {
      overflow_ = true;
      return;
    }
    std::copy(text.begin(), text.end(), buffer_ + size_);
    size_ += text.size();
    buffer_[size_] = '\0';
    has_field_[nested_] = true;
  }

  void Append(const char *text) { Append(std::string_view(text)); }

  void AppendUint(std::uint64_t value) {
    char buffer[24];
    Append(UintText(value, buffer));
  }

  // Nesting is at most one level; the flags of the open scopes are kept for
  // both levels so a nested object starts its own field list.
  static constexpr std::size_t kMaxDepth = 2;
  char *buffer_;
  std::size_t capacity_;
  std::size_t size_{0};
  bool overflow_{false};
  std::size_t nested_{0};
  std::array<bool, kMaxDepth + 1> has_field_{};
};

// One coherent snapshot of this tick for the run recorder: the state the
// runtime published, the target that was in force for it - the same frame the
// trace records, so the two files describe the same run - the feedback the tick
// used and the host send path's answer. It is built from values the tick
// already holds; nothing here reads a device, a clock or a file.
telemetry::RecordSample MakeRecordSample(const ExecutionFsm &fsm,
                                         const runtime::TickResult &result,
                                         bool feedback_valid) {
  telemetry::RecordSample sample;
  sample.stamp_us = result.state.stamp_us;
  sample.state_seq = result.state.state_seq;
  sample.execution_state = result.state.execution_state;
  sample.output_action = result.state.output_action;
  sample.stop_reason = result.state.stop_reason;
  sample.fault_id = result.state.fault_id;
  sample.healthy = result.state.healthy;
  sample.damping_eligible = result.state.damping_eligible;
  sample.feedback_seq = result.state.feedback.snapshot_seq;
  sample.feedback_stamp_us = result.state.feedback.stamp_us;
  sample.feedback_valid = feedback_valid;
  sample.imu_valid = result.state.feedback.imu.valid;
  sample.send_accepted = result.write_result.accepted;
  sample.output_valid = result.output_valid;
  sample.joint_count = std::min<std::uint32_t>(result.state.feedback.count,
                                               protocol::kMaxJoints);
  // The measured feedback of this very tick, in the configuration's order. It
  // is meaningful only for the joints the feedback snapshot really carried and
  // only while `feedback_valid` holds; the flag, not the numbers, is the
  // contract, so a reader never reads a zero a failed read left behind as a
  // measurement.
  for (std::uint32_t index = 0;
       index < result.state.feedback.count && index < protocol::kMaxJoints;
       ++index) {
    sample.measured_q[index] = result.state.feedback.joints[index].q;
    sample.measured_dq[index] = result.state.feedback.joints[index].dq;
  }
  // The final output of this tick: the torques the output stage produced for
  // the backend, from `TickResult::output`. `send_accepted` above is the host
  // path's answer, not a device acknowledgement.
  for (std::uint32_t index = 0;
       index < result.output.count && index < sample.joint_count; ++index) {
    sample.output_torque[index] = result.output.torque[index];
  }
  if (fsm.has_target()) {
    const protocol::JointCommand &target = fsm.current_target();
    sample.command_present = true;
    sample.producer_id = target.producer_id;
    sample.epoch = target.epoch;
    sample.command_seq = target.command_seq;
    sample.source_state_seq = target.source_state_seq;
    sample.command_rx_us = fsm.target_rx_us();
    sample.q = target.joints.q;
    sample.dq = target.joints.dq;
    sample.tau_ff = target.joints.tau_ff;
    sample.kp = target.joints.kp;
    sample.kd = target.joints.kd;
  }
  return sample;
}

// One session event for the recorder's side channel: the authorization context
// this tick observes - the runtime instance and session, the producer and epoch
// that hold it - and the gains of the first command adopted with it. It is
// built from the state and target the tick already holds; nothing here reads a
// device, a clock or a file, and nothing allocates.
telemetry::RecordSessionEvent MakeSessionEvent(
    const ExecutionFsm &fsm, const runtime::TickResult &result) {
  telemetry::RecordSessionEvent event;
  event.runtime_id = result.state.runtime_id;
  event.session_id = result.state.session_id;
  event.producer_id = result.state.producer_id;
  event.epoch = result.state.epoch;
  event.stamp_us = result.state.stamp_us;
  event.state_seq = result.state.state_seq;
  // The model digest belongs to the application, the only side that loaded a
  // model: the runtime links no inference and has no digest to write, so it
  // writes the literal "unknown" here rather than inventing one. A real digest
  // travels in the application's own session record.
  std::memcpy(event.model_sha256.data(), "unknown", sizeof("unknown"));
  if (fsm.has_target()) {
    const protocol::JointCommand &target = fsm.current_target();
    event.joint_count =
        std::min<std::uint32_t>(target.joints.count, protocol::kMaxJoints);
    event.kp = target.joints.kp;
    event.kd = target.joints.kd;
  }
  return event;
}

OutputTraceRecord MakeTraceRecord(const ExecutionFsm &fsm,
                                  const runtime::TickResult &result,
                                  const hardware::OutputRecord &output) {
  OutputTraceRecord record;
  record.tick = fsm.tick();
  record.stamp_us = output.write_stamp_us;
  record.execution_state = result.state.execution_state;
  record.output_action = result.state.output_action;
  record.runtime_id = result.state.runtime_id;
  record.session_id = result.state.session_id;
  record.producer_id = result.state.producer_id;
  record.epoch = result.state.epoch;
  record.fault_id = result.state.fault_id;
  record.last_applied_command_seq = result.state.last_applied_command_seq;
  record.write_accepted = output.result.accepted;
  record.has_target = fsm.has_target();
  if (record.has_target) {
    const protocol::JointCommand &target = fsm.current_target();
    record.target_rx_us = fsm.target_rx_us();
    record.target_command_seq = target.command_seq;
    record.target_epoch = target.epoch;
    record.target_source_state_seq = target.source_state_seq;
    record.target_producer_id = target.producer_id;
    record.target_q = target.joints.q;
    record.target_dq = target.joints.dq;
    record.target_kp = target.joints.kp;
    record.target_kd = target.joints.kd;
    record.target_tau_ff = target.joints.tau_ff;
  }
  record.joint_count = std::min<std::uint32_t>(result.state.feedback.count,
                                               protocol::kMaxJoints);
  for (std::uint32_t index = 0; index < record.joint_count; ++index) {
    record.measured_q[index] = result.state.feedback.joints[index].q;
    record.measured_dq[index] = result.state.feedback.joints[index].dq;
  }
  record.torque = output.output.torque;
  return record;
}

// Control-thread front end of the trace: accepts fixed-size records into a
// bounded channel and owns the writer thread and the file.
class OutputTrace {
 public:
  explicit OutputTrace(std::filesystem::path path)
      : path_(std::move(path)), line_(new char[kTraceLineCapacity]) {}

  OutputTrace(const OutputTrace &) = delete;
  OutputTrace &operator=(const OutputTrace &) = delete;

  ~OutputTrace() { Stop(); }

  bool Start();
  void TryRecord(const OutputTraceRecord &record);
  void Stop();

  // What the trace did, as one snapshot. Only the joined state may hand it out:
  // Stop() joins the writer thread first, and that thread is the only producer
  // of `recorded` and `write_failures`, so a snapshot taken afterwards is final
  // and no reader can observe a value that is still being written.
  struct Counters {
    bool active{false};
    std::uint64_t recorded{0};
    std::uint64_t dropped{0};
    std::uint64_t write_failures{0};
  };

  Counters counters() const {
    Counters snapshot;
    snapshot.active = true;
    snapshot.recorded = recorded_.load(std::memory_order_relaxed);
    snapshot.dropped = dropped_;
    snapshot.write_failures = write_failures_.load(std::memory_order_relaxed);
    return snapshot;
  }

 private:
  void Run();
  void Render(const OutputTraceRecord &record);

  std::filesystem::path path_;
  std::ofstream file_;
  std::unique_ptr<char[]> line_;
  transport::BoundedChannel<OutputTraceRecord, kTraceChannelCapacity> channel_;
  std::thread worker_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> recorded_{0};
  std::atomic<std::uint64_t> write_failures_{0};
  std::uint64_t dropped_{0};
  bool running_{false};
};

bool OutputTrace::Start() {
  file_.open(path_, std::ios::out | std::ios::trunc);
  if (!file_) {
    StderrFormat("robot_runtime: cannot open the output trace %s\n",
                 path_.c_str());
    return false;
  }
  worker_ = std::thread([this]() { Run(); });
  running_ = true;
  return true;
}

void OutputTrace::TryRecord(const OutputTraceRecord &record) {
  if (running_ && !channel_.TryPush(record)) {
    // A dropped record is a hole in the trace and is reported as one; it is
    // never papered over by delaying the control loop.
    ++dropped_;
  }
}

void OutputTrace::Stop() {
  if (!running_) {
    return;
  }
  stop_requested_.store(true, std::memory_order_relaxed);
  worker_.join();
  running_ = false;
  // The end record makes a truncated file distinguishable from a complete one:
  // a consumer that does not find it knows the writer never finished.
  JsonLine line(line_.get(), kTraceLineCapacity);
  line.Begin();
  line.Field("kind", "end");
  line.Field("recorded", recorded_.load(std::memory_order_relaxed));
  line.Field("dropped", dropped_);
  line.Field("write_failures", write_failures_.load(std::memory_order_relaxed));
  line.End();
  if (line.ok()) {
    std::string_view text = line.line();
    file_.write(text.data(), static_cast<std::streamsize>(text.size()));
    file_.put('\n');
  }
  file_.flush();
  file_.close();
}

void OutputTrace::Run() {
  OutputTraceRecord record;
  while (true) {
    if (channel_.TryPop(record)) {
      Render(record);
      continue;
    }
    if (stop_requested_.load(std::memory_order_relaxed)) {
      // The control thread is the only producer and has already stopped
      // pushing.
      while (channel_.TryPop(record)) {
        Render(record);
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kTraceIdleSleepUs));
  }
}

void OutputTrace::Render(const OutputTraceRecord &record) {
  JsonLine line(line_.get(), kTraceLineCapacity);
  line.Begin();
  line.Field("kind", "tick");
  line.Field("tick", record.tick);
  line.Field("stamp_us", record.stamp_us);
  line.IdField("runtime_id", record.runtime_id);
  line.IdField("session_id", record.session_id);
  line.IdField("producer_id", record.producer_id);
  line.Field("state", StateName(record.execution_state));
  line.Field("action", OutputActionName(record.output_action));
  line.Field("epoch", record.epoch);
  line.Field("fault_id", record.fault_id);
  line.Field("last_applied_command_seq", record.last_applied_command_seq);
  line.Field("write_accepted", record.write_accepted);
  line.Field("target_present", record.has_target);
  line.OpenObject("target");
  line.Field("rx_us", record.target_rx_us);
  line.Field("command_seq", record.target_command_seq);
  line.Field("epoch", record.target_epoch);
  line.Field("source_state_seq", record.target_source_state_seq);
  line.IdField("producer_id", record.target_producer_id);
  line.CloseObject();
  line.Field("joint_count", static_cast<std::uint64_t>(record.joint_count));
  line.ArrayField("target_q", record.target_q.data(), record.joint_count);
  line.ArrayField("target_dq", record.target_dq.data(), record.joint_count);
  line.ArrayField("target_kp", record.target_kp.data(), record.joint_count);
  line.ArrayField("target_kd", record.target_kd.data(), record.joint_count);
  line.ArrayField("target_tau_ff", record.target_tau_ff.data(),
                  record.joint_count);
  line.ArrayField("measured_q", record.measured_q.data(), record.joint_count);
  line.ArrayField("measured_dq", record.measured_dq.data(), record.joint_count);
  line.ArrayField("torque", record.torque.data(), record.joint_count);
  line.End();
  if (!line.ok()) {
    // Refused instead of truncated: the consumer sees the tick missing.
    write_failures_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const std::string_view text = line.line();
  file_.write(text.data(), static_cast<std::streamsize>(text.size()));
  file_.put('\n');
  if (!file_) {
    if (write_failures_.load(std::memory_order_relaxed) == 0) {
      StderrFormat("robot_runtime: the output trace %s cannot be written\n",
                   path_.c_str());
    }
    write_failures_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  recorded_.fetch_add(1, std::memory_order_relaxed);
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
                const protocol::Id &runtime_id, protocol::ExecutionState state,
                protocol::TimeUs stamp_us) {
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
// reported, never a rendered number. The state machine is read only for the
// lease-expiry line, which must name the silence that ended the lease.
void LogTickEvents(telemetry::Logger &logger, const ExecutionFsm &fsm,
                   const TickResult &result) {
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
    if (event.code == EventCode::SupervisorTimeout) {
      // The lease's own line names the silence: the sequence of the last
      // heartbeat the machine accepted, how long ago it accepted it, and the
      // two stamps the lease was judged against - captured before the expiry
      // cleared the session, which is why they are read through the expiry
      // accessors and not from the live fields. A lease that expires while the
      // acceptance is fresh and the watermark is not (or the other way round)
      // says which of the two decided.
      const protocol::TimeUs accepted_us = fsm.last_accepted_heartbeat_us();
      const protocol::TimeUs lease_us = fsm.expired_lease_rx_us();
      const protocol::TimeUs tick_us = fsm.expired_lease_tick_us();
      record.UintField("last_heartbeat_seq", fsm.last_accepted_heartbeat_seq());
      record.UintField("heartbeat_age_us",
                       accepted_us == 0 ? 0
                                        : static_cast<std::uint64_t>(
                                              event.stamp_us - accepted_us));
      record.UintField("lease_rx_us", lease_us);
      record.UintField("lease_age_us",
                       lease_us == 0 || tick_us < lease_us
                           ? 0
                           : static_cast<std::uint64_t>(tick_us - lease_us));
      record.UintField("expiry_tick_us", tick_us);
      record.UintField("expiry_stamp_us", event.stamp_us);
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
  // The heartbeat chain, one field per step: what the transport read from DDS,
  // what the machine accepted and discarded, and how long ago it accepted the
  // last one. A silence is read straight off the age.
  record.UintField("heartbeats_read", transport.heartbeats_read_count());
  record.UintField("heartbeats_accepted", fsm.heartbeats_accepted_count());
  record.UintField("heartbeats_rejected", fsm.heartbeats_rejected_count());
  record.UintField("last_heartbeat_seq", fsm.last_accepted_heartbeat_seq());
  const protocol::TimeUs accepted_us = fsm.last_accepted_heartbeat_us();
  record.UintField("heartbeat_age_us",
                   accepted_us == 0
                       ? 0
                       : static_cast<std::uint64_t>(stamp_us - accepted_us));
  // The cadence, so a hole is a number rather than something inferred from a
  // summary count: the worst tick-to-tick interval between accepted heartbeats
  // and how many of them the runtime judged late.
  record.UintField("worst_accepted_heartbeat_gap_us",
                   fsm.worst_accepted_heartbeat_gap_us());
  record.UintField("heartbeats_late_count", fsm.heartbeats_late_count());
  logger.TryLog(record.event());
}

// Final line on stderr: the logger is already stopped here, so the numbers are
// final. Nothing about a clean stop is claimed before this point, and the line
// itself is bounded: a full pipe must not turn the shutdown into a wait, so it
// is rendered on the stack and written through the bounded diagnostic writer.
void ReportCleanStop(const telemetry::Logger &logger,
                     const LoopCounters &counters, const ExecutionFsm &fsm,
                     const transport::DdsTransport &transport,
                     const OutputTrace::Counters &trace,
                     protocol::TimeUs stamp_us) {
  const protocol::TimeUs accepted_us = fsm.last_accepted_heartbeat_us();
  const std::uint64_t heartbeat_age_us =
      accepted_us == 0 ? 0 : static_cast<std::uint64_t>(stamp_us - accepted_us);
  std::array<char, 1024> line{};
  const int length = std::snprintf(
      line.data(), line.size(),
      "robot_runtime: clean shutdown: state=ZeroTorque ticks=%llu "
      "overruns=%llu worst_overrun_us=%llu written=%llu undrained=%llu "
      "dropped=%llu filtered=%llu errors=%llu transport_dropped=%llu "
      "request_dropped=%llu command_dropped=%llu heartbeat_dropped=%llu "
      "heartbeats_read=%llu heartbeats_accepted=%llu heartbeats_rejected=%llu "
      "last_heartbeat_seq=%llu heartbeat_age_us=%llu "
      "worst_accepted_heartbeat_gap_us=%llu heartbeats_late_count=%llu "
      "trace_active=%d trace_written=%llu trace_dropped=%llu "
      "trace_failures=%llu\n",
      static_cast<unsigned long long>(counters.ticks),
      static_cast<unsigned long long>(counters.overruns),
      static_cast<unsigned long long>(counters.worst_overrun_us),
      static_cast<unsigned long long>(logger.Written()),
      static_cast<unsigned long long>(logger.Undrained()),
      static_cast<unsigned long long>(logger.Dropped()),
      static_cast<unsigned long long>(logger.Filtered()),
      static_cast<unsigned long long>(logger.error_count()),
      static_cast<unsigned long long>(TransportDropped(transport)),
      static_cast<unsigned long long>(transport.dropped_request_count()),
      static_cast<unsigned long long>(transport.dropped_command_count()),
      static_cast<unsigned long long>(transport.dropped_heartbeat_count()),
      static_cast<unsigned long long>(transport.heartbeats_read_count()),
      static_cast<unsigned long long>(fsm.heartbeats_accepted_count()),
      static_cast<unsigned long long>(fsm.heartbeats_rejected_count()),
      static_cast<unsigned long long>(fsm.last_accepted_heartbeat_seq()),
      static_cast<unsigned long long>(heartbeat_age_us),
      static_cast<unsigned long long>(fsm.worst_accepted_heartbeat_gap_us()),
      static_cast<unsigned long long>(fsm.heartbeats_late_count()),
      trace.active ? 1 : 0, static_cast<unsigned long long>(trace.recorded),
      static_cast<unsigned long long>(trace.dropped),
      static_cast<unsigned long long>(trace.write_failures));
  if (length <= 0) {
    return;
  }
  const std::size_t size =
      std::min<std::size_t>(static_cast<std::size_t>(length), line.size() - 1);
  StderrLine(std::string_view(line.data(), size));
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
        parsed.error_code = static_cast<std::uint32_t>(
            CountValue(*error, Join(joint_path, "error_code"),
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
  auto effective_count = static_cast<std::uint32_t>(config.joint_count());
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
    parsed.elapsed_us =
        CountValue(RequiredMember(entry, "elapsed_us", entry_path),
                   Join(entry_path, "elapsed_us"),
                   std::numeric_limits<protocol::TimeUs>::max());
    if (has_previous && parsed.elapsed_us <= previous_elapsed) {
      ScenarioFail(Join(entry_path, "elapsed_us"),
                   "must be strictly increasing");
    }
    has_previous = true;
    previous_elapsed = parsed.elapsed_us;
    if (const Json *paused = FindMember(entry, "paused")) {
      parsed.paused = BoolValue(*paused, Join(entry_path, "paused"));
    }
    if (const Json *feedback = FindMember(entry, "feedback")) {
      parsed.feedback = ParseFeedback(*feedback, Join(entry_path, "feedback"),
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
  // Why the loop keeps running after a stop was requested. Reported once per
  // change, never at control frequency.
  enum class Notice {
    None,
    Boot,
    Motion,
    MotionWithoutSession,
    Fault,
    WriteRefused
  };

  // Returns the input of this tick. A stop signal while motion may still be
  // authorized asks the state machine for the runtime's own ZeroTorque stop
  // (TickInput::local_stop_requested); every other tick passes the transport
  // input through. No supervisor identity, session or request id is fabricated,
  // every received stop of the tick is still answered, and no received stop can
  // be displaced because the local stop is not part of a lane.
  const TickInput *PlanStop(const TickInput &inbound, const ExecutionFsm &fsm) {
    const protocol::ExecutionState state = fsm.execution_state();
    if (state != protocol::ExecutionState::Damping &&
        state != protocol::ExecutionState::Tracking) {
      return &inbound;
    }
    if (fsm.session_id() == protocol::kNoId) {
      // Damping holds without a session only when a supervisor stopped
      // tracking and its lease expired, or when it stopped from Tracking. The
      // local stop is only taken while a session can still be answered, so the
      // runtime reports this and keeps commanding damping torque, which is
      // exactly what "never report a stop that did not happen" requires.
      Record(Notice::MotionWithoutSession);
      return &inbound;
    }
    if (!stop_input_) {
      stop_input_ = std::make_unique<TickInput>();
    }
    // The only copy of an inbound batch this loop makes, and only while motion
    // may still be authorized. The state machine applies the local stop in the
    // same tick, so this is not a retry loop: the next tick sees ZeroTorque and
    // passes the batch through again.
    *stop_input_ = inbound;
    stop_input_->local_stop_requested = true;
    // Requesting the stop and telling the operator that the runtime still waits
    // for the confirmation belong together: the request alone is not a stop.
    Record(Notice::Motion);
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
        Record(Notice::WriteRefused);
        return false;
      case protocol::ExecutionState::Fault:
        Record(Notice::Fault);
        return false;
      case protocol::ExecutionState::Boot:
        Record(Notice::Boot);
        return false;
      case protocol::ExecutionState::Damping:
      case protocol::ExecutionState::Tracking:
        Record(Notice::Motion);
        return false;
    }
    return false;
  }

  // One tick makes two decisions - what to request and whether to leave - and
  // both may have a reason to report. Only the last decision of the tick is
  // reported, and only when it differs from the previous report, so a state the
  // runtime cannot leave on its own never prints at control frequency.
  void Record(Notice notice) { pending_ = notice; }

  void FlushNotice() {
    if (pending_ == last_notice_) {
      return;
    }
    last_notice_ = pending_;
    Print(pending_);
  }

 private:
  void Print(Notice notice) {
    switch (notice) {
      case Notice::None:
        return;
      case Notice::Boot:
        StderrLine(
            "robot_runtime: exit requested during the startup "
            "self-check; waiting for ZeroTorque\n");
        return;
      case Notice::Motion:
        StderrLine(
            "robot_runtime: exit requested while motion may be "
            "authorized; ZeroTorque stop requested, an explicit "
            "ZeroTorque confirmation is still required\n");
        return;
      case Notice::MotionWithoutSession:
        StderrLine(
            "robot_runtime: exit requested while damping holds "
            "without a supervision session; no request path is "
            "available, an explicit ZeroTorque confirmation is still "
            "required\n");
        return;
      case Notice::Fault:
        StderrLine(
            "robot_runtime: exit requested with a latched fault; a "
            "Reset is required, the fault is not cleared\n");
        return;
      case Notice::WriteRefused:
        StderrLine(
            "robot_runtime: exit requested but the torque write of "
            "this tick was not accepted; not exiting\n");
        return;
    }
  }

  // The runtime asks for its own stop through TickInput::local_stop_requested:
  // the machine is the only source of that flag, so the operator's signal is a
  // trusted local stop and never a fabricated protocol request.
  Notice pending_{Notice::None};
  Notice last_notice_{Notice::None};
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
         "[--fake-scenario PATH] [--output-trace PATH]";
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
    if (argument == "--output-trace") {
      if (!has_value) {
        result.error = "--output-trace requires a path";
        return result;
      }
      result.options.output_trace_path = argv[++index];
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
      if (consumed != value.size() || domain > configuration::kMaxDdsDomain) {
        result.error = "--domain must be between 0 and " +
                       std::to_string(configuration::kMaxDdsDomain) +
                       ", got \"" + value + "\"";
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
  // The scenario timeline is measured from process start, so its base is taken
  // before anything else of this run happens - including validation, which is
  // allowed to fail and end the process.
  const protocol::TimeUs process_start_us = MonotonicUs();

  // A domain override is applied by the configuration layer, so the identity
  // digest covers the configuration in effect and not the file on disk.
  configuration::ConfigOverrides overrides;
  if (options.dds_domain) {
    overrides.dds_domain = *options.dds_domain;
  }
  configuration::RuntimeConfig config;
  try {
    config = configuration::LoadConfig(options.config_path, overrides);
  } catch (const std::exception &error) {
    StderrFormat("robot_runtime: configuration error: %s\n", error.what());
    return kExitStartupError;
  }
  // The fake-only options are refused before a device is opened, a log
  // directory is created or DDS starts: a real deployment has no scenario and
  // no fake output history to trace.
  const bool fake_selected = config.backend == "fake";
  if (!fake_selected &&
      (options.fake_scenario_path || options.output_trace_path)) {
    StderrLine(
        "robot_runtime: --fake-scenario and --output-trace are fake-only "
        "options\n");
    return kExitStartupError;
  }

  std::optional<FakeScenario> scenario;
  if (fake_selected && options.fake_scenario_path) {
    try {
      scenario = LoadFakeScenario(*options.fake_scenario_path, config);
    } catch (const std::exception &error) {
      StderrFormat("robot_runtime: scenario error: %s\n", error.what());
      return kExitStartupError;
    }
  }

  // The metadata of one recorded run: what a reader needs to interpret its rows
  // without any other file. The digests identify the configuration and the
  // joint layout in effect, and the units are the protocol's own.
  const auto record_metadata = [](const configuration::RuntimeConfig &config) {
    Json metadata;
    metadata["schema_version"] = telemetry::kRecordSchemaVersion;
    metadata["config_id"] = config.config_id;
    metadata["layout_id"] = config.layout_id;
    metadata["backend"] = config.backend;
    metadata["control_hz"] = config.control_hz;
    metadata["sample_hz"] = config.telemetry.recorder.sample_hz;
    metadata["joint_count"] = config.joint_count();
    metadata["joints"] = config.joint_names;
    metadata["units"] = {{"position", "rad"},    {"velocity", "rad_s"},
                         {"torque", "Nm"},       {"kp", "Nm_per_rad"},
                         {"kd", "Nm_s_per_rad"}, {"time", "us"}};
    metadata["row"] =
        "one control tick: the published state, the target in force, the "
        "feedback it used and the host send result; the joint columns follow "
        "`joints` in order";
    return metadata.dump(1);
  };

  // The output trace records the fake backend's own output history, so it is
  // part of the fake deployment only. A trace file that cannot be created is a
  // startup error, not a silently missing measurement.
  std::unique_ptr<OutputTrace> trace;
  if (options.output_trace_path) {
    trace = std::make_unique<OutputTrace>(*options.output_trace_path);
    if (!trace->Start()) {
      return kExitStartupError;
    }
  }

  const RuntimeIdentity identity = MakeRuntimeId();
  std::unique_ptr<hardware::Backend> backend;
  try {
    backend = hardware::MakeBackend(config);
    backend->Initialize();
  } catch (const std::exception &error) {
    StderrFormat("robot_runtime: backend start failed: %s\n", error.what());
    if (backend) {
      backend->Close();
    }
    trace.reset();
    return kExitStartupError;
  }
  hardware::FakeBackend *fake_backend =
      fake_selected ? dynamic_cast<hardware::FakeBackend *>(backend.get())
                    : nullptr;

  ExecutionFsm fsm(config, identity.id);
  telemetry::Logger logger(config.telemetry);
  if (!logger.Start()) {
    StderrLine("robot_runtime: the logger did not start\n");
    trace.reset();
    backend->Close();
    return kExitStartupError;
  }
  // The run recorder: the control thread offers one coherent snapshot per
  // recorded tick and never waits for it, so the disk cannot delay a tick. A
  // run directory that cannot be created is a startup error, like a missing log
  // directory, instead of a silent absence of evidence.
  // The recorder samples at its own rate from a fixed phase: every
  // `record_ratio`-th control tick, counted from the first one, so a run's rows
  // are evenly spaced and its rate is the configured one rather than the
  // control rate. The stride is what the recorder needs to tell a missing
  // sample from the next recorded tick.
  //
  // It is a whole number because LoadConfig refuses a sampling rate that does
  // not divide the control rate exactly: the ratio is the sample period over
  // the control period, both in microseconds, and not a rounded ratio of the
  // two frequencies, which is what would turn 200 Hz on a 500 Hz loop into one
  // sample per three ticks and a run that records 166.7 Hz.
  const std::uint32_t record_ratio = [&config]() {
    if (!config.telemetry.recorder.enabled ||
        config.telemetry.recorder.sample_hz == 0) {
      return 0U;
    }
    const auto sample_period_us = static_cast<std::uint64_t>(std::llround(
        1000000.0 / static_cast<double>(config.telemetry.recorder.sample_hz)));
    const std::uint64_t control_period_us =
        config.control_period_us == 0 ? 1 : config.control_period_us;
    const std::uint64_t ratio = sample_period_us / control_period_us;
    return static_cast<std::uint32_t>(ratio == 0 ? 1 : ratio);
  }();
  std::uint32_t record_tick = 0;
  // The authorization context the recorder's session channel last reported. The
  // runtime emits one session event when it changes - a new producer, a new
  // epoch, or the context ending - rather than one per tick, and the zero
  // identity is the "no context" a run starts in.
  protocol::Id session_producer{};
  protocol::Sequence session_epoch = 0;
  std::unique_ptr<telemetry::Recorder> recorder;
  if (config.telemetry.recorder.enabled) {
    telemetry::RecorderConfig recorder_config;
    recorder_config.enabled = true;
    recorder_config.sample_hz = config.telemetry.recorder.sample_hz;
    recorder_config.queue_capacity = config.telemetry.recorder.queue_capacity;
    recorder_config.stride = record_ratio;
    recorder_config.directory = config.telemetry.recorder.directory;
    recorder = std::make_unique<telemetry::Recorder>();
    std::string recorder_error;
    if (!recorder->Start(recorder_config, record_metadata(config),
                         &recorder_error)) {
      StderrFormat("robot_runtime: %s\n", recorder_error.c_str());
      logger.Stop();
      trace.reset();
      backend->Close();
      return kExitStartupError;
    }
  }
  if (!InstallStopHandlers()) {
    StderrLine("robot_runtime: cannot install the stop handlers\n");
    if (recorder) {
      static_cast<void>(recorder->Stop());
    }
    logger.Stop();
    trace.reset();
    backend->Close();
    return kExitStartupError;
  }

  transport::DdsTransport transport(config);
  try {
    transport.Start();
  } catch (const std::exception &error) {
    StderrFormat("robot_runtime: transport start failed: %s\n", error.what());
    logger.Stop();
    trace.reset();
    backend->Close();
    return kExitStartupError;
  }

  std::optional<ScenarioApplier> applier;
  if (fake_backend != nullptr) {
    applier.emplace(scenario.value_or(FakeScenario{}), config);
  }
  LogStartup(logger, config, identity.id, fsm.execution_state(),
             process_start_us);
  if (identity.entropy_fallback) {
    LogRecord record(configuration::LogLevel::Warning, process_start_us, 0,
                     "runtime",
                     "system entropy unusable; runtime id is only process "
                     "unique");
    char rendered[33];
    record.Field("runtime_id", HexId(identity.id, rendered));
    logger.TryLog(record.event());
  }

  const protocol::TimeUs period_us =
      config.control_period_us == 0 ? 1 : config.control_period_us;
  LoopCounters counters;
  protocol::TimeUs next_status_us = process_start_us + kStatusIntervalUs;
  auto deadline = Clock::now();
  ShutdownPlanner shutdown;

  while (true) {
    const protocol::TimeUs now = MonotonicUs();
    // Due scenario entries are applied before the read of this tick, so the
    // snapshot the state machine sees is the one the scenario describes at this
    // instant - including the entry due at process start.
    if (applier) {
      applier->ApplyDue(now - process_start_us, now, *fake_backend);
    }

    const bool stop_requested =
        g_stop_requested.load(std::memory_order_relaxed);
    const TickInput *input = &transport.TakeInput();
    if (stop_requested) {
      input = shutdown.PlanStop(*input, fsm);
    }
    const protocol::FeedbackSnapshot feedback = backend->Read(now);
    const TickResult result = fsm.Tick(now, feedback, *input, *backend);
    transport.Publish(result);
    ++counters.ticks;
    counters.fsm_dropped =
        result.dropped_result_count + result.dropped_event_count;
    LogTickEvents(logger, fsm, result);

    if (recorder && record_ratio != 0) {
      if (record_tick % record_ratio == 0) {
        // The feedback of this tick is usable only when it describes every
        // configured joint: a snapshot that is short or carries an invalid
        // sample is recorded as such, never smoothed over.
        bool feedback_valid =
            result.state.feedback.count == config.joint_count();
        for (std::uint32_t index = 0;
             index < result.state.feedback.count && feedback_valid; ++index) {
          feedback_valid = result.state.feedback.joints[index].valid;
        }
        static_cast<void>(
            recorder->TryRecord(MakeRecordSample(fsm, result, feedback_valid)));
      }
      ++record_tick;
    }

    if (recorder) {
      // One session event per authorization context the tick observes, not one
      // per tick: emit only when the producer or epoch changes - a context
      // adopted, re-authorized, or ended - derived from the state the tick
      // already published. A full session channel costs the event and is
      // counted; the control thread never waits for it, and the call neither
      // allocates nor throws.
      if (result.state.producer_id != session_producer ||
          result.state.epoch != session_epoch) {
        session_producer = result.state.producer_id;
        session_epoch = result.state.epoch;
        static_cast<void>(
            recorder->TryRecordSession(MakeSessionEvent(fsm, result)));
      }
    }

    if (trace && fake_backend != nullptr) {
      // The torque the backend really recorded, one record per write the tick
      // made: hand it to the writer thread, never to a file.
      hardware::OutputRecord written[kTraceDrainBatch];
      std::size_t drained = 0;
      while ((drained = fake_backend->DrainOutputHistory(
                  written, kTraceDrainBatch)) > 0) {
        for (std::size_t index = 0; index < drained; ++index) {
          trace->TryRecord(MakeTraceRecord(fsm, result, written[index]));
        }
      }
    }

    if (stop_requested) {
      shutdown.FlushNotice();
    }
    if (stop_requested && shutdown.MayExit(result, fsm)) {
      LogSummary(logger, "shutdown", fsm, counters, transport, logger.Dropped(),
                 now);
      transport.Stop();
      logger.Stop();
      // The trace writer owns its counters until it is joined: stop it first,
      // so every number the shutdown line reports is final. Stop() also writes
      // the end record, which is what lets the consumer tell a complete trace
      // from a truncated one.
      if (trace) {
        trace->Stop();
      }
      // The recorder is closed before the summary line, so everything it could
      // not keep is on disk by the time the process reports it stopped. Its own
      // index carries the counts a reader needs - rows, chunks, gaps, drops and
      // failed writes - which is where they belong, next to the samples they
      // describe.
      if (recorder) {
        static_cast<void>(recorder->Stop());
        recorder.reset();
      }
      ReportCleanStop(logger, counters, fsm, transport,
                      trace ? trace->counters() : OutputTrace::Counters{}, now);
      trace.reset();
      backend->Close();
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
    counters.worst_overrun_us = std::max(counters.worst_overrun_us,
                                         static_cast<protocol::TimeUs>(late));
    if (late >= static_cast<std::chrono::microseconds::rep>(period_us)) {
      deadline = work_end;
    }
  }
}

}  // namespace irmv::runtime
