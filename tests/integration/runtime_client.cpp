// Test client of the standalone runtime. One process plays exactly one role of
// the frozen protocol over real DDS and reports everything it observed as JSON
// lines on stdout, so a harness can assert on the traffic instead of on the
// client's own conclusions:
//
//   runtime_client --mode supervisor|producer|observer --scenario NAME
//                  --domain N [--config PATH] [--runtime-id HEX] [--rate HZ]
//                  [--producer-id HEX] [--epoch N] [--replay FILE]
//
// `--mode` selects the role; `--scenario` selects the behaviour of that role,
// which is what lets one binary cover the failure matrix: the supervisor
// acquires a session, stops, authorizes tracking and drives the recovery
// scenarios; the producer publishes complete joint commands while the runtime
// authorizes it; the observer only listens and stamps what arrives. Every
// request is confirmed through irmv::test::ConfirmationLog: an answer counts
// only when the result and a state published at or after it agree.
//
// `--replay FILE` is the packet file of the replay scenarios. A scenario whose
// name ends in `_record` appends every packet it sends to that file; a scenario
// whose name ends in `_replay` re-sends the saved packets unchanged and reports
// what the runtime answers.
//
// SIGSTOP and SIGCONT need no handling: the process simply stops running.
// SIGINT and SIGTERM only set a flag, so the client leaves its loop, flushes
// its observations and reports what it had reached.
//
// Exit codes: 0 all checks of the scenario passed, 1 a check failed, 2 the
// command line was rejected, 3 startup (configuration or DDS) failed.

#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dds/dds.hpp>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "execution.hpp"
#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"
#include "support/confirmation.hpp"

namespace {

namespace codec = irmv::transport::codec;
namespace wire = irmv::protocol::wire;
using json = nlohmann::json;

using irmv::configuration::RuntimeConfig;
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
using irmv::protocol::StopReason;
using irmv::protocol::SupervisorHeartbeat;
using irmv::protocol::TimeUs;
using irmv::test::ConfirmationExpectations;
using irmv::test::ConfirmationLog;
using irmv::test::ConfirmationOrder;
using irmv::test::ConfirmationStatus;
using irmv::test::ResultObservation;
using irmv::test::TrackedRequest;

constexpr int kExitOk = 0;
constexpr int kExitFailure = 1;
constexpr int kExitUsage = 2;
constexpr int kExitStartup = 3;

// Request ids of the supervisor start far above the ids of the flood client, so
// a flood that overwrites the session's request record can never make a
// supervisor request look stale.
constexpr Sequence kRequestBase = 1ULL << 40;
constexpr Sequence kFloodRequestBase = 1ULL << 30;
TimeUs NowUs() {
  return static_cast<TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Bands of request ids the flood clients draw from: 30000 bands of 2^25 ids fit
// strictly between the flood base and the supervisor base.
constexpr Sequence kFloodBandWidth = 1ULL << 25;
constexpr Sequence kFloodBandCount = 30000;

// Random band of this process. Two live floods collide with probability
// 1/30000, and a collision is caught by the client's own answered/sent
// cross-check rather than corrupting the drop count.
Sequence MakeFloodBand() {
  std::uint64_t seed = 0;
  try {
    std::random_device entropy;
    seed = (static_cast<std::uint64_t>(entropy()) << 32) ^ entropy();
  } catch (const std::exception &) {
    // No entropy: the process id and the clock still separate two processes.
    unsigned long long mixed = static_cast<unsigned long long>(::getpid());
    mixed *= 0x9e3779b97f4a7c15ULL;
    mixed ^= static_cast<unsigned long long>(NowUs());
    mixed *= 0xbf58476d1ce4e5b9ULL;
    mixed ^= mixed >> 29;
    seed = static_cast<std::uint64_t>(mixed);
  }
  return static_cast<Sequence>(seed % kFloodBandCount);
}

constexpr TimeUs kHeartbeatIntervalUs = 50'000;
constexpr TimeUs kRetryIntervalUs = 50'000;
constexpr TimeUs kPollSleepUs = 500;
// Longest a result may be held back so the state published by the same tick is
// observed first; the runtime publishes both from the same tick.
constexpr TimeUs kResultHoldUs = 200'000;
// Every client leaves its loop eventually, whatever the scenario does, so a
// broken runtime fails the scenario instead of hanging the harness.
constexpr TimeUs kGlobalCapUs = 30'000'000;
constexpr TimeUs kObserverCapUs = 25'000'000;

std::string IdHex(const Id &id) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string text(2 * id.size(), '0');
  for (std::size_t index = 0; index < id.size(); ++index) {
    text[2 * index] = kHex[(id[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHex[id[index] & 0x0f];
  }
  return text;
}

bool ParseHexId(std::string_view text, Id *id) {
  if (text.size() != 2 * id->size()) {
    return false;
  }
  for (std::size_t index = 0; index < id->size(); ++index) {
    unsigned value = 0;
    for (std::size_t nibble = 0; nibble < 2; ++nibble) {
      const unsigned char byte =
          static_cast<unsigned char>(text[2 * index + nibble]);
      if (byte >= '0' && byte <= '9') {
        value = value * 16 + (byte - '0');
      } else if (byte >= 'a' && byte <= 'f') {
        value = value * 16 + (byte - 'a' + 10);
      } else if (byte >= 'A' && byte <= 'F') {
        value = value * 16 + (byte - 'A' + 10);
      } else {
        return false;
      }
    }
    (*id)[index] = static_cast<std::uint8_t>(value);
  }
  return true;
}

Id MakeId(std::uint8_t seed) {
  Id id{};
  id.fill(seed);
  return id;
}

// Distinct per process: two clients of one scenario must never share a
// supervisor identity. A replayed identity is read from the packet file
// instead.
Id ProcessSupervisorId() {
  Id id{};
  const std::uint64_t pid = static_cast<std::uint64_t>(::getpid());
  id[0] = 0x5A;
  id[13] = static_cast<std::uint8_t>((pid >> 16) & 0xFFu);
  id[14] = static_cast<std::uint8_t>((pid >> 8) & 0xFFu);
  id[15] = static_cast<std::uint8_t>(pid & 0xFFu);
  return id;
}

const char *StateName(ExecutionState state) {
  switch (state) {
    case ExecutionState::Boot:
      return "Boot";
    case ExecutionState::ZeroTorque:
      return "ZeroTorque";
    case ExecutionState::Damping:
      return "Damping";
    case ExecutionState::Tracking:
      return "Tracking";
    case ExecutionState::Fault:
      return "Fault";
  }
  return "Unknown";
}

const char *ActionName(OutputAction action) {
  switch (action) {
    case OutputAction::ZeroTorque:
      return "ZeroTorque";
    case OutputAction::Damping:
      return "Damping";
    case OutputAction::Tracking:
      return "Tracking";
  }
  return "Unknown";
}

const char *StatusName(ResultStatus status) {
  return status == ResultStatus::Applied ? "Applied" : "Rejected";
}

const char *OperationName(Operation operation) {
  switch (operation) {
    case Operation::AcquireSession:
      return "AcquireSession";
    case Operation::Damping:
      return "Damping";
    case Operation::ZeroTorque:
      return "ZeroTorque";
    case Operation::Tracking:
      return "Tracking";
    case Operation::Reset:
      return "Reset";
  }
  return "Unknown";
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
  return "Unknown";
}

const char *StopReasonName(StopReason reason) {
  switch (reason) {
    case StopReason::None:
      return "None";
    case StopReason::Requested:
      return "Requested";
    case StopReason::CommandTimeout:
      return "CommandTimeout";
    case StopReason::SourceStateTimeout:
      return "SourceStateTimeout";
    case StopReason::SupervisorTimeout:
      return "SupervisorTimeout";
  }
  return "Unknown";
}

// Pins the participant to the interface the runtime binds, exactly like the
// transport does: both processes must meet on one domain and one network.
std::string InterfaceConfig(const std::string &interface_name) {
  return "<CycloneDDS><Domain "
         "id=\"any\"><General><Interfaces><NetworkInterface "
         "name=\"" +
         interface_name + "\"/></Interfaces></General></Domain></CycloneDDS>";
}

void Diagnostic(std::string_view text) {
  std::fprintf(stderr, "runtime_client: %.*s\n", static_cast<int>(text.size()),
               text.data());
}

// ---- command line ---------------------------------------------------------

struct Options {
  std::string mode;
  std::string scenario;
  std::uint32_t domain{0};
  std::filesystem::path config_path{"config/sdk.json"};
  std::optional<Id> runtime_id;
  double rate_hz{50.0};
  std::optional<Id> producer_id;
  std::uint64_t epoch{0};
  std::optional<std::filesystem::path> replay_path;
};

const char *UsageLine() {
  return "usage: runtime_client --mode supervisor|producer|observer --scenario "
         "NAME --domain N [--config PATH] [--runtime-id HEX] [--rate HZ] "
         "[--producer-id HEX] [--epoch N] [--replay FILE]";
}

bool ParseOptions(int argc, char **argv, Options *options, std::string *error) {
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    const bool has_value = index + 1 < argc;
    const auto value = [&]() -> std::string {
      return has_value ? std::string(argv[++index]) : std::string();
    };
    if (argument == "--mode") {
      if (!has_value) {
        *error = "--mode requires a value";
        return false;
      }
      options->mode = value();
      continue;
    }
    if (argument == "--scenario") {
      if (!has_value) {
        *error = "--scenario requires a value";
        return false;
      }
      options->scenario = value();
      continue;
    }
    if (argument == "--domain") {
      if (!has_value) {
        *error = "--domain requires a value";
        return false;
      }
      const std::string text = value();
      std::size_t consumed = 0;
      std::uint64_t domain = 0;
      try {
        domain = std::stoull(text, &consumed);
      } catch (const std::exception &) {
        *error = "--domain must be an integer, got \"" + text + "\"";
        return false;
      }
      if (consumed != text.size() || domain > 230) {
        *error = "--domain must be between 0 and 230, got \"" + text + "\"";
        return false;
      }
      options->domain = static_cast<std::uint32_t>(domain);
      continue;
    }
    if (argument == "--config") {
      if (!has_value) {
        *error = "--config requires a path";
        return false;
      }
      options->config_path = value();
      continue;
    }
    if (argument == "--runtime-id" || argument == "--producer-id") {
      if (!has_value) {
        *error = std::string(argument) + " requires a hex identity";
        return false;
      }
      Id id{};
      if (!ParseHexId(value(), &id) || id == irmv::protocol::kNoId) {
        *error = std::string(argument) + " must be 32 hex digits and non-zero";
        return false;
      }
      if (argument == "--runtime-id") {
        options->runtime_id = id;
      } else {
        options->producer_id = id;
      }
      continue;
    }
    if (argument == "--rate") {
      if (!has_value) {
        *error = "--rate requires a value";
        return false;
      }
      const std::string text = value();
      try {
        options->rate_hz = std::stod(text);
      } catch (const std::exception &) {
        *error = "--rate must be a number, got \"" + text + "\"";
        return false;
      }
      if (!(options->rate_hz > 0.0) || !std::isfinite(options->rate_hz)) {
        *error = "--rate must be finite and positive";
        return false;
      }
      continue;
    }
    if (argument == "--epoch") {
      if (!has_value) {
        *error = "--epoch requires a value";
        return false;
      }
      const std::string text = value();
      std::size_t consumed = 0;
      try {
        options->epoch = std::stoull(text, &consumed);
      } catch (const std::exception &) {
        *error = "--epoch must be an integer, got \"" + text + "\"";
        return false;
      }
      if (consumed != text.size()) {
        *error = "--epoch must be an integer, got \"" + text + "\"";
        return false;
      }
      continue;
    }
    if (argument == "--replay") {
      if (!has_value) {
        *error = "--replay requires a path";
        return false;
      }
      options->replay_path = value();
      continue;
    }
    *error = "unknown argument \"" + std::string(argument) + "\"";
    return false;
  }
  if (options->mode.empty() || options->scenario.empty()) {
    *error = "--mode and --scenario are required";
    return false;
  }
  if (options->mode != "supervisor" && options->mode != "producer" &&
      options->mode != "observer") {
    *error = "--mode must be supervisor, producer or observer";
    return false;
  }
  return true;
}

bool EndsWith(std::string_view text, std::string_view suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// ---- traffic --------------------------------------------------------------

// The client side of the five frozen topics: one writer per topic the client
// sends on and one reader per topic it listens to, all on one participant
// pinned to the interface of the runtime configuration.
class Traffic {
 public:
  Traffic(std::uint32_t domain, const std::string &interface_name)
      : participant_(domain, dds::domain::qos::DomainParticipantQos(), nullptr,
                     dds::core::status::StatusMask::none(),
                     interface_name.empty() ? std::string()
                                            : InterfaceConfig(interface_name)),
        publisher_(participant_),
        subscriber_(participant_),
        request_topic_(
            participant_,
            std::string(irmv::protocol::kExecutionRequestTopic.name)),
        command_topic_(participant_,
                       std::string(irmv::protocol::kJointCommandTopic.name)),
        heartbeat_topic_(
            participant_,
            std::string(irmv::protocol::kSupervisorHeartbeatTopic.name)),
        result_topic_(participant_,
                      std::string(irmv::protocol::kExecutionResultTopic.name)),
        state_topic_(participant_,
                     std::string(irmv::protocol::kRobotStateTopic.name)),
        request_writer_(
            publisher_, request_topic_,
            codec::ToWriterQos(irmv::protocol::kExecutionRequestTopic.qos)),
        command_writer_(
            publisher_, command_topic_,
            codec::ToWriterQos(irmv::protocol::kJointCommandTopic.qos)),
        heartbeat_writer_(
            publisher_, heartbeat_topic_,
            codec::ToWriterQos(irmv::protocol::kSupervisorHeartbeatTopic.qos)),
        result_reader_(
            subscriber_, result_topic_,
            codec::ToReaderQos(irmv::protocol::kExecutionResultTopic.qos)),
        state_reader_(
            subscriber_, state_topic_,
            codec::ToReaderQos(irmv::protocol::kRobotStateTopic.qos)) {}

  bool SendRequest(const ExecutionRequest &request) {
    wire::ExecutionRequest converted;
    codec::ToWire(request, converted);
    return Write(request_writer_, converted);
  }

  bool SendCommand(const JointCommand &command) {
    wire::JointCommand converted;
    codec::ToWire(command, converted);
    return Write(command_writer_, converted);
  }

  bool SendHeartbeat(const SupervisorHeartbeat &heartbeat) {
    wire::SupervisorHeartbeat converted;
    codec::ToWire(heartbeat, converted);
    return Write(heartbeat_writer_, converted);
  }

  std::size_t TakeResults(std::vector<ExecutionResult> *out) {
    out->clear();
    return Take(result_reader_, out);
  }

  std::size_t TakeStates(std::vector<RobotState> *out) {
    out->clear();
    return Take(state_reader_, out);
  }

  std::uint64_t write_failures() const { return write_failures_; }

 private:
  template <typename Writer, typename Sample>
  bool Write(Writer &writer, const Sample &sample) {
    try {
      writer.write(sample);
      return true;
    } catch (const std::exception &) {
      // A reliable writer refuses a sample when its history stays full past the
      // blocking time; the client counts it and keeps going.
      ++write_failures_;
      return false;
    }
  }

  template <typename Reader, typename ProtocolT>
  std::size_t Take(Reader &reader, std::vector<ProtocolT> *out) {
    std::size_t taken = 0;
    try {
      auto samples =
          reader.select().state(dds::sub::status::DataState::any()).take();
      for (const auto &sample : samples) {
        if (!sample.info().valid()) {
          continue;
        }
        ProtocolT converted;
        if (codec::FromWire(sample.data(), converted)) {
          out->push_back(converted);
          ++taken;
        }
      }
    } catch (const std::exception &) {
      ++read_failures_;
    }
    return taken;
  }

  dds::domain::DomainParticipant participant_;
  dds::pub::Publisher publisher_;
  dds::sub::Subscriber subscriber_;
  dds::topic::Topic<wire::ExecutionRequest> request_topic_;
  dds::topic::Topic<wire::JointCommand> command_topic_;
  dds::topic::Topic<wire::SupervisorHeartbeat> heartbeat_topic_;
  dds::topic::Topic<wire::ExecutionResult> result_topic_;
  dds::topic::Topic<wire::RobotState> state_topic_;
  dds::pub::DataWriter<wire::ExecutionRequest> request_writer_;
  dds::pub::DataWriter<wire::JointCommand> command_writer_;
  dds::pub::DataWriter<wire::SupervisorHeartbeat> heartbeat_writer_;
  dds::sub::DataReader<wire::ExecutionResult> result_reader_;
  dds::sub::DataReader<wire::RobotState> state_reader_;
  std::uint64_t write_failures_{0};
  std::uint64_t read_failures_{0};
};

// ---- packet files ---------------------------------------------------------

json CommandJson(const JointCommand &command) {
  const std::uint32_t count =
      std::min<std::uint32_t>(command.joints.count, irmv::protocol::kMaxJoints);
  json joints = json::array();
  for (std::uint32_t index = 0; index < count; ++index) {
    joints.push_back(json{{"q", command.joints.q[index]},
                          {"dq", command.joints.dq[index]},
                          {"tau_ff", command.joints.tau_ff[index]},
                          {"kp", command.joints.kp[index]},
                          {"kd", command.joints.kd[index]}});
  }
  return json{{"protocol_version", command.protocol_version},
              {"runtime_id", IdHex(command.runtime_id)},
              {"session_id", IdHex(command.session_id)},
              {"producer_id", IdHex(command.producer_id)},
              {"epoch", command.epoch},
              {"command_seq", command.command_seq},
              {"source_state_seq", command.source_state_seq},
              {"stamp_us", command.stamp_us},
              {"layout_id", std::string(command.layout_id.View())},
              {"joints", joints}};
}

JointCommand CommandFromJson(const json &record) {
  JointCommand command;
  command.protocol_version = record.at("protocol_version").get<std::uint32_t>();
  static_cast<void>(ParseHexId(record.at("runtime_id").get<std::string>(),
                               &command.runtime_id));
  static_cast<void>(ParseHexId(record.at("session_id").get<std::string>(),
                               &command.session_id));
  static_cast<void>(ParseHexId(record.at("producer_id").get<std::string>(),
                               &command.producer_id));
  command.epoch = record.at("epoch").get<Sequence>();
  command.command_seq = record.at("command_seq").get<Sequence>();
  command.source_state_seq = record.at("source_state_seq").get<Sequence>();
  command.stamp_us = record.at("stamp_us").get<TimeUs>();
  static_cast<void>(
      command.layout_id.Assign(record.at("layout_id").get<std::string>()));
  const json &joints = record.at("joints");
  command.joints.count = static_cast<std::uint32_t>(joints.size());
  for (std::size_t index = 0; index < joints.size(); ++index) {
    command.joints.q[index] = joints[index].at("q").get<double>();
    command.joints.dq[index] = joints[index].at("dq").get<double>();
    command.joints.tau_ff[index] = joints[index].at("tau_ff").get<double>();
    command.joints.kp[index] = joints[index].at("kp").get<double>();
    command.joints.kd[index] = joints[index].at("kd").get<double>();
  }
  return command;
}

json RequestJson(const ExecutionRequest &request) {
  return json{{"protocol_version", request.protocol_version},
              {"supervisor_id", IdHex(request.supervisor_id)},
              {"request_id", request.request_id},
              {"session_id", IdHex(request.session_id)},
              {"runtime_id", IdHex(request.runtime_id)},
              {"expected_epoch", request.expected_epoch},
              {"proposed_epoch", request.proposed_epoch},
              {"producer_id", IdHex(request.producer_id)},
              {"fault_id", request.fault_id},
              {"operation", OperationName(request.operation)},
              {"first_command", CommandJson(request.first_command)}};
}

ExecutionRequest RequestFromJson(const json &record) {
  ExecutionRequest request;
  request.protocol_version = record.at("protocol_version").get<std::uint32_t>();
  static_cast<void>(ParseHexId(record.at("supervisor_id").get<std::string>(),
                               &request.supervisor_id));
  request.request_id = record.at("request_id").get<Sequence>();
  static_cast<void>(ParseHexId(record.at("session_id").get<std::string>(),
                               &request.session_id));
  static_cast<void>(ParseHexId(record.at("runtime_id").get<std::string>(),
                               &request.runtime_id));
  request.expected_epoch = record.at("expected_epoch").get<Sequence>();
  request.proposed_epoch = record.at("proposed_epoch").get<Sequence>();
  static_cast<void>(ParseHexId(record.at("producer_id").get<std::string>(),
                               &request.producer_id));
  request.fault_id = record.at("fault_id").get<Sequence>();
  const std::string operation = record.at("operation").get<std::string>();
  for (const Operation candidate :
       {Operation::AcquireSession, Operation::Damping, Operation::ZeroTorque,
        Operation::Tracking, Operation::Reset}) {
    if (operation == OperationName(candidate)) {
      request.operation = candidate;
      break;
    }
  }
  request.first_command = CommandFromJson(record.at("first_command"));
  return request;
}

json HeartbeatJson(const SupervisorHeartbeat &heartbeat) {
  return json{{"protocol_version", heartbeat.protocol_version},
              {"supervisor_id", IdHex(heartbeat.supervisor_id)},
              {"session_id", IdHex(heartbeat.session_id)},
              {"runtime_id", IdHex(heartbeat.runtime_id)},
              {"heartbeat_seq", heartbeat.heartbeat_seq},
              {"stamp_us", heartbeat.stamp_us}};
}

SupervisorHeartbeat HeartbeatFromJson(const json &record) {
  SupervisorHeartbeat heartbeat;
  heartbeat.protocol_version =
      record.at("protocol_version").get<std::uint32_t>();
  static_cast<void>(ParseHexId(record.at("supervisor_id").get<std::string>(),
                               &heartbeat.supervisor_id));
  static_cast<void>(ParseHexId(record.at("session_id").get<std::string>(),
                               &heartbeat.session_id));
  static_cast<void>(ParseHexId(record.at("runtime_id").get<std::string>(),
                               &heartbeat.runtime_id));
  heartbeat.heartbeat_seq = record.at("heartbeat_seq").get<Sequence>();
  heartbeat.stamp_us = record.at("stamp_us").get<TimeUs>();
  return heartbeat;
}

// ---- signals --------------------------------------------------------------

volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void HandleStopSignal(int signal_number) {
  static_cast<void>(signal_number);
  g_stop_requested = 1;
}

bool StopRequested() { return g_stop_requested != 0; }

// ---- client ---------------------------------------------------------------

// Compact view of one received RobotState. The full struct carries 64 feedback
// entries and 64 fault details, far more than any assertion needs, and the
// runtime publishes it at control frequency.
struct StateView {
  TimeUs t_us{0};
  TimeUs stamp_us{0};
  Sequence state_seq{0};
  ExecutionState execution_state{ExecutionState::Boot};
  OutputAction output_action{OutputAction::ZeroTorque};
  StopReason stop_reason{StopReason::None};
  Sequence epoch{0};
  Sequence fault_id{0};
  Sequence last_applied_command_seq{0};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  bool healthy{false};
  bool damping_eligible{false};
};

class Client {
 public:
  Client(const Options &options, const RuntimeConfig &config)
      : options_(options),
        config_(config),
        supervisor_id_(ProcessSupervisorId()),
        producer_id_(options.producer_id.value_or(MakeId(0x33))),
        runtime_id_(options.runtime_id.value_or(irmv::protocol::kNoId)),
        traffic_(options.domain, config.dds.interface) {}

  int Run();

 private:
  // ---- plumbing ----------------------------------------------------------

  TimeUs now() const { return NowUs(); }

  std::uint32_t joint_count() const {
    return static_cast<std::uint32_t>(config_.joint_count());
  }

  void Emit(const json &record);

  void Note(std::string_view text);

  // Reads both published topics, reports every observation and feeds the
  // confirmation log in the requested arrival order. `result_first` decides
  // which observation of one tick is presented first; the other order is
  // produced by holding this poll's results back until the next poll.
  void Poll(bool result_first);

  void PumpHeartbeat();

  bool WaitFor(const std::function<bool()> &ready, TimeUs timeout_us,
               std::string_view what);

  void Check(bool condition, std::string_view name,
             std::string_view detail = {});

  int Finish(std::string_view role);

  // ---- requests ----------------------------------------------------------

  Sequence NextRequestId() { return ++request_id_; }

  ExecutionRequest MakeRequest(Operation operation, Sequence request_id);

  bool SendAndConfirm(ExecutionRequest request,
                      const ConfirmationExpectations &expectations,
                      bool result_first, int drop_results, TimeUs timeout_us,
                      TrackedRequest *observed);

  // ---- roles -------------------------------------------------------------

  int RunSupervisor();
  int RunProducer();
  int RunObserver();
  int RunReplay();
  int RunFlood();

  // ---- supervisor helpers ------------------------------------------------

  bool WaitForZeroTorque();
  bool EnsureSession();
  // `force` sends the stop request even when the state already reads Damping:
  // the flood scenarios measure the latency of a request that really travelled.
  bool EnsureDamping(bool result_first, bool force = false);
  bool EnsureTracking(bool result_first, int drop_results);
  bool RequestZeroTorque(bool result_first);
  // Waits up to `duration_us` for the runtime to leave tracking. A fallback is
  // only recognised on a state published at or after `since_us`, so the state
  // that was already known before the authorization cannot be mistaken for one.
  bool HoldTracking(TimeUs duration_us, bool stop_on_fallback, TimeUs since_us,
                    bool *fell_back);
  bool Recover();

  // ---- producer helpers --------------------------------------------------

  struct CommandPlan {
    bool hold_measured_q{false};
    double q_target{0.0};
    double dq_target{0.0};
    double kp{0.0};
    double kd{0.0};
    double tau_ff{0.0};
    bool publish{true};
  };

  CommandPlan PlanForWindow(TimeUs elapsed_us) const;
  JointCommand MakeCommand(const RobotState &state, const CommandPlan &plan);
  JointCommand MakeFirstCommand(const RobotState &state, Sequence command_seq,
                                Sequence proposed_epoch);
  void SendProducerCommand(const RobotState &state);

  const RobotState *latest_state() const {
    return latest_state_.has_value() ? &*latest_state_ : nullptr;
  }

  // ---- packet file -------------------------------------------------------

  void SavePacket(const json &record);
  bool LoadPackets(std::vector<json> *records);

  // ---- state -------------------------------------------------------------

  Options options_;
  RuntimeConfig config_;
  Id supervisor_id_;
  Id producer_id_;
  Id runtime_id_;
  Traffic traffic_;

  ConfirmationLog confirm_;
  std::vector<ExecutionResult> read_results_;
  std::vector<RobotState> read_states_;
  std::optional<RobotState> latest_state_;
  TimeUs latest_state_rx_us_{0};
  // Results held back so the state published by the same tick is seen first.
  std::vector<std::pair<ExecutionResult, TimeUs>> held_results_;
  TimeUs first_held_us_{0};
  // Answers kept for the scenario's own measurements. The full result is small;
  // states are reduced to the fields the assertions use.
  std::vector<std::pair<ExecutionResult, TimeUs>> all_results_;
  std::vector<StateView> all_states_;

  bool poll_result_first_{true};
  bool emit_traffic_{true};

  Id session_id_{};
  Sequence last_known_epoch_{0};
  Sequence request_id_{kRequestBase};
  Sequence heartbeat_seq_{0};
  TimeUs next_heartbeat_us_{0};
  TimeUs next_command_us_{0};
  Sequence command_seq_{0};
  Sequence window_epoch_{0};
  TimeUs window_start_us_{0};
  std::uint64_t window_index_{0};
  bool tracking_seen_{false};
  TimeUs zero_torque_since_us_{0};

  // Simulated answer loss: the next `suppress_remaining_` results of
  // `suppress_request_id_` are read and reported, but never handed to the
  // confirmation log.
  Sequence suppress_request_id_{0};
  int suppress_remaining_{0};

  json checks_ = json::object();
  json measurements_ = json::object();
  std::vector<std::string> errors_;

  std::uint64_t requests_sent_{0};
  std::uint64_t commands_sent_{0};
  std::uint64_t heartbeat_sent_{0};
  std::uint64_t flood_sent_{0};
  TimeUs flood_emit_us_{0};
  std::vector<Sequence> command_source_seqs_;
  std::uint64_t replay_applied_{0};
  std::uint64_t replay_bootstrap_applied_{0};
  std::uint64_t replay_requests_{0};
};

// ---- output ---------------------------------------------------------------

void Client::Emit(const json &record) {
  std::cout << record.dump() << '\n';
  std::cout.flush();
}

void Client::Note(std::string_view text) {
  Emit(json{{"kind", "note"}, {"t_us", now()}, {"text", std::string(text)}});
}

json StateRecord(const StateView &view) {
  return json{{"kind", "state"},
              {"t_us", view.t_us},
              {"stamp_us", view.stamp_us},
              {"state_seq", view.state_seq},
              {"runtime_id", IdHex(view.runtime_id)},
              {"session_id", IdHex(view.session_id)},
              {"producer_id", IdHex(view.producer_id)},
              {"epoch", view.epoch},
              {"fault_id", view.fault_id},
              {"last_applied_command_seq", view.last_applied_command_seq},
              {"execution_state", StateName(view.execution_state)},
              {"output_action", ActionName(view.output_action)},
              {"stop_reason", StopReasonName(view.stop_reason)},
              {"healthy", view.healthy},
              {"damping_eligible", view.damping_eligible}};
}

json ResultRecord(const ExecutionResult &result, TimeUs rx_us,
                  std::string_view kind) {
  return json{{"kind", std::string(kind)},
              {"t_us", rx_us},
              {"stamp_us", result.stamp_us},
              {"runtime_id", IdHex(result.runtime_id)},
              {"session_id", IdHex(result.session_id)},
              {"supervisor_id", IdHex(result.supervisor_id)},
              {"producer_id", IdHex(result.producer_id)},
              {"request_id", result.request_id},
              {"epoch", result.epoch},
              {"fault_id", result.fault_id},
              {"execution_state", StateName(result.execution_state)},
              {"output_action", ActionName(result.output_action)},
              {"status", StatusName(result.status)},
              {"reason", ReasonName(result.reason)}};
}

// ---- polling --------------------------------------------------------------

void Client::Poll(bool result_first) {
  const TimeUs now_us = now();
  poll_result_first_ = result_first;
  traffic_.TakeResults(&read_results_);
  traffic_.TakeStates(&read_states_);

  const auto report_result = [this](const ExecutionResult &result, TimeUs rx_us,
                                    bool suppressed) {
    all_results_.push_back({result, rx_us});
    if (emit_traffic_) {
      Emit(ResultRecord(result, rx_us,
                        suppressed ? "result_suppressed" : "result"));
    }
    if (!suppressed) {
      confirm_.ObserveResult(result, rx_us);
    }
  };

  const auto report_state = [this](const RobotState &state, TimeUs rx_us) {
    StateView view;
    view.t_us = rx_us;
    view.stamp_us = state.stamp_us;
    view.state_seq = state.state_seq;
    view.execution_state = state.execution_state;
    view.output_action = state.output_action;
    view.stop_reason = state.stop_reason;
    view.epoch = state.epoch;
    view.fault_id = state.fault_id;
    view.last_applied_command_seq = state.last_applied_command_seq;
    view.runtime_id = state.runtime_id;
    view.session_id = state.session_id;
    view.producer_id = state.producer_id;
    view.healthy = state.healthy;
    view.damping_eligible = state.damping_eligible;
    all_states_.push_back(view);
    if (emit_traffic_) {
      Emit(StateRecord(view));
    }
    latest_state_ = state;
    latest_state_rx_us_ = rx_us;
    if (state.execution_state == ExecutionState::Tracking) {
      tracking_seen_ = true;
      zero_torque_since_us_ = 0;
    } else if (state.execution_state == ExecutionState::ZeroTorque &&
               zero_torque_since_us_ == 0) {
      zero_torque_since_us_ = now();
    }
    if (state.session_id != irmv::protocol::kNoId) {
      session_id_ = state.session_id;
    }
    if (state.runtime_id != irmv::protocol::kNoId) {
      runtime_id_ = state.runtime_id;
    }
    last_known_epoch_ = state.epoch;
    // Only a state at or after the first send of a pending request can witness
    // its confirmation; anything older is reported but never stored, which is
    // also what keeps the log bounded over a long run.
    const TrackedRequest *earliest = nullptr;
    for (const TrackedRequest &entry : confirm_.tracked()) {
      if (entry.status != ConfirmationStatus::Pending) {
        continue;
      }
      if (earliest == nullptr ||
          entry.first_send_us < earliest->first_send_us) {
        earliest = &entry;
      }
    }
    if (earliest != nullptr && state.stamp_us >= earliest->first_send_us) {
      confirm_.ObserveState(state, rx_us);
    }
  };

  const auto was_suppressed = [this](const ExecutionResult &result) {
    if (suppress_remaining_ <= 0 || result.request_id != suppress_request_id_) {
      return false;
    }
    --suppress_remaining_;
    Note("result for request " + std::to_string(result.request_id) +
         " suppressed by the client (simulated answer loss)");
    return true;
  };

  if (result_first) {
    for (const ExecutionResult &result : read_results_) {
      report_result(result, now_us, was_suppressed(result));
    }
    for (const RobotState &state : read_states_) {
      report_state(state, now_us);
    }
    for (const auto &held : held_results_) {
      report_result(held.first, held.second, false);
    }
    held_results_.clear();
    return;
  }

  // State first: the states of this poll are reported now and this poll's
  // results are held until the next poll, so the state published by the tick
  // that answered a request is always observed before the answer.
  for (const RobotState &state : read_states_) {
    report_state(state, now_us);
  }
  for (const auto &held : held_results_) {
    report_result(held.first, held.second, false);
  }
  held_results_.clear();
  for (const ExecutionResult &result : read_results_) {
    if (was_suppressed(result)) {
      report_result(result, now_us, true);
      continue;
    }
    if (held_results_.empty()) {
      first_held_us_ = now_us;
    }
    held_results_.push_back({result, now_us});
  }
  if (!held_results_.empty() && now_us - first_held_us_ > kResultHoldUs) {
    for (const auto &held : held_results_) {
      report_result(held.first, held.second, false);
    }
    held_results_.clear();
  }
}

void Client::PumpHeartbeat() {
  if (session_id_ == irmv::protocol::kNoId) {
    return;
  }
  const TimeUs now_us = now();
  if (now_us < next_heartbeat_us_) {
    return;
  }
  next_heartbeat_us_ = now_us + kHeartbeatIntervalUs;
  SupervisorHeartbeat heartbeat;
  heartbeat.protocol_version = irmv::protocol::kProtocolVersion;
  heartbeat.supervisor_id = supervisor_id_;
  heartbeat.session_id = session_id_;
  heartbeat.runtime_id = runtime_id_;
  heartbeat.heartbeat_seq = ++heartbeat_seq_;
  heartbeat.stamp_us = now_us;
  traffic_.SendHeartbeat(heartbeat);
  ++heartbeat_sent_;
  SavePacket(json{{"kind", "heartbeat"}, {"packet", HeartbeatJson(heartbeat)}});
}

bool Client::WaitFor(const std::function<bool()> &ready, TimeUs timeout_us,
                     std::string_view what) {
  const TimeUs deadline = now() + timeout_us;
  while (now() < deadline) {
    if (StopRequested()) {
      return false;
    }
    Poll(poll_result_first_);
    PumpHeartbeat();
    if (ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
  }
  Diagnostic(std::string("timed out waiting for ") + std::string(what));
  return false;
}

void Client::Check(bool condition, std::string_view name,
                   std::string_view detail) {
  checks_[std::string(name)] = condition;
  if (!condition) {
    errors_.push_back(std::string(name) + ": " + std::string(detail));
    Diagnostic("check failed: " + std::string(name) + ": " +
               std::string(detail));
  }
}

int Client::Finish(std::string_view role) {
  const bool ok = errors_.empty();
  Emit(json{{"kind", "client_summary"},
            {"role", std::string(role)},
            {"scenario", options_.scenario},
            {"ok", ok},
            {"pid", static_cast<std::uint64_t>(::getpid())},
            {"requests_sent", requests_sent_},
            {"commands_sent", commands_sent_},
            {"heartbeats_sent", heartbeat_sent_},
            {"checks", checks_},
            {"measurements", measurements_},
            {"errors", errors_}});
  return ok ? kExitOk : kExitFailure;
}

void Client::SavePacket(const json &record) {
  if (!options_.replay_path || !EndsWith(options_.scenario, "_record")) {
    return;
  }
  std::ofstream file(*options_.replay_path, std::ios::out | std::ios::app);
  if (!file) {
    Diagnostic("cannot append to the packet file");
    return;
  }
  file << record.dump() << '\n';
}

bool Client::LoadPackets(std::vector<json> *records) {
  if (!options_.replay_path) {
    Diagnostic("--replay is required for a replay scenario");
    return false;
  }
  std::ifstream file(*options_.replay_path);
  if (!file) {
    Diagnostic("cannot read the packet file");
    return false;
  }
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    try {
      records->push_back(json::parse(line));
    } catch (const json::exception &) {
      Diagnostic("ignoring an unreadable packet line");
    }
  }
  return true;
}

// ---- requests -------------------------------------------------------------

ExecutionRequest Client::MakeRequest(Operation operation, Sequence request_id) {
  ExecutionRequest request;
  request.protocol_version = irmv::protocol::kProtocolVersion;
  request.supervisor_id = supervisor_id_;
  request.request_id = request_id;
  request.session_id = session_id_;
  request.runtime_id = runtime_id_;
  request.expected_epoch = last_known_epoch_;
  request.proposed_epoch = 0;
  request.producer_id = irmv::protocol::kNoId;
  request.fault_id = 0;
  request.operation = operation;
  // The wire codec validates the embedded command of every request, including
  // the operations that carry no target, so a request always carries a
  // well-formed frame; the runtime reads it only for Tracking.
  request.first_command.protocol_version = irmv::protocol::kProtocolVersion;
  request.first_command.runtime_id = request.runtime_id;
  request.first_command.session_id = request.session_id;
  request.first_command.producer_id = producer_id_;
  request.first_command.stamp_us = now();
  static_cast<void>(request.first_command.layout_id.Assign(config_.layout_id));
  request.first_command.joints.count = joint_count();
  return request;
}

bool Client::SendAndConfirm(ExecutionRequest request,
                            const ConfirmationExpectations &expectations,
                            bool result_first, int drop_results,
                            TimeUs timeout_us, TrackedRequest *observed) {
  const TimeUs send_us = now();
  suppress_request_id_ = request.request_id;
  suppress_remaining_ = drop_results;
  confirm_.Track(request, send_us, expectations);
  const bool sent = traffic_.SendRequest(request);
  ++requests_sent_;
  Emit(json{{"kind", "request"},
            {"t_us", send_us},
            {"operation", OperationName(request.operation)},
            {"request_id", request.request_id},
            {"session_id", IdHex(request.session_id)},
            {"expected_epoch", request.expected_epoch},
            {"proposed_epoch", request.proposed_epoch},
            {"producer_id", IdHex(request.producer_id)},
            {"send_index", 1},
            {"sent", sent}});
  SavePacket(json{{"kind", "request"}, {"packet", RequestJson(request)}});

  const TimeUs deadline = send_us + timeout_us;
  TimeUs last_send_us = send_us;
  int send_index = 1;
  while (now() < deadline && !StopRequested()) {
    Poll(result_first);
    PumpHeartbeat();
    if (confirm_.confirmed(request.request_id)) {
      const TrackedRequest *entry = confirm_.Find(request.request_id);
      if (observed != nullptr && entry != nullptr) {
        *observed = *entry;
      }
      Emit(json{
          {"kind", "confirmed"},
          {"t_us", now()},
          {"request_id", request.request_id},
          {"operation", OperationName(request.operation)},
          {"order",
           entry != nullptr && entry->order == ConfirmationOrder::StateFirst
               ? "state_first"
               : "result_first"},
          {"send_count", entry == nullptr ? 0 : entry->send_count},
          {"session_id", IdHex(entry == nullptr ? irmv::protocol::kNoId
                                                : entry->confirmed_session)},
          {"suppressed_results", drop_results}});
      if (latest_state() != nullptr) {
        last_known_epoch_ = latest_state()->epoch;
      }
      return true;
    }
    // A retry repeats the identical payload: the runtime answers the recorded
    // result of the same request id instead of applying the request twice.
    if (now() - last_send_us >= kRetryIntervalUs) {
      last_send_us = now();
      confirm_.Track(request, last_send_us, expectations);
      traffic_.SendRequest(request);
      ++requests_sent_;
      ++send_index;
      Emit(json{{"kind", "request"},
                {"t_us", last_send_us},
                {"operation", OperationName(request.operation)},
                {"request_id", request.request_id},
                {"session_id", IdHex(request.session_id)},
                {"expected_epoch", request.expected_epoch},
                {"proposed_epoch", request.proposed_epoch},
                {"producer_id", IdHex(request.producer_id)},
                {"send_index", send_index},
                {"sent", true}});
      SavePacket(json{{"kind", "request"}, {"packet", RequestJson(request)}});
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
  }

  const ResultObservation *answer = confirm_.result_of(request.request_id);
  const std::string detail =
      answer == nullptr ? "no answer"
                        : std::string(StatusName(answer->result.status)) + "/" +
                              ReasonName(answer->result.reason);
  Diagnostic("request " + std::to_string(request.request_id) + " (" +
             OperationName(request.operation) + ") not confirmed: " + detail);
  Emit(json{{"kind", "unconfirmed"},
            {"t_us", now()},
            {"request_id", request.request_id},
            {"operation", OperationName(request.operation)},
            {"answer", detail},
            {"send_count", send_index}});
  return false;
}

// ---- supervisor -----------------------------------------------------------

bool Client::WaitForZeroTorque() {
  const bool ready = WaitFor(
      [this]() {
        const RobotState *state = latest_state();
        return state != nullptr &&
               state->execution_state == ExecutionState::ZeroTorque;
      },
      12'000'000, "the runtime to publish ZeroTorque");
  if (!ready) {
    Check(false, "runtime_reaches_zero_torque", "no ZeroTorque state observed");
    return false;
  }
  runtime_id_ = latest_state()->runtime_id;
  Note("runtime instance " + IdHex(runtime_id_) + " reached ZeroTorque");
  return true;
}

bool Client::EnsureSession() {
  if (session_id_ != irmv::protocol::kNoId) {
    return true;
  }
  const Sequence request_id = NextRequestId();
  ExecutionRequest request = MakeRequest(Operation::AcquireSession, request_id);
  request.session_id = irmv::protocol::kNoId;
  // The bootstrap is legal from Boot, ZeroTorque and Damping alike, so the
  // answer is proven by the state that agrees with it, never by a state this
  // request cannot know when it is sent.
  const ConfirmationExpectations expectations;
  TrackedRequest observed;
  const bool confirmed =
      SendAndConfirm(request, expectations, false, 0, 3'000'000, &observed);
  Check(confirmed, "session_acquired",
        "the bootstrap request was not confirmed");
  if (!confirmed) {
    return false;
  }
  session_id_ = observed.confirmed_session;
  next_heartbeat_us_ = now();
  heartbeat_seq_ = 0;
  Note("session " + IdHex(session_id_) + " acquired");
  return true;
}

bool Client::EnsureDamping(bool result_first, bool force) {
  const RobotState *state = latest_state();
  if (!force && state != nullptr &&
      state->execution_state == ExecutionState::Damping &&
      state->session_id == session_id_) {
    return true;
  }
  const Sequence request_id = NextRequestId();
  ExecutionRequest request = MakeRequest(Operation::Damping, request_id);
  ConfirmationExpectations expectations;
  expectations.execution_state = ExecutionState::Damping;
  expectations.output_action = OutputAction::Damping;
  expectations.producer_id = irmv::protocol::kNoId;
  TrackedRequest observed;
  const bool confirmed = SendAndConfirm(request, expectations, result_first, 0,
                                        3'000'000, &observed);
  Check(confirmed, "damping_applied", "the stop request was not confirmed");
  if (!confirmed) {
    return false;
  }
  return true;
}

bool Client::EnsureTracking(bool result_first, int drop_results) {
  const RobotState *state = latest_state();
  if (state == nullptr) {
    Check(false, "tracking_state_known", "no state to build a command from");
    return false;
  }
  const Sequence request_id = NextRequestId();
  const Sequence expected = state->epoch;
  const Sequence proposed = options_.epoch != 0 ? options_.epoch : expected + 1;
  ExecutionRequest request = MakeRequest(Operation::Tracking, request_id);
  request.expected_epoch = expected;
  request.proposed_epoch = proposed;
  request.producer_id = producer_id_;
  request.first_command = MakeFirstCommand(*state, command_seq_ + 1, proposed);
  command_seq_ = request.first_command.command_seq;
  ConfirmationExpectations expectations;
  expectations.execution_state = ExecutionState::Tracking;
  expectations.output_action = OutputAction::Tracking;
  expectations.epoch = proposed;
  expectations.producer_id = producer_id_;
  TrackedRequest observed;
  const bool confirmed = SendAndConfirm(request, expectations, result_first,
                                        drop_results, 3'000'000, &observed);
  Check(confirmed, "tracking_granted",
        "the tracking request was not confirmed");
  if (!confirmed) {
    return false;
  }
  return true;
}

bool Client::RequestZeroTorque(bool result_first) {
  const Sequence request_id = NextRequestId();
  ExecutionRequest request = MakeRequest(Operation::ZeroTorque, request_id);
  ConfirmationExpectations expectations;
  expectations.execution_state = ExecutionState::ZeroTorque;
  expectations.output_action = OutputAction::ZeroTorque;
  expectations.producer_id = irmv::protocol::kNoId;
  TrackedRequest observed;
  const bool confirmed = SendAndConfirm(request, expectations, result_first, 0,
                                        3'000'000, &observed);
  Check(confirmed, "zero_torque_applied", "the stop request was not confirmed");
  if (!confirmed) {
    return false;
  }
  return true;
}

bool Client::HoldTracking(TimeUs duration_us, bool stop_on_fallback,
                          TimeUs since_us, bool *fell_back) {
  const TimeUs deadline = now() + duration_us;
  if (fell_back != nullptr) {
    *fell_back = false;
  }
  while (now() < deadline && !StopRequested()) {
    Poll(true);
    PumpHeartbeat();
    const RobotState *state = latest_state();
    if (state != nullptr && state->stamp_us >= since_us &&
        (state->execution_state == ExecutionState::Damping ||
         state->execution_state == ExecutionState::ZeroTorque) &&
        state->stop_reason != StopReason::None) {
      if (fell_back != nullptr) {
        *fell_back = true;
      }
      if (stop_on_fallback) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
  }
  return !StopRequested();
}

JointCommand Client::MakeFirstCommand(const RobotState &state,
                                      Sequence command_seq,
                                      Sequence proposed_epoch) {
  // The entry frame of a tracking authorization reproduces the damping torque
  // of the measured state: it is the only frame the runtime checks for
  // continuity, and that check compares the torque the frame would produce with
  // the torque the current state produces.
  JointCommand command;
  command.protocol_version = irmv::protocol::kProtocolVersion;
  command.runtime_id = state.runtime_id;
  command.session_id = state.session_id;
  command.producer_id = producer_id_;
  command.epoch = proposed_epoch;
  command.command_seq = command_seq;
  command.source_state_seq = state.state_seq;
  command.stamp_us = now();
  static_cast<void>(command.layout_id.Assign(config_.layout_id));
  command.joints.count = joint_count();
  for (std::uint32_t index = 0; index < joint_count(); ++index) {
    const double q = state.feedback.joints[index].q;
    const double dq = state.feedback.joints[index].dq;
    command.joints.q[index] = q;
    command.joints.dq[index] = dq;
    command.joints.kp[index] = 0.0;
    command.joints.kd[index] = config_.damping_kd[index];
    command.joints.tau_ff[index] = -config_.damping_kd[index] * dq;
  }
  return command;
}

bool Client::Recover() {
  // The runtime revoked the session while this supervisor was not running: the
  // whole handshake has to be made again, which is what the pause and kill
  // scenarios must show.
  Note("supervision session gone; re-acquiring");
  session_id_ = irmv::protocol::kNoId;
  heartbeat_seq_ = 0;
  if (!EnsureSession()) {
    return false;
  }
  return EnsureDamping(true) && EnsureTracking(true, 0);
}

int Client::RunSupervisor() {
  const std::string &scenario = options_.scenario;
  if (scenario == "flood" || scenario == "log_volume") {
    return RunFlood();
  }
  if (EndsWith(scenario, "_replay")) {
    return RunReplay();
  }
  if (!WaitForZeroTorque()) {
    return Finish("supervisor");
  }

  if (scenario == "normal") {
    // Both confirmation orders are exercised: the handshake is confirmed with
    // the state of the tick observed first, the tracking and the stops with the
    // answer observed first.
    EnsureSession();
    EnsureDamping(false);
    if (EnsureTracking(true, 0)) {
      bool fell_back = false;
      const TimeUs hold_start = now();
      HoldTracking(4'000'000, true, hold_start, &fell_back);
      Check(fell_back, "target_expired_to_damping",
            "no command timeout fallback observed");
      if (fell_back) {
        measurements_["fallback_observed_us"] = now() - hold_start;
      }
      if (EnsureTracking(true, 0)) {
        HoldTracking(400'000, false, now(), nullptr);
      }
      EnsureDamping(true);
      RequestZeroTorque(true);
    }
    return Finish("supervisor");
  }

  if (scenario == "request_retry") {
    EnsureSession();
    EnsureDamping(true);
    const RobotState *state = latest_state();
    if (state == nullptr) {
      Check(false, "retry_state_known", "no state observed");
      return Finish("supervisor");
    }
    const Sequence watermark = state->epoch;
    const Sequence proposed =
        options_.epoch != 0 ? options_.epoch : watermark + 1;
    const Sequence request_id = NextRequestId();
    ExecutionRequest request = MakeRequest(Operation::Tracking, request_id);
    request.expected_epoch = watermark;
    request.proposed_epoch = proposed;
    request.producer_id = producer_id_;
    request.first_command =
        MakeFirstCommand(*state, command_seq_ + 1, proposed);
    command_seq_ = request.first_command.command_seq;
    ConfirmationExpectations expectations;
    expectations.execution_state = ExecutionState::Tracking;
    expectations.output_action = OutputAction::Tracking;
    expectations.epoch = proposed;
    expectations.producer_id = producer_id_;
    TrackedRequest observed;
    // The answer of the first send is lost on purpose: only the retry can
    // confirm, and the retry repeats the identical payload.
    const bool confirmed =
        SendAndConfirm(request, expectations, true, 1, 4'000'000, &observed);
    Check(confirmed, "retry_confirmed",
          "the retried tracking request was not confirmed");
    measurements_["retry_send_count"] = observed.send_count;
    measurements_["retry_request_id"] = request_id;

    std::size_t answers = 0;
    bool identical = true;
    json seen = json::array();
    for (const auto &entry : all_results_) {
      if (entry.first.request_id != request_id) {
        continue;
      }
      ++answers;
      seen.push_back(json{{"status", StatusName(entry.first.status)},
                          {"reason", ReasonName(entry.first.reason)},
                          {"epoch", entry.first.epoch},
                          {"session_id", IdHex(entry.first.session_id)},
                          {"stamp_us", entry.first.stamp_us},
                          {"t_us", entry.second}});
      if (entry.first.status != ResultStatus::Applied ||
          entry.first.epoch != proposed ||
          entry.first.session_id != observed.confirmed_session) {
        identical = false;
      }
    }
    measurements_["retry_answers"] = seen;
    Check(answers >= 2, "retry_answered_twice",
          "the runtime did not answer the repeated request");
    Check(identical, "retry_same_answer",
          "the repeated request was not answered with the stored result");

    Sequence max_epoch = 0;
    for (const StateView &view : all_states_) {
      max_epoch = std::max(max_epoch, view.epoch);
    }
    measurements_["max_epoch"] = max_epoch;
    Check(max_epoch <= proposed, "retry_no_second_authorization",
          "a second authorization epoch appeared");
    HoldTracking(400'000, false, now(), nullptr);
    EnsureDamping(true);
    RequestZeroTorque(true);
    return Finish("supervisor");
  }

  if (scenario == "stale_context_record" ||
      scenario == "runtime_restarted_record") {
    EnsureSession();
    EnsureDamping(true);
    if (EnsureTracking(true, 0)) {
      // A few commands belong to the saved traffic: a replayed command must not
      // refresh a target either.
      const TimeUs until = now() + 600'000;
      TimeUs next_command = now();
      while (now() < until && !StopRequested()) {
        Poll(true);
        PumpHeartbeat();
        const RobotState *state = latest_state();
        if (state != nullptr &&
            state->execution_state == ExecutionState::Tracking &&
            state->producer_id == producer_id_ && now() >= next_command) {
          next_command = now() + 20'000;
          if (state->epoch != window_epoch_) {
            window_epoch_ = state->epoch;
            window_start_us_ = now();
            ++window_index_;
            command_seq_ =
                std::max(command_seq_, state->last_applied_command_seq);
          }
          SendProducerCommand(*state);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
      }
    }
    EnsureDamping(true);
    RequestZeroTorque(true);
    Emit(json{{"kind", "packets_saved"},
              {"t_us", now()},
              {"requests", requests_sent_},
              {"commands", commands_sent_},
              {"heartbeats", heartbeat_sent_}});
    measurements_["packets_saved_requests"] = requests_sent_;
    measurements_["packets_saved_commands"] = commands_sent_;
    return Finish("supervisor");
  }

  if (scenario == "supervisor_killed" ||
      scenario == "supervisor_killed_record" ||
      scenario == "supervisor_paused") {
    EnsureSession();
    EnsureDamping(true);
    if (!EnsureTracking(true, 0)) {
      return Finish("supervisor");
    }
    // Stay in tracking and keep the lease alive; the harness stops or kills
    // this process and the client reports the recovery it had to make
    // afterwards.
    const TimeUs deadline = now() + 25'000'000;
    std::uint64_t recoveries = 0;
    bool recovered_and_held = false;
    TimeUs recovery_deadline = 0;
    while (now() < deadline && !StopRequested()) {
      Poll(true);
      if (session_id_ != irmv::protocol::kNoId) {
        PumpHeartbeat();
      }
      const RobotState *state = latest_state();
      if (state == nullptr) {
        std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
        continue;
      }
      if (state->session_id == irmv::protocol::kNoId) {
        if (session_id_ != irmv::protocol::kNoId) {
          Note("session " + IdHex(session_id_) + " was revoked by the runtime");
        }
        session_id_ = irmv::protocol::kNoId;
        heartbeat_seq_ = 0;
        if (Recover()) {
          ++recoveries;
          recovery_deadline = now() + 1'500'000;
          Note("re-acquired after the revocation");
        }
        continue;
      }
      if (recovery_deadline != 0 &&
          state->execution_state == ExecutionState::Tracking) {
        if (!recovered_and_held) {
          recovered_and_held = true;
          Note("tracking is live again with epoch " +
               std::to_string(state->epoch));
        }
        if (now() >= recovery_deadline) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
    }
    measurements_["recoveries"] = recoveries;
    Check(recovered_and_held, "recovered_after_fallback",
          "no re-authorization after the revocation");
    if (recovered_and_held) {
      EnsureDamping(true);
      RequestZeroTorque(true);
    }
    return Finish("supervisor");
  }

  if (scenario == "write_refused") {
    // The fake scenario refuses the torque write of a tracking tick. The
    // runtime must latch the refusal as a fault, command zero torque instead,
    // and leave the fault only for a matching Reset once the write path is
    // healthy again.
    EnsureSession();
    EnsureDamping(true);
    if (!EnsureTracking(true, 0)) {
      return Finish("supervisor");
    }
    bool faulted = false;
    Sequence fault_id = 0;
    const TimeUs fault_deadline = now() + 6'000'000;
    while (now() < fault_deadline && !StopRequested()) {
      Poll(true);
      PumpHeartbeat();
      const RobotState *state = latest_state();
      if (state != nullptr && state->execution_state == ExecutionState::Fault &&
          state->fault_id != 0) {
        faulted = true;
        fault_id = state->fault_id;
        Note("fault " + std::to_string(fault_id) +
             " latched by the refused write");
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
    }
    Check(faulted, "write_failure_latched",
          "a refused torque write did not latch a fault");
    measurements_["fault_id"] = fault_id;
    if (faulted) {
      // A rejected Reset is answered, not applied, so each attempt repeats with
      // a new id: the recovery window of the state machine needs a healthy
      // window with an accepted write path before it releases the fault.
      bool reset = false;
      std::uint64_t attempts = 0;
      const TimeUs reset_deadline = now() + 6'000'000;
      while (!reset && now() < reset_deadline && !StopRequested()) {
        ++attempts;
        const Sequence request_id = NextRequestId();
        ExecutionRequest request = MakeRequest(Operation::Reset, request_id);
        request.fault_id = fault_id;
        ConfirmationExpectations expectations;
        expectations.execution_state = ExecutionState::ZeroTorque;
        expectations.output_action = OutputAction::ZeroTorque;
        expectations.producer_id = irmv::protocol::kNoId;
        TrackedRequest observed;
        reset =
            SendAndConfirm(request, expectations, true, 0, 500'000, &observed);
        // The pause keeps the lease alive: the supervisor timeout is 300 ms, so
        // waiting without a heartbeat would cost the session, not just the
        // attempt.
        const TimeUs retry_at = now() + 250'000;
        while (!reset && now() < retry_at && !StopRequested()) {
          Poll(true);
          PumpHeartbeat();
          std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
        }
      }
      measurements_["reset_attempts"] = attempts;
      Check(reset, "reset_applied",
            "the fault was never cleared by a matching Reset");
      Check(RequestZeroTorque(true), "zero_torque_applied",
            "the stop after the reset was not confirmed");
    }
    return Finish("supervisor");
  }

  if (scenario == "producer_paused") {
    EnsureSession();
    EnsureDamping(true);
    if (!EnsureTracking(true, 0)) {
      return Finish("supervisor");
    }
    const Sequence pre_fallback_seq =
        latest_state() == nullptr ? 0
                                  : latest_state()->last_applied_command_seq;
    bool fell_back = false;
    const TimeUs grant_us = now();
    HoldTracking(5'000'000, true, grant_us, &fell_back);
    Check(fell_back, "command_timeout_damping",
          "the command timeout did not stop tracking");
    const RobotState *fallback = latest_state();
    Check(fallback != nullptr && fallback->session_id != irmv::protocol::kNoId,
          "fallback_keeps_session", "the command timeout revoked the session");
    measurements_["pre_fallback_command_seq"] = pre_fallback_seq;
    // Re-authorizing is the only way back: the producer resumes under the epoch
    // of a second tracking request.
    if (EnsureTracking(true, 0)) {
      HoldTracking(1'200'000, false, now(), nullptr);
      const Sequence after = latest_state() == nullptr
                                 ? 0
                                 : latest_state()->last_applied_command_seq;
      measurements_["post_recovery_command_seq"] = after;
      Check(after > pre_fallback_seq, "commands_resumed",
            "no command was applied after the recovery");
    }
    EnsureDamping(true);
    RequestZeroTorque(true);
    return Finish("supervisor");
  }

  if (scenario == "command_flood" || scenario == "slow_logger") {
    EnsureSession();
    EnsureDamping(true);
    if (!EnsureTracking(true, 0)) {
      return Finish("supervisor");
    }
    HoldTracking(1'000'000, false, now(), nullptr);
    const Sequence applied_before =
        latest_state() == nullptr ? 0
                                  : latest_state()->last_applied_command_seq;
    // The stop travels on the dedicated stop lane and must not be starved,
    // whatever the ordinary request lane is carrying.
    // The state may well have fallen back to Damping already (a starved command
    // lane expires a target), so the stop is forced: this measures a request
    // that really travelled on the stop lane while the flood ran.
    const TimeUs stop_send_us = now();
    const bool damped = EnsureDamping(true, true);
    measurements_["stop_latency_us"] = now() - stop_send_us;
    Check(damped, "stop_under_flood",
          "the stop was not applied while the flood ran");
    measurements_["applied_command_seq_before_stop"] = applied_before;
    RequestZeroTorque(true);
    return Finish("supervisor");
  }

  Diagnostic("unknown supervisor scenario \"" + scenario + "\"");
  return kExitUsage;
}

// ---- target flood ---------------------------------------------------------

int Client::RunFlood() {
  // A target flood: ordinary tracking requests on the request lane, at a rate
  // the control core cannot answer one by one. Every one of them is rejected,
  // because the expected epoch never matches the live watermark, so the flood
  // never changes an authorization. `--rate` alone decides the rate.
  emit_traffic_ = false;
  const TimeUs period_us = static_cast<TimeUs>(1'000'000.0 / options_.rate_hz);
  const TimeUs deadline = now() + 8'000'000;
  // Every flood owns a private band of request ids, drawn from the system
  // entropy: two floods running at once therefore cannot count each other's
  // answers, and no band reaches the supervisor's own ids. The client also
  // cross-checks its count against what it sent (a band collision would show up
  // as more answers than sends instead of a silently wrong drop count).
  const Sequence flood_base =
      kFloodRequestBase + MakeFloodBand() * kFloodBandWidth;
  Sequence request_id = flood_base;
  Id observed_session = irmv::protocol::kNoId;
  bool ever_had_session = false;
  flood_emit_us_ = now();
  while (now() < deadline && !StopRequested()) {
    Poll(true);
    const RobotState *state = latest_state();
    if (state != nullptr) {
      if (state->session_id != irmv::protocol::kNoId) {
        observed_session = state->session_id;
        ever_had_session = true;
      } else if (ever_had_session &&
                 state->execution_state == ExecutionState::ZeroTorque) {
        // The scenario is over once the supervised phase has ended.
        break;
      }
    }
    if (observed_session == irmv::protocol::kNoId ||
        runtime_id_ == irmv::protocol::kNoId) {
      std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
      continue;
    }
    if (now() >= flood_emit_us_ + period_us) {
      flood_emit_us_ = now();
      // Built through MakeRequest so the frame carries a well-formed command:
      // the wire codec validates the embedded command of every request.
      ExecutionRequest request = MakeRequest(Operation::Tracking, ++request_id);
      request.session_id = observed_session;
      request.runtime_id = runtime_id_;
      // The expected epoch never matches the live watermark, and no producer is
      // named: the flood can therefore never change an authorization.
      request.expected_epoch = 0;
      request.proposed_epoch = 0;
      request.producer_id = irmv::protocol::kNoId;
      traffic_.SendRequest(request);
      ++flood_sent_;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  measurements_["flood_sent"] = flood_sent_;
  measurements_["flood_write_failures"] = traffic_.write_failures();
  std::uint64_t answers = 0;
  for (const auto &entry : all_results_) {
    if (entry.first.request_id > flood_base &&
        entry.first.request_id <= request_id) {
      ++answers;
    }
  }
  measurements_["flood_answered"] = answers;
  measurements_["flood_band"] = flood_base / kFloodBandWidth;
  Check(flood_sent_ > 0, "flood_sent_requests", "the flood sent nothing");
  // Only this process's own band is counted: more answers than sends would mean
  // two bands collided and the drop count cannot be trusted.
  Check(answers <= flood_sent_, "flood_count_consistent",
        "more answers than requests: two flood id bands overlap");
  // `flood` must exceed what the runtime can answer one by one, so the requests
  // it drops are the drops the scenario counts; `log_volume` only has to stay
  // inside the inbound capacity while it produces events, which is what the
  // slow-consumer scenario needs.
  if (options_.scenario == "flood") {
    Check(answers < flood_sent_, "flood_dropped_requests",
          "every flooded request was answered");
  }
  return Finish("supervisor");
}

// ---- producer -------------------------------------------------------------

Client::CommandPlan Client::PlanForWindow(TimeUs elapsed_us) const {
  CommandPlan plan;
  // The first window drives the per-tick acceptance: distinct measured
  // velocities, a binding clamp, a free torque step, and then silence so the
  // hold deadline can be measured. Later windows only keep a target fresh.
  if (window_index_ > 1) {
    plan.hold_measured_q = true;
    plan.dq_target = 0.0;
    plan.kd = 3.0;
    return plan;
  }
  if (elapsed_us < 200'000) {
    plan.hold_measured_q = true;
    plan.dq_target = 0.0;
    plan.kd = 3.0;
    return plan;
  }
  if (elapsed_us < 600'000) {
    plan.q_target = 1.0;
    plan.dq_target = 0.0;
    plan.kp = 40.0;
    return plan;
  }
  if (elapsed_us < 700'000) {
    plan.tau_ff = -2.0;
    return plan;
  }
  if (elapsed_us < 1'000'000) {
    plan.tau_ff = 13.0;
    return plan;
  }
  plan.publish = false;
  return plan;
}

JointCommand Client::MakeCommand(const RobotState &state,
                                 const CommandPlan &plan) {
  JointCommand command;
  command.protocol_version = irmv::protocol::kProtocolVersion;
  command.runtime_id = state.runtime_id;
  command.session_id = state.session_id;
  command.producer_id = producer_id_;
  command.epoch = state.epoch;
  command.command_seq = ++command_seq_;
  // The source state sequence is the newest state the producer really received;
  // an old computation is never re-labelled as a fresh one.
  command.source_state_seq = state.state_seq;
  command.stamp_us = now();
  static_cast<void>(command.layout_id.Assign(config_.layout_id));
  command.joints.count = joint_count();
  for (std::uint32_t index = 0; index < joint_count(); ++index) {
    command.joints.q[index] =
        plan.hold_measured_q ? state.feedback.joints[index].q : plan.q_target;
    command.joints.dq[index] = plan.dq_target;
    command.joints.kp[index] = plan.kp;
    command.joints.kd[index] = plan.kd;
    command.joints.tau_ff[index] = plan.tau_ff;
  }
  return command;
}

void Client::SendProducerCommand(const RobotState &state) {
  const TimeUs now_us = now();
  const CommandPlan plan = PlanForWindow(now_us - window_start_us_);
  if (!plan.publish) {
    return;
  }
  const JointCommand command = MakeCommand(state, plan);
  traffic_.SendCommand(command);
  ++commands_sent_;
  command_source_seqs_.push_back(command.source_state_seq);
  SavePacket(json{{"kind", "command"}, {"packet", CommandJson(command)}});
  if (options_.rate_hz <= 200.0 || commands_sent_ % 50 == 0) {
    Emit(json{{"kind", "command"},
              {"t_us", now_us},
              {"command_seq", command.command_seq},
              {"epoch", command.epoch},
              {"source_state_seq", command.source_state_seq},
              {"kp", command.joints.kp[0]},
              {"kd", command.joints.kd[0]},
              {"tau_ff", command.joints.tau_ff[0]}});
  }
}

int Client::RunProducer() {
  const TimeUs period_us = static_cast<TimeUs>(1'000'000.0 / options_.rate_hz);
  const TimeUs deadline = now() + kGlobalCapUs;
  TimeUs quiet_since_us = 0;
  while (now() < deadline && !StopRequested()) {
    Poll(true);
    const RobotState *state = latest_state();
    const TimeUs now_us = now();
    if (state == nullptr) {
      std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
      continue;
    }
    const bool authorized =
        state->execution_state == ExecutionState::Tracking &&
        state->producer_id == producer_id_ &&
        state->session_id != irmv::protocol::kNoId;
    if (!authorized) {
      // Nothing is published without an authorization: a producer that keeps
      // commanding a revoked epoch would only be ignored by the runtime.
      if (tracking_seen_ && quiet_since_us == 0) {
        quiet_since_us = now_us;
      }
      if (zero_torque_since_us_ != 0 && tracking_seen_ &&
          now_us - zero_torque_since_us_ > 400'000) {
        break;
      }
      if (quiet_since_us != 0 && now_us - quiet_since_us > 3'000'000) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    if (state->epoch != window_epoch_) {
      window_epoch_ = state->epoch;
      window_start_us_ = now_us;
      ++window_index_;
      command_seq_ = std::max(command_seq_, state->last_applied_command_seq);
      next_command_us_ = now_us;
      Note("authorized for epoch " + std::to_string(window_epoch_) +
           " (window " + std::to_string(window_index_) + ")");
    }
    if (now_us >= next_command_us_) {
      next_command_us_ = now_us + period_us;
      SendProducerCommand(*state);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  const std::set<Sequence> distinct(command_source_seqs_.begin(),
                                    command_source_seqs_.end());
  Emit(json{{"kind", "command_summary"},
            {"t_us", now()},
            {"count", commands_sent_},
            {"distinct_source_state_seqs", distinct.size()},
            {"write_failures", traffic_.write_failures()}});
  measurements_["commands_sent"] = commands_sent_;
  measurements_["command_write_failures"] = traffic_.write_failures();
  measurements_["distinct_source_state_seqs"] = distinct.size();
  Check(commands_sent_ > 0, "producer_commanded",
        "no command was ever published");
  Check(distinct.size() > 1, "producer_source_state_fresh",
        "the producer re-used one source state sequence");
  return Finish("producer");
}

// ---- observer -------------------------------------------------------------

int Client::RunObserver() {
  // The observer only listens: it stamps every state and result it receives and
  // leaves the assertions to the harness.
  const TimeUs deadline = now() + kObserverCapUs;
  while (now() < deadline && !StopRequested()) {
    Poll(true);
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  Poll(true);
  std::uint64_t tracking_states = 0;
  for (const StateView &view : all_states_) {
    if (view.execution_state == ExecutionState::Tracking) {
      ++tracking_states;
    }
  }
  measurements_["states"] = static_cast<std::uint64_t>(all_states_.size());
  measurements_["results"] = static_cast<std::uint64_t>(all_results_.size());
  measurements_["tracking_states"] = tracking_states;
  return Finish("observer");
}

// ---- replay ---------------------------------------------------------------

int Client::RunReplay() {
  std::vector<json> packets;
  if (!LoadPackets(&packets)) {
    Check(false, "replay_file_readable", "the packet file could not be read");
    return Finish("supervisor");
  }
  Check(!packets.empty(), "replay_file_not_empty", "no packet was saved");
  Note("replaying " + std::to_string(packets.size()) + " saved packets");
  for (const json &record : packets) {
    if (record.value("kind", "") != "request") {
      continue;
    }
    const ExecutionRequest request = RequestFromJson(record.at("packet"));
    Emit(json{{"kind", "replay_packet"},
              {"t_us", now()},
              {"packet", "request"},
              {"request_id", request.request_id},
              {"operation", OperationName(request.operation)},
              {"runtime_id", IdHex(request.runtime_id)},
              {"session_id", IdHex(request.session_id)},
              {"epoch", request.expected_epoch},
              {"producer_id", IdHex(request.producer_id)}});
  }

  const TimeUs deadline = now() + 4'000'000;
  bool saw_instance = false;
  while (now() < deadline && !saw_instance && !StopRequested()) {
    Poll(true);
    if (latest_state() != nullptr &&
        latest_state()->runtime_id != irmv::protocol::kNoId) {
      saw_instance = true;
      Note("live runtime instance " + IdHex(latest_state()->runtime_id));
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
  }

  for (int round = 0; round < 3 && !StopRequested(); ++round) {
    for (const json &record : packets) {
      const std::string kind = record.at("kind").get<std::string>();
      if (kind == "request") {
        const ExecutionRequest request = RequestFromJson(record.at("packet"));
        const TimeUs sent = now();
        traffic_.SendRequest(request);
        ++requests_sent_;
        ++replay_requests_;
        Emit(json{{"kind", "replay"},
                  {"t_us", sent},
                  {"packet", "request"},
                  {"round", round},
                  {"operation", OperationName(request.operation)},
                  {"request_id", request.request_id},
                  {"sent_runtime_id", IdHex(request.runtime_id)},
                  {"sent_session_id", IdHex(request.session_id)}});
        const TimeUs answer_deadline = now() + 150'000;
        bool answered = false;
        while (now() < answer_deadline && !answered && !StopRequested()) {
          Poll(true);
          // A replayed packet is answered by an instance that may not be the
          // one the packet names, so the answer is matched on the request id
          // and the arrival time only: the identity check of the confirmation
          // log would hide exactly the rejection this scenario measures.
          const ExecutionResult *answer = nullptr;
          for (const auto &entry : all_results_) {
            if (entry.first.request_id == request.request_id &&
                entry.second >= sent) {
              answer = &entry.first;
              break;
            }
          }
          if (answer != nullptr) {
            answered = true;
            const bool session_scoped =
                request.session_id != irmv::protocol::kNoId ||
                request.expected_epoch != 0;
            if (answer->status == ResultStatus::Applied) {
              if (session_scoped) {
                ++replay_applied_;
              } else {
                ++replay_bootstrap_applied_;
              }
            }
            Emit(json{{"kind", "replay_answer"},
                      {"t_us", now()},
                      {"request_id", request.request_id},
                      {"operation", OperationName(request.operation)},
                      {"sent_session_id", IdHex(request.session_id)},
                      {"sent_epoch", request.expected_epoch},
                      {"status", StatusName(answer->status)},
                      {"reason", ReasonName(answer->reason)},
                      {"runtime_id", IdHex(answer->runtime_id)},
                      {"session_id", IdHex(answer->session_id)}});
          }
          std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
        }
      } else if (kind == "heartbeat") {
        const SupervisorHeartbeat heartbeat =
            HeartbeatFromJson(record.at("packet"));
        traffic_.SendHeartbeat(heartbeat);
        Emit(json{{"kind", "replay"},
                  {"t_us", now()},
                  {"packet", "heartbeat"},
                  {"round", round},
                  {"heartbeat_seq", heartbeat.heartbeat_seq},
                  {"session_id", IdHex(heartbeat.session_id)}});
      } else if (kind == "command") {
        const JointCommand command = CommandFromJson(record.at("packet"));
        traffic_.SendCommand(command);
        Emit(json{{"kind", "replay"},
                  {"t_us", now()},
                  {"packet", "command"},
                  {"round", round},
                  {"command_seq", command.command_seq},
                  {"epoch", command.epoch},
                  {"session_id", IdHex(command.session_id)}});
      }
      std::this_thread::sleep_for(std::chrono::microseconds(2'000));
    }
  }

  // Observe the aftermath: the runtime must not have been re-authorized.
  const TimeUs settle = now() + 500'000;
  Sequence max_epoch = 0;
  std::uint64_t tracking_states = 0;
  while (now() < settle && !StopRequested()) {
    Poll(true);
    if (latest_state() != nullptr) {
      max_epoch = std::max(max_epoch, latest_state()->epoch);
      if (latest_state()->execution_state == ExecutionState::Tracking) {
        ++tracking_states;
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(kPollSleepUs));
  }
  measurements_["replay_session_scoped_applied"] = replay_applied_;
  measurements_["replay_bootstrap_applied"] = replay_bootstrap_applied_;
  measurements_["replay_applied"] = replay_applied_;
  measurements_["replay_requests"] = replay_requests_;
  measurements_["replay_max_epoch"] = max_epoch;
  measurements_["replay_tracking_states"] = tracking_states;
  Check(replay_applied_ == 0, "replayed_session_scoped_rejected",
        "a replayed packet that named a session or an epoch was applied");
  Check(tracking_states == 0, "no_automatic_tracking",
        "the runtime entered Tracking without a new request");
  return Finish("supervisor");
}

int Client::Run() {
  if (options_.mode == "supervisor") {
    return RunSupervisor();
  }
  if (options_.mode == "producer") {
    return RunProducer();
  }
  return RunObserver();
}

}  // namespace

int main(int argc, char **argv) {
  Options options;
  std::string error;
  if (!ParseOptions(argc, argv, &options, &error)) {
    std::fprintf(stderr, "runtime_client: %s\n%s\n", error.c_str(),
                 UsageLine());
    return kExitUsage;
  }
  if (std::signal(SIGINT, HandleStopSignal) == SIG_ERR ||
      std::signal(SIGTERM, HandleStopSignal) == SIG_ERR) {
    std::fprintf(stderr, "runtime_client: cannot install the stop handlers\n");
    return kExitStartup;
  }
  RuntimeConfig config;
  try {
    config = irmv::configuration::LoadConfig(options.config_path);
  } catch (const std::exception &config_error) {
    std::fprintf(stderr, "runtime_client: configuration error: %s\n",
                 config_error.what());
    return kExitStartup;
  }
  config.dds.domain = options.domain;
  try {
    Client client(options, config);
    return client.Run();
  } catch (const std::exception &runtime_error) {
    std::fprintf(stderr, "runtime_client: %s\n", runtime_error.what());
    return kExitStartup;
  }
}
