// Test peer of the fault-recording acceptance: it subscribes to the frozen
// `irmv/v1/robot_state` topic on one domain and prints one JSON line per
// received state, so a harness can assert on what the DDS report held rather
// than on the recorder's file.
//
//   fault_recording_peer --config PATH --domain N [--seconds S]
//
// Every state line carries the published `fault_id`, the state sequence, the
// execution state and the complete fault report (reason/condition names, joint,
// error code, values, stamps, output action and message). A fault report is a
// bounded, published fact; this peer never interprets it, deduplicates it or
// substitutes one occurrence for another.
//
// SIGINT and SIGTERM only set a flag: the loop leaves after the current poll
// and the process exits 0. `--seconds` is a backstop so a harness can never
// hang on a peer that misses a signal.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dds/dds.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <thread>

#include "execution.hpp"
#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"
#include "irmv/transport/codec.hpp"

namespace {

namespace codec = irmv::transport::codec;
namespace wire = irmv::protocol::wire;
using Json = nlohmann::json;

std::atomic<bool> g_stop{false};

void HandleSignal(int) { g_stop.store(true, std::memory_order_relaxed); }

std::string InterfaceConfig(const std::string &interface_name) {
  return "<CycloneDDS><Domain "
         "id=\"any\"><General><Interfaces><NetworkInterface "
         "name=\"" +
         interface_name + "\"/></Interfaces></General></Domain></CycloneDDS>";
}

// JSON has no non-finite double. The fault schema encodes these values as the
// explicit strings the recorder uses, so a subscriber sees the same fact the
// file records instead of a null.
Json EncodeDouble(double value) {
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value > 0.0 ? "+Inf" : "-Inf";
  }
  return value;
}

const char *ReasonName(irmv::protocol::FaultReason reason) {
  switch (reason) {
    case irmv::protocol::FaultReason::None:
      return "None";
    case irmv::protocol::FaultReason::BootFailed:
      return "BootFailed";
    case irmv::protocol::FaultReason::FeedbackInvalid:
      return "FeedbackInvalid";
    case irmv::protocol::FaultReason::DriverError:
      return "DriverError";
    case irmv::protocol::FaultReason::ImuInvalid:
      return "ImuInvalid";
    case irmv::protocol::FaultReason::JointPositionExceeded:
      return "JointPositionExceeded";
    case irmv::protocol::FaultReason::JointVelocityExceeded:
      return "JointVelocityExceeded";
    case irmv::protocol::FaultReason::CommandInvalid:
      return "CommandInvalid";
    case irmv::protocol::FaultReason::WriteFailed:
      return "WriteFailed";
    case irmv::protocol::FaultReason::ProtocolExhausted:
      return "ProtocolExhausted";
  }
  return "Unknown";
}

void PrintState(const irmv::protocol::RobotState &state) {
  Json line;
  line["event"] = "state";
  line["stamp_us"] = state.stamp_us;
  line["state_seq"] = state.state_seq;
  line["execution_state"] = static_cast<std::uint32_t>(state.execution_state);
  line["output_action"] = static_cast<std::uint32_t>(state.output_action);
  line["healthy"] = state.healthy;
  line["damping_eligible"] = state.damping_eligible;
  line["fault_id"] = state.fault_id;
  line["fault_count"] = state.faults.count;
  Json details = Json::array();
  const std::size_t count = state.faults.count < irmv::protocol::kMaxJoints
                                ? state.faults.count
                                : irmv::protocol::kMaxJoints;
  for (std::size_t index = 0; index < count; ++index) {
    const irmv::protocol::FaultDetail &detail = state.faults.details[index];
    Json entry;
    entry["reason"] = static_cast<std::uint32_t>(detail.reason);
    entry["reason_name"] = ReasonName(detail.reason);
    entry["condition"] = static_cast<std::uint32_t>(detail.condition);
    entry["condition_name"] =
        irmv::protocol::FaultConditionName(detail.condition);
    if (detail.joint_index == irmv::protocol::kNoJointIndex) {
      entry["joint_index"] = nullptr;
    } else {
      entry["joint_index"] = detail.joint_index;
    }
    entry["error_code"] = detail.error_code;
    entry["observed_value"] = EncodeDouble(detail.observed_value);
    entry["limit"] = EncodeDouble(detail.limit);
    entry["sample_stamp_us"] = detail.sample_stamp_us;
    entry["detected_stamp_us"] = detail.detected_stamp_us;
    entry["output_action"] = static_cast<std::uint32_t>(detail.output_action);
    entry["message"] = std::string(detail.message.View());
    details.push_back(std::move(entry));
  }
  line["faults"] = std::move(details);
  const std::string text =
      line.dump(-1, ' ', false, Json::error_handler_t::replace);
  std::fputs(text.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

struct Options {
  std::string config_path{"config/sdk.json"};
  std::uint32_t domain{0};
  double seconds{0.0};
  // Seconds after startup at which the peer asks the runtime to reset the
  // latched fault. A reset is the only way out of Fault, and the harness needs
  // the recorder to finalize its fault stream before the process leaves. Zero
  // keeps the peer a pure listener.
  double reset_at_seconds{0.0};
};

void PrintEvent(const char *event, std::uint64_t stamp_us,
                irmv::protocol::Sequence fault_id) {
  Json line;
  line["event"] = event;
  line["stamp_us"] = stamp_us;
  line["fault_id"] = fault_id;
  const std::string text =
      line.dump(-1, ' ', false, Json::error_handler_t::replace);
  std::fputs(text.c_str(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    const bool has_value = index + 1 < argc;
    if (argument == "--config" && has_value) {
      options.config_path = argv[++index];
    } else if (argument == "--domain" && has_value) {
      options.domain =
          static_cast<std::uint32_t>(std::strtoul(argv[++index], nullptr, 10));
    } else if (argument == "--seconds" && has_value) {
      options.seconds = std::strtod(argv[++index], nullptr);
    } else if (argument == "--reset-at-seconds" && has_value) {
      options.reset_at_seconds = std::strtod(argv[++index], nullptr);
    } else {
      std::fprintf(stderr,
                   "usage: fault_recording_peer --config PATH --domain N "
                   "[--seconds S] [--reset-at-seconds S]\n");
      return 2;
    }
  }

  irmv::configuration::ConfigOverrides overrides;
  overrides.dds_domain = options.domain;
  irmv::configuration::RuntimeConfig config;
  try {
    config = irmv::configuration::LoadConfig(options.config_path, overrides);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "fault_recording_peer: %s\n", error.what());
    return 1;
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  try {
    dds::domain::DomainParticipant participant(
        config.dds.domain, dds::domain::qos::DomainParticipantQos(), nullptr,
        dds::core::status::StatusMask::none(),
        config.dds.interface.empty() ? std::string()
                                     : InterfaceConfig(config.dds.interface));
    dds::sub::Subscriber subscriber(participant);
    dds::pub::Publisher publisher(participant);
    dds::topic::Topic<wire::RobotState> state_topic(
        participant, std::string(irmv::protocol::kRobotStateTopic.name));
    dds::topic::Topic<wire::ExecutionRequest> request_topic(
        participant, std::string(irmv::protocol::kExecutionRequestTopic.name));
    dds::sub::DataReader<wire::RobotState> reader(
        subscriber, state_topic,
        codec::ToReaderQos(irmv::protocol::kRobotStateTopic.qos));
    dds::pub::DataWriter<wire::ExecutionRequest> request_writer(
        publisher, request_topic,
        codec::ToWriterQos(irmv::protocol::kExecutionRequestTopic.qos));

    // The identity the runtime answers. A reset is a normal request of the
    // session that acquired the context, so the peer bootstraps one session
    // and then uses it; no safety rule is bypassed and no request is
    // fabricated from a published identity it did not acquire.
    irmv::protocol::Id supervisor_id{};
    supervisor_id.fill(0x5a);
    irmv::protocol::Id runtime_id{};
    irmv::protocol::Id session_id{};
    std::uint64_t request_id = 0;
    bool session_announced = false;
    bool reset_sent = false;
    bool reset_applied = false;
    double last_acquire = -1.0;
    double last_reset = -1.0;

    const auto send =
        [&request_writer](const irmv::protocol::ExecutionRequest &request) {
          wire::ExecutionRequest converted;
          codec::ToWire(request, converted);
          try {
            request_writer.write(converted);
            return true;
          } catch (const std::exception &) {
            return false;
          }
        };

    const auto started = std::chrono::steady_clock::now();
    while (!g_stop.load(std::memory_order_relaxed)) {
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
      if (options.seconds > 0.0 && elapsed >= options.seconds) {
        break;
      }
      auto samples =
          reader.select().state(dds::sub::status::DataState::any()).take();
      for (const auto &sample : samples) {
        if (!sample.info().valid()) {
          continue;
        }
        irmv::protocol::RobotState state;
        if (!codec::FromWire(sample.data(), state)) {
          continue;
        }
        PrintState(state);
        if (runtime_id == irmv::protocol::kNoId) {
          runtime_id = state.runtime_id;
        }
        if (state.session_id != irmv::protocol::kNoId) {
          session_id = state.session_id;
          if (!session_announced) {
            session_announced = true;
            PrintEvent("session_acquired", state.stamp_us, state.fault_id);
          }
        } else {
          // The runtime expires a session whose lease lapsed, so a reset has to
          // be issued under a *current* session. Forget the old id and acquire
          // again; the reset branch below waits for the issued one.
          session_id = irmv::protocol::kNoId;
          session_announced = false;
        }
        // Bootstrap the session the reset will be issued under. A request can
        // be lost before discovery completes, so it is repeated at a bounded
        // rate until the issued session is observed.
        if (session_id == irmv::protocol::kNoId &&
            runtime_id != irmv::protocol::kNoId &&
            elapsed - last_acquire >= 0.2) {
          last_acquire = elapsed;
          irmv::protocol::ExecutionRequest request;
          request.protocol_version = irmv::protocol::kProtocolVersion;
          request.operation = irmv::protocol::Operation::AcquireSession;
          request.supervisor_id = supervisor_id;
          request.request_id = ++request_id;
          request.runtime_id = runtime_id;
          request.session_id = irmv::protocol::kNoId;
          static_cast<void>(send(request));
        }
        // The only way out of a latched fault is Reset, and the harness needs
        // the recorder to finalize before the runtime leaves. Reset is issued
        // under the acquired session, with the published fault id.
        if (options.reset_at_seconds > 0.0 &&
            elapsed >= options.reset_at_seconds &&
            session_id != irmv::protocol::kNoId &&
            state.execution_state == irmv::protocol::ExecutionState::Fault &&
            elapsed - last_reset >= 0.2) {
          last_reset = elapsed;
          reset_sent = true;
          irmv::protocol::ExecutionRequest request;
          request.protocol_version = irmv::protocol::kProtocolVersion;
          request.operation = irmv::protocol::Operation::Reset;
          request.supervisor_id = supervisor_id;
          request.request_id = ++request_id;
          request.session_id = session_id;
          request.runtime_id = runtime_id;
          request.expected_epoch = state.epoch;
          request.proposed_epoch = state.epoch;
          request.fault_id = state.fault_id;
          static_cast<void>(send(request));
        }
        if (reset_sent && !reset_applied &&
            state.execution_state ==
                irmv::protocol::ExecutionState::ZeroTorque) {
          reset_applied = true;
          PrintEvent("reset_applied", state.stamp_us, state.fault_id);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  } catch (const std::exception &error) {
    std::fprintf(stderr, "fault_recording_peer: %s\n", error.what());
    return 1;
  }
  return 0;
}
