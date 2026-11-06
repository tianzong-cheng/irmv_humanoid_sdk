// The recorder worker's live tail: envelope identity, transport sequence,
// bounded sends and the promise that a missing receiver costs the tail only.
//
// The core case installs a deterministic send hook, so no test depends on a
// service being up. One case uses a real Unix datagram socket to prove the
// production path still writes the same envelope, and the recorder cases prove
// that a refused tail never changes what the run recorded.
#include "irmv/telemetry/live_telemetry.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/fault_record.hpp"
#include "irmv/telemetry/recorder.hpp"
#include "irmv/telemetry/velocity_record.hpp"
#include "support/check.hpp"

namespace {

using Json = nlohmann::json;
using irmv::protocol::ExecutionState;
using irmv::protocol::FaultCondition;
using irmv::protocol::FaultReason;
using irmv::protocol::Id;
using irmv::protocol::OutputAction;
using irmv::protocol::StopReason;
using irmv::telemetry::EncodeLiveSample;
using irmv::telemetry::EncodeLiveVelocity;
using irmv::telemetry::LiveTelemetryOptions;
using irmv::telemetry::LiveTelemetrySender;
using irmv::telemetry::RecordedFault;
using irmv::telemetry::RecordSample;
using irmv::telemetry::Recorder;
using irmv::telemetry::SetLiveTelemetrySendHookForTest;
using irmv::telemetry::VelocityRecorder;
using irmv::telemetry::VelocitySample;

struct Captured {
  std::vector<std::string> messages;
  std::size_t failures_left{0};
  std::size_t attempts{0};
};

int Capture(void *context, std::string_view bytes) {
  Captured *captured = static_cast<Captured *>(context);
  captured->messages.emplace_back(bytes);
  captured->attempts += 1;
  if (captured->failures_left > 0) {
    --captured->failures_left;
    return -1;
  }
  return 0;
}

std::string Metadata() {
  Json metadata;
  metadata["schema_version"] = 4;
  metadata["config_id"] = std::string(64, 'a');
  metadata["layout_id"] = std::string(64, 'b');
  metadata["sample_hz"] = 100;
  metadata["clock"] = {{"clock", "linux_clock_monotonic"},
                       {"boot_id", "boot-live"},
                       {"available", true}};
  metadata["source"] = "runtime";
  metadata["run_path"] = "/tmp/irmv-live-run";
  return metadata.dump();
}

RecordSample MakeSample(std::uint64_t seq) {
  RecordSample sample;
  sample.stamp_us = 1000 + seq * 10000;
  sample.state_seq = seq;
  sample.execution_state = ExecutionState::Tracking;
  sample.output_action = OutputAction::Tracking;
  sample.stop_reason = StopReason::None;
  sample.healthy = true;
  sample.command_present = true;
  sample.feedback_valid = true;
  sample.imu_valid = true;
  sample.send_accepted = true;
  sample.output_valid = true;
  sample.joint_count = 2;
  for (std::uint32_t index = 0; index < 2; ++index) {
    sample.q[index] = 0.5 + 0.01 * index;
    sample.measured_q[index] = 0.25;
    sample.joint_valid[index] = true;
  }
  sample.measured_tau_est[0] = std::nan("");
  return sample;
}

RecordedFault MakeFault(std::uint64_t seq) {
  RecordedFault fault;
  fault.record_seq = seq;
  fault.runtime_id.fill(0x11);
  fault.state_seq = 100 + seq;
  fault.fault_id = seq;
  fault.detail.reason = FaultReason::JointPositionExceeded;
  fault.detail.condition = FaultCondition::JointPositionExceeded;
  fault.detail.joint_index = 0;
  fault.detail.error_code = 7;
  fault.detail.observed_value = std::nan("");
  fault.detail.limit = 1.0;
  fault.detail.sample_stamp_us = 1000;
  fault.detail.detected_stamp_us = 1010;
  fault.detail.output_action = OutputAction::ZeroTorque;
  fault.detail.message.Assign("live fault");
  return fault;
}

std::string MakeSocketPath() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return "/tmp/irmv-live-" + std::to_string(now) + ".sock";
}

}  // namespace

int main() {
  Captured captured;
  SetLiveTelemetrySendHookForTest(Capture, &captured);
  LiveTelemetrySender sender(LiveTelemetryOptions{MakeSocketPath()},
                             Metadata());
  CHECK(sender.enabled());

  CHECK(sender.TrySend("hello", R"({"record_kind":"runtime"})"));
  CHECK(sender.TrySend("sample", EncodeLiveSample(MakeSample(1))));
  CHECK(sender.transport_seq() == 2);
  CHECK(captured.messages.size() == 2);
  Json hello = Json::parse(captured.messages[0], nullptr, false);
  CHECK(!hello.is_discarded());
  CHECK(hello["version"] == 1);
  CHECK(hello["source"] == "runtime");
  CHECK(hello["run_path"] == "/tmp/irmv-live-run");
  CHECK(hello["kind"] == "hello");
  CHECK(hello["transport_seq"] == "1");
  CHECK(hello["emitter_id"].get<std::string>().size() == 32);
  const std::string emitter = hello["emitter_id"].get<std::string>();
  Json first_sample = Json::parse(captured.messages[1], nullptr, false);
  CHECK(first_sample["transport_seq"] == "2");
  CHECK(first_sample["payload"]["stamp_us"] == "11000");
  CHECK(first_sample["payload"]["state_seq"] == "1");
  CHECK(first_sample["payload"]["measured_tau_est"][0] == "NaN");
  CHECK(first_sample["payload"]["q"][0] == 0.5);

  // A refused send advances the transport sequence and is counted, but it does
  // not disturb the emitter identity or throw.
  captured.failures_left = 2;
  CHECK(!sender.TrySend("sample", EncodeLiveSample(MakeSample(2))));
  CHECK(!sender.TrySend("sample", EncodeLiveSample(MakeSample(3))));
  CHECK(sender.failures() == 2);
  CHECK(sender.transport_seq() == 4);
  CHECK(sender.TrySend("sample", EncodeLiveSample(MakeSample(4))));
  CHECK(sender.transport_seq() == 5);
  Json recovered = Json::parse(captured.messages.back(), nullptr, false);
  CHECK(recovered["emitter_id"] == emitter);
  CHECK(recovered["transport_seq"] == "5");

  // Oversize and malformed payloads are refused without touching the hook.
  const std::size_t before_oversize = captured.messages.size();
  const std::string oversize(70000, 'x');
  CHECK(!sender.TrySend("sample", oversize));
  CHECK(sender.oversize() == 1);
  CHECK(captured.messages.size() == before_oversize);
  CHECK(!sender.TrySend("sample", "{not json"));
  CHECK(!sender.TrySend("sample", R"({"value":NaN})"));
  CHECK(!sender.TrySend("sample", std::string_view("\xff\xfe", 2)));
  CHECK(sender.transport_seq() == 9);

  // A real datagram receiver proves the production path, not just the hook.
  SetLiveTelemetrySendHookForTest(nullptr, nullptr);
  {
    const std::string path = MakeSocketPath();
    const int receiver = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    CHECK(receiver >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, path.c_str(), path.size());
    CHECK(::bind(receiver, reinterpret_cast<sockaddr *>(&address),
                 static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                        path.size() + 1)) == 0);
    LiveTelemetrySender real(LiveTelemetryOptions{path}, Metadata());
    CHECK(real.enabled());
    CHECK(real.TrySend("status", R"({"ok":true})"));
    char buffer[65536];
    const ssize_t received =
        ::recv(receiver, buffer, sizeof(buffer), MSG_DONTWAIT);
    CHECK(received > 0);
    Json envelope = Json::parse(std::string(buffer, received), nullptr, false);
    CHECK(envelope["kind"] == "status");
    CHECK(envelope["source"] == "runtime");
    ::close(receiver);
    ::unlink(path.c_str());
  }

  // No receiver: the sender reports failure and remains usable.
  {
    LiveTelemetrySender absent(LiveTelemetryOptions{MakeSocketPath()},
                               Metadata());
    CHECK(absent.enabled());
    CHECK(!absent.TrySend("status", R"({"ok":true})"));
    CHECK(absent.failures() == 1);
    CHECK(absent.transport_seq() == 1);
    SetLiveTelemetrySendHookForTest(Capture, &captured);
  }

  // The recorder integration: a refused outlet never changes what is recorded.
  {
    Captured recorder_capture;
    recorder_capture.failures_left = 1000000;
    SetLiveTelemetrySendHookForTest(Capture, &recorder_capture);
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        ("irmv-live-record-test-" +
         std::to_string(std::chrono::steady_clock::now()
                            .time_since_epoch()
                            .count()));
    std::filesystem::create_directories(root);
    irmv::telemetry::RecorderConfig config;
    config.enabled = true;
    config.sample_hz = 100;
    config.queue_capacity = 64;
    config.directory = root;
    LiveTelemetryOptions options;
    options.socket_path = MakeSocketPath();
    Recorder recorder;
    std::string error;
    CHECK(recorder.Start(config, Metadata(), &error, options));
    for (std::uint64_t seq = 0; seq < 10; ++seq) {
      CHECK(recorder.TryRecord(MakeSample(seq)));
    }
    CHECK(recorder.TryRecordFault(MakeFault(0)));
    CHECK(recorder.TryRecordFault(MakeFault(1)));
    CHECK(recorder.Stop());
    CHECK(recorder.recorded() == 10);
    CHECK(recorder.faults_recorded() == 2);
    CHECK(recorder.faults_written() == 2);

    Recorder offline;
    CHECK(offline.Start(config, Metadata(), &error));
    for (std::uint64_t seq = 0; seq < 10; ++seq) {
      CHECK(offline.TryRecord(MakeSample(seq)));
    }
    CHECK(offline.TryRecordFault(MakeFault(0)));
    CHECK(offline.TryRecordFault(MakeFault(1)));
    CHECK(offline.Stop());
    CHECK(offline.recorded() == recorder.recorded());
    CHECK(offline.faults_written() == recorder.faults_written());
    CHECK(recorder_capture.attempts >= 12);
    std::size_t samples = 0;
    std::size_t faults = 0;
    for (const std::string &message : recorder_capture.messages) {
      Json envelope = Json::parse(message, nullptr, false);
      CHECK(!envelope.is_discarded());
      if (envelope["kind"] == "sample") {
        ++samples;
      } else if (envelope["kind"] == "fault") {
        ++faults;
      }
    }
    CHECK(samples == 10);
    CHECK(faults == 2);
    std::filesystem::remove_all(root);
  }

  // The application stream has its own envelope source and payload.
  {
    Captured app_capture;
    app_capture.failures_left = 1000000;
    SetLiveTelemetrySendHookForTest(Capture, &app_capture);
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        ("irmv-live-app-test-" +
         std::to_string(std::chrono::steady_clock::now()
                            .time_since_epoch()
                            .count()));
    std::filesystem::create_directories(root);
    irmv::configuration::RecorderConfig config;
    config.enabled = true;
    config.sample_hz = 100;
    config.queue_capacity = 64;
    config.directory = root;
    Json metadata = Json::parse(Metadata(), nullptr, false);
    metadata["schema"] = "irmv.application_velocity";
    metadata["version"] = 1;
    metadata["source"] = "local";
    VelocityRecorder recorder;
    LiveTelemetryOptions options;
    options.socket_path = MakeSocketPath();
    std::string error;
    CHECK(recorder.Start(config, metadata.dump(), &error, options));
    VelocitySample sample;
    sample.stamp_us = 1000;
    sample.sample_seq = 0;
    sample.identity_known = true;
    sample.runtime_id.fill(1);
    sample.session_id.fill(2);
    sample.latest_state_seq = 9;
    sample.forward = 0.5;
    sample.lateral = -0.25;
    sample.yaw = 0.125;
    CHECK(recorder.TryRecord(sample));
    CHECK(recorder.Stop());
    CHECK(recorder.recorded() == 1);
    bool saw_application = false;
    for (const std::string &message : app_capture.messages) {
      Json envelope = Json::parse(message, nullptr, false);
      CHECK(!envelope.is_discarded());
      if (envelope["kind"] == "sample") {
        CHECK(envelope["source"] == "application");
        CHECK(envelope["payload"]["forward"] == 0.5);
        saw_application = true;
      }
    }
    CHECK(saw_application);
    std::filesystem::remove_all(root);
  }

  SetLiveTelemetrySendHookForTest(nullptr, nullptr);
  return 0;
}
