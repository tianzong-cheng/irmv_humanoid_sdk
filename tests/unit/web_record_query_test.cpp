// The read-only experiment catalog and metadata bridge: what a browser can
// learn about a records root without reading a single value row.
//
// The run bytes are produced by the real recorders, so the catalog is tested
// against files this build actually wrote. The contract fixture fixes the
// physical meaning of every exposed field; the test checks the bridge reports
// that meaning rather than inferring one from a unit string or an array
// position.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/record.hpp"
#include "irmv/telemetry/record_reader.hpp"
#include "irmv/telemetry/recorder.hpp"
#include "irmv/telemetry/velocity_record.hpp"
#include "irmv/web/record_query.hpp"
#include "support/check.hpp"

namespace {

using Json = nlohmann::json;
using irmv::protocol::ExecutionState;
using irmv::protocol::Id;
using irmv::protocol::OutputAction;
using irmv::protocol::StopReason;
using irmv::telemetry::RecordedFault;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;
using irmv::telemetry::RecordRange;
using irmv::telemetry::RecordReader;
using irmv::telemetry::RecordSample;
using irmv::telemetry::RecordSessionEvent;
using irmv::telemetry::VelocityRecorder;
using irmv::telemetry::VelocitySample;
using irmv::web::RecordQuery;

constexpr std::uint64_t kNever = static_cast<std::uint64_t>(-1);

std::filesystem::path MakeRoot() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path root =
      std::filesystem::temp_directory_path() /
      ("irmv-web-query-test-" + std::to_string(now));
  std::filesystem::create_directories(root);
  return root;
}

Id MakeId(std::uint8_t fill) {
  Id id{};
  id.fill(fill);
  return id;
}

std::string Hex(const Id &id) {
  static const char kDigits[] = "0123456789abcdef";
  std::string text;
  for (const std::uint8_t byte : id) {
    text.push_back(kDigits[byte >> 4]);
    text.push_back(kDigits[byte & 0x0FU]);
  }
  return text;
}

std::string EscapeJoint(const std::string &name) {
  static const char kDigits[] = "0123456789ABCDEF";
  std::string escaped;
  for (const char raw : name) {
    const unsigned char byte = static_cast<unsigned char>(raw);
    if ((byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z') ||
        (byte >= '0' && byte <= '9') || raw == '_' || raw == '-' ||
        raw == '.') {
      escaped.push_back(raw);
    } else {
      escaped.push_back('%');
      escaped.push_back(kDigits[byte >> 4]);
      escaped.push_back(kDigits[byte & 0x0FU]);
    }
  }
  return escaped;
}

Json ReadFixture(const char *path) {
  std::ifstream file(path);
  CHECK(file.good());
  Json document = Json::parse(file, nullptr, false);
  CHECK(document.is_object());
  return document;
}

RecorderConfig MakeRecorderConfig(const std::filesystem::path &directory) {
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.stride = 1;
  config.queue_capacity = 64;
  config.directory = directory;
  return config;
}

std::string RuntimeMeta(const Json &contract,
                        const std::vector<std::string> &joints,
                        const std::string &boot_id) {
  Json metadata;
  metadata["schema_version"] = irmv::telemetry::kRecordSchemaVersion;
  metadata["config_id"] = std::string(64, 'a');
  metadata["layout_id"] = std::string(64, 'b');
  metadata["backend"] = "fake";
  metadata["control_hz"] = 500.0;
  metadata["sample_hz"] = 100;
  metadata["joint_count"] = joints.size();
  metadata["joints"] = joints;
  metadata["sdk_git_sha"] = contract["sdk_git_sha"];
  metadata["clock"] = {
      {"clock", contract["clock"]}, {"boot_id", boot_id}, {"available", true}};
  metadata["units"] = contract["units"];
  metadata["imu"] = {{"orientation_rpy",
                      {{"order", "roll,pitch,yaw"},
                       {"unit", "rad"},
                       {"axis_frame", "unknown"}}}};
  metadata["row"] = "one control tick";
  return metadata.dump(1);
}

bool WriteRuntime(const std::filesystem::path &root, const Json &contract,
                  const std::vector<std::string> &joints,
                  const std::string &boot_id, std::uint64_t last_seq,
                  std::uint64_t last_stamp, std::uint8_t id_fill,
                  std::uint8_t session_fill, std::filesystem::path *directory) {
  Recorder recorder;
  std::string error;
  if (!recorder.Start(MakeRecorderConfig(root),
                      RuntimeMeta(contract, joints, boot_id), &error)) {
    std::cerr << "runtime recorder: " << error << '\n';
    return false;
  }
  for (int offset = 1; offset >= 0; --offset) {
    RecordSample sample;
    sample.stamp_us = last_stamp - static_cast<std::uint64_t>(2 * offset);
    sample.state_seq = last_seq - static_cast<std::uint64_t>(offset);
    sample.execution_state = ExecutionState::Tracking;
    sample.output_action = OutputAction::Tracking;
    sample.stop_reason = StopReason::None;
    sample.fault_id = 7;
    sample.healthy = true;
    sample.damping_eligible = true;
    sample.command_present = true;
    sample.producer_id[0] = 0x5A;
    sample.epoch = 9;
    sample.command_seq = sample.state_seq;
    sample.source_state_seq = sample.state_seq;
    sample.command_rx_us = sample.stamp_us + 3;
    sample.feedback_seq = sample.state_seq;
    sample.feedback_stamp_us = sample.stamp_us - 100;
    sample.feedback_valid = true;
    sample.imu_valid = true;
    sample.imu_orientation_rpy = {0.11, -0.22, 0.33};
    sample.imu_angular_velocity = {1.1, -2.2, 3.3};
    sample.imu_linear_acceleration = {4.4, -5.5, 6.6};
    sample.imu_rx_stamp_us = sample.stamp_us + 7;
    sample.send_accepted = true;
    sample.output_valid = true;
    sample.joint_count = static_cast<std::uint32_t>(joints.size());
    for (std::uint32_t index = 0; index < joints.size(); ++index) {
      sample.q[index] = 0.5 + 0.01 * index;
      sample.dq[index] = 0.25;
      sample.tau_ff[index] = 0.125;
      sample.kp[index] = 50.0;
      sample.kd[index] = 2.0;
      sample.measured_q[index] = 0.75 + 0.02 * index;
      sample.measured_dq[index] = 0.35;
      sample.measured_tau_est[index] = 0.45;
      sample.output_torque[index] = 0.15;
      sample.joint_valid[index] = true;
    }
    // A value behind a false validity flag is still the raw value. The bridge
    // must never replace it with a valid zero.
    sample.measured_q[0] = 987.654;
    sample.joint_valid[0] = false;
    if (!recorder.TryRecord(sample)) {
      std::cerr << "runtime recorder refused a sample\n";
      return false;
    }
  }
  RecordSessionEvent session;
  session.runtime_id = MakeId(id_fill);
  session.session_id = MakeId(session_fill);
  session.producer_id = MakeId(0x33);
  session.epoch = 1;
  session.stamp_us = last_stamp;
  session.state_seq = last_seq;
  session.joint_count = static_cast<std::uint32_t>(joints.size());
  session.kp[0] = 50.0;
  session.kd[0] = 2.0;
  std::memcpy(session.model_sha256.data(), "unknown", sizeof("unknown"));
  if (!recorder.TryRecordSession(session)) {
    std::cerr << "runtime recorder refused a session\n";
    return false;
  }
  if (!recorder.Stop()) {
    std::cerr << "runtime recorder did not stop\n";
    return false;
  }
  *directory = recorder.run_directory();
  return true;
}

std::string ApplicationMeta(const Json &contract, bool matching_boot,
                            std::uint64_t last_state_seq,
                            std::uint64_t last_stamp) {
  Json metadata;
  metadata["schema"] = std::string(irmv::telemetry::kVelocitySchema);
  metadata["version"] = irmv::telemetry::kVelocitySchemaVersion;
  metadata["sdk_git_sha"] = contract["sdk_git_sha"];
  metadata["application_config_id"] = std::string(64, 'c');
  metadata["config_id"] = std::string(64, 'a');
  metadata["layout_id"] = std::string(64, 'b');
  metadata["joint_count"] = 3;
  metadata["sample_hz"] = 100;
  metadata["loop_period_us"] = 10000;
  metadata["sampling"] = {{"paced_by_deadline", false}};
  metadata["source"] = "local";
  metadata["clock"] = {
      {"clock", contract["clock"]},
      {"boot_id", matching_boot ? contract["boot_id"] : "boot-runtime-other"},
      {"available", true}};
  metadata["units"] = {
      {"forward", "m_s"}, {"lateral", "m_s"}, {"yaw", "rad_s"}, {"time", "us"}};
  metadata["model"] = {{"status", "loaded"},
                       {"name", "policy.onnx"},
                       {"path", nullptr},
                       {"sha256", std::string(64, 'd')},
                       {"contract_sha256", std::string(64, 'e')}};
  metadata["row"] = "one application velocity sample";
  metadata["last_state_seq"] = last_state_seq;
  metadata["last_stamp_us"] = last_stamp;
  return metadata.dump(1);
}

bool WriteApplication(const std::filesystem::path &root, const Json &contract,
                      bool matching_boot, std::uint64_t last_state_seq,
                      std::uint64_t last_stamp,
                      std::filesystem::path *directory) {
  VelocityRecorder recorder;
  std::string error;
  irmv::configuration::RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.queue_capacity = 64;
  config.directory = root;
  if (!recorder.Start(
          config,
          ApplicationMeta(contract, matching_boot, last_state_seq, last_stamp),
          &error)) {
    std::cerr << "velocity recorder: " << error << '\n';
    return false;
  }
  for (std::uint64_t seq = 0; seq < 2; ++seq) {
    VelocitySample sample;
    sample.stamp_us = last_stamp - 100 + 100 * seq;
    sample.sample_seq = seq;
    sample.identity_known = true;
    sample.runtime_id = MakeId(0x11);
    sample.session_id = MakeId(0x22);
    sample.latest_state_seq = seq == 0 ? last_state_seq - 1 : last_state_seq;
    sample.forward = 0.5 + 0.1 * seq;
    sample.lateral = -0.25;
    sample.yaw = 0.125;
    if (!recorder.TryRecord(sample)) {
      std::cerr << "velocity recorder refused a sample\n";
      return false;
    }
  }
  if (!recorder.Stop()) {
    std::cerr << "velocity recorder did not stop\n";
    return false;
  }
  *directory = recorder.run_directory();
  return true;
}

Json Call(RecordQuery *query, Json request) {
  if (!request.contains("request_id")) {
    request["request_id"] = "req";
  }
  const std::string text = query->Execute(request.dump());
  Json reply = Json::parse(text, nullptr, false);
  CHECK(!reply.is_discarded());
  CHECK(reply.value("request_id", std::string()) ==
        request["request_id"].get<std::string>());
  return reply;
}

const Json *FindSignal(const Json &context, const std::string &id) {
  for (const Json &signal : context["signals"]) {
    if (signal.value("id", std::string()) == id) {
      return &signal;
    }
  }
  return nullptr;
}

const Json *FindRun(const Json &reply, const std::string &name) {
  for (const Json &run : reply["runs"]) {
    if (run.value("name", std::string()) == name) {
      return &run;
    }
  }
  return nullptr;
}

std::string RunIdByName(const Json &reply, const std::string &name) {
  const Json *run = FindRun(reply, name);
  if (run == nullptr) {
    return std::string();
  }
  return run->value("run_id", std::string());
}

int CheckRuntimeSignals(const Json &context, const Json &contract,
                        const std::vector<std::string> &joints) {
  for (const Json &expected : contract["signals"]) {
    const std::string field = expected["field"].get<std::string>();
    if (expected.value("joint", false)) {
      for (const std::string &joint : joints) {
        const std::string id = "runtime/" + EscapeJoint(joint) + "/" + field;
        const Json *signal = FindSignal(context, id);
        CHECK(signal != nullptr);
        CHECK((*signal)["unit"] == expected["unit"]);
        CHECK((*signal)["quantity"] == expected["quantity"]);
        CHECK((*signal)["value_kind"] == expected["value_kind"]);
        CHECK((*signal)["validity"] == expected["validity"]);
        CHECK((*signal)["source"] == "runtime");
        CHECK((*signal)["joint_name"] == joint);
      }
    } else {
      const std::string id = "runtime//" + field;
      const Json *signal = FindSignal(context, id);
      CHECK(signal != nullptr);
      CHECK((*signal)["unit"] == expected["unit"]);
      CHECK((*signal)["quantity"] == expected["quantity"]);
      CHECK((*signal)["value_kind"] == expected["value_kind"]);
      CHECK((*signal)["validity"] == expected["validity"]);
    }
  }
  for (const Json &signal : context["signals"]) {
    const std::string field = signal["field"].get<std::string>();
    CHECK(field.find("temperature") == std::string::npos);
  }
  return 0;
}

int CheckApplicationSignals(const Json &context, const Json &contract) {
  for (const Json &expected : contract["signals"]) {
    const std::string id =
        "application//" + expected["field"].get<std::string>();
    const Json *signal = FindSignal(context, id);
    CHECK(signal != nullptr);
    CHECK((*signal)["unit"] == expected["unit"]);
    CHECK((*signal)["quantity"] == expected["quantity"]);
    CHECK((*signal)["value_kind"] == expected["value_kind"]);
    CHECK((*signal)["validity"] == expected["validity"]);
    CHECK((*signal)["source"] == "application");
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  const char *fixture_path =
      argc > 1 ? argv[1] : "tests/support/web_query_contract.json";
  const Json fixture = ReadFixture(fixture_path);
  const Json &runtime_contract = fixture["runtime"];
  const Json &application_contract = fixture["application"];
  const std::vector<std::string> joints =
      runtime_contract["joints"].get<std::vector<std::string>>();

  const std::filesystem::path root = MakeRoot();
  std::filesystem::path runtime_a;
  std::filesystem::path runtime_b;
  std::filesystem::path application_a;
  std::filesystem::path application_b;
  CHECK(WriteRuntime(root, runtime_contract, joints,
                     runtime_contract["boot_id"], 9007199254740993ULL,
                     9007199254740995ULL, 0x11, 0x22, &runtime_a));
  std::vector<std::string> reordered = joints;
  std::reverse(reordered.begin(), reordered.end());
  CHECK(WriteRuntime(root, runtime_contract, reordered, "boot-runtime-b", 101,
                     2000, 0x44, 0x55, &runtime_b));
  CHECK(WriteApplication(root, application_contract, true, 42, 5000,
                         &application_a));
  CHECK(WriteApplication(root, application_contract, false, 43, 6000,
                         &application_b));

  // A run of another schema is listed as unreadable, never reinterpreted.
  const std::filesystem::path old_run = root / "run_schema3";
  CHECK(std::filesystem::create_directory(old_run));
  {
    std::ofstream metadata(old_run / "meta.json");
    CHECK(metadata.good());
    metadata << R"({"schema_version":3,"joint_count":1,"joints":["j"]})";
  }

  // A symlink that leaves the configured root is not a run of this workbench.
  const std::filesystem::path outside =
      root.parent_path() / (root.filename().string() + "-outside");
  std::filesystem::remove_all(outside);
  CHECK(std::filesystem::create_directory(outside));
  {
    std::ofstream metadata(outside / "meta.json");
    CHECK(metadata.good());
    metadata << R"({"schema_version":4,"joint_count":1,"joints":["j"]})";
  }
  std::error_code link_code;
  std::filesystem::create_symlink(outside, root / "run_escaped", link_code);
  CHECK(!link_code);

  RecordQuery query(root);
  Json catalog = Call(&query, {{"op", "describe"}});
  CHECK(catalog["ok"] == true);
  CHECK(catalog["runs"].is_array());
  CHECK(RunIdByName(catalog, "run_escaped").empty());
  CHECK(!RunIdByName(catalog, runtime_a.filename().string()).empty());
  CHECK(!RunIdByName(catalog, runtime_b.filename().string()).empty());
  CHECK(!RunIdByName(catalog, application_a.filename().string()).empty());
  const Json *old_entry = FindRun(catalog, "run_schema3");
  CHECK(old_entry != nullptr);
  CHECK((*old_entry)["readable"] == false);
  CHECK((*old_entry)["error"].get<std::string>().find("schema version 3") !=
        std::string::npos);

  const std::string runtime_a_id =
      RunIdByName(catalog, runtime_a.filename().string());
  const std::string runtime_b_id =
      RunIdByName(catalog, runtime_b.filename().string());
  const std::string application_a_id =
      RunIdByName(catalog, application_a.filename().string());
  const std::string application_b_id =
      RunIdByName(catalog, application_b.filename().string());

  Json described = Call(&query, {{"op", "describe"}, {"run_id", runtime_a_id}});
  CHECK(described["ok"] == true);
  const Json &context = described["context"];
  CHECK(context["kind"] == "runtime");
  CHECK(context["state_seq"] == "9007199254740993");
  CHECK(context["stamp_us"] == "9007199254740995");
  CHECK(context["sdk_git_sha"] == runtime_contract["sdk_git_sha"]);
  CHECK(context["clock"]["boot_id"] == runtime_contract["boot_id"]);
  CHECK(context["joints"] == joints);
  CHECK(CheckRuntimeSignals(context, runtime_contract, joints) == 0);

  // Reordering joints in another run must not change a semantic identity.
  Json described_b =
      Call(&query, {{"op", "describe"}, {"run_id", runtime_b_id}});
  CHECK(described_b["ok"] == true);
  CHECK(CheckRuntimeSignals(described_b["context"], runtime_contract,
                            reordered) == 0);
  for (const std::string &joint : joints) {
    CHECK(FindSignal(described["context"],
                     "runtime/" + EscapeJoint(joint) + "/q") != nullptr);
    CHECK(FindSignal(described_b["context"],
                     "runtime/" + EscapeJoint(joint) + "/q") != nullptr);
  }

  Json described_app =
      Call(&query, {{"op", "describe"}, {"run_id", application_a_id}});
  CHECK(described_app["ok"] == true);
  CHECK(described_app["context"]["kind"] == "application");
  CHECK(described_app["context"]["state_seq"] ==
        application_contract["state_seq"]);
  CHECK(described_app["context"]["stamp_us"] ==
        application_contract["stamp_us"]);
  CHECK(CheckApplicationSignals(described_app["context"],
                                application_contract) == 0);

  Json associated = Call(&query, {{"op", "context"},
                                  {"run_id", runtime_a_id},
                                  {"app_run_id", application_a_id}});
  CHECK(associated["ok"] == true);
  CHECK(associated["context"]["association"]["boot_id_match"] == true);
  CHECK(associated["context"]["association"]["identity_match"] == true);
  CHECK(associated["context"]["association"]["ok"] == true);
  CHECK(associated["context"]["application"]["run_id"] == application_a_id);

  Json mismatched = Call(&query, {{"op", "context"},
                                  {"run_id", runtime_a_id},
                                  {"app_run_id", application_b_id}});
  CHECK(mismatched["ok"] == true);
  CHECK(mismatched["context"]["association"]["boot_id_match"] == false);
  CHECK(mismatched["context"]["association"]["ok"] == false);
  CHECK(mismatched["context"]["association"]["reason"] == "boot_id mismatch");

  Json refused = Call(
      &query,
      {{"op", "describe"}, {"run_id", RunIdByName(catalog, "run_schema3")}});
  CHECK(refused["ok"] == false);
  CHECK(refused["error"].get<std::string>().find("schema version 3") !=
        std::string::npos);

  Json unimplemented =
      Call(&query, {{"op", "subscribe"}, {"run_id", runtime_a_id}});
  CHECK(unimplemented["ok"] == false);
  CHECK(unimplemented["error"].get<std::string>().find("not implemented") !=
        std::string::npos);

  // The raw reader still holds the invalid value exactly: the catalog's
  // validity contract is about meaning, not about rewriting bytes.
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(runtime_a, &reader, &error));
  RecordRange range;
  CHECK(reader.ReadRange(0, kNever, &range));
  CHECK(range.samples.size() == 2);
  CHECK(range.samples.back().state_seq == 9007199254740993ULL);
  CHECK(range.samples.back().stamp_us == 9007199254740995ULL);
  CHECK(range.samples.back().measured_q[0] == 987.654);
  CHECK(!range.samples.back().joint_valid[0]);

  // Simulation clock proof augments, never replaces runtime/session proof.
  Id simulation_run{};
  simulation_run.fill(0x12);
  const Json sim_clock = Json::parse(irmv::telemetry::EncodeRecordClock(
      irmv::telemetry::SimulationRecordClock(simulation_run, 1000, 2000)));
  auto set_clock = [](const std::filesystem::path &dir, const Json &clock) {
    Json meta;
    {
      std::ifstream input(dir / "meta.json");
      input >> meta;
    }
    meta["clock"] = clock;
    std::ofstream output(dir / "meta.json");
    output << meta.dump();
  };
  set_clock(runtime_a, sim_clock);
  set_clock(application_a, sim_clock);
  auto sim_association = [&]() {
    RecordQuery fresh(root);
    return Call(&fresh,
                {{"op", "context"},
                 {"run_id", runtime_a_id},
                 {"app_run_id", application_a_id}})["context"]["association"];
  };
  CHECK(sim_association()["ok"] == true);
  for (const char *key : {"clock_run_id", "logical_origin_us", "physics_dt_us",
                          "control_dt_us", "clock_domain"}) {
    Json foreign = sim_clock;
    if (std::string(key) == "clock_run_id")
      foreign[key] = std::string(32, 'b');
    else if (std::string(key) == "clock_domain")
      foreign[key] = "host_monotonic";
    else
      foreign[key] = sim_clock[key].get<std::uint64_t>() * 2;
    set_clock(application_a, foreign);
    CHECK(sim_association()["ok"] == false);
  }
  set_clock(application_a, sim_clock);
  CHECK(sim_association()["ok"] == true);
  // C++ metadata exposes extension, while old v4 metadata above stayed
  // readable.
  RecordReader simulation_reader;
  CHECK(RecordReader::Open(runtime_a, &simulation_reader, &error));
  CHECK(simulation_reader.metadata().clock.clock_domain == "simulation");
  CHECK(simulation_reader.metadata().clock.logical_origin_us == 1);

  {
    // A metadata reference must not bypass the web records-root boundary.
    const auto path = outside / "simulation.jsonl";
    std::ofstream sidecar(path);
    sidecar << Json{{"schema", "irmv.simulation_evidence"},
                    {"version", 1},
                    {"record_type", "header"},
                    {"clock", sim_clock}}
                   .dump()
            << '\n';
    sidecar
        << R"({"record_type":"footer","offered":0,"written":0,"dropped":0,"write_failures":0})"
        << '\n';
    sidecar.close();
    Json metadata;
    {
      std::ifstream input(runtime_a / "meta.json");
      input >> metadata;
    }
    metadata["simulation"] = {
        {"sidecar",
         std::filesystem::relative(path, runtime_a).generic_string()}};
    {
      std::ofstream output(runtime_a / "meta.json");
      output << metadata.dump();
    }
    RecordQuery bounded(root);
    const auto described =
        Call(&bounded, {{"op", "context"}, {"run_id", runtime_a_id}});
    CHECK(described["context"]["simulation_evidence"]["evidence_complete"] ==
          false);
    CHECK(described["context"]["simulation_evidence"]["error"] ==
          "invalid simulation reference");
  }

  std::filesystem::remove_all(root);
  std::filesystem::remove_all(outside);
  return 0;
}
