// Bounded range, exact cursor and paged fault queries of the experiment web
// bridge.
//
// The test writes real recorder runs, so the gaps, loss spans, fault lines and
// chunk checksums are the ones this build actually produces. Pure model checks
// cover the reduction budget and the held-forward boundary, which must not
// depend on a file at all.
#include "irmv/web/record_query.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/fault_record.hpp"
#include "irmv/telemetry/record.hpp"
#include "irmv/telemetry/record_reader.hpp"
#include "irmv/telemetry/recorder.hpp"
#include "irmv/telemetry/velocity_record.hpp"
#include "irmv/web/range_reduce.hpp"
#include "support/check.hpp"

namespace {

using Json = nlohmann::json;
using irmv::protocol::ExecutionState;
using irmv::protocol::FaultCondition;
using irmv::protocol::FaultReason;
using irmv::protocol::Id;
using irmv::protocol::OutputAction;
using irmv::protocol::StopReason;
using irmv::telemetry::RecordedFault;
using irmv::telemetry::RecordRange;
using irmv::telemetry::RecordReader;
using irmv::telemetry::RecordSample;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;
using irmv::telemetry::VelocitySample;
using irmv::web::HoldVelocityAt;
using irmv::web::RangeReducer;
using irmv::web::RecordQuery;
using irmv::web::SeriesPoint;
using irmv::web::VelocityField;

constexpr std::uint64_t kNever = static_cast<std::uint64_t>(-1);

std::filesystem::path MakeRoot() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path root =
      std::filesystem::temp_directory_path() /
      ("irmv-web-range-test-" + std::to_string(now));
  std::filesystem::create_directories(root);
  return root;
}

Id MakeId(std::uint8_t fill) {
  Id id{};
  id.fill(fill);
  return id;
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

std::string RuntimeMeta(const std::vector<std::string> &joints,
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
  metadata["sdk_git_sha"] = "range-test-sha";
  metadata["clock"] = {{"clock", "linux_clock_monotonic"},
                       {"boot_id", boot_id},
                       {"available", true}};
  metadata["units"] = {{"position", "rad"},
                       {"velocity", "rad_s"},
                       {"torque", "Nm"},
                       {"kp", "Nm_per_rad"},
                       {"kd", "Nm_s_per_rad"},
                       {"time", "us"},
                       {"imu_orientation_rpy", "rad"},
                       {"imu_angular_velocity", "rad_s"},
                       {"imu_linear_acceleration", "m_s2"}};
  metadata["imu"] = {{"orientation_rpy", {{"axis_frame", "unknown"}}}};
  return metadata.dump(1);
}

RecordSample MakeSample(std::uint64_t stamp_us, std::uint64_t state_seq,
                        const std::vector<std::string> &joints) {
  RecordSample sample;
  sample.stamp_us = stamp_us;
  sample.state_seq = state_seq;
  sample.execution_state = ExecutionState::Tracking;
  sample.output_action = OutputAction::Tracking;
  sample.stop_reason = StopReason::None;
  sample.healthy = true;
  sample.command_present = true;
  sample.feedback_valid = true;
  sample.imu_valid = true;
  sample.send_accepted = true;
  sample.output_valid = true;
  sample.joint_count = static_cast<std::uint32_t>(joints.size());
  for (std::uint32_t index = 0; index < joints.size(); ++index) {
    sample.q[index] = 0.0;
    sample.dq[index] = 0.0;
    sample.tau_ff[index] = 0.0;
    sample.kp[index] = 1.0;
    sample.kd[index] = 0.1;
    sample.measured_q[index] = 0.0;
    sample.measured_dq[index] = 0.0;
    sample.measured_tau_est[index] = 0.0;
    sample.output_torque[index] = 0.0;
    sample.joint_valid[index] = true;
  }
  return sample;
}

bool WriteRuntime(const std::filesystem::path &root,
                  const std::vector<std::string> &joints,
                  const std::vector<RecordSample> &samples,
                  const std::vector<RecordedFault> &faults,
                  std::filesystem::path *directory) {
  Recorder recorder;
  std::string error;
  if (!recorder.Start(MakeRecorderConfig(root),
                      RuntimeMeta(joints, "boot-range"), &error)) {
    std::cerr << "runtime recorder: " << error << '\n';
    return false;
  }
  for (const RecordSample &sample : samples) {
    if (!recorder.TryRecord(sample)) {
      std::cerr << "runtime recorder refused a sample\n";
      return false;
    }
  }
  for (const RecordedFault &fault : faults) {
    if (!recorder.TryRecordFault(fault)) {
      std::cerr << "runtime recorder refused a fault\n";
      return false;
    }
  }
  if (!recorder.Stop()) {
    std::cerr << "runtime recorder did not stop\n";
    return false;
  }
  *directory = recorder.run_directory();
  return true;
}

bool WriteApplication(const std::filesystem::path &root,
                      const std::vector<VelocitySample> &rows,
                      std::filesystem::path *directory) {
  Json metadata;
  metadata["schema"] = std::string(irmv::telemetry::kVelocitySchema);
  metadata["version"] = irmv::telemetry::kVelocitySchemaVersion;
  metadata["sdk_git_sha"] = "range-test-sha";
  metadata["application_config_id"] = std::string(64, 'c');
  metadata["config_id"] = std::string(64, 'a');
  metadata["layout_id"] = std::string(64, 'b');
  metadata["joint_count"] = 1;
  metadata["sample_hz"] = 100;
  metadata["loop_period_us"] = 10000;
  metadata["source"] = "local";
  metadata["clock"] = {{"clock", "linux_clock_monotonic"},
                       {"boot_id", "boot-range"},
                       {"available", true}};
  metadata["units"] = {{"forward", "m_s"},
                       {"lateral", "m_s"},
                       {"yaw", "rad_s"},
                       {"time", "us"}};
  metadata["model"] = {{"status", "loaded"}, {"name", "policy.onnx"}};

  irmv::telemetry::VelocityRecorder recorder;
  irmv::configuration::RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.queue_capacity = 64;
  config.directory = root;
  std::string error;
  if (!recorder.Start(config, metadata.dump(1), &error)) {
    std::cerr << "velocity recorder: " << error << '\n';
    return false;
  }
  for (const VelocitySample &row : rows) {
    if (!recorder.TryRecord(row)) {
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
  request["request_id"] = "req";
  const std::string text = query->Execute(request.dump());
  Json reply = Json::parse(text, nullptr, false);
  CHECK(!reply.is_discarded());
  return reply;
}

std::string RunIdByName(const Json &catalog, const std::string &name) {
  for (const Json &run : catalog["runs"]) {
    if (run.value("name", std::string()) == name) {
      return run.value("run_id", std::string());
    }
  }
  return std::string();
}

RecordedFault MakeFault(std::uint64_t seq, std::uint64_t stamp,
                        double observed, const std::string &message) {
  RecordedFault fault;
  fault.record_seq = seq;
  fault.runtime_id = MakeId(0x11);
  fault.state_seq = 100 + seq;
  fault.fault_id = seq;
  fault.detail.reason = FaultReason::JointPositionExceeded;
  fault.detail.condition = FaultCondition::JointPositionExceeded;
  fault.detail.joint_index = 0;
  fault.detail.error_code = static_cast<std::uint32_t>(0xDEAD0000 + seq);
  fault.detail.observed_value = observed;
  fault.detail.limit = 1.0;
  fault.detail.sample_stamp_us = stamp - 10;
  fault.detail.detected_stamp_us = stamp;
  fault.detail.output_action = OutputAction::ZeroTorque;
  fault.detail.message.Assign(message);
  return fault;
}

int CheckReductionBudget() {
  RangeReducer reducer(0, 100000, 10);
  for (int index = 0; index < 1000; ++index) {
    reducer.Push(SeriesPoint{static_cast<std::uint64_t>(index) * 100,
                             static_cast<std::uint64_t>(index),
                             static_cast<double>(index % 7), true, 0});
  }
  const auto series = reducer.Finish();
  CHECK(series.points.size() <= 40);
  CHECK(series.input_points == 1000);
  return 0;
}

int CheckNarrowSpike() {
  RangeReducer reducer(0, 1000000, 1);
  reducer.Push(SeriesPoint{100, 0, 1.0, true, 0});
  reducer.Push(SeriesPoint{500, 1, 5.0, true, 0});
  reducer.Push(SeriesPoint{900, 2, 2.0, true, 0});
  const auto series = reducer.Finish();
  CHECK(series.points.size() <= 4);
  bool found = false;
  for (const auto &point : series.points) {
    if (point.stamp_us == 500 && point.value == 5.0) {
      found = true;
      CHECK(point.aggregation.find("max") != std::string::npos);
    }
  }
  CHECK(found);
  return 0;
}

int CheckHoldBoundary() {
  std::vector<VelocitySample> rows(3);
  rows[0].stamp_us = 1000000;
  rows[0].sample_seq = 0;
  rows[0].identity_known = true;
  rows[0].runtime_id = MakeId(1);
  rows[0].session_id = MakeId(2);
  rows[0].forward = 1.0;
  rows[1] = rows[0];
  rows[1].stamp_us = 1019000;
  rows[1].sample_seq = 1;
  rows[1].forward = 2.0;
  rows[2] = rows[0];
  rows[2].stamp_us = 1030001;
  rows[2].sample_seq = 2;
  rows[2].forward = 2.2;
  irmv::telemetry::VelocityLossSpan loss;
  loss.first_stamp_us = 1019001;
  loss.last_stamp_us = 1025000;
  loss.first_sample_seq = 2;
  loss.last_sample_seq = 2;
  std::vector<irmv::telemetry::VelocityLossSpan> losses{loss};
  const auto at_1019000 =
      HoldVelocityAt(rows, losses, 1019000, 10, VelocityField::Forward);
  CHECK(at_1019000.found);
  CHECK(at_1019000.value == 2.0);
  CHECK(!HoldVelocityAt(rows, losses, 1020000, 10, VelocityField::Forward)
             .found);
  CHECK(!HoldVelocityAt(rows, losses, 1022000, 10, VelocityField::Forward)
             .found);
  const auto at_1040000 =
      HoldVelocityAt(rows, losses, 1040000, 10, VelocityField::Forward);
  CHECK(at_1040000.found);
  CHECK(at_1040000.value == 2.2);
  std::reverse(losses.begin(), losses.end());
  CHECK(HoldVelocityAt(rows, losses, 1019000, 10, VelocityField::Forward)
            .value == 2.0);
  CHECK(!HoldVelocityAt(rows, losses, 1020000, 10, VelocityField::Forward)
             .found);
  CHECK(HoldVelocityAt(rows, losses, 1040000, 10, VelocityField::Forward)
            .value == 2.2);
  return 0;
}

}  // namespace

int main(int /*argc*/, char ** /*argv*/) {
  CHECK(CheckReductionBudget() == 0);
  CHECK(CheckNarrowSpike() == 0);
  CHECK(CheckHoldBoundary() == 0);

  const std::filesystem::path root = MakeRoot();
  const std::vector<std::string> joints{"left_hip", "right_hip"};

  // A cursor run: the middle bucket holds a much larger value than the sample
  // nearest the cursor. A cursor must return the raw sample, not the bucket max.
  std::vector<RecordSample> cursor_samples;
  cursor_samples.push_back(MakeSample(1000, 1, joints));
  cursor_samples.back().q[0] = 1.0;
  cursor_samples.push_back(MakeSample(2000, 2, joints));
  cursor_samples.back().q[0] = 2.0;
  cursor_samples.push_back(MakeSample(4000, 3, joints));
  cursor_samples.back().q[0] = 100.0;
  std::filesystem::path cursor_run;
  CHECK(WriteRuntime(root, joints, cursor_samples, {}, &cursor_run));

  // A clean fault stream: the recorder creates faults.jsonl even when no fault
  // is ever reported, so an empty complete stream is distinguishable from a
  // missing file.
  std::filesystem::path empty_faults_run;
  CHECK(WriteRuntime(root, joints, {MakeSample(1000, 1, joints)}, {},
                     &empty_faults_run));

  // Five occurrences at the same tick: pagination must neither drop nor repeat
  // one of them.
  std::vector<RecordedFault> same_tick;
  for (std::uint64_t seq = 0; seq < 5; ++seq) {
    same_tick.push_back(MakeFault(
        seq, 1234, seq == 0 ? std::nan("") : static_cast<double>(seq),
        "same tick " + std::to_string(seq)));
  }
  std::filesystem::path fault_run;
  CHECK(WriteRuntime(root, joints, {MakeSample(1234, 1, joints)}, same_tick,
                     &fault_run));

  // A run with exactly one committed chunk, so corrupting its payload is an
  // unambiguous whole-chunk refusal.
  std::filesystem::path corrupt_run;
  CHECK(WriteRuntime(root, joints, {MakeSample(1000, 1, joints)}, {},
                     &corrupt_run));
  {
    std::filesystem::path chunk;
    for (const auto &entry : std::filesystem::directory_iterator(corrupt_run)) {
      if (entry.path().extension() == ".bin") {
        chunk = entry.path();
      }
    }
    CHECK(!chunk.empty());
    std::fstream file(chunk, std::ios::binary | std::ios::in | std::ios::out);
    CHECK(file.good());
    file.seekg(-1, std::ios::end);
    char byte = 0;
    file.get(byte);
    byte = static_cast<char>(byte ^ 0x5A);
    file.clear();
    file.seekp(-1, std::ios::end);
    file.put(byte);
    file.close();
    RecordReader reader;
    std::string error;
    CHECK(RecordReader::Open(corrupt_run, &reader, &error));
    RecordRange range;
    CHECK(reader.ReadRange(0, kNever, &range));
    CHECK(range.corrupt);
    CHECK(!range.error.empty());
    CHECK(range.samples.empty());
  }

  std::vector<VelocitySample> app_rows(2);
  app_rows[0].stamp_us = 1000;
  app_rows[0].sample_seq = 0;
  app_rows[0].identity_known = true;
  app_rows[0].runtime_id = MakeId(0x11);
  app_rows[0].session_id = MakeId(0x22);
  app_rows[0].latest_state_seq = 10;
  app_rows[0].forward = 0.1;
  app_rows[0].lateral = -0.2;
  app_rows[0].yaw = 0.3;
  app_rows[1] = app_rows[0];
  app_rows[1].stamp_us = 2000;
  app_rows[1].sample_seq = 1;
  app_rows[1].latest_state_seq = 11;
  app_rows[1].forward = 0.2;
  std::filesystem::path app_run;
  CHECK(WriteApplication(root, app_rows, &app_run));

  RecordQuery query(root);
  Json catalog = Call(&query, {{"op", "describe"}});
  CHECK(catalog["ok"] == true);
  const std::string cursor_id =
      RunIdByName(catalog, cursor_run.filename().string());
  const std::string empty_faults_id =
      RunIdByName(catalog, empty_faults_run.filename().string());
  const std::string fault_id = RunIdByName(catalog, fault_run.filename().string());
  const std::string corrupt_id =
      RunIdByName(catalog, corrupt_run.filename().string());
  const std::string app_id = RunIdByName(catalog, app_run.filename().string());
  CHECK(!cursor_id.empty());
  CHECK(!empty_faults_id.empty());
  CHECK(!fault_id.empty());
  CHECK(!corrupt_id.empty());
  CHECK(!app_id.empty());

  // Range: half-open, bounded, and the narrow spike keeps its own timestamp.
  Json range = Call(&query, {{"op", "range"},
                             {"run_id", cursor_id},
                             {"signal_ids", {"runtime/left_hip/q"}},
                             {"begin_us", "1000"},
                             {"end_us", "5000"},
                             {"pixel_width", 1}});
  CHECK(range["ok"] == true);
  CHECK(range["series"].size() == 1);
  CHECK(range["series"][0]["points"].size() <= 4);
  bool saw_high = false;
  for (const Json &point : range["series"][0]["points"]) {
    if (point["t"] == "4000") {
      saw_high = point["v"] == 100.0;
    }
  }
  CHECK(saw_high);

  Json exclusive = Call(&query, {{"op", "range"},
                                 {"run_id", cursor_id},
                                 {"signal_ids", {"runtime/left_hip/q"}},
                                 {"begin_us", "1000"},
                                 {"end_us", "2000"},
                                 {"pixel_width", 10}});
  CHECK(exclusive["ok"] == true);
  CHECK(exclusive["series"][0]["points"].size() == 1);
  CHECK(exclusive["series"][0]["points"][0]["t"] == "1000");

  // Cursor: the raw sample nearest before the cursor, never the bucket max.
  Json cursor = Call(&query, {{"op", "cursor"},
                              {"run_id", cursor_id},
                              {"signal_ids", {"runtime/left_hip/q"}},
                              {"cursor_us", "3500"}});
  CHECK(cursor["ok"] == true);
  CHECK(cursor["series"][0]["valid"] == true);
  CHECK(cursor["series"][0]["value"] == 2.0);
  CHECK(cursor["series"][0]["sample_stamp_us"] == "2000");
  CHECK(cursor["series"][0]["age_us"] == "1500");

  Json before = Call(&query, {{"op", "cursor"},
                              {"run_id", cursor_id},
                              {"signal_ids", {"runtime/left_hip/q"}},
                              {"cursor_us", "999"}});
  CHECK(before["ok"] == true);
  CHECK(before["series"][0]["valid"] == false);
  CHECK(before["series"][0]["value"].is_null());

  // Invalid raw values stay invalid: the cursor refuses the latest sample when
  // its validity flag is false, and the range carries valid=false.
  Json corrupt = Call(&query, {{"op", "range"},
                               {"run_id", corrupt_id},
                               {"signal_ids", {"runtime/left_hip/q"}},
                               {"begin_us", "0"},
                               {"end_us", "5000"},
                               {"pixel_width", 10}});
  CHECK(corrupt["ok"] == true);
  CHECK(corrupt["integrity"]["runtime"]["corrupt"] == true);
  CHECK(corrupt["series"][0]["points"].empty());

  // Application range and held-forward cursor.
  Json app_range = Call(&query, {{"op", "range"},
                                 {"run_id", cursor_id},
                                 {"app_run_id", app_id},
                                 {"signal_ids", {"application//forward"}},
                                 {"begin_us", "0"},
                                 {"end_us", "3000"},
                                 {"pixel_width", 10}});
  CHECK(app_range["ok"] == true);
  CHECK(app_range["series"][0]["points"].size() == 2);
  Json app_cursor = Call(&query, {{"op", "cursor"},
                                  {"run_id", cursor_id},
                                  {"app_run_id", app_id},
                                  {"signal_ids", {"application//forward"}},
                                  {"cursor_us", "2500"}});
  CHECK(app_cursor["ok"] == true);
  CHECK(app_cursor["series"][0]["valid"] == true);
  CHECK(app_cursor["series"][0]["value"] == 0.2);
  CHECK(app_cursor["series"][0]["age_us"] == "500");

  // Faults: a complete empty stream is a success; a missing file is not.
  Json empty_events = Call(&query, {{"op", "events"},
                                    {"run_id", empty_faults_id},
                                    {"begin_us", "0"},
                                    {"end_us", std::to_string(kNever)}});
  CHECK(empty_events["ok"] == true);
  CHECK(empty_events["events"].empty());
  CHECK(empty_events["integrity"]["faults"]["complete"] == true);

  CHECK(std::filesystem::remove(empty_faults_run / "faults.jsonl"));
  Json missing_events = Call(&query, {{"op", "events"},
                                      {"run_id", empty_faults_id},
                                      {"begin_us", "0"},
                                      {"end_us", std::to_string(kNever)}});
  CHECK(missing_events["ok"] == false);
  CHECK(missing_events["error"].get<std::string>().find("cannot find") !=
        std::string::npos);

  // Same-tick paging: every occurrence appears exactly once across pages.
  std::set<std::string> event_ids;
  std::string continuation;
  int pages = 0;
  do {
    Json page_request = {{"op", "events"},
                         {"run_id", fault_id},
                         {"begin_us", "0"},
                         {"end_us", std::to_string(kNever)},
                         {"max_rows", 2}};
    if (!continuation.empty()) {
      page_request["continuation"] = continuation;
    }
    Json page = Call(&query, page_request);
    CHECK(page["ok"] == true);
    for (const Json &event : page["events"]) {
      const std::string id = event["event_id"].get<std::string>();
      CHECK(event_ids.count(id) == 0);
      event_ids.insert(id);
      CHECK(event["detected_stamp_us"] == "1234");
    }
    continuation = page.contains("continuation")
                       ? page["continuation"].get<std::string>()
                       : std::string();
    ++pages;
  } while (!continuation.empty() && pages < 10);
  CHECK(pages == 3);
  CHECK(event_ids.size() == 5);

  Json detail = Call(&query, {{"op", "events"},
                              {"run_id", fault_id},
                              {"begin_us", "0"},
                              {"end_us", std::to_string(kNever)},
                              {"max_rows", 100}});
  CHECK(detail["ok"] == true);
  CHECK(detail["events"].size() == 5);
  CHECK(detail["events"][0]["observed_value"] == "NaN");
  CHECK(detail["events"][0]["joint_name"] == "left_hip");
  CHECK(detail["events"][0]["output_action_name"] == "ZeroTorque");
  CHECK(detail["events"][0]["message"] == "same tick 0");
  CHECK(!detail["events"][0]["message_hex"].get<std::string>().empty());

  Json bad_signal = Call(&query, {{"op", "range"},
                                  {"run_id", cursor_id},
                                  {"signal_ids", {"runtime/unknown/q"}},
                                  {"begin_us", "0"},
                                  {"end_us", "5000"},
                                  {"pixel_width", 10}});
  CHECK(bad_signal["ok"] == false);

  std::filesystem::remove_all(root);
  return 0;
}
