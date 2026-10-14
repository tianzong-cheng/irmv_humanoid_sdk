// The application-side policy session record: what the run was joined to.
//
// The entry checks the promises this record makes: a confirmed context
// round-trips through the sidecar with its join key, its model summary and its
// gains; the two fillers that build those gains name exactly the right rows; a
// full handoff costs the event rather than the control tick; and a directory
// that cannot be used is reported instead of thrown.
#include "irmv/application/policy_session_record.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "irmv/policy/action.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::FillFrameGains;
using irmv::application::FillGainOverrides;
using irmv::application::kPolicySessionFile;
using irmv::application::PolicyGainOverride;
using irmv::application::PolicySessionEvent;
using irmv::application::PolicySessionRecorder;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;

std::filesystem::path MakeDirectory() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("irmv-policy-session-test-" + std::to_string(now));
  std::filesystem::create_directory(directory);
  return directory;
}

std::filesystem::path SidecarPath(const std::filesystem::path &directory) {
  return directory / std::string(kPolicySessionFile);
}

Id MakeId(std::uint8_t fill) {
  Id id{};
  id.fill(fill);
  return id;
}

// A local session: no producer, no epoch, no gains.
PolicySessionEvent MakeLocalEvent() {
  PolicySessionEvent event;
  event.runtime_id = MakeId(0x11);
  event.session_id = MakeId(0x22);
  event.external = false;
  event.model_sha256 = std::string(64, 'a');
  event.model_path = "/models/policy.onnx";
  event.model_input = "obs";
  event.model_output = "action";
  event.contract_sha256 = std::string(64, 'b');
  return event;
}

std::vector<std::string> ReadLines(const std::filesystem::path &path) {
  std::ifstream file(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      lines.push_back(line);
    }
  }
  return lines;
}

}  // namespace

// An external session whose gains came from the first frame and a local session
// without gains both survive the round trip: the file holds exactly one JSON
// object per event, and every join key, model field and gain value reads back.
int RoundTrip() {
  const std::filesystem::path directory = MakeDirectory();
  {
    PolicySessionRecorder recorder(directory.string());
    CHECK(recorder.started());
    CHECK(recorder.error().empty());

    PolicySessionEvent external;
    external.runtime_id = MakeId(0x11);
    external.session_id = MakeId(0x22);
    external.producer_process_id = MakeId(0x33);
    external.epoch = 7;
    external.external = true;
    external.model_sha256 = std::string(64, 'a');
    external.model_path = "/models/policy.onnx";
    external.model_input = "obs";
    external.model_output = "action";
    external.contract_sha256 = std::string(64, 'b');
    JointCommand frame;
    frame.joints.count = 2;
    frame.joints.kp[0] = 60.0;
    frame.joints.kp[1] = 30.0;
    frame.joints.kd[0] = 4.0;
    frame.joints.kd[1] = 2.5;
    FillFrameGains(frame, {"left_hip", "right_knee"}, &external.gains);
    CHECK(recorder.TryRecord(external));

    PolicySessionEvent local = MakeLocalEvent();
    CHECK(recorder.TryRecord(local));
    CHECK(recorder.recorded() == 2);
    CHECK(recorder.dropped() == 0);
    recorder.Stop();
    // Stopping twice is not an error.
    recorder.Stop();
  }

  const std::vector<std::string> lines = ReadLines(SidecarPath(directory));
  CHECK(lines.size() == 2);

  const nlohmann::json first = nlohmann::json::parse(lines[0]);
  CHECK(first["schema"] == "irmv.policy_session");
  CHECK(first["version"] == 1);
  CHECK(first["host_us"].get<std::uint64_t>() > 0);
  CHECK(first["monotonic_us"].get<std::uint64_t>() > 0);
  CHECK(first["runtime_id"] == std::string(32, '1'));
  CHECK(first["session_id"] == std::string(32, '2'));
  CHECK(first["producer_process_id"] == std::string(32, '3'));
  CHECK(first["epoch"] == 7);
  CHECK(first["source"] == "external");
  CHECK(first["model"]["sha256"] == std::string(64, 'a'));
  CHECK(first["model"]["path"] == "/models/policy.onnx");
  CHECK(first["model"]["input"] == "obs");
  CHECK(first["model"]["output"] == "action");
  CHECK(first["model"]["contract_sha256"] == std::string(64, 'b'));
  // An external session writes both kinds of every declared joint, kp first.
  CHECK(first["gains"].size() == 4);
  CHECK(first["gains"][0]["joint"] == "left_hip");
  CHECK(first["gains"][0]["gain"] == "kp");
  CHECK(first["gains"][0]["model"].get<double>() == 60.0);
  CHECK(first["gains"][0]["effective"].get<double>() == 60.0);
  CHECK(first["gains"][1]["joint"] == "left_hip");
  CHECK(first["gains"][1]["gain"] == "kd");
  CHECK(first["gains"][1]["model"].get<double>() == 4.0);
  CHECK(first["gains"][2]["joint"] == "right_knee");
  CHECK(first["gains"][2]["gain"] == "kp");
  CHECK(first["gains"][2]["model"].get<double>() == 30.0);
  CHECK(first["gains"][3]["joint"] == "right_knee");
  CHECK(first["gains"][3]["gain"] == "kd");
  CHECK(first["gains"][3]["effective"].get<double>() == 2.5);

  const nlohmann::json second = nlohmann::json::parse(lines[1]);
  CHECK(second["source"] == "local");
  CHECK(second["producer_process_id"] == "");
  CHECK(second["epoch"] == 0);
  CHECK(second["gains"].empty());
  CHECK(second["runtime_id"] == std::string(32, '1'));
  CHECK(second["session_id"] == std::string(32, '2'));

  std::filesystem::remove_all(directory);
  return 0;
}

// The local filler copies exactly the overrides the policy layer resolved, and
// the recorder writes exactly the overridden kinds: a matched joint that
// overrode nothing contributes no row.
int GainOverridesAreFilled() {
  const std::filesystem::path directory = MakeDirectory();
  {
    irmv::policy::ResolvedGains resolved;
    resolved.override_count = 2;
    resolved.overrides[0].joint.Assign("left_hip");
    resolved.overrides[0].kp_overridden = true;
    resolved.overrides[0].model_kp = 60.0;
    resolved.overrides[0].applied_kp = 70.0;
    resolved.overrides[0].model_kd = 4.0;  // kd was not overridden
    resolved.overrides[1].joint.Assign("right_knee");
    resolved.overrides[1].model_kp = 30.0;  // neither kind was overridden
    resolved.overrides[1].model_kd = 2.5;

    std::vector<PolicyGainOverride> gains;
    FillGainOverrides(resolved, &gains);
    CHECK(gains.size() == 2);
    CHECK(std::string(gains[0].joint.View()) == "left_hip");
    CHECK(gains[0].kp_overridden);
    CHECK(gains[0].model_kp == 60.0);
    CHECK(gains[0].applied_kp == 70.0);
    CHECK(!gains[0].kd_overridden);
    CHECK(std::string(gains[1].joint.View()) == "right_knee");
    CHECK(!gains[1].kp_overridden);
    CHECK(!gains[1].kd_overridden);

    PolicySessionRecorder recorder(directory.string());
    PolicySessionEvent event = MakeLocalEvent();
    event.gains = gains;
    CHECK(recorder.TryRecord(event));
    recorder.Stop();
  }

  const std::vector<std::string> lines = ReadLines(SidecarPath(directory));
  CHECK(lines.size() == 1);
  const nlohmann::json entry = nlohmann::json::parse(lines[0]);
  CHECK(entry["source"] == "local");
  // Exactly the one overridden kind: the joint that overrode nothing writes no
  // row, and the un-overridden kd of the other writes none either.
  CHECK(entry["gains"].size() == 1);
  CHECK(entry["gains"][0]["joint"] == "left_hip");
  CHECK(entry["gains"][0]["gain"] == "kp");
  CHECK(entry["gains"][0]["model"].get<double>() == 60.0);
  CHECK(entry["gains"][0]["effective"].get<double>() == 70.0);
  std::filesystem::remove_all(directory);
  return 0;
}

// The external filler names every joint of the frame from the canonical table,
// in the frame's order, with declared equal to effective, and the recorder
// writes both kinds per joint. A frame the table cannot name is refused whole.
int FrameGainsAreFilled() {
  const std::filesystem::path directory = MakeDirectory();
  JointCommand frame;
  frame.joints.count = 3;
  frame.joints.kp[0] = 10.0;
  frame.joints.kp[1] = 20.0;
  frame.joints.kp[2] = 30.0;
  frame.joints.kd[0] = 1.0;
  frame.joints.kd[1] = 2.0;
  frame.joints.kd[2] = 3.0;
  const std::vector<std::string> names{"a", "b", "c"};

  std::vector<PolicyGainOverride> gains;
  FillFrameGains(frame, names, &gains);
  CHECK(gains.size() == 3);
  CHECK(std::string(gains[0].joint.View()) == "a");
  CHECK(std::string(gains[1].joint.View()) == "b");
  CHECK(std::string(gains[2].joint.View()) == "c");
  for (const PolicyGainOverride &gain : gains) {
    CHECK(!gain.kp_overridden);
    CHECK(!gain.kd_overridden);
    CHECK(gain.model_kp == gain.applied_kp);
    CHECK(gain.model_kd == gain.applied_kd);
  }

  // A frame longer than the canonical table produces no rows at all.
  frame.joints.count = 4;
  FillFrameGains(frame, names, &gains);
  CHECK(gains.empty());
  // A frame with no joints leaves nothing.
  frame.joints.count = 0;
  FillFrameGains(frame, names, &gains);
  CHECK(gains.empty());

  frame.joints.count = 3;
  {
    PolicySessionRecorder recorder(directory.string());
    PolicySessionEvent event = MakeLocalEvent();
    event.external = true;
    event.producer_process_id = MakeId(0x33);
    event.epoch = 9;
    FillFrameGains(frame, names, &event.gains);
    CHECK(recorder.TryRecord(event));
    recorder.Stop();
  }

  const std::vector<std::string> lines = ReadLines(SidecarPath(directory));
  CHECK(lines.size() == 1);
  const nlohmann::json entry = nlohmann::json::parse(lines[0]);
  CHECK(entry["source"] == "external");
  CHECK(entry["producer_process_id"] == std::string(32, '3'));
  CHECK(entry["epoch"] == 9);
  // Two rows per joint, kp before kd, in the frame's order.
  CHECK(entry["gains"].size() == 6);
  const std::string kOrder[3] = {"a", "b", "c"};
  const double kKp[3] = {10.0, 20.0, 30.0};
  const double kKd[3] = {1.0, 2.0, 3.0};
  for (std::size_t index = 0; index < 3; ++index) {
    const nlohmann::json &kp = entry["gains"][2 * index];
    const nlohmann::json &kd = entry["gains"][2 * index + 1];
    CHECK(kp["joint"] == kOrder[index]);
    CHECK(kp["gain"] == "kp");
    CHECK(kp["model"].get<double>() == kKp[index]);
    CHECK(kp["effective"].get<double>() == kKp[index]);
    CHECK(kd["joint"] == kOrder[index]);
    CHECK(kd["gain"] == "kd");
    CHECK(kd["model"].get<double>() == kKd[index]);
    CHECK(kd["effective"].get<double>() == kKd[index]);
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A handoff nobody drains costs the events, not the caller: the queue is fixed
// capacity by construction, every offer returns immediately, and the excess is
// refused and counted - the control thread is bounded, whatever the disk does.
int BoundedQueueDropsAndCounts() {
  const std::filesystem::path directory = MakeDirectory();
  PolicySessionRecorder recorder(directory.string());
  CHECK(recorder.started());
  // The worker waits before its first pass, so a burst pushed immediately
  // overruns the bounded handoff; the tail is refused. Every call still
  // returns, which is the whole point of the fixed capacity.
  const std::uint64_t burst = 4096;
  std::uint64_t accepted = 0;
  for (std::uint64_t index = 1; index <= burst; ++index) {
    if (recorder.TryRecord(MakeLocalEvent())) {
      ++accepted;
    }
  }
  CHECK(accepted < burst);
  CHECK(accepted > 0);
  CHECK(recorder.dropped() == burst - accepted);
  CHECK(recorder.recorded() == accepted);
  CHECK(recorder.dropped() > 0);
  recorder.Stop();
  // Every event that was accepted is written before the file closes: the
  // handoff drops, it does not lose.
  const std::vector<std::string> lines = ReadLines(SidecarPath(directory));
  CHECK(lines.size() == accepted);
  for (const std::string &line : lines) {
    CHECK(nlohmann::json::parse(line).is_object());
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A directory that cannot hold the sidecar is reported, not thrown: the
// recorder is left stopped, error() names the reason, and every TryRecord
// refuses without an exception. This is how the rest of the project reports an
// unusable telemetry path.
int UnusableDirectoryIsReported() {
  const std::filesystem::path directory = MakeDirectory();
  {
    PolicySessionRecorder recorder((directory / "missing").string());
    CHECK(!recorder.started());
    CHECK(!recorder.error().empty());
    PolicySessionEvent event = MakeLocalEvent();
    CHECK(!recorder.TryRecord(event));
    CHECK(recorder.dropped() == 1);
    CHECK(recorder.recorded() == 0);
    recorder.Stop();
  }
  {
    // A regular file where the directory should be is refused the same way.
    const std::filesystem::path file_path = directory / "not_a_directory";
    std::ofstream(file_path) << "x\n";
    PolicySessionRecorder recorder(file_path.string());
    CHECK(!recorder.started());
    CHECK(!recorder.error().empty());
    recorder.Stop();
  }
  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += RoundTrip();
  failures += GainOverridesAreFilled();
  failures += FrameGainsAreFilled();
  failures += BoundedQueueDropsAndCounts();
  failures += UnusableDirectoryIsReported();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
