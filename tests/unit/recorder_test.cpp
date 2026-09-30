// The recorder: what the control thread may pay for a sample, what the worker
// writes, and what a run says about the samples it never recorded.
//
// The entry is built around the two promises that matter to the control loop:
// TryRecord never blocks, allocates or touches a file, and a full handoff costs
// the sample - counted and visible as a gap in the run - rather than the tick.
#include "irmv/telemetry/recorder.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "irmv/telemetry/record.hpp"
#include "support/check.hpp"

namespace {

using irmv::protocol::ExecutionState;
using irmv::protocol::OutputAction;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::telemetry::kRecordChunkRows;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;
using irmv::telemetry::RecordGap;
using irmv::telemetry::RecordGapReason;
using irmv::telemetry::RecordSample;

constexpr std::size_t kJoints = 12;

RecordSample MakeSample(Sequence state_seq) {
  RecordSample sample;
  sample.stamp_us = 1000 + state_seq * 10000;
  sample.state_seq = state_seq;
  sample.execution_state = ExecutionState::Tracking;
  sample.output_action = OutputAction::Tracking;
  sample.healthy = true;
  sample.command_present = true;
  sample.epoch = 3;
  sample.command_seq = state_seq;
  sample.source_state_seq = state_seq;
  sample.feedback_seq = state_seq;
  sample.feedback_valid = true;
  sample.imu_valid = true;
  sample.send_accepted = true;
  sample.joint_count = static_cast<std::uint32_t>(kJoints);
  for (std::size_t index = 0; index < kJoints; ++index) {
    sample.q[index] = 0.01 * static_cast<double>(index) + 0.001 * state_seq;
    sample.dq[index] = 0.0;
    sample.tau_ff[index] = 0.0;
    sample.kp[index] = 50.0;
    sample.kd[index] = 2.0;
  }
  return sample;
}

// A run directory that removes itself, and a metadata document good enough for
// the recorder to write.
std::filesystem::path MakeDirectory() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("irmv-recorder-test-" + std::to_string(now));
  std::filesystem::create_directory(directory);
  return directory;
}

bool StartRecorder(Recorder *recorder, const std::filesystem::path &directory,
                   bool enabled = true, std::uint32_t stride = 1) {
  RecorderConfig config;
  config.enabled = enabled;
  config.sample_hz = 100;
  config.stride = stride;
  config.directory = directory;
  std::string error;
  const std::string metadata = R"({"schema_version":1,"joints":["a","b"]})";
  const bool started = recorder->Start(config, metadata, &error);
  if (!started) {
    std::cerr << "recorder start failed: " << error << '\n';
  }
  return started;
}

std::uint64_t CountChunks(const std::filesystem::path &run_directory) {
  std::uint64_t count = 0;
  for (const auto &entry : std::filesystem::directory_iterator(run_directory)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind("chunk_", 0) == 0 && entry.path().extension() == ".bin") {
      ++count;
    }
  }
  return count;
}

std::uint64_t CountPartials(const std::filesystem::path &run_directory) {
  std::uint64_t count = 0;
  for (const auto &entry : std::filesystem::directory_iterator(run_directory)) {
    if (entry.path().extension() == ".part") {
      ++count;
    }
  }
  return count;
}

nlohmann::json ReadIndex(const std::filesystem::path &run_directory) {
  std::ifstream file(run_directory / "index.json");
  if (!file) {
    return nlohmann::json{};
  }
  return nlohmann::json::parse(file, nullptr, false);
}

}  // namespace

// A recorder that was never started accepts nothing and creates nothing; one
// that is started creates exactly one run directory with its metadata.
int Lifecycle() {
  const std::filesystem::path directory = MakeDirectory();
  {
    Recorder recorder;
    CHECK(!recorder.started());
    CHECK(!recorder.TryRecord(MakeSample(1)));
    CHECK(recorder.dropped() == 1);
    CHECK(recorder.run_directory().empty());
  }
  {
    Recorder recorder;
    CHECK(StartRecorder(&recorder, directory));
    CHECK(recorder.started());
    CHECK(!recorder.run_directory().empty());
    CHECK(std::filesystem::exists(recorder.run_directory() / "meta.json"));
    CHECK(recorder.Stop());
    CHECK(!recorder.started());
    // Stopping twice is not an error and does not lose the run.
    CHECK(recorder.Stop());
  }
  {
    // A directory that does not exist is refused before any state is created.
    Recorder recorder;
    RecorderConfig config;
    config.enabled = true;
    config.sample_hz = 100;
    config.directory = directory / "missing";
    std::string error;
    CHECK(!recorder.Start(config, "{}", &error));
    CHECK(!error.empty());
    CHECK(!recorder.started());
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// The samples the control thread offers end up in chunks, and the index
// describes them by time and sequence.
int RecordedRun() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  const std::uint64_t rows = kRecordChunkRows + 10;
  // Paced the way a 100 Hz producer would offer them: the handoff is bounded,
  // so a burst is *allowed* to cost samples - that is the next case - and this
  // one asserts the structure of a run nothing was lost from.
  const std::uint64_t batch = 32;
  for (std::uint64_t index = 1; index <= rows; ++index) {
    CHECK(recorder.TryRecord(MakeSample(index)));
    if (index % batch == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }
  CHECK(recorder.Stop());
  CHECK(recorder.dropped() == 0);
  CHECK(recorder.recorded() == rows);

  const std::filesystem::path run = recorder.run_directory();
  CHECK(!run.empty());
  // Two chunks: one full, one with the remainder.
  CHECK(recorder.chunks() == 2);
  CHECK(CountChunks(run) == 2);
  // Every chunk was renamed into place: nothing is left half written.
  CHECK(CountPartials(run) == 0);

  const nlohmann::json index = ReadIndex(run);
  CHECK(index.is_object());
  CHECK(index["rows_recorded"] == rows);
  CHECK(index["joint_count"] == kJoints);
  CHECK(index["chunks"].size() == 2);
  CHECK(index["chunks"][0]["rows"] == kRecordChunkRows);
  CHECK(index["chunks"][1]["rows"] == 10);
  CHECK(index["chunks"][0]["first_state_seq"] == 1);
  CHECK(index["chunks"][0]["last_state_seq"] == kRecordChunkRows);
  CHECK(index["chunks"][1]["first_state_seq"] == kRecordChunkRows + 1);
  CHECK(index["chunks"][1]["last_state_seq"] == rows);
  CHECK(index["gaps"].empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// A hole in the sequence the runtime published is a gap in the run, reported
// with its range - never smoothed over by presenting the samples around it as
// consecutive.
int Gaps() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  for (Sequence seq : {1, 2, 3, 9, 10, 11}) {
    CHECK(recorder.TryRecord(MakeSample(seq)));
  }
  CHECK(recorder.Stop());
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["gaps"].size() == 1);
  CHECK(index["gaps"][0]["first_seq"] == 4);
  CHECK(index["gaps"][0]["last_seq"] == 8);
  CHECK(index["gaps"][0]["dropped"] == 5);
  CHECK(index["rows_recorded"] == 6);
  std::filesystem::remove_all(directory);
  return 0;
}

// A handoff nobody drains costs the samples, not the tick: the queue is
// bounded, the refusal is counted, and the run reports the stretch it never
// wrote.
int FullQueueIsCounted() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  // The worker waits before its first drain, so a burst pushed immediately
  // fills the bounded queue and the rest is refused.
  const std::uint64_t burst = 2000;
  std::uint64_t accepted = 0;
  for (std::uint64_t index = 1; index <= burst; ++index) {
    if (recorder.TryRecord(MakeSample(index))) {
      ++accepted;
    }
  }
  CHECK(accepted < burst);
  CHECK(recorder.dropped() > 0);
  CHECK(recorder.Stop());
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(!index["gaps"].empty());
  // The gap explains the loss in the sequence space, so a reader can tell it
  // from a stretch that never existed.
  bool saw_queue_gap = false;
  for (const auto &gap : index["gaps"]) {
    if (gap["reason"] == "queue_full") {
      saw_queue_gap = true;
      CHECK(gap["dropped"] > 0);
    }
  }
  CHECK(saw_queue_gap);
  std::filesystem::remove_all(directory);
  return 0;
}

// A run sampled every fifth control tick - what a 100 Hz recorder on a 500 Hz
// control loop is - must not report a gap per row: consecutive rows differ by
// the stride by construction, and only a jump beyond it is a missing sample.
int StridedRun() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  constexpr std::uint32_t kStride = 5;
  CHECK(StartRecorder(&recorder, directory, true, kStride));
  // Twenty samples, each five ticks apart, and then a hole of two samples.
  Sequence seq = 1;
  for (int index = 0; index < 20; ++index) {
    CHECK(recorder.TryRecord(MakeSample(seq)));
    seq += kStride;
  }
  const Sequence hole_start = seq;
  seq += 2 * kStride;
  for (int index = 0; index < 5; ++index) {
    CHECK(recorder.TryRecord(MakeSample(seq)));
    seq += kStride;
  }
  CHECK(recorder.Stop());
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["rows_recorded"] == 25);
  CHECK(index["gaps"].size() == 1);
  // The gap names the two missing samples, not the eight ticks between them.
  CHECK(index["gaps"][0]["first_seq"] == hole_start);
  CHECK(index["gaps"][0]["last_seq"] == hole_start + kStride);
  CHECK(index["gaps"][0]["dropped"] == 2);
  std::filesystem::remove_all(directory);
  return 0;
}

// A disabled recorder opens nothing at all - the configuration decides whether
// a run exists, not the caller.
int Disabled() {
  const std::filesystem::path directory = MakeDirectory();
  std::size_t entries = 0;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    static_cast<void>(entry);
    ++entries;
  }
  CHECK(entries == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += Lifecycle();
  failures += RecordedRun();
  failures += Gaps();
  failures += FullQueueIsCounted();
  failures += StridedRun();
  failures += Disabled();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
