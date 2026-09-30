// The read side of a recorded run: a run written by the recorder is read back
// row by row, by time, with its gaps and its truncations reported rather than
// smoothed over.
//
// The round trip is deliberately through the recorder itself: the reader is
// tested against bytes the writer really produced, not against a fixture that
// could agree with a format neither of them implements.
#include "irmv/telemetry/record_reader.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "irmv/telemetry/recorder.hpp"
#include "support/check.hpp"

namespace {

using irmv::protocol::ExecutionState;
using irmv::protocol::OutputAction;
using irmv::protocol::Sequence;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;
using irmv::telemetry::RecordGapReason;
using irmv::telemetry::RecordRange;
using irmv::telemetry::RecordReader;
using irmv::telemetry::RecordSample;

constexpr std::size_t kJoints = 4;
constexpr std::uint32_t kStride = 5;
constexpr TimeUs kStampStepUs = 10000;

RecordSample MakeSample(Sequence state_seq) {
  RecordSample sample;
  sample.stamp_us = 1000 + state_seq * kStampStepUs;
  sample.state_seq = state_seq;
  sample.execution_state = ExecutionState::Tracking;
  sample.output_action = OutputAction::Tracking;
  sample.stop_reason = StopReason::None;
  sample.fault_id = 7;
  sample.healthy = true;
  sample.damping_eligible = true;
  sample.command_present = true;
  sample.producer_id[0] = 0x5A;
  sample.epoch = 9;
  sample.command_seq = state_seq;
  sample.source_state_seq = state_seq;
  sample.command_rx_us = sample.stamp_us + 3;
  sample.feedback_seq = state_seq;
  sample.feedback_stamp_us = sample.stamp_us - 100;
  sample.feedback_valid = true;
  sample.imu_valid = true;
  sample.send_accepted = true;
  sample.joint_count = static_cast<std::uint32_t>(kJoints);
  for (std::size_t index = 0; index < kJoints; ++index) {
    sample.q[index] = 0.5 + 0.001 * state_seq + 0.01 * index;
    sample.dq[index] = 0.25 + 0.002 * state_seq;
    sample.tau_ff[index] = 0.125 + 0.003 * state_seq;
    sample.kp[index] = 50.0 + index;
    sample.kd[index] = 2.0 + 0.1 * index;
  }
  return sample;
}

std::filesystem::path MakeDirectory() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("irmv-reader-test-" + std::to_string(now));
  std::filesystem::create_directory(directory);
  return directory;
}

// Writes a run of `count` samples with the given stride, skipping the samples
// whose state sequence is in [hole_from, hole_to] so the run has one hole.
bool WriteRun(const std::filesystem::path &directory, int count,
              Sequence hole_from, Sequence hole_to,
              std::vector<RecordSample> *written,
              std::filesystem::path *run_directory) {
  Recorder recorder;
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.stride = kStride;
  config.directory = directory;
  std::string error;
  const std::string metadata =
      R"({"schema_version":1,"config_id":"cfg","layout_id":"lay","backend":"fake","control_hz":500.0,"sample_hz":100,"joint_count":4,"joints":["j1","j2","j3","j4"],"units":{"position":"rad"}})";
  if (!recorder.Start(config, metadata, &error)) {
    std::cerr << "recorder start failed: " << error << '\n';
    return false;
  }
  Sequence seq = 1;
  for (int index = 0; index < count; ++index) {
    if (!(seq >= hole_from && seq <= hole_to)) {
      const RecordSample sample = MakeSample(seq);
      if (!recorder.TryRecord(sample)) {
        std::cerr << "the recorder refused a sample\n";
        return false;
      }
      if (written != nullptr) {
        written->push_back(sample);
      }
    }
    seq += kStride;
  }
  const bool stopped = recorder.Stop();
  if (stopped && run_directory != nullptr) {
    *run_directory = recorder.run_directory();
  }
  return stopped;
}

}  // namespace

// A run read back whole: every field of every row, and the metadata the run
// declares about itself.
int FullRead() {
  const std::filesystem::path directory = MakeDirectory();
  std::vector<RecordSample> written;
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, &written, &run));
  CHECK(written.size() == 30);

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.metadata().schema_version == 1);
  CHECK(reader.metadata().config_id == "cfg");
  CHECK(reader.metadata().layout_id == "lay");
  CHECK(reader.metadata().sample_hz == 100);
  CHECK(reader.metadata().joint_count == kJoints);
  CHECK(reader.metadata().joints.size() == kJoints);
  CHECK(reader.metadata().joints[0] == "j1");
  CHECK(reader.chunks().size() == 1);
  CHECK(reader.gaps().empty());

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(!range.truncated);
  CHECK(range.samples.size() == written.size());
  for (std::size_t index = 0; index < written.size(); ++index) {
    CHECK(range.samples[index] == written[index]);
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A range read returns exactly the rows inside it, and the gap that overlaps it
// is reported with the range.
int RangeRead() {
  const std::filesystem::path directory = MakeDirectory();
  std::vector<RecordSample> written;
  const Sequence hole_from = 21;
  const Sequence hole_to = 30;  // two samples of stride five
  std::filesystem::path run;
  CHECK(WriteRun(directory, 40, hole_from, hole_to, &written, &run));

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));

  const RecordSample &first = written.front();
  const RecordSample &middle = written[written.size() / 3];
  RecordRange range;
  // The window covers the first row and a few after it.
  CHECK(reader.ReadRange(first.stamp_us, middle.stamp_us, &range));
  CHECK(range.ok);
  CHECK(!range.samples.empty());
  for (const RecordSample &sample : range.samples) {
    CHECK(sample.stamp_us >= first.stamp_us);
    CHECK(sample.stamp_us < middle.stamp_us);
  }
  // The first row of the run is inside the window and is exactly what was
  // written.
  CHECK(range.samples.front() == first);

  // A window that covers the hole reports it.
  const TimeUs hole_start_us = 1000 + hole_from * kStampStepUs;
  const TimeUs hole_end_us = 1000 + (hole_to + kStride) * kStampStepUs;
  RecordRange hole_range;
  CHECK(reader.ReadRange(hole_start_us, hole_end_us, &hole_range));
  CHECK(hole_range.gaps.size() == 1);
  // The gap is expressed on the recorded grid: with a stride of five, the two
  // samples that are missing are the ones at 21 and 26, not the eight control
  // ticks between them.
  CHECK(hole_range.gaps[0].first_seq == hole_from);
  CHECK(hole_range.gaps[0].last_seq == hole_to - kStride + 1);
  CHECK(hole_range.gaps[0].dropped == 2);
  CHECK(hole_range.gaps[0].reason == RecordGapReason::QueueFull);

  // An empty window reads nothing and is not an error.
  RecordRange empty;
  CHECK(reader.ReadRange(500, 500, &empty));
  CHECK(empty.ok);
  CHECK(empty.samples.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk that was not written whole is reported as a truncation, and the rows
// it did hold are still read.
int TruncatedChunk() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  const std::filesystem::path chunk = run / reader.chunks().front().file;
  const auto size = std::filesystem::file_size(chunk);
  std::filesystem::resize_file(chunk, size / 2);

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.truncated);
  CHECK(!range.samples.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// A run of another schema version is refused whole rather than reinterpreted.
int UnknownVersion() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 5, 0, 0, nullptr, &run));
  std::ofstream file(run / "meta.json", std::ios::trunc);
  file << R"({"schema_version":7})";
  file.close();
  RecordReader reader;
  std::string error;
  CHECK(!RecordReader::Open(run, &reader, &error));
  CHECK(!error.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += FullRead();
  failures += RangeRead();
  failures += TruncatedChunk();
  failures += UnknownVersion();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
