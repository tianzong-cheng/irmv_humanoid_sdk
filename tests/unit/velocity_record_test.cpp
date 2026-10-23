// The application's independent velocity stream: what the samples of one
// application run are, what the worker writes, and what a reader can tell about
// the rows and the losses of a run.
//
// The entry checks the promises the recorder makes to the control loop: one
// fixed 256-slot handoff, no allocation, blocking or file work on the caller's
// thread, and a full queue or a stopped recorder costs the sample and is
// counted instead of delaying the tick. It also pins the stream's contract: a
// row per due sample - including the all-zero initial intent - with its own
// identity, a monotonic sample sequence and 64-bit values that survive the JSON
// round trip; losses named in the index; and a trailing partial line, a
// malformed complete line and an unknown schema each reported as the different
// fact they are.
#include "irmv/telemetry/velocity_record.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/record_context.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::RecorderConfig;
using irmv::protocol::Id;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::telemetry::kVelocityFile;
using irmv::telemetry::kVelocityIndexFile;
using irmv::telemetry::kVelocityMetadataFile;
using irmv::telemetry::ReadVelocityRange;
using irmv::telemetry::SampleSchedule;
using irmv::telemetry::SetVelocityLineWriteHookForTest;
using irmv::telemetry::VelocityLossReason;
using irmv::telemetry::VelocityRecorder;
using irmv::telemetry::VelocityRecordRange;
using irmv::telemetry::VelocitySample;

constexpr TimeUs kNever = std::numeric_limits<TimeUs>::max();

std::filesystem::path MakeDirectory() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("irmv-velocity-record-test-" + std::to_string(now));
  std::filesystem::create_directory(directory);
  return directory;
}

Id MakeId(std::uint8_t fill) {
  Id id{};
  id.fill(fill);
  return id;
}

std::string MakeMeta(std::uint32_t sample_hz, std::uint32_t version = 1) {
  nlohmann::json metadata;
  metadata["schema"] = "irmv.application_velocity";
  metadata["version"] = version;
  metadata["sdk_git_sha"] = std::string(40, 'c');
  metadata["application_config_id"] = std::string(64, 'a');
  metadata["config_id"] = std::string(64, 'b');
  metadata["sample_hz"] = sample_hz;
  metadata["clock"] = {{"clock", "linux_clock_monotonic"},
                       {"boot_id", "1234-5678"},
                       {"available", true}};
  metadata["units"] = {
      {"forward", "m_s"}, {"lateral", "m_s"}, {"yaw", "rad_s"}, {"time", "us"}};
  metadata["model"] = {{"status", "none"}, {"name", nullptr}};
  return metadata.dump(2);
}

bool StartRecorder(VelocityRecorder *recorder,
                   const std::filesystem::path &directory,
                   std::size_t queue_capacity = 256,
                   std::uint32_t sample_hz = 100,
                   std::uint32_t meta_version = 1) {
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = sample_hz;
  config.queue_capacity = queue_capacity;
  config.directory = directory;
  std::string error;
  const bool started =
      recorder->Start(config, MakeMeta(sample_hz, meta_version), &error);
  if (!started) {
    std::cerr << "velocity recorder start failed: " << error << '\n';
  }
  return started;
}

VelocitySample MakeSample(std::uint64_t seq, TimeUs stamp_us, double forward,
                          double lateral, double yaw,
                          std::uint8_t runtime_fill = 0x11,
                          std::uint8_t session_fill = 0x22,
                          Sequence latest_state_seq = 1) {
  VelocitySample sample;
  sample.stamp_us = stamp_us;
  sample.sample_seq = seq;
  sample.identity_known = runtime_fill != 0;
  if (sample.identity_known) {
    sample.runtime_id = MakeId(runtime_fill);
    sample.session_id = MakeId(session_fill);
  }
  sample.latest_state_seq = latest_state_seq;
  sample.forward = forward;
  sample.lateral = lateral;
  sample.yaw = yaw;
  return sample;
}

nlohmann::json ReadIndex(const std::filesystem::path &run_directory) {
  std::ifstream file(run_directory / std::string(kVelocityIndexFile));
  nlohmann::json index;
  file >> index;
  return index;
}

std::string ReadText(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

}  // namespace

// One run is one directory named by the host clock; it carries the metadata the
// caller handed over, an empty stream that still exists, and an index that says
// the run is not complete until the worker finalized it.
int DirectoryMetadataAndEmptyStream() {
  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));
  CHECK(recorder.started());
  CHECK(recorder.error().empty());

  const std::filesystem::path run = recorder.run_directory();
  CHECK(run.parent_path() == parent);
  const std::string name = run.filename().string();
  CHECK(name.rfind("app_run_", 0) == 0);
  CHECK(run != parent);

  const nlohmann::json metadata =
      nlohmann::json::parse(ReadText(run / std::string(kVelocityMetadataFile)));
  CHECK(metadata["schema"] == "irmv.application_velocity");
  CHECK(metadata["version"] == 1);
  CHECK(metadata["sample_hz"] == 100);
  CHECK(metadata["application_config_id"] == std::string(64, 'a'));
  CHECK(metadata["clock"]["available"] == true);

  CHECK(std::filesystem::is_regular_file(run / std::string(kVelocityFile)));
  CHECK(std::filesystem::file_size(run / std::string(kVelocityFile)) == 0);
  CHECK(ReadIndex(run)["complete"] == false);

  CHECK(recorder.Stop());
  const nlohmann::json index = ReadIndex(run);
  CHECK(index["complete"] == true);
  CHECK(index["rows_written"] == 0);
  CHECK(index["samples_dropped"] == 0);
  CHECK(index["write_failures"] == 0);

  VelocityRecordRange range;
  CHECK(ReadVelocityRange(run, 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.empty());
  CHECK(!range.truncated);
  CHECK(range.complete);

  // A second recorder in the same parent is a second run, never the same one.
  VelocityRecorder other;
  CHECK(StartRecorder(&other, parent));
  CHECK(other.run_directory() != run);
  CHECK(other.Stop());
  CHECK(recorder.Stop());

  std::filesystem::remove_all(parent);
  return 0;
}

// Every due sample is a row: constant and continuously changing values both
// survive, the all-zero initial intent is a valid row rather than a "no data"
// marker, and 64-bit stamps and sequences are preserved exactly.
int RoundTripAndRangeFilter() {
  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));

  constexpr std::uint64_t kHugeSeq = 9007199254740995ULL;  // 2^53 + 3
  constexpr TimeUs kHugeStamp = 9007199254740993ULL;       // 2^53 + 1
  constexpr Sequence kHugeStateSeq = 9007199254740997ULL;  // 2^53 + 5

  CHECK(recorder.TryRecord(MakeSample(0, 100, 0.0, 0.0, 0.0)));
  CHECK(recorder.TryRecord(MakeSample(1, 200, 0.25, 0.5, -0.75)));
  CHECK(recorder.TryRecord(MakeSample(2, 300, 0.5, 0.5, -0.75)));
  CHECK(recorder.TryRecord(MakeSample(kHugeSeq, kHugeStamp, 1.5, -2.0, 3.25,
                                      0x33, 0x44, kHugeStateSeq)));
  CHECK(recorder.recorded() == 4);
  CHECK(recorder.Stop());
  CHECK(recorder.written() == 4);

  VelocityRecordRange all;
  CHECK(ReadVelocityRange(recorder.run_directory(), 0, kNever, &all));
  CHECK(all.ok);
  CHECK(all.complete);
  CHECK(!all.truncated);
  CHECK(all.rows.size() == 4);
  if (all.rows.size() == 4) {
    CHECK(all.rows[0].stamp_us == 100);
    CHECK(all.rows[0].sample_seq == 0);
    CHECK(all.rows[0].identity_known);
    CHECK(all.rows[0].runtime_id == MakeId(0x11));
    CHECK(all.rows[0].session_id == MakeId(0x22));
    CHECK(all.rows[0].forward == 0.0);
    CHECK(all.rows[1].forward == 0.25);
    CHECK(all.rows[1].lateral == 0.5);
    CHECK(all.rows[2].forward == 0.5);
    CHECK(all.rows[3].stamp_us == kHugeStamp);
    CHECK(all.rows[3].sample_seq == kHugeSeq);
    CHECK(all.rows[3].latest_state_seq == kHugeStateSeq);
    CHECK(all.rows[3].runtime_id == MakeId(0x33));
    CHECK(all.rows[3].session_id == MakeId(0x44));
    CHECK(all.rows[3].yaw == 3.25);
  }

  VelocityRecordRange window;
  CHECK(ReadVelocityRange(recorder.run_directory(), 200, 300, &window));
  CHECK(window.ok);
  CHECK(window.rows.size() == 1);
  if (window.rows.size() == 1) {
    CHECK(window.rows[0].stamp_us == 200);
  }

  VelocityRecordRange empty;
  CHECK(ReadVelocityRange(recorder.run_directory(), 300, 300, &empty));
  CHECK(empty.ok);
  CHECK(empty.rows.empty());

  std::filesystem::remove_all(parent);
  return 0;
}

// The identity is the sample's own fact. A run that starts before the runtime
// is discovered records that state; when a different instance and session
// appear, the new sample carries the new identity and never the old one.
int IdentityTransitionDoesNotReuse() {
  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));

  VelocitySample unknown = MakeSample(0, 100, 0.0, 0.0, 0.0, 0, 0, 0);
  CHECK(!unknown.identity_known);
  CHECK(recorder.TryRecord(unknown));
  CHECK(recorder.TryRecord(MakeSample(1, 200, 0.1, 0.0, 0.0, 0x01, 0x02, 11)));
  CHECK(recorder.TryRecord(MakeSample(2, 300, 0.2, 0.0, 0.0, 0x03, 0x04, 12)));
  CHECK(recorder.Stop());

  VelocityRecordRange range;
  CHECK(ReadVelocityRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.size() == 3);
  if (range.rows.size() == 3) {
    CHECK(!range.rows[0].identity_known);
    CHECK(range.rows[0].runtime_id == irmv::protocol::kNoId);
    CHECK(range.rows[0].session_id == irmv::protocol::kNoId);
    CHECK(range.rows[0].latest_state_seq == 0);
    CHECK(range.rows[1].identity_known);
    CHECK(range.rows[1].runtime_id == MakeId(0x01));
    CHECK(range.rows[1].session_id == MakeId(0x02));
    CHECK(range.rows[1].latest_state_seq == 11);
    CHECK(range.rows[2].identity_known);
    CHECK(range.rows[2].runtime_id == MakeId(0x03));
    CHECK(range.rows[2].session_id == MakeId(0x04));
    CHECK(range.rows[2].latest_state_seq == 12);
  }

  std::filesystem::remove_all(parent);
  return 0;
}

// A handoff that is full costs the sample rather than the tick, the index names
// the exact sequence stretch, and the run still finalizes once the device
// drains.
int QueueFullIsCountedAndSpanned() {
  struct BlockHook {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered{false};
    bool released{false};
  } state;
  SetVelocityLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        auto *hook = static_cast<BlockHook *>(context);
        std::unique_lock<std::mutex> lock(hook->mutex);
        hook->entered = true;
        hook->cv.notify_all();
        hook->cv.wait(lock, [hook] { return hook->released; });
        return 0;
      },
      &state);

  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent, 1));
  CHECK(recorder.TryRecord(MakeSample(0, 100, 0.1, 0.0, 0.0)));
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    CHECK(state.cv.wait_for(lock, std::chrono::seconds(2),
                            [&state] { return state.entered; }));
  }
  // The worker is parked with the first row. One more sample fits the one-slot
  // handoff; the next two are refused and counted, not waited for.
  CHECK(recorder.TryRecord(MakeSample(1, 200, 0.2, 0.0, 0.0)));
  CHECK(!recorder.TryRecord(MakeSample(2, 300, 0.3, 0.0, 0.0)));
  CHECK(!recorder.TryRecord(MakeSample(3, 400, 0.4, 0.0, 0.0)));
  CHECK(recorder.recorded() == 2);
  CHECK(recorder.dropped() == 2);

  {
    const std::lock_guard<std::mutex> lock(state.mutex);
    state.released = true;
  }
  state.cv.notify_all();
  CHECK(recorder.Stop());
  CHECK(recorder.written() == 2);
  CHECK(recorder.dropped() == 2);

  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["complete"] == true);
  CHECK(index["rows_written"] == 2);
  CHECK(index["samples_dropped"] == 2);
  CHECK(index["losses"].size() == 1);
  if (index["losses"].size() == 1) {
    CHECK(index["losses"][0]["reason"] == "queue_full");
    CHECK(index["losses"][0]["dropped"] == 2);
    CHECK(index["losses"][0]["first_sample_seq"] == "0000000000000002");
    CHECK(index["losses"][0]["last_sample_seq"] == "0000000000000003");
  }

  SetVelocityLineWriteHookForTest(nullptr, nullptr);
  std::filesystem::remove_all(parent);
  return 0;
}

// A device that blocks must not hold Stop() past its deadline: Stop() returns
// false while the worker still owns the writer, and the worker finalizes the
// run on its own once the device drains.
int BlockedWriterBoundsStop() {
  struct BlockHook {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered{false};
    bool released{false};
  } state;
  SetVelocityLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        auto *hook = static_cast<BlockHook *>(context);
        std::unique_lock<std::mutex> lock(hook->mutex);
        hook->entered = true;
        hook->cv.notify_all();
        hook->cv.wait(lock, [hook] { return hook->released; });
        return 0;
      },
      &state);

  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));
  CHECK(recorder.TryRecord(MakeSample(0, 100, 0.1, 0.0, 0.0)));
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    CHECK(state.cv.wait_for(lock, std::chrono::seconds(2),
                            [&state] { return state.entered; }));
  }

  const auto started = std::chrono::steady_clock::now();
  CHECK(!recorder.Stop());
  const auto elapsed = std::chrono::steady_clock::now() - started;
  // Returning while the writer is still parked is the point; the exact
  // two-second boundary may stretch under a sanitizer or a loaded host.
  CHECK(elapsed >= std::chrono::milliseconds(500));
  CHECK(elapsed < std::chrono::seconds(10));
  CHECK(ReadIndex(recorder.run_directory())["complete"] == false);

  {
    const std::lock_guard<std::mutex> lock(state.mutex);
    state.released = true;
  }
  state.cv.notify_all();

  bool joined = false;
  for (int attempt = 0; attempt < 500 && !joined; ++attempt) {
    joined = recorder.Stop();
    if (!joined) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  CHECK(joined);
  CHECK(recorder.written() == 1);

  VelocityRecordRange range;
  CHECK(ReadVelocityRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.complete);
  CHECK(range.rows.size() == 1);

  SetVelocityLineWriteHookForTest(nullptr, nullptr);
  std::filesystem::remove_all(parent);
  return 0;
}

// A torn write is rolled back to the last committed line: the previous rows
// stay readable, the lost sample is named in the index, and the run is marked
// incomplete instead of claiming a row it never persisted.
int TornLineRollsBack() {
  struct TornHook {
    std::atomic<bool> first{true};
  } state;
  SetVelocityLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        auto *hook = static_cast<TornHook *>(context);
        return hook->first.exchange(false) ? 2 : 0;
      },
      &state);

  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));
  CHECK(recorder.TryRecord(MakeSample(0, 100, 0.1, 0.0, 0.0)));
  CHECK(recorder.TryRecord(MakeSample(1, 200, 0.2, 0.0, 0.0)));
  CHECK(recorder.TryRecord(MakeSample(2, 300, 0.3, 0.0, 0.0)));
  CHECK(recorder.Stop());
  CHECK(recorder.recorded() == 3);
  CHECK(recorder.written() == 2);
  CHECK(recorder.write_failures() == 1);

  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["complete"] == false);
  CHECK(index["write_failures"] == 1);
  CHECK(index["losses"].size() == 1);
  if (index["losses"].size() == 1) {
    CHECK(index["losses"][0]["reason"] == "write_failed");
    CHECK(index["losses"][0]["dropped"] == 1);
  }

  VelocityRecordRange range;
  CHECK(ReadVelocityRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(!range.truncated);
  CHECK(!range.complete);
  CHECK(range.rows.size() == 2);
  if (range.rows.size() == 2) {
    CHECK(range.rows[0].sample_seq == 1);
    CHECK(range.rows[1].sample_seq == 2);
  }

  SetVelocityLineWriteHookForTest(nullptr, nullptr);
  std::filesystem::remove_all(parent);
  return 0;
}

// The tail, a malformed complete line and a version this build does not know
// are three different facts: a truncated tail is ignored, a complete malformed
// line is an error, and an unknown schema/version is refused at the metadata,
// index and line level.
int TruncationCorruptAndUnknownVersion() {
  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));
  CHECK(recorder.TryRecord(MakeSample(0, 100, 0.1, 0.0, 0.0)));
  CHECK(recorder.Stop());
  const std::filesystem::path run = recorder.run_directory();
  const std::filesystem::path stream = run / std::string(kVelocityFile);
  const std::filesystem::path index_path =
      run / std::string(kVelocityIndexFile);
  const std::filesystem::path metadata_path =
      run / std::string(kVelocityMetadataFile);

  // A trailing line without a newline is a write that never finished: the
  // complete prefix is read and the tail is named truncated.
  {
    std::ofstream file(stream, std::ios::app | std::ios::binary);
    file << "{\"schema\":\"irmv.application_velocity\",\"version\":1";
  }
  VelocityRecordRange truncated;
  CHECK(ReadVelocityRange(run, 0, kNever, &truncated));
  CHECK(truncated.ok);
  CHECK(truncated.rows.size() == 1);
  CHECK(truncated.truncated);
  CHECK(truncated.complete);

  // Without the final index marker the run is readable but incomplete.
  const std::string index_text = ReadText(index_path);
  std::filesystem::remove(index_path);
  VelocityRecordRange unmarked;
  CHECK(ReadVelocityRange(run, 0, kNever, &unmarked));
  CHECK(unmarked.ok);
  CHECK(!unmarked.complete);
  {
    std::ofstream file(index_path, std::ios::binary | std::ios::trunc);
    file << index_text;
  }

  // A newline-terminated malformed line is an error, not a line to skip.
  {
    std::ofstream file(stream, std::ios::binary | std::ios::trunc);
    file << "{\"schema\":\"irmv.application_velocity\",\"version\":\n";
  }
  VelocityRecordRange corrupt;
  CHECK(!ReadVelocityRange(run, 0, kNever, &corrupt));
  CHECK(!corrupt.ok);
  CHECK(!corrupt.error.empty());

  // A complete line of another version is refused too, with its own message.
  {
    std::ofstream file(stream, std::ios::binary | std::ios::trunc);
    file << "{\"schema\":\"irmv.application_velocity\",\"version\":2}\n";
  }
  VelocityRecordRange unknown_line;
  CHECK(!ReadVelocityRange(run, 0, kNever, &unknown_line));
  CHECK(!unknown_line.ok);
  CHECK(!unknown_line.error.empty());

  // The metadata owns the run's schema identity; a reader refuses a version it
  // does not know instead of interpreting the fields of another one.
  const std::string metadata_text = ReadText(metadata_path);
  {
    nlohmann::json metadata = nlohmann::json::parse(metadata_text);
    metadata["version"] = 2;
    std::ofstream file(metadata_path, std::ios::binary | std::ios::trunc);
    file << metadata.dump(2);
  }
  VelocityRecordRange unknown_metadata;
  CHECK(!ReadVelocityRange(run, 0, kNever, &unknown_metadata));
  CHECK(!unknown_metadata.ok);
  CHECK(!unknown_metadata.error.empty());
  {
    std::ofstream file(metadata_path, std::ios::binary | std::ios::trunc);
    file << metadata_text;
  }

  // The index's schema identity is enforced the same way.
  {
    nlohmann::json index = nlohmann::json::parse(index_text);
    index["version"] = 2;
    std::ofstream file(index_path, std::ios::binary | std::ios::trunc);
    file << index.dump(2);
  }
  VelocityRecordRange unknown_index;
  CHECK(!ReadVelocityRange(run, 0, kNever, &unknown_index));
  CHECK(!unknown_index.ok);
  CHECK(!unknown_index.error.empty());

  // A recorder whose metadata declares another schema version never starts.
  VelocityRecorder refused;
  CHECK(!StartRecorder(&refused, parent, 256, 100, 2));
  CHECK(!refused.started());
  CHECK(!refused.error().empty());

  std::filesystem::remove_all(parent);
  return 0;
}

// A due sample is stored once, with the timestamp it was offered with. A
// schedule that missed several deadlines reports one sample, not a
// reconstructed burst; the recorder stores exactly what it was given.
int GapsAreNotBackfilled() {
  SampleSchedule schedule(100);  // 10 ms
  CHECK(schedule.Enabled());
  CHECK(schedule.NextDeadlineUs() == 0);

  CHECK(schedule.Due(1'000'000));
  CHECK(schedule.NextDeadlineUs() == 1'010'000);
  // Five periods later the schedule is due once and its next deadline is the
  // first grid point after now, not five rows into the past.
  CHECK(schedule.Due(1'050'000));
  CHECK(schedule.NextDeadlineUs() == 1'060'000);
  CHECK(!schedule.Due(1'050'001));

  const std::filesystem::path parent = MakeDirectory();
  VelocityRecorder recorder;
  CHECK(StartRecorder(&recorder, parent));
  SampleSchedule recording_schedule(100);
  std::uint64_t seq = 0;
  const TimeUs instants[] = {1'000'000, 1'050'000, 1'060'000};
  for (const TimeUs instant : instants) {
    if (!recording_schedule.Due(instant)) {
      continue;
    }
    CHECK(recorder.TryRecord(
        MakeSample(seq, instant, static_cast<double>(seq), 0.0, 0.0)));
    ++seq;
  }
  CHECK(recorder.Stop());

  VelocityRecordRange range;
  CHECK(ReadVelocityRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.size() == 3);
  if (range.rows.size() == 3) {
    CHECK(range.rows[0].stamp_us == 1'000'000);
    CHECK(range.rows[1].stamp_us == 1'050'000);
    CHECK(range.rows[2].stamp_us == 1'060'000);
    CHECK(range.rows[1].sample_seq == 1);
    CHECK(range.rows[2].sample_seq == 2);
  }

  std::filesystem::remove_all(parent);
  return 0;
}

// A directory that cannot be used is reported through error() and Start
// refuses; nothing is thrown and TryRecord stays a counted refusal.
int UnusableDirectoryIsReported() {
  VelocityRecorder recorder;
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.directory = "/nonexistent/irmv-velocity-directory";
  std::string error;
  CHECK(!recorder.Start(config, MakeMeta(100), &error));
  CHECK(!error.empty());
  CHECK(!recorder.started());
  CHECK(!recorder.TryRecord(MakeSample(0, 100, 0.0, 0.0, 0.0)));

  const std::filesystem::path parent = MakeDirectory();
  {
    const std::filesystem::path file_path = parent / "not_a_directory";
    std::ofstream(file_path) << "x\n";
    VelocityRecorder other;
    CHECK(!StartRecorder(&other, file_path));
    CHECK(!other.started());
    CHECK(!other.error().empty());
    CHECK(!other.TryRecord(MakeSample(0, 100, 0.0, 0.0, 0.0)));
  }
  std::filesystem::remove_all(parent);
  return 0;
}

int main() {
  int failures = 0;
  failures += DirectoryMetadataAndEmptyStream();
  failures += RoundTripAndRangeFilter();
  failures += IdentityTransitionDoesNotReuse();
  failures += QueueFullIsCountedAndSpanned();
  failures += BlockedWriterBoundsStop();
  failures += TornLineRollsBack();
  failures += TruncationCorruptAndUnknownVersion();
  failures += GapsAreNotBackfilled();
  failures += UnusableDirectoryIsReported();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
