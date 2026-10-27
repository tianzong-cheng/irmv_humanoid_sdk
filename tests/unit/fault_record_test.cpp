// Fault persistence: what the control thread may pay to report a fault, what
// the worker writes to faults.jsonl, and what a reader can tell about the
// occurrences it holds and the ones the run lost.
//
// The entry is built around the same two promises as the sample recorder:
// TryRecordFault never blocks, allocates or touches a file, and a full handoff
// costs the event - counted and visible as a loss span in the index - rather
// than the tick. A worker parked in a write must not hold Stop() past its
// deadline either.
#include "irmv/telemetry/fault_record.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "irmv/telemetry/recorder.hpp"
#include "support/check.hpp"
#include "support/fault_record_test_hooks.hpp"

namespace {

using irmv::protocol::FaultCondition;
using irmv::protocol::FaultDetail;
using irmv::protocol::FaultReason;
using irmv::protocol::OutputAction;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::telemetry::FaultLossReason;
using irmv::telemetry::FaultRecordRange;
using irmv::telemetry::kFaultRecordSchemaVersion;
using irmv::telemetry::kRecordGapSegments;
using irmv::telemetry::ReadFaultRange;
using irmv::telemetry::RecordedFault;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;

constexpr TimeUs kNever = std::numeric_limits<TimeUs>::max();

std::filesystem::path MakeDirectory() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("irmv-fault-record-test-" + std::to_string(now));
  std::filesystem::create_directory(directory);
  return directory;
}

bool StartRecorder(Recorder *recorder, const std::filesystem::path &directory) {
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.directory = directory;
  std::string error;
  const std::string metadata =
      std::string(R"({"schema_version":)") +
      std::to_string(irmv::telemetry::kRecordSchemaVersion) +
      R"(,"joints":["a","b"]})";
  const bool started = recorder->Start(config, metadata, &error);
  if (!started) {
    std::cerr << "recorder start failed: " << error << '\n';
  }
  return started;
}

nlohmann::json ReadIndex(const std::filesystem::path &run_directory) {
  std::ifstream file(run_directory / "index.json");
  if (!file) {
    return nlohmann::json{};
  }
  return nlohmann::json::parse(file, nullptr, false);
}

RecordedFault MakeFault(std::uint64_t record_seq, Sequence state_seq,
                        Sequence fault_id, TimeUs detected_stamp_us) {
  RecordedFault fault;
  fault.record_seq = record_seq;
  fault.runtime_id[0] = 0xab;
  fault.runtime_id[15] = 0xcd;
  fault.state_seq = state_seq;
  fault.fault_id = fault_id;
  fault.detail.reason = FaultReason::JointPositionExceeded;
  fault.detail.condition = FaultCondition::JointPositionExceeded;
  fault.detail.joint_index = 3;
  fault.detail.error_code = 0xDEADBEEFU;
  fault.detail.observed_value = 1.25;
  fault.detail.limit = 1.0;
  fault.detail.sample_stamp_us = detected_stamp_us - 7;
  fault.detail.detected_stamp_us = detected_stamp_us;
  fault.detail.output_action = OutputAction::ZeroTorque;
  static_cast<void>(fault.detail.message.Assign("joint position limit"));
  return fault;
}

bool SameFault(const RecordedFault &left, const RecordedFault &right) {
  return left.record_seq == right.record_seq &&
         left.runtime_id == right.runtime_id &&
         left.state_seq == right.state_seq && left.fault_id == right.fault_id &&
         left.detail.reason == right.detail.reason &&
         left.detail.condition == right.detail.condition &&
         left.detail.joint_index == right.detail.joint_index &&
         left.detail.error_code == right.detail.error_code &&
         left.detail.sample_stamp_us == right.detail.sample_stamp_us &&
         left.detail.detected_stamp_us == right.detail.detected_stamp_us &&
         left.detail.output_action == right.detail.output_action &&
         left.detail.message.View() == right.detail.message.View();
}

std::string ReadFile(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

// Three different details in the order they occurred, with 64-bit sequences,
// codes and stamps, and a range read that returns exactly one of them. The
// recorder is not allowed to deduplicate: a repeated occurrence is a second
// event the state machine already decided to report.
int ThreeFaultsInOrder() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));

  const std::uint64_t big_record = (1ULL << 63) + 11;
  const Sequence big_state = (1ULL << 63) + 5;
  const Sequence big_fault = (1ULL << 63) + 7;
  const TimeUs big_stamp = (1ULL << 63) + 1234;
  RecordedFault first = MakeFault(big_record, big_state, big_fault, big_stamp);
  first.detail.reason = FaultReason::FeedbackInvalid;
  first.detail.condition = FaultCondition::JointFeedbackNonFinite;
  first.detail.joint_index = 0;
  first.detail.observed_value = std::numeric_limits<double>::quiet_NaN();
  first.detail.limit = 0.5;
  static_cast<void>(first.detail.message.Assign("non-finite feedback"));
  RecordedFault second =
      MakeFault(big_record + 1, big_state + 1, big_fault, big_stamp + 10);
  second.detail.reason = FaultReason::ImuInvalid;
  second.detail.condition = FaultCondition::ImuFeedbackNonFinite;
  second.detail.joint_index = irmv::protocol::kNoJointIndex;
  second.detail.error_code = 0;
  second.detail.observed_value = 2.0;
  second.detail.limit = std::numeric_limits<double>::infinity();
  static_cast<void>(second.detail.message.Assign("imu non-finite"));
  RecordedFault third =
      MakeFault(big_record + 2, big_state + 2, big_fault + 1, big_stamp + 20);
  third.detail.reason = FaultReason::DriverError;
  third.detail.condition = FaultCondition::JointDriverError;
  third.detail.joint_index = 63;
  third.detail.observed_value = -3.0;
  third.detail.limit = -std::numeric_limits<double>::infinity();
  static_cast<void>(third.detail.message.Assign("driver refused"));
  // The same detail as the second occurrence, reported again on a later tick:
  // a duplicate of the condition is still a distinct event.
  RecordedFault duplicate = second;
  duplicate.record_seq = big_record + 3;
  duplicate.state_seq = big_state + 3;
  duplicate.detail.detected_stamp_us = big_stamp + 30;
  duplicate.detail.sample_stamp_us = big_stamp + 23;

  CHECK(recorder.TryRecordFault(first));
  CHECK(recorder.TryRecordFault(second));
  CHECK(recorder.TryRecordFault(third));
  CHECK(recorder.TryRecordFault(duplicate));
  CHECK(recorder.Stop());
  CHECK(recorder.faults_recorded() == 4);
  CHECK(recorder.faults_written() == 4);
  CHECK(recorder.fault_write_failures() == 0);
  CHECK(recorder.faults_dropped() == 0);
  CHECK(recorder.faults_generation_dropped() == 0);

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.error.empty());
  CHECK(range.complete);
  CHECK(!range.truncated);
  CHECK(range.losses.empty());
  CHECK(range.rows.size() == 4);
  if (range.rows.size() == 4) {
    CHECK(SameFault(range.rows[0], first));
    CHECK(SameFault(range.rows[1], second));
    CHECK(SameFault(range.rows[2], third));
    CHECK(SameFault(range.rows[3], duplicate));
    // The doubles that JSON cannot represent are explicit strings on disk and
    // are restored by the reader; null is never used for them.
    CHECK(std::isnan(range.rows[0].detail.observed_value));
    CHECK(range.rows[0].detail.limit == 0.5);
    CHECK(std::isinf(range.rows[1].detail.limit));
    CHECK(range.rows[1].detail.limit > 0.0);
    CHECK(std::isinf(range.rows[2].detail.limit));
    CHECK(range.rows[2].detail.limit < 0.0);
    CHECK(range.rows[1].detail.joint_index == irmv::protocol::kNoJointIndex);
    CHECK(range.rows[2].detail.joint_index == 63);
    CHECK(range.rows[0].runtime_id == first.runtime_id);
    CHECK(range.rows[0].state_seq == big_state);
    CHECK(range.rows[0].fault_id == big_fault);
    CHECK(std::string(range.rows[0].detail.message.View()) ==
          "non-finite feedback");
  }

  // The range is half open and filters on the detected stamp, not on the order
  // of the lines.
  FaultRecordRange second_only;
  CHECK(ReadFaultRange(recorder.run_directory(), big_stamp + 10, big_stamp + 20,
                       &second_only));
  CHECK(second_only.rows.size() == 1);
  if (second_only.rows.size() == 1) {
    CHECK(second_only.rows[0].record_seq == big_record + 1);
  }
  FaultRecordRange empty;
  CHECK(ReadFaultRange(recorder.run_directory(), big_stamp + 20, big_stamp + 20,
                       &empty));
  CHECK(empty.ok);
  CHECK(empty.rows.empty());
  CHECK(empty.losses.empty());

  // One JSON object per line, each terminated: a run writes exactly the
  // accepted occurrences.
  const std::string lines = ReadFile(recorder.run_directory() / "faults.jsonl");
  CHECK(!lines.empty());
  CHECK(lines.back() == '\n');
  CHECK(static_cast<std::size_t>(
            std::count(lines.begin(), lines.end(), '\n')) == 4);
  CHECK(lines.find("\"NaN\"") != std::string::npos);
  CHECK(lines.find("\"+Inf\"") != std::string::npos);
  CHECK(lines.find("\"-Inf\"") != std::string::npos);
  CHECK(lines.find("\"state_seq\":\"8000000000000005\"") != std::string::npos);
  // The line schema is the fault stream's own version, not the binary chunk
  // schema the run metadata uses.
  const nlohmann::json first_line =
      nlohmann::json::parse(lines.substr(0, lines.find('\n')), nullptr, false);
  CHECK(!first_line.is_discarded());
  CHECK(first_line["schema_version"] ==
        irmv::telemetry::kFaultRecordSchemaVersion);
  CHECK(first_line["schema_version"] != irmv::telemetry::kRecordSchemaVersion);

  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["faults"]["complete"] == true);
  CHECK(index["faults"]["recorded"] == 4);
  CHECK(index["faults"]["written"] == 4);
  CHECK(index["faults"]["queue_dropped"] == 0);
  CHECK(index["faults"]["generation_dropped"] == 0);
  CHECK(index["faults"]["losses"].empty());

  std::filesystem::remove_all(directory);
  return 0;
}

// A run that saw no fault still has a faults.jsonl and a complete index, and a
// reader reports zero rows without calling that an error.
int NoFaultsIsComplete() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.Stop());
  CHECK(std::filesystem::exists(recorder.run_directory() / "faults.jsonl"));
  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.empty());
  CHECK(range.losses.empty());
  CHECK(range.complete);
  CHECK(!range.truncated);
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["faults"]["complete"] == true);
  CHECK(index["faults"]["written"] == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

// The recorder creates `faults.jsonl` before it accepts the first occurrence,
// so its absence is lost evidence, never a run that simply saw no faults. This
// holds for a run that recorded occurrences and for a finalized zero-fault run.
int MissingSidecarIsAHardError() {
  {
    const std::filesystem::path directory = MakeDirectory();
    Recorder recorder;
    CHECK(StartRecorder(&recorder, directory));
    CHECK(recorder.TryRecordFault(MakeFault(1, 10, 20, 1000)));
    CHECK(recorder.Stop());
    const std::filesystem::path faults_path =
        recorder.run_directory() / "faults.jsonl";
    CHECK(std::filesystem::exists(faults_path));
    std::filesystem::remove(faults_path);

    FaultRecordRange range;
    CHECK(!ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
    CHECK(!range.ok);
    CHECK(range.rows.empty());
    CHECK(range.error.find("faults.jsonl") != std::string::npos);
    std::filesystem::remove_all(directory);
  }
  {
    const std::filesystem::path directory = MakeDirectory();
    Recorder recorder;
    CHECK(StartRecorder(&recorder, directory));
    CHECK(recorder.Stop());
    const std::filesystem::path faults_path =
        recorder.run_directory() / "faults.jsonl";
    CHECK(std::filesystem::exists(faults_path));
    const nlohmann::json index = ReadIndex(recorder.run_directory());
    CHECK(index["faults"]["complete"] == true);
    CHECK(index["faults"]["written"] == 0);
    std::filesystem::remove(faults_path);

    FaultRecordRange range;
    CHECK(!ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
    CHECK(!range.ok);
    CHECK(range.rows.empty());
    CHECK(range.error.find("faults.jsonl") != std::string::npos);
    std::filesystem::remove_all(directory);
  }
  return 0;
}

// A trailing line without its newline is a write that never finished: the
// complete prefix is read and the tail is reported truncated. Without the final
// completion marker in the index the run reads complete=false.
int TrailingPartialAndMissingCompletion() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(MakeFault(1, 10, 20, 1000)));
  CHECK(recorder.Stop());

  const std::filesystem::path faults_path =
      recorder.run_directory() / "faults.jsonl";
  {
    std::ofstream file(faults_path, std::ios::app | std::ios::binary);
    file << "{\"schema_version\":1,\"record_seq\":2";
  }
  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.size() == 1);
  CHECK(range.truncated);
  CHECK(range.complete);

  // The final marker is what proves the worker finalized the fault stream.
  std::filesystem::remove(recorder.run_directory() / "index.json");
  FaultRecordRange unmarked;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &unmarked));
  CHECK(unmarked.ok);
  CHECK(unmarked.rows.size() == 1);
  CHECK(unmarked.truncated);
  CHECK(!unmarked.complete);

  // A newline-terminated line that is not a valid occurrence is an error, not
  // a line to skip.
  {
    std::ofstream file(faults_path, std::ios::binary | std::ios::trunc);
    file << "{\"schema_version\":1,\"record_seq\":1,\"runtime_id\":"
            "\"ab0000000000000000000000000000cd\",\"state_seq\":"
            "\"000000000000000a\",\"fault_id\":\"0000000000000014\","
            "\"reason\":5,\"reason_name\":\"JointPositionExceeded\","
            "\"condition\":8,\"condition_name\":\"JointPositionExceeded\","
            "\"joint_index\":3,\"error_code\":3735928559,"
            "\"observed_value\":1.25,\"limit\":1.0,"
            "\"sample_stamp_us\":993,\"detected_stamp_us\":1000,"
            "\"output_action\":0,\"message\":\"ok\","
            "\"message_hex\":\"6f6b\"}\n"
            "{\"schema_version\":1,\"record_seq\":\n";
  }
  FaultRecordRange malformed;
  CHECK(!ReadFaultRange(recorder.run_directory(), 0, kNever, &malformed));
  CHECK(!malformed.ok);
  CHECK(!malformed.error.empty());

  std::filesystem::remove_all(directory);
  return 0;
}

// A failed line write is counted, reported as a loss, and never claims
// persistence: the final fault completion marker stays false and the row is
// absent from the file.
int WriteFailureDoesNotClaimPersistence() {
  struct FailHook {
    std::atomic<bool> fail{true};
  } state;
  irmv::telemetry::SetFaultLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        return static_cast<FailHook *>(context)->fail.load() ? 1 : 0;
      },
      &state);

  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(MakeFault(7, 70, 700, 7000)));
  CHECK(recorder.TryRecordFault(MakeFault(8, 71, 700, 7010)));
  CHECK(recorder.Stop());
  CHECK(recorder.fault_write_failures() == 2);
  CHECK(recorder.faults_written() == 0);
  CHECK(recorder.faults_recorded() == 2);

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.empty());
  CHECK(!range.complete);
  CHECK(range.losses.size() == 1);
  if (range.losses.size() == 1) {
    CHECK(range.losses[0].reason == FaultLossReason::WriteFailed);
    CHECK(range.losses[0].dropped == 2);
    CHECK(range.losses[0].first_record_seq == 7);
    CHECK(range.losses[0].last_record_seq == 8);
    CHECK(range.losses[0].first_state_seq == 70);
    CHECK(range.losses[0].last_state_seq == 71);
  }
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["faults"]["complete"] == false);
  CHECK(index["faults"]["written"] == 0);
  CHECK(index["faults"]["write_failures"] == 2);
  CHECK(index["faults"]["losses"].size() == 1);
  CHECK(index["faults"]["losses"][0]["reason"] == "write_failed");

  irmv::telemetry::SetFaultLineWriteHookForTest(nullptr, nullptr);
  state.fail.store(false);
  std::filesystem::remove_all(directory);
  return 0;
}

// A worker parked inside a fault write must not hold Stop(): the deadline
// expires, the worker keeps its own state, and once the device drains it
// finalizes the run. Generation-side and queue-side losses are counted apart,
// and a full fault queue does not consume the sample channel's room.
int BlockedWriterAndLossAccounting() {
  struct BlockHook {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered{false};
    bool released{false};
    bool block_first{true};
  } state;
  irmv::telemetry::SetFaultLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        auto *hook = static_cast<BlockHook *>(context);
        std::unique_lock<std::mutex> lock(hook->mutex);
        if (!hook->block_first) {
          return 0;
        }
        hook->block_first = false;
        hook->entered = true;
        hook->cv.notify_all();
        hook->cv.wait(lock, [hook] { return hook->released; });
        return 0;
      },
      &state);

  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(MakeFault(1, 100, 900, 10000)));
  {
    std::unique_lock<std::mutex> lock(state.mutex);
    const bool entered = state.cv.wait_for(lock, std::chrono::seconds(2),
                                           [&state] { return state.entered; });
    CHECK(entered);
  }

  // The worker is parked in the first line. The fault handoff now fills and
  // refuses; the sample handoff is a different lane and still takes samples.
  constexpr std::uint64_t kFlood = 1200;
  for (std::uint64_t index = 0; index < kFlood; ++index) {
    const std::uint64_t record_seq = 2 + index;
    const Sequence state_seq = 101 + index;
    static_cast<void>(recorder.TryRecordFault(
        MakeFault(record_seq, state_seq, 900, 10000 + state_seq)));
  }
  CHECK(recorder.faults_dropped() > 0);
  CHECK(recorder.faults_recorded() > 0);
  const std::uint64_t sample_dropped_before = recorder.dropped();
  CHECK(recorder.TryRecord(irmv::telemetry::RecordSample{}));
  CHECK(recorder.dropped() == sample_dropped_before);
  CHECK(recorder.recorded() == 1);

  // Generation-side loss is a different fact: the state machine could not even
  // enumerate these occurrences. It is counted apart from the queue refusals.
  recorder.ReportFaultGenerationLoss(20000, 5000, 3);
  recorder.ReportFaultGenerationLoss(20010, 5001, 6);
  CHECK(recorder.faults_generation_dropped() == 9);

  const auto started = std::chrono::steady_clock::now();
  CHECK(!recorder.Stop());
  const auto elapsed = std::chrono::steady_clock::now() - started;
  // The deadline is what bounds the return, not the wall clock: under load or a
  // sanitizer the exact two seconds may stretch. Returning at all while the
  // worker is still parked proves Stop() did not wait for the writer.
  CHECK(elapsed >= std::chrono::milliseconds(500));
  CHECK(elapsed < std::chrono::seconds(10));
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["faults"]["complete"] == false);

  {
    const std::lock_guard<std::mutex> lock(state.mutex);
    state.released = true;
  }
  state.cv.notify_all();

  // The second Stop observes the worker's own completion and joins it.
  bool joined = false;
  for (int attempt = 0; attempt < 500 && !joined; ++attempt) {
    joined = recorder.Stop();
    if (!joined) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  CHECK(joined);
  CHECK(recorder.fault_write_failures() == 0);
  CHECK(recorder.faults_generation_dropped() == 9);
  const std::uint64_t written = recorder.faults_written();
  CHECK(written == recorder.faults_recorded());
  CHECK(written >= 1024);
  if (written >= 1024) {
    CHECK(written <= 1025);
  }

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.complete);
  CHECK(!range.truncated);
  CHECK(range.rows.size() == written);
  if (!range.rows.empty()) {
    CHECK(range.rows.front().record_seq == 1);
    // The accepted flood occurrences are the ones that filled the lane: their
    // ordinals are 2..written.
    CHECK(range.rows.back().record_seq == written);
  }
  // Losses: one queue-full stretch whose bounds name the refused occurrences,
  // and two generation reports that never had a record sequence of their own.
  std::uint64_t queue_dropped = 0;
  std::uint64_t generation_dropped = 0;
  std::size_t generation_spans = 0;
  std::size_t queue_spans = 0;
  for (const auto &loss : range.losses) {
    if (loss.reason == FaultLossReason::Generation) {
      generation_dropped += loss.dropped;
      ++generation_spans;
      CHECK(loss.first_record_seq == 0);
      CHECK(loss.last_record_seq == 0);
    } else if (loss.reason == FaultLossReason::QueueFull) {
      queue_dropped += loss.dropped;
      ++queue_spans;
    }
  }
  CHECK(generation_dropped == 9);
  CHECK(generation_spans == 2);
  CHECK(queue_spans == 1);
  CHECK(queue_dropped == recorder.faults_dropped());
  CHECK(queue_dropped == kFlood - (written - 1));
  if (queue_spans == 1) {
    for (const auto &loss : range.losses) {
      if (loss.reason != FaultLossReason::QueueFull) {
        continue;
      }
      CHECK(loss.first_record_seq == 2 + (written - 1));
      CHECK(loss.last_record_seq == kFlood + 1);
      CHECK(loss.dropped == kFlood - (written - 1));
      CHECK(loss.first_state_seq == 101 + (written - 1));
      CHECK(loss.last_state_seq == 101 + kFlood - 1);
      CHECK(loss.first_stamp_us == 10000 + loss.first_state_seq);
      CHECK(loss.last_stamp_us == 10000 + loss.last_state_seq);
    }
  }

  const nlohmann::json index_after = ReadIndex(recorder.run_directory());
  CHECK(index_after["faults"]["complete"] == true);
  CHECK(index_after["faults"]["generation_dropped"] == 9);
  CHECK(index_after["faults"]["queue_dropped"] == recorder.faults_dropped());

  irmv::telemetry::SetFaultLineWriteHookForTest(nullptr, nullptr);
  std::filesystem::remove_all(directory);
  return 0;
}

// A torn write is rolled back to the last committed line. Without the rollback
// the next complete line would merge with the torn bytes and the reader would
// reject the whole sidecar as malformed, losing every earlier row too.
int TornLineRollsBack() {
  struct TornHook {
    std::atomic<bool> first{true};
  } state;
  irmv::telemetry::SetFaultLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        auto *hook = static_cast<TornHook *>(context);
        return hook->first.exchange(false) ? 2 : 0;
      },
      &state);

  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(MakeFault(1, 10, 20, 1000)));
  CHECK(recorder.TryRecordFault(MakeFault(2, 11, 20, 1010)));
  CHECK(recorder.TryRecordFault(MakeFault(3, 12, 20, 1020)));
  CHECK(recorder.Stop());
  CHECK(recorder.faults_recorded() == 3);
  CHECK(recorder.faults_written() == 2);
  CHECK(recorder.fault_write_failures() == 1);

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(!range.truncated);
  CHECK(!range.complete);
  CHECK(range.rows.size() == 2);
  if (range.rows.size() == 2) {
    CHECK(range.rows[0].record_seq == 2);
    CHECK(range.rows[1].record_seq == 3);
  }
  CHECK(range.losses.size() == 1);
  if (range.losses.size() == 1) {
    CHECK(range.losses[0].reason == FaultLossReason::WriteFailed);
    CHECK(range.losses[0].dropped == 1);
    CHECK(range.losses[0].first_record_seq == 1);
    CHECK(range.losses[0].last_record_seq == 1);
  }
  const std::string content =
      ReadFile(recorder.run_directory() / "faults.jsonl");
  CHECK(static_cast<std::size_t>(
            std::count(content.begin(), content.end(), '\n')) == 2);
  CHECK(content.find("\"record_seq\":1,") == std::string::npos);
  CHECK(content.find("\"record_seq\":2,") != std::string::npos);
  CHECK(content.find("\"record_seq\":3,") != std::string::npos);

  irmv::telemetry::SetFaultLineWriteHookForTest(nullptr, nullptr);
  std::filesystem::remove_all(directory);
  return 0;
}

// The protocol message is raw fixed-capacity bytes, not UTF-8. The exact bytes
// - including invalid sequences, NULs and the 256-byte boundary - must return
// unchanged, even though the human `message` field had to replace them.
int MessageBytesRoundTrip() {
  std::string raw(256, 'x');
  raw[0] = static_cast<char>(0xFF);
  raw[1] = static_cast<char>(0xFE);
  raw[2] = '\0';
  raw[3] = '"';
  raw[4] = '\\';
  raw[255] = static_cast<char>(0x80);
  RecordedFault fault = MakeFault(1, 10, 20, 1000);
  CHECK(fault.detail.message.Assign(raw));

  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(fault));
  CHECK(recorder.Stop());
  CHECK(recorder.faults_written() == 1);

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.complete);
  CHECK(range.rows.size() == 1);
  if (range.rows.size() == 1) {
    CHECK(std::string(range.rows[0].detail.message.View()) == raw);
  }
  const std::string content =
      ReadFile(recorder.run_directory() / "faults.jsonl");
  CHECK(content.find("\"message_hex\":\"fffe00225c") != std::string::npos);
  CHECK(content.find("80\"") != std::string::npos);

  std::filesystem::remove_all(directory);
  return 0;
}

// The producer-side loss table is fixed capacity. Past 64 separate generation
// reports the last span absorbs the rest: the span count degrades, but no loss
// disappears from the total.
int LossSpanTableFoldRetainsTotals() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  for (std::uint64_t index = 0; index < 70; ++index) {
    recorder.ReportFaultGenerationLoss(1000 + index, 100 + index, 1);
  }
  CHECK(recorder.faults_generation_dropped() == 70);
  CHECK(recorder.Stop());

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.complete);
  CHECK(range.losses.size() == 64);
  std::uint64_t total = 0;
  for (const auto &loss : range.losses) {
    CHECK(loss.reason == FaultLossReason::Generation);
    total += loss.dropped;
  }
  CHECK(total == 70);
  if (range.losses.size() == 64) {
    // Spans 1..64 are stored; reports 65..70 fold into the 64th span.
    CHECK(range.losses[63].first_state_seq == 163);
    CHECK(range.losses[63].last_state_seq == 169);
    CHECK(range.losses[63].dropped == 7);
  }

  std::filesystem::remove_all(directory);
  return 0;
}

// The worker-side write-failure span table is bounded like the producer
// tables. Alternating failures and successes create one span per failure; past
// 64 they fold into the last one, which must keep the total without growing the
// index.
int WriteFailureLossSpansAreBounded() {
  struct AlternatingHook {
    std::atomic<std::uint64_t> calls{0};
  } state;
  irmv::telemetry::SetFaultLineWriteHookForTest(
      [](void *context, std::string_view) -> int {
        auto *hook = static_cast<AlternatingHook *>(context);
        return (hook->calls.fetch_add(1) % 2 == 0) ? 1 : 0;
      },
      &state);

  constexpr std::uint64_t kEvents = 600;
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  for (std::uint64_t index = 0; index < kEvents; ++index) {
    CHECK(recorder.TryRecordFault(MakeFault(
        index + 1, 1000 + index, 5000 + index, 100'000 + index * 10)));
  }
  CHECK(recorder.Stop());
  const std::uint64_t failures = recorder.fault_write_failures();
  CHECK(failures == kEvents / 2);
  CHECK(recorder.faults_written() == kEvents / 2);
  CHECK(recorder.faults_recorded() == kEvents);

  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(!range.complete);
  CHECK(range.losses.size() <= kRecordGapSegments);
  std::uint64_t dropped = 0;
  for (const auto &loss : range.losses) {
    CHECK(loss.reason == FaultLossReason::WriteFailed);
    dropped += loss.dropped;
    // The folded table retains the first span's start and extends the last.
    CHECK(loss.first_record_seq != 0);
    CHECK(loss.last_record_seq >= loss.first_record_seq);
  }
  CHECK(dropped == failures);

  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index.is_object());
  CHECK(index["faults"]["complete"] == false);
  CHECK(index["faults"]["write_failures"] == failures);
  CHECK(index["faults"]["losses"].size() <= kRecordGapSegments);
  std::uint64_t index_dropped = 0;
  for (const auto &loss : index["faults"]["losses"]) {
    index_dropped += loss["dropped"].get<std::uint64_t>();
  }
  CHECK(index_dropped == failures);
  if (index["faults"]["losses"].size() == kRecordGapSegments) {
    // The first failure starts the retained first span; the last reported
    // failure ends the folded last span.
    CHECK(index["faults"]["losses"][0]["first_record_seq"] == 1);
    CHECK(index["faults"]["losses"].back()["last_record_seq"] == kEvents - 1);
  }

  irmv::telemetry::SetFaultLineWriteHookForTest(nullptr, nullptr);
  std::filesystem::remove_all(directory);
  return 0;
}

// An index without the fault block (a run from a build before fault
// persistence, or one whose section was not published) is readable and
// incomplete, not an error.
int IndexWithoutFaultBlockReadsIncomplete() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(MakeFault(1, 10, 20, 1000)));
  CHECK(recorder.Stop());
  {
    std::ofstream file(recorder.run_directory() / "index.json",
                       std::ios::binary | std::ios::trunc);
    file << "{\"schema_version\":4,\"complete\":true}\n";
  }
  FaultRecordRange range;
  CHECK(ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(range.ok);
  CHECK(range.rows.size() == 1);
  CHECK(range.losses.empty());
  CHECK(!range.complete);
  CHECK(!range.truncated);
  std::filesystem::remove_all(directory);
  return 0;
}

}  // namespace

// A complete fault line of another schema version is refused by name, never
// skipped or reinterpreted. The writer first produces a valid line, which is
// then relabelled, so every other field is exactly what this build writes.
int WrongLineSchemaVersionIsRefused() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  CHECK(recorder.TryRecordFault(MakeFault(1, 10, 20, 1000)));
  CHECK(recorder.Stop());

  const std::filesystem::path faults_path =
      recorder.run_directory() / "faults.jsonl";
  std::string text = ReadFile(faults_path);
  const std::uint32_t wrong_version = kFaultRecordSchemaVersion + 1;
  const std::string from =
      "\"schema_version\":" + std::to_string(kFaultRecordSchemaVersion);
  const std::size_t position = text.find(from);
  CHECK(position != std::string::npos);
  if (position != std::string::npos) {
    text.replace(position, from.size(),
                 "\"schema_version\":" + std::to_string(wrong_version));
  }
  {
    std::ofstream file(faults_path, std::ios::binary | std::ios::trunc);
    file << text;
  }

  FaultRecordRange range;
  CHECK(!ReadFaultRange(recorder.run_directory(), 0, kNever, &range));
  CHECK(!range.ok);
  CHECK(!range.error.empty());
  CHECK(range.error.find("schema") != std::string::npos);
  CHECK(range.error.find(std::to_string(wrong_version)) != std::string::npos);
  CHECK(range.rows.empty());

  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += ThreeFaultsInOrder();
  failures += NoFaultsIsComplete();
  failures += MissingSidecarIsAHardError();
  failures += TrailingPartialAndMissingCompletion();
  failures += WriteFailureDoesNotClaimPersistence();
  failures += BlockedWriterAndLossAccounting();
  failures += TornLineRollsBack();
  failures += MessageBytesRoundTrip();
  failures += LossSpanTableFoldRetainsTotals();
  failures += WriteFailureLossSpansAreBounded();
  failures += IndexWithoutFaultBlockReadsIncomplete();
  failures += WrongLineSchemaVersionIsRefused();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
