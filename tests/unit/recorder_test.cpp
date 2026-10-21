// The recorder: what the control thread may pay for a sample, what the worker
// writes, and what a run says about the samples it never recorded.
//
// The entry is built around the two promises that matter to the control loop:
// TryRecord never blocks, allocates or touches a file, and a full handoff costs
// the sample - counted and visible as a gap in the run - rather than the tick.
#include "irmv/telemetry/recorder.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "irmv/telemetry/record.hpp"
#include "irmv/telemetry/record_reader.hpp"
#include "support/check.hpp"
#include "support/recorder_test_hooks.hpp"

namespace {

using irmv::protocol::ExecutionState;
using irmv::protocol::OutputAction;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::telemetry::kRecordChunkRows;
using irmv::telemetry::kRecordGapSegments;
using irmv::telemetry::kRecordQueueCapacity;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;
using irmv::telemetry::RecordGap;
using irmv::telemetry::RecordGapReason;
using irmv::telemetry::RecordReader;
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

// One authorization context the session tests hand to the recorder: the
// producer and epoch of the adopted context, the gains of its first command,
// and the literal digest the runtime writes because it holds no model.
irmv::telemetry::RecordSessionEvent MakeSessionEvent(Sequence epoch) {
  irmv::telemetry::RecordSessionEvent event;
  event.runtime_id[0] = 0x11;
  event.session_id[0] = 0x22;
  event.producer_id[0] = 0x33;
  event.epoch = epoch;
  event.stamp_us = 5000 + epoch * 10;
  event.state_seq = epoch;
  event.joint_count = 2;
  event.kp[0] = 60.0;
  event.kp[1] = 61.0;
  event.kd[0] = 3.0;
  event.kd[1] = 4.0;
  std::memcpy(event.model_sha256.data(), "unknown", sizeof("unknown"));
  return event;
}

// Starts a recorder whose metadata a reader can open: the joint table and count
// must agree, because the reader checks them before handing out a row.
bool StartRecorderForSessions(Recorder *recorder,
                              const std::filesystem::path &directory) {
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.directory = directory;
  std::string error;
  const std::string metadata =
      std::string(R"({"schema_version":)") +
      std::to_string(irmv::telemetry::kRecordSchemaVersion) +
      R"(,"config_id":"cfg","layout_id":"lay","backend":"fake","control_hz":100.0,"sample_hz":100,"joint_count":2,"joints":["a","b"],"units":{"position":"rad"}})";
  return recorder->Start(config, metadata, &error);
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

std::uint64_t CountChunks(const std::filesystem::path &run_directory) {
  std::uint64_t count = 0;
  for (const auto &entry : std::filesystem::directory_iterator(run_directory)) {
    const std::string name = entry.path().filename().string();
    // A regular file only: a directory that happens to be named like a chunk is
    // how a later test obstructs a rename, and it is not a written chunk.
    if (name.rfind("chunk_", 0) == 0 && entry.path().extension() == ".bin" &&
        std::filesystem::is_regular_file(entry.path())) {
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

// One sample offered to a recorder and whether the handoff took it. A test that
// asserts on refusals has to know which offers were refused: the handoff is
// drained by a worker on its own clock, so the offers are logged and the
// expectation is derived from what the recorder really did rather than from
// when the worker happened to wake up.
struct OfferRecord {
  Sequence seq{0};
  TimeUs stamp_us{0};
  bool accepted{false};
};

bool Offer(Recorder *recorder, std::vector<OfferRecord> *log, Sequence seq) {
  const RecordSample sample = MakeSample(seq);
  const bool accepted = recorder->TryRecord(sample);
  log->push_back(OfferRecord{seq, sample.stamp_us, accepted});
  return accepted;
}

// Offers a sample until the handoff takes it, the way a control loop that wants
// every sample recorded behaves: the queue is bounded and the worker drains it
// on its own clock, so a busy handoff is room to wait for, not a loss.
bool OfferUntilAccepted(Recorder *recorder, std::vector<OfferRecord> *log,
                        Sequence seq) {
  for (int attempt = 0; attempt < 500; ++attempt) {
    if (Offer(recorder, log, seq)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

// Groups the refusals a test saw the way the control thread groups them: a
// refusal extends the current stretch when it is the same sample offered again
// or the next sample of the stride, and nothing was accepted since the stretch
// opened; anything else starts a new stretch. The table must agree with this
// offer by offer, which is what makes "two refusals a minute apart are two
// losses" checkable.
std::vector<irmv::telemetry::RecordGap> ExpectedQueueGaps(
    const std::vector<OfferRecord> &offers, Sequence stride) {
  std::vector<irmv::telemetry::RecordGap> gaps;
  bool open = false;
  for (const OfferRecord &offer : offers) {
    if (offer.accepted) {
      open = false;
      continue;
    }
    const bool continues = open && (offer.seq == gaps.back().last_seq ||
                                    offer.seq == gaps.back().last_seq + stride);
    if (!continues) {
      irmv::telemetry::RecordGap gap;
      gap.first_seq = offer.seq;
      gap.first_stamp_us = offer.stamp_us;
      gap.reason = RecordGapReason::QueueFull;
      gaps.push_back(gap);
      open = true;
    }
    gaps.back().last_seq = offer.seq;
    gaps.back().last_stamp_us = offer.stamp_us;
  }
  for (irmv::telemetry::RecordGap &gap : gaps) {
    gap.dropped = (gap.last_seq - gap.first_seq) / stride + 1;
  }
  return gaps;
}

std::vector<irmv::telemetry::RecordGap> QueueGaps(
    const std::vector<irmv::telemetry::RecordGap> &gaps) {
  std::vector<irmv::telemetry::RecordGap> queue_gaps;
  for (const irmv::telemetry::RecordGap &gap : gaps) {
    if (gap.reason == RecordGapReason::QueueFull) {
      queue_gaps.push_back(gap);
    }
  }
  return queue_gaps;
}

RecordGapReason ReasonFromName(const std::string &name) {
  if (name == "queue_full") {
    return RecordGapReason::QueueFull;
  }
  if (name == "write_failed") {
    return RecordGapReason::WriteFailed;
  }
  if (name == "stopped") {
    return RecordGapReason::Stopped;
  }
  return RecordGapReason::None;
}

// The gap table as the run's own index carries it: what a reader of the files
// sees, field by field.
std::vector<irmv::telemetry::RecordGap> IndexGaps(const nlohmann::json &index) {
  std::vector<irmv::telemetry::RecordGap> gaps;
  for (const auto &entry : index["gaps"]) {
    irmv::telemetry::RecordGap gap;
    gap.first_seq = entry["first_seq"].get<Sequence>();
    gap.last_seq = entry["last_seq"].get<Sequence>();
    gap.first_stamp_us = entry["first_stamp_us"].get<TimeUs>();
    gap.last_stamp_us = entry["last_stamp_us"].get<TimeUs>();
    gap.dropped = entry["dropped"].get<std::uint64_t>();
    gap.reason = ReasonFromName(entry["reason"].get<std::string>());
    gaps.push_back(gap);
  }
  return gaps;
}

int CheckSameGaps(const std::vector<irmv::telemetry::RecordGap> &actual,
                  const std::vector<irmv::telemetry::RecordGap> &expected) {
  CHECK(actual.size() == expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    CHECK(actual[index].first_seq == expected[index].first_seq);
    CHECK(actual[index].last_seq == expected[index].last_seq);
    CHECK(actual[index].first_stamp_us == expected[index].first_stamp_us);
    CHECK(actual[index].last_stamp_us == expected[index].last_stamp_us);
    CHECK(actual[index].dropped == expected[index].dropped);
    CHECK(actual[index].reason == expected[index].reason);
  }
  return 0;
}

// No refusal may cover a sample the run recorded: one stretch of refusals
// spanning the samples that went in fine is exactly what this table exists to
// avoid. (A chunk that could not be written covers samples the handoff did
// accept - those are the rows the failed write lost - so only refusals are
// checked here.)
int CheckRefusalGapsMissRecordedOffers(
    const std::vector<irmv::telemetry::RecordGap> &gaps,
    const std::vector<OfferRecord> &offers) {
  for (const irmv::telemetry::RecordGap &gap : gaps) {
    if (gap.reason != RecordGapReason::QueueFull) {
      continue;
    }
    for (const OfferRecord &offer : offers) {
      if (offer.accepted) {
        CHECK(!(offer.seq >= gap.first_seq && offer.seq <= gap.last_seq));
      }
    }
  }
  return 0;
}

// The producer pause is a plain function pointer, so the state it uses lives
// here and the hook touches no recorder object.
struct DecrementPause {
  std::mutex mutex;
  std::condition_variable cv;
  bool entered{false};
  bool released{false};
};

DecrementPause *g_decrement_pause = nullptr;

void PauseAfterDecrement() {
  DecrementPause *const pause = g_decrement_pause;
  std::unique_lock<std::mutex> lock(pause->mutex);
  pause->entered = true;
  pause->cv.notify_all();
  pause->cv.wait(lock, [pause] { return pause->released; });
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

// A jump in the sequence the runtime published is not a loss. The recorder
// reports the samples the handoff refused, the chunks it could not write and
// the tail it could not drain; a stretch the runtime never sampled is none of
// those, and guessing a hole there (with a stamp range the recorder never saw)
// would double-report real losses and invent samples.
int SequenceJumpIsNotALoss() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  for (Sequence seq : {1, 2, 3, 9, 10, 11}) {
    CHECK(recorder.TryRecord(MakeSample(seq)));
  }
  CHECK(recorder.Stop());
  CHECK(recorder.dropped() == 0);
  CHECK(recorder.gaps().empty());
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["gaps"].empty());
  CHECK(index["rows_recorded"] == 6);
  std::filesystem::remove_all(directory);
  return 0;
}

// A handoff nobody drains costs the samples, not the tick: the queue is
// bounded, the refusal is counted, and the run reports the stretch it never
// wrote - one stretch, covering exactly the refusals.
int FullQueueIsCounted() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  // The worker waits before its first drain, so a burst pushed immediately
  // fills the bounded queue and the rest is refused.
  const std::uint64_t burst = 2000;
  std::vector<OfferRecord> offers;
  std::uint64_t accepted = 0;
  for (std::uint64_t index = 1; index <= burst; ++index) {
    if (Offer(&recorder, &offers, index)) {
      ++accepted;
    }
  }
  CHECK(accepted < burst);
  CHECK(recorder.dropped() == burst - accepted);
  CHECK(recorder.Stop());
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  // The gap explains the loss in the sequence space, so a reader can tell it
  // from a stretch that never existed - and it is the stretch the recorder
  // really refused, not a hole guessed from the rows around it.
  const std::vector<RecordGap> expected = ExpectedQueueGaps(offers, 1);
  CHECK(!expected.empty());
  CHECK(CheckSameGaps(IndexGaps(index), expected) == 0);
  CHECK(CheckSameGaps(recorder.gaps(), expected) == 0);
  CHECK(CheckRefusalGapsMissRecordedOffers(expected, offers) == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

// Two stretches of refusals with a closed chunk in between are two losses, each
// with its own sequence and stamp range: one gap spanning everything between
// them would report the samples that were written fine as missing.
int RefusalStretches() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  std::vector<OfferRecord> offers;
  // The worker waits before its first drain, so the tail of this burst is
  // refused: the first stretch of loss.
  Sequence seq = 1;
  for (int index = 0; index < 300; ++index) {
    Offer(&recorder, &offers, seq);
    ++seq;
  }
  // Wait for the handoff to be drained, then record a chunk's worth of samples
  // the way a control loop paced by the writer's own clock would offer them:
  // small batches with room to spare, so every one of them is recorded. The
  // chunk that closes and is renamed into place here is what separates the two
  // losses.
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  for (int batch = 0; batch < 10; ++batch) {
    for (int index = 0; index < 100; ++index) {
      CHECK(Offer(&recorder, &offers, seq));
      ++seq;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
  }
  CHECK(CountChunks(recorder.run_directory()) == 1);
  // A second burst after the chunk was written: these refusals start again.
  for (int index = 0; index < 300; ++index) {
    Offer(&recorder, &offers, seq);
    ++seq;
  }
  CHECK(recorder.Stop());

  const std::vector<RecordGap> expected = ExpectedQueueGaps(offers, 1);
  CHECK(expected.size() == 2);
  // The first stretch is the tail of the first burst: the handoff took exactly
  // its capacity.
  CHECK(expected[0].first_seq == kRecordQueueCapacity + 1);
  CHECK(expected[0].last_seq == 300);
  CHECK(expected[0].dropped == 44);
  // The second is the tail of the second burst, and the two are far apart: the
  // samples recorded between them are not part of either loss.
  CHECK(expected[1].first_seq == 1557);
  CHECK(expected[1].last_seq == 1600);
  CHECK(expected[1].dropped == 44);
  CHECK(expected[1].first_stamp_us == MakeSample(1557).stamp_us);
  CHECK(expected[1].last_stamp_us == MakeSample(1600).stamp_us);

  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(CheckSameGaps(IndexGaps(index), expected) == 0);
  CHECK(CheckSameGaps(recorder.gaps(), expected) == 0);
  CHECK(CheckRefusalGapsMissRecordedOffers(expected, offers) == 0);
  // Every accepted sample is in a chunk, and every refused one is a loss: the
  // two stretches account for the whole difference.
  CHECK(recorder.recorded() == 256 + 1000 + 256);
  CHECK(index["rows_recorded"] == recorder.recorded());
  CHECK(recorder.dropped() == 44 + 44);
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk that cannot be written is one loss covering exactly the rows that
// chunk held - the rows it had already taken out of the handoff - and the run
// keeps recording after it.
int WriteFailureIsReported() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  // A directory where the chunk's file has to go: the worker writes and flushes
  // its temporary file, but the rename into place cannot land on a directory.
  // That is the failure a full disk or a permission change produces, and the
  // run's own directory survives it, so its index can still be written.
  CHECK(std::filesystem::create_directory(recorder.run_directory() /
                                          "chunk_0.bin"));
  std::vector<OfferRecord> offers;
  // A chunk and a bit, paced the way a control loop offers them - small batches
  // with room to spare, so every offer is taken and the only loss in the run is
  // the one the failed writes make. The first chunk closes at 900 rows and
  // cannot be renamed into place; the rows after it are recorded, and the next
  // write fails the same way: the run keeps going.
  Sequence seq = 1;
  const std::uint64_t rows = kRecordChunkRows + 300;
  for (std::uint64_t batch = 0; batch < rows / 100; ++batch) {
    for (std::uint64_t index = 0; index < 100; ++index) {
      CHECK(Offer(&recorder, &offers, seq));
      ++seq;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
  }
  CHECK(recorder.Stop());
  CHECK(recorder.recorded() == rows);
  CHECK(recorder.chunks() == 0);
  CHECK(recorder.write_failures() == 2);

  const std::vector<RecordGap> gaps = recorder.gaps();
  // One loss per failed write, each covering exactly the rows that chunk held:
  // the rows already written are not swept in, and neither are the rows that
  // came after.
  CHECK(gaps.size() == 2);
  CHECK(gaps[0].reason == RecordGapReason::WriteFailed);
  CHECK(gaps[0].first_seq == 1);
  CHECK(gaps[0].last_seq == kRecordChunkRows);
  CHECK(gaps[0].dropped == kRecordChunkRows);
  CHECK(gaps[0].first_stamp_us == MakeSample(1).stamp_us);
  CHECK(gaps[0].last_stamp_us == MakeSample(kRecordChunkRows).stamp_us);
  CHECK(gaps[1].reason == RecordGapReason::WriteFailed);
  CHECK(gaps[1].first_seq == kRecordChunkRows + 1);
  CHECK(gaps[1].last_seq == rows);
  CHECK(gaps[1].dropped == rows - kRecordChunkRows);
  CHECK(gaps[1].first_stamp_us == MakeSample(kRecordChunkRows + 1).stamp_us);
  CHECK(gaps[1].last_stamp_us == MakeSample(rows).stamp_us);
  CHECK(CheckRefusalGapsMissRecordedOffers(gaps, offers) == 0);

  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["write_failures"] == 2);
  CHECK(index["rows_recorded"] == 0);
  CHECK(CheckSameGaps(IndexGaps(index), gaps) == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

// Stop closes the handoff first, so a producer cannot keep refilling it while
// the worker finalizes. Every sample the handoff accepted is drained and
// written; no accepted sample is left behind as a Stopped tail.
int ProducerShutdownDrainsAcceptedSamples() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  // A control loop that keeps offering samples while another thread stops the
  // recorder: the worker drains the handoff and leaves it, and what lands
  // behind its last take is the tail. The producer is a tight loop, so the
  // handoff is refilled before Stop() gets to it.
  std::atomic<bool> producing{true};
  std::thread producer([&recorder, &producing] {
    Sequence seq = 1;
    while (producing.load(std::memory_order_relaxed)) {
      recorder.TryRecord(MakeSample(seq));
      ++seq;
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  CHECK(recorder.Stop());
  producing.store(false, std::memory_order_relaxed);
  producer.join();

  const std::vector<RecordGap> gaps = recorder.gaps();
  for (const RecordGap &gap : gaps) {
    CHECK(gap.reason != RecordGapReason::Stopped);
  }
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["complete"] == true);
  // Everything the handoff accepted before it closed is in a committed chunk.
  CHECK(index["rows_recorded"] == recorder.recorded());
  CHECK(CheckSameGaps(IndexGaps(index), gaps) == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

// gaps() and the index are one table: a caller reading the object and a reader
// reading the files must not be able to disagree about what a run lost.
int GapTableMatchesIndex() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  // A chunk that cannot be renamed into place, then a stretch of refusals: a
  // table with more than one reason in it.
  CHECK(std::filesystem::create_directory(recorder.run_directory() /
                                          "chunk_0.bin"));
  std::vector<OfferRecord> offers;
  Sequence seq = 1;
  for (std::uint64_t index = 0; index < kRecordChunkRows; ++index) {
    CHECK(OfferUntilAccepted(&recorder, &offers, seq));
    ++seq;
  }
  for (int index = 0; index < 300; ++index) {
    Offer(&recorder, &offers, seq);
    ++seq;
  }
  CHECK(recorder.Stop());

  const std::vector<RecordGap> gaps = recorder.gaps();
  CHECK(gaps.size() >= 2);
  bool saw_write_failure = false;
  bool saw_refusal = false;
  for (const RecordGap &gap : gaps) {
    saw_write_failure =
        saw_write_failure || gap.reason == RecordGapReason::WriteFailed;
    saw_refusal = saw_refusal || gap.reason == RecordGapReason::QueueFull;
  }
  CHECK(saw_write_failure);
  CHECK(saw_refusal);
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(CheckSameGaps(IndexGaps(index), gaps) == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

// More separate refusal stretches than the control thread's fixed table holds:
// the table stops growing, the loss does not go silent - the last stretch there
// absorbs it - and no recorded sample is ever reported as missing.
int RefusalTableOverflow() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  std::vector<OfferRecord> offers;
  // The handoff is 256 samples, so the first 256 offers here are recorded and
  // every offer after them is refused. The refused sequences are two ticks
  // apart, so each one is a stretch of its own: far more stretches than the
  // table has entries.
  Sequence seq = 1;
  for (int index = 0; index < 500; ++index) {
    Offer(&recorder, &offers, seq);
    seq += 2;
  }
  CHECK(recorder.Stop());
  const std::vector<RecordGap> gaps = recorder.gaps();
  const std::vector<RecordGap> queue_gaps = QueueGaps(gaps);
  CHECK(queue_gaps.size() == kRecordGapSegments);
  const std::uint64_t refusals = static_cast<std::uint64_t>(
      std::count_if(offers.begin(), offers.end(),
                    [](const OfferRecord &offer) { return !offer.accepted; }));
  CHECK(refusals > kRecordGapSegments);
  // The last entry covers the newest loss - the resolution of the stretches
  // folded into it is what is lost, not the loss itself - and every entry stays
  // inside the stretch the refusals really happened in.
  CHECK(queue_gaps.back().last_seq == offers.back().seq);
  CHECK(queue_gaps.front().first_seq == offers[256].seq);
  CHECK(CheckRefusalGapsMissRecordedOffers(gaps, offers) == 0);
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(CheckSameGaps(IndexGaps(index), gaps) == 0);
  std::filesystem::remove_all(directory);
  return 0;
}

// A run sampled every fifth control tick - what a 100 Hz recorder on a 500 Hz
// control loop is - must not report a gap per row: consecutive rows differ by
// the stride by construction, and a loss is only the samples the handoff really
// refused. The stretch it reports is counted on the recorded grid: three
// samples five ticks apart are three missing samples, not the ticks between
// them.
int StridedRun() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  constexpr std::uint32_t kStride = 5;
  CHECK(StartRecorder(&recorder, directory, true, kStride));
  std::vector<OfferRecord> offers;
  // The worker waits before its first drain, so a burst larger than the handoff
  // loses its tail: three samples of the recorded grid, five ticks apart.
  Sequence seq = 1;
  for (int index = 0; index < 259; ++index) {
    Offer(&recorder, &offers, seq);
    seq += kStride;
  }
  CHECK(recorder.Stop());
  CHECK(recorder.recorded() == kRecordQueueCapacity);

  const std::vector<RecordGap> expected = ExpectedQueueGaps(offers, kStride);
  CHECK(expected.size() == 1);
  // The stretch names the three samples that are missing, not the ticks between
  // them.
  CHECK(expected[0].first_seq == 1 + 256 * kStride);
  CHECK(expected[0].last_seq == 1 + 258 * kStride);
  CHECK(expected[0].dropped == 3);
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["rows_recorded"] == kRecordQueueCapacity);
  CHECK(CheckSameGaps(IndexGaps(index), expected) == 0);
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

// One session event written to `sessions.json` and read back field by field:
// the sidecar carries the schema version, the producer and epoch of the adopted
// context, its gains and the literal model digest the runtime writes.
int SessionEventIsWritten() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorderForSessions(&recorder, directory));
  CHECK(recorder.TryRecordSession(MakeSessionEvent(1)));
  CHECK(recorder.Stop());
  CHECK(recorder.session_dropped() == 0);
  CHECK(recorder.sessions_written() == 1);

  // The sidecar is a JSON array of objects, in arrival order.
  std::ifstream file(recorder.run_directory() / "sessions.json");
  CHECK(static_cast<bool>(file));
  const nlohmann::json sessions = nlohmann::json::parse(file, nullptr, false);
  CHECK(sessions.is_array());
  CHECK(sessions.size() == 1);
  CHECK(sessions[0]["schema_version"] == irmv::telemetry::kRecordSchemaVersion);
  CHECK(sessions[0]["epoch"] == 1);
  // The runtime holds no model, so it writes "unknown" rather than a digest it
  // would have had to invent.
  CHECK(sessions[0]["model_sha256"] == "unknown");

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(recorder.run_directory(), &reader, &error));
  CHECK(reader.sessions().size() == 1);
  // Every field survives the sidecar: the context's producer and epoch, its
  // gains, and the digest verbatim.
  CHECK(reader.sessions()[0] == MakeSessionEvent(1));
  std::filesystem::remove_all(directory);
  return 0;
}

// A session channel nobody drains costs the events, not the tick: the control
// call always returns, the loss is counted, and what the channel did accept is
// what the sidecar and the reader report.
int SessionChannelIsBounded() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorderForSessions(&recorder, directory));
  // More contexts than the channel holds, pushed without giving the worker a
  // chance to drain - the worker waits before its first pass, so the tail is
  // refused. Each call still returns.
  const std::uint64_t burst = 200;
  std::uint64_t accepted = 0;
  for (std::uint64_t index = 1; index <= burst; ++index) {
    if (recorder.TryRecordSession(MakeSessionEvent(index))) {
      ++accepted;
    }
  }
  CHECK(accepted < burst);
  CHECK(recorder.Stop());
  // Every event is either written or counted as dropped: the channel invents no
  // silence.
  CHECK(accepted + recorder.session_dropped() == burst);
  CHECK(recorder.sessions_written() == accepted);

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(recorder.run_directory(), &reader, &error));
  CHECK(reader.sessions().size() == recorder.sessions_written());
  for (const irmv::telemetry::RecordSessionEvent &event : reader.sessions()) {
    CHECK(std::string(event.model_sha256.data()) == "unknown");
    CHECK(event.joint_count == 2);
    CHECK(event.kp[0] == 60.0);
    CHECK(event.kd[1] == 4.0);
    CHECK(event.runtime_id[0] == 0x11);
    CHECK(event.producer_id[0] == 0x33);
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A writer blocked past Stop()'s deadline must not hold the caller: Stop()
// returns false at the deadline, performs no filesystem write itself, and the
// worker finishes safely once the device drains. The worker and its queues are
// shared state, so the detached thread never references the Recorder.
int BlockedWriterDoesNotBlockStop() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartRecorder(&recorder, directory));
  const std::filesystem::path fifo =
      recorder.run_directory() / "chunk_0.bin.part";
  CHECK(::mkfifo(fifo.c_str(), 0600) == 0);
  const int read_fd = ::open(fifo.c_str(), O_RDONLY | O_NONBLOCK);
  CHECK(read_fd >= 0);

  // Fill and close the first chunk. The worker opens the FIFO for its temporary
  // file and blocks writing the payload once the pipe buffer is full.
  for (std::uint64_t seq = 1; seq <= kRecordChunkRows; ++seq) {
    bool accepted = false;
    for (int attempt = 0; attempt < 500 && !accepted; ++attempt) {
      accepted = recorder.TryRecord(MakeSample(static_cast<Sequence>(seq)));
      if (!accepted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    CHECK(accepted);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  CHECK(recorder.chunks() == 0);
  CHECK(recorder.write_failures() == 0);

  const auto started = std::chrono::steady_clock::now();
  CHECK(!recorder.Stop());
  const auto elapsed = std::chrono::steady_clock::now() - started;
  CHECK(elapsed < std::chrono::milliseconds(3000));
  // Stop() itself did not finalize the run; only the worker may write it.
  const nlohmann::json index = ReadIndex(recorder.run_directory());
  CHECK(index["complete"] == false);

  // Drain the pipe so the blocked write can finish, then let the worker close
  // the run. The second Stop observes the worker's own completion.
  std::array<char, 64 * 1024> buffer{};
  const auto drain_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(10);
  for (;;) {
    const ssize_t count = ::read(read_fd, buffer.data(), buffer.size());
    if (count > 0) {
      continue;
    }
    if (count == 0) {
      break;
    }
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      break;
    }
    if (std::chrono::steady_clock::now() >= drain_deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ::close(read_fd);
  CHECK(recorder.Stop());
  std::filesystem::remove_all(directory);
  return 0;
}

// The producer's decrement is its final access to the shared state. Pause it
// immediately after that decrement, let the worker finalize and Stop() join,
// destroy the wrapper, then resume the producer: any notify or other shared
// access after the decrement is a use-after-free (caught by the sanitizer run)
// and the deterministic interleaving makes that window observable.
int ProducerAfterDecrementDoesNotTouchSharedState() {
  const std::filesystem::path directory = MakeDirectory();
  auto recorder = std::make_unique<Recorder>();
  CHECK(StartRecorder(recorder.get(), directory));

  DecrementPause pause;
  g_decrement_pause = &pause;
  irmv::telemetry::SetProducerAfterDecrementHookForTest(&PauseAfterDecrement);
  std::atomic<bool> producer_done{false};
  std::thread producer([&recorder, &producer_done] {
    static_cast<void>(recorder->TryRecord(MakeSample(1)));
    producer_done.store(true, std::memory_order_release);
  });
  {
    std::unique_lock<std::mutex> lock(pause.mutex);
    CHECK(pause.cv.wait_for(lock, std::chrono::seconds(5),
                            [&pause] { return pause.entered; }));
  }

  CHECK(recorder->Stop());
  // The worker has been joined, so the wrapper now holds the last reference to
  // the shared state. Destroying it frees the queue, refusal table and
  // condition variable while the producer is still parked after its decrement.
  recorder.reset();
  {
    const std::lock_guard<std::mutex> lock(pause.mutex);
    pause.released = true;
  }
  pause.cv.notify_all();
  producer.join();
  CHECK(producer_done.load(std::memory_order_acquire));

  irmv::telemetry::SetProducerAfterDecrementHookForTest(nullptr);
  g_decrement_pause = nullptr;
  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += Lifecycle();
  failures += RecordedRun();
  failures += SequenceJumpIsNotALoss();
  failures += FullQueueIsCounted();
  failures += RefusalStretches();
  failures += WriteFailureIsReported();
  failures += ProducerShutdownDrainsAcceptedSamples();
  failures += GapTableMatchesIndex();
  failures += RefusalTableOverflow();
  failures += StridedRun();
  failures += SessionEventIsWritten();
  failures += SessionChannelIsBounded();
  failures += Disabled();
  failures += BlockedWriterDoesNotBlockStop();
  failures += ProducerAfterDecrementDoesNotTouchSharedState();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
