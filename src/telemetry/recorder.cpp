#include "irmv/telemetry/recorder.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace irmv::telemetry {
namespace {

using Json = nlohmann::json;

// The magic of a chunk file: a reader refuses a file that does not start with
// it instead of reading whatever the bytes happen to mean.
constexpr char kChunkMagic[8] = {'I', 'R', 'M', 'V', 'C', 'H', '0', '1'};

// Column order of a chunk, fixed by the schema version. The joint columns are
// `rows * joint_count` wide: a deployment with twelve joints does not pay for
// the sixty-four the protocol allows.
void AppendU8(std::vector<std::uint8_t> *out, std::uint8_t value) {
  out->push_back(value);
}

void AppendU32(std::vector<std::uint8_t> *out, std::uint32_t value) {
  for (int shift = 0; shift < 32; shift += 8) {
    out->push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

void AppendU64(std::vector<std::uint8_t> *out, std::uint64_t value) {
  for (int shift = 0; shift < 64; shift += 8) {
    out->push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

void AppendDouble(std::vector<std::uint8_t> *out, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "double is eight bytes here");
  std::memcpy(&bits, &value, sizeof(bits));
  AppendU64(out, bits);
}

void AppendBytes(std::vector<std::uint8_t> *out, const std::uint8_t *bytes,
                 std::size_t count) {
  out->insert(out->end(), bytes, bytes + count);
}

// One chunk of the index.
struct ChunkEntry {
  std::string file;
  std::uint32_t rows{0};
  protocol::TimeUs first_stamp_us{0};
  protocol::TimeUs last_stamp_us{0};
  protocol::Sequence first_state_seq{0};
  protocol::Sequence last_state_seq{0};
};

std::string GapReasonName(RecordGapReason reason) {
  switch (reason) {
    case RecordGapReason::None:
      return "none";
    case RecordGapReason::QueueFull:
      return "queue_full";
    case RecordGapReason::WriteFailed:
      return "write_failed";
    case RecordGapReason::Stopped:
      return "stopped";
  }
  return "unknown";
}

}  // namespace

struct Recorder::Impl {
  explicit Impl(Recorder &recorder) : owner(recorder) {}

  bool Prepare(const RecorderConfig &recorder_config, std::string meta_json,
               std::string *error);
  void Run();
  // Takes what the queue holds into the chunk buffer, and closes the chunk when
  // it is full. Returns how many samples it took, so the final drain can stop
  // when the queue is empty instead of guessing.
  std::size_t DrainInto();
  void WriteChunk();
  void RecordGaps(const RecordSample &sample);
  void WriteIndex() const;
  void Finish();

  Recorder &owner;
  RecorderConfig config;
  std::filesystem::path run_directory;
  std::string metadata;
  std::string index_json;
  std::thread worker;
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool finished{false};

  // Worker-owned.
  std::vector<RecordSample> buffer;
  std::vector<ChunkEntry> chunks;
  std::vector<RecordGap> gaps;
  std::uint32_t joint_count{0};
  bool have_joint_count{false};
  bool have_last{false};
  protocol::Sequence last_state_seq{0};
  bool write_failed{false};
  std::uint64_t write_failures{0};
};

bool Recorder::Impl::Prepare(const RecorderConfig &recorder_config,
                             std::string meta_json, std::string *error) {
  config = recorder_config;
  metadata = std::move(meta_json);
  if (config.queue_capacity == 0 ||
      config.queue_capacity > kRecordQueueCapacity) {
    *error = "the recorder queue capacity must be 1.." +
             std::to_string(kRecordQueueCapacity) + ", got " +
             std::to_string(config.queue_capacity);
    return false;
  }
  if (config.directory.empty()) {
    *error = "the recorder has no directory";
    return false;
  }
  std::error_code code;
  if (!std::filesystem::is_directory(config.directory, code) || code) {
    *error = "the recorder directory " + config.directory.string() +
             " does not exist";
    return false;
  }
  // One directory per run, named by the host clock so two runs can never share
  // one. The creation is atomic: a directory that already exists is refused
  // instead of being written into.
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto micros =
      std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  run_directory = config.directory / ("run_" + std::to_string(micros));
  if (!std::filesystem::create_directory(run_directory, code) || code) {
    *error = "the recorder could not create " + run_directory.string();
    return false;
  }
  std::ofstream metadata_file(run_directory / std::string(kRecordMetadataFile));
  if (!metadata_file) {
    *error = "the recorder could not write " + run_directory.string();
    return false;
  }
  metadata_file << metadata;
  if (!metadata_file.good()) {
    *error = "the recorder could not write " + run_directory.string();
    return false;
  }
  buffer.reserve(kRecordChunkRows);
  return true;
}

void Recorder::Impl::RecordGaps(const RecordSample &sample) {
  if (!have_last) {
    have_last = true;
    last_state_seq = sample.state_seq;
    return;
  }
  if (sample.state_seq > last_state_seq + 1) {
    // Everything between the last sample that was written and this one is
    // missing: the queue refused it, or a chunk write failed, or both. The gap
    // is recorded in the sequence space the runtime published, so a reader can
    // tell a missing stretch from a stretch that never existed.
    RecordGap gap;
    gap.first_seq = last_state_seq + 1;
    gap.last_seq = sample.state_seq - 1;
    gap.first_stamp_us = 0;
    gap.last_stamp_us = sample.stamp_us;
    gap.dropped = gap.last_seq - gap.first_seq + 1;
    gap.reason = write_failed ? RecordGapReason::WriteFailed
                              : RecordGapReason::QueueFull;
    gaps.push_back(gap);
  }
  last_state_seq = sample.state_seq;
}

void Recorder::Impl::WriteChunk() {
  if (buffer.empty()) {
    return;
  }
  const std::uint32_t rows = static_cast<std::uint32_t>(buffer.size());
  std::vector<std::uint8_t> bytes;
  bytes.reserve(64 + rows * (120 + 5 * joint_count * 8));
  AppendBytes(&bytes, reinterpret_cast<const std::uint8_t *>(kChunkMagic),
              sizeof(kChunkMagic));
  AppendU32(&bytes, kRecordSchemaVersion);
  AppendU32(&bytes, joint_count);
  AppendU32(&bytes, rows);
  AppendU64(&bytes, buffer.front().stamp_us);
  AppendU64(&bytes, buffer.back().stamp_us);
  AppendU64(&bytes, buffer.front().state_seq);
  AppendU64(&bytes, buffer.back().state_seq);

  for (const RecordSample &sample : buffer) {
    AppendU64(&bytes, sample.stamp_us);
    AppendU64(&bytes, sample.state_seq);
    AppendU32(&bytes, static_cast<std::uint32_t>(sample.execution_state));
    AppendU32(&bytes, static_cast<std::uint32_t>(sample.output_action));
    AppendU32(&bytes, static_cast<std::uint32_t>(sample.stop_reason));
    AppendU64(&bytes, sample.fault_id);
    AppendU8(&bytes, sample.healthy ? 1U : 0U);
    AppendU8(&bytes, sample.damping_eligible ? 1U : 0U);
    AppendU8(&bytes, sample.command_present ? 1U : 0U);
    AppendU8(&bytes, sample.feedback_valid ? 1U : 0U);
    AppendU8(&bytes, sample.imu_valid ? 1U : 0U);
    AppendU8(&bytes, sample.send_accepted ? 1U : 0U);
    AppendBytes(&bytes, sample.producer_id.data(), sample.producer_id.size());
    AppendU64(&bytes, sample.epoch);
    AppendU64(&bytes, sample.command_seq);
    AppendU64(&bytes, sample.source_state_seq);
    AppendU64(&bytes, sample.command_rx_us);
    AppendU64(&bytes, sample.feedback_seq);
    AppendU64(&bytes, sample.feedback_stamp_us);
  }
  // The joint columns, one column at a time and only as wide as the run needs.
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&bytes, sample.q[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&bytes, sample.dq[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&bytes, sample.tau_ff[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&bytes, sample.kp[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&bytes, sample.kd[joint]);
    }
  }

  const std::string name =
      std::string(kRecordChunkPrefix) + std::to_string(chunks.size()) + ".bin";
  const std::filesystem::path final_path = run_directory / name;
  // The chunk is written under a temporary name and renamed into place only
  // once every byte is on the device, so a run that ends abruptly leaves a
  // temporary file a reader ignores rather than a chunk it would have to judge
  // complete or not.
  const std::filesystem::path temporary = final_path.string() + ".part";
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    if (!file) {
      write_failed = true;
      ++write_failures;
      buffer.clear();
      return;
    }
    file.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    file.flush();
    if (!file.good()) {
      write_failed = true;
      ++write_failures;
      buffer.clear();
      return;
    }
  }
  std::error_code code;
  std::filesystem::rename(temporary, final_path, code);
  if (code) {
    write_failed = true;
    ++write_failures;
    std::filesystem::remove(temporary, code);
    buffer.clear();
    return;
  }

  ChunkEntry entry;
  entry.file = name;
  entry.rows = rows;
  entry.first_stamp_us = buffer.front().stamp_us;
  entry.last_stamp_us = buffer.back().stamp_us;
  entry.first_state_seq = buffer.front().state_seq;
  entry.last_state_seq = buffer.back().state_seq;
  chunks.push_back(std::move(entry));
  write_failed = false;
  buffer.clear();
}

std::size_t Recorder::Impl::DrainInto() {
  std::size_t taken = 0;
  RecordSample sample;
  // The queue is the FIFO the control thread pushes into; a take either hands a
  // sample over or reports that none is waiting, and it never blocks.
  while (buffer.size() < kRecordChunkRows && owner.queue_.TryPop(sample)) {
    ++taken;
    if (!have_joint_count) {
      joint_count = sample.joint_count;
      have_joint_count = true;
    }
    if (sample.joint_count != joint_count) {
      // A run describes one layout: a sample of another joint count is not the
      // same run and is refused rather than silently truncated.
      continue;
    }
    RecordGaps(sample);
    buffer.push_back(sample);
  }
  if (buffer.size() >= kRecordChunkRows) {
    WriteChunk();
  }
  return taken;
}

void Recorder::Impl::Run() {
  while (!stop_requested.load(std::memory_order_relaxed)) {
    {
      std::unique_lock<std::mutex> lock(done_mutex);
      done_cv.wait_for(lock, std::chrono::milliseconds(50),
                       [this] { return stop_requested.load(); });
    }
    static_cast<void>(DrainInto());
  }
  // Drain what is left inside the bound Stop() gives, then close the run. The
  // loop ends when the queue has nothing left, and the pass bound keeps a
  // producer that keeps pushing from holding the stop forever.
  for (int pass = 0; pass < 200; ++pass) {
    if (DrainInto() == 0) {
      break;
    }
  }
  if (!buffer.empty()) {
    WriteChunk();
  }
  WriteIndex();
  {
    const std::lock_guard<std::mutex> lock(done_mutex);
    finished = true;
  }
  done_cv.notify_all();
}

void Recorder::Impl::WriteIndex() const {
  Json index;
  index["schema_version"] = kRecordSchemaVersion;
  index["joint_count"] = joint_count;
  index["rows_recorded"] = 0;
  Json chunk_list = Json::array();
  std::uint64_t rows_total = 0;
  for (const ChunkEntry &entry : chunks) {
    Json chunk;
    chunk["file"] = entry.file;
    chunk["rows"] = entry.rows;
    chunk["first_stamp_us"] = entry.first_stamp_us;
    chunk["last_stamp_us"] = entry.last_stamp_us;
    chunk["first_state_seq"] = entry.first_state_seq;
    chunk["last_state_seq"] = entry.last_state_seq;
    chunk_list.push_back(std::move(chunk));
    rows_total += entry.rows;
  }
  index["chunks"] = std::move(chunk_list);
  index["rows_recorded"] = rows_total;
  Json gap_list = Json::array();
  for (const RecordGap &gap : gaps) {
    Json entry;
    entry["first_seq"] = gap.first_seq;
    entry["last_seq"] = gap.last_seq;
    entry["first_stamp_us"] = gap.first_stamp_us;
    entry["last_stamp_us"] = gap.last_stamp_us;
    entry["dropped"] = gap.dropped;
    entry["reason"] = GapReasonName(gap.reason);
    gap_list.push_back(std::move(entry));
  }
  // The samples the queue refused are a gap of their own: the worker never saw
  // them, so they cannot show up as a hole between two samples it did write.
  const protocol::Sequence drop_first =
      owner.dropped_first_seq_.load(std::memory_order_relaxed);
  const protocol::Sequence drop_last =
      owner.dropped_last_seq_.load(std::memory_order_relaxed);
  if (drop_first != 0 && drop_last >= drop_first) {
    Json entry;
    entry["first_seq"] = drop_first;
    entry["last_seq"] = drop_last;
    entry["first_stamp_us"] = 0;
    entry["last_stamp_us"] = 0;
    entry["dropped"] = drop_last - drop_first + 1;
    entry["reason"] = GapReasonName(RecordGapReason::QueueFull);
    gap_list.push_back(std::move(entry));
  }
  index["gaps"] = std::move(gap_list);
  index["write_failures"] = write_failures;
  const std::filesystem::path path =
      run_directory / std::string(kRecordIndexFile);
  std::ofstream file(path, std::ios::trunc);
  if (!file) {
    return;
  }
  file << index.dump(1);
}

void Recorder::Impl::Finish() {
  stop_requested.store(true, std::memory_order_relaxed);
  done_cv.notify_all();
  if (!worker.joinable()) {
    return;
  }
  {
    std::unique_lock<std::mutex> lock(done_mutex);
    done_cv.wait_for(lock, std::chrono::milliseconds(2000),
                     [this] { return finished; });
  }
  worker.join();
}

Recorder::Recorder() = default;

Recorder::~Recorder() { Stop(); }

bool Recorder::Start(const RecorderConfig &config, std::string meta_json,
                     std::string *error) {
  if (impl_) {
    return true;
  }
  std::string message;
  std::unique_ptr<Impl> impl = std::make_unique<Impl>(*this);
  if (!impl->Prepare(config, std::move(meta_json), &message)) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  }
  impl_ = std::move(impl);
  run_directory_ = impl_->run_directory;
  started_.store(true, std::memory_order_release);
  impl_->worker = std::thread([this] { impl_->Run(); });
  return true;
}

bool Recorder::TryRecord(const RecordSample &sample) noexcept {
  // On the control thread: one copy into a fixed-capacity queue and nothing
  // else. A disabled or stopped recorder, and a full queue, refuse the sample
  // and count it - the tick that produced it does not wait for the disk.
  if (!started_.load(std::memory_order_acquire)) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!queue_.TryPush(sample)) {
    if (dropped_first_seq_.load(std::memory_order_relaxed) == 0) {
      dropped_first_seq_.store(sample.state_seq, std::memory_order_relaxed);
    }
    dropped_last_seq_.store(sample.state_seq, std::memory_order_relaxed);
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  recorded_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool Recorder::Stop() {
  if (!impl_) {
    started_.store(false, std::memory_order_release);
    return true;
  }
  impl_->Finish();
  // A queue that still holds samples is a loss the run reports: they were
  // accepted and never written.
  std::uint64_t left = 0;
  RecordSample sample;
  while (queue_.TryPop(sample)) {
    ++left;
  }
  if (left != 0) {
    dropped_.fetch_add(left, std::memory_order_relaxed);
  }
  chunks_.store(static_cast<std::uint64_t>(impl_->chunks.size()),
                std::memory_order_relaxed);
  write_failures_.store(impl_->write_failures, std::memory_order_relaxed);
  impl_.reset();
  started_.store(false, std::memory_order_release);
  return true;
}

bool Recorder::started() const {
  return started_.load(std::memory_order_acquire);
}

const std::filesystem::path &Recorder::run_directory() const {
  return run_directory_;
}

std::uint64_t Recorder::dropped() const {
  return dropped_.load(std::memory_order_relaxed);
}

std::uint64_t Recorder::recorded() const {
  return recorded_.load(std::memory_order_relaxed);
}

std::uint64_t Recorder::chunks() const {
  return chunks_.load(std::memory_order_relaxed);
}

std::uint64_t Recorder::write_failures() const {
  return write_failures_.load(std::memory_order_relaxed);
}

std::vector<RecordGap> Recorder::gaps() const {
  return impl_ ? impl_->gaps : std::vector<RecordGap>{};
}

}  // namespace irmv::telemetry
