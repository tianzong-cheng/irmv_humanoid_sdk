#include "irmv/telemetry/recorder.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace irmv::telemetry {
namespace {

using Json = nlohmann::json;

// Column order of a chunk, fixed by the schema version. The joint columns are
// `rows * joint_count` wide: a deployment with twelve joints does not pay for
// the sixty-four the protocol allows. Version 2 writes eight columns - the
// applied target (`q`, `dq`, `tau_ff`, `kp`, `kd`), the measured feedback
// (`measured_q`, `measured_dq`) and the output stage's torque (`output_torque`)
// - so a reader can see what the machine measured and what it was asked to
// apply in the same tick.
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

// The samples a stretch of a run stands for. The recorder samples the control
// sequence every `stride` ticks, so a stretch of eight ticks at a stride of
// five is two missing samples, not eight: every count in a run is on the
// recorded grid, and getting this wrong would make every strided run look
// lossy.
std::uint64_t MissingSamples(protocol::Sequence first_seq,
                             protocol::Sequence last_seq,
                             protocol::Sequence stride) noexcept {
  if (last_seq < first_seq || stride == 0) {
    return 1;
  }
  return (last_seq - first_seq) / stride + 1;
}

// A 16-byte identity as its lower-case hex form, the same rendering the log,
// the trace and read_record use: an identity is opaque text, not a number.
std::string IdHex(const protocol::Id &id) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string text(2 * id.size(), '0');
  for (std::size_t index = 0; index < id.size(); ++index) {
    text[2 * index] = kHex[(id[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHex[id[index] & 0x0f];
  }
  return text;
}

// The digest text of a session event, read verbatim from its fixed storage. The
// runtime writes the literal "unknown"; a real digest belongs to the
// application, and this side never substitutes one.
std::string DigestText(const std::array<char, kModelDigestBytes> &digest) {
  std::size_t length = 0;
  while (length < digest.size() && digest[length] != '\0') {
    ++length;
  }
  return std::string(digest.data(), length);
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
  // Records the rows the chunk buffer held as one loss of the given reason and
  // drops them. They were already taken out of the queue, so nothing else can
  // see them: without this the run would only show a hole in the sequence.
  void DropChunk(RecordGapReason reason);
  void WriteIndex(const std::vector<RecordGap> &gaps) const;
  // Takes what the session channel holds into the worker's session list. Like
  // DrainInto it never blocks: it stops when the channel is empty.
  void DrainSessionsInto();
  // Writes the session list to `sessions.json` as a JSON array, in the order
  // the events arrived.
  void WriteSessions() const;
  void Finish();
  // The sampling stride of the run, normalized: a stride of zero is one tick.
  protocol::Sequence Stride() const noexcept {
    return config.stride == 0 ? 1 : config.stride;
  }

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
  // The session events the worker took off the channel, in arrival order.
  // Stop() drains what is left before writing the sidecar, so a run's sessions
  // are what the channel held when it closed rather than what the worker
  // happened to have woken up for.
  std::vector<RecordSessionEvent> sessions;
  std::vector<ChunkEntry> chunks;
  // The losses the worker itself saw - one per chunk it could not write - in
  // the order they happened. Stop() adds the refusals the control thread saw
  // and the tail it could not drain, and writes the merged table.
  std::vector<RecordGap> gaps;
  std::uint32_t joint_count{0};
  bool have_joint_count{false};
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

void Recorder::Impl::DropChunk(RecordGapReason reason) {
  if (!buffer.empty()) {
    RecordGap loss;
    loss.first_seq = buffer.front().state_seq;
    loss.last_seq = buffer.back().state_seq;
    loss.first_stamp_us = buffer.front().stamp_us;
    loss.last_stamp_us = buffer.back().stamp_us;
    loss.dropped = MissingSamples(loss.first_seq, loss.last_seq, Stride());
    loss.reason = reason;
    // Only the rows this chunk held are lost: the chunk is a closed range of
    // the run, so the loss is that range and nothing else. Rows already written
    // by earlier chunks are not swept into it, and rows that follow are not
    // either - the run keeps recording after a failed write.
    gaps.push_back(loss);
  }
  buffer.clear();
}

void Recorder::Impl::WriteChunk() {
  if (buffer.empty()) {
    return;
  }
  const std::uint32_t rows = static_cast<std::uint32_t>(buffer.size());
  // The payload - the fixed-width per-row fields and then the column-major
  // joint columns - is built on its own so the header written below can carry
  // its exact length and checksum before a byte of it reaches the file. A
  // reader then tells a chunk whose bytes were corrupted from one that was cut
  // short without trusting either.
  std::vector<std::uint8_t> payload;
  payload.reserve(ChunkPayloadBytes(rows, joint_count));

  for (const RecordSample &sample : buffer) {
    AppendU64(&payload, sample.stamp_us);
    AppendU64(&payload, sample.state_seq);
    AppendU32(&payload, static_cast<std::uint32_t>(sample.execution_state));
    AppendU32(&payload, static_cast<std::uint32_t>(sample.output_action));
    AppendU32(&payload, static_cast<std::uint32_t>(sample.stop_reason));
    AppendU64(&payload, sample.fault_id);
    AppendU8(&payload, sample.healthy ? 1U : 0U);
    AppendU8(&payload, sample.damping_eligible ? 1U : 0U);
    AppendU8(&payload, sample.command_present ? 1U : 0U);
    AppendU8(&payload, sample.feedback_valid ? 1U : 0U);
    AppendU8(&payload, sample.imu_valid ? 1U : 0U);
    AppendU8(&payload, sample.send_accepted ? 1U : 0U);
    AppendU8(&payload, sample.output_valid ? 1U : 0U);
    AppendBytes(&payload, sample.producer_id.data(), sample.producer_id.size());
    AppendU64(&payload, sample.epoch);
    AppendU64(&payload, sample.command_seq);
    AppendU64(&payload, sample.source_state_seq);
    AppendU64(&payload, sample.command_rx_us);
    AppendU64(&payload, sample.feedback_seq);
    AppendU64(&payload, sample.feedback_stamp_us);
  }
  // The joint columns, one column at a time and only as wide as the run needs.
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.q[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.dq[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.tau_ff[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.kp[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.kd[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.measured_q[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.measured_dq[joint]);
    }
  }
  for (const RecordSample &sample : buffer) {
    for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
      AppendDouble(&payload, sample.output_torque[joint]);
    }
  }

  // The header is assembled after the payload it describes, so its length and
  // checksum fields hold real values rather than being patched in afterwards.
  // The header's own checksum covers everything written above it - the length
  // among them - so a chunk whose length field was corrupted in place fails the
  // header check and is named as corrupt instead of being read as a short file.
  std::vector<std::uint8_t> chunk;
  chunk.reserve(kChunkHeaderBytes + payload.size());
  AppendBytes(&chunk, reinterpret_cast<const std::uint8_t *>(kChunkMagic),
              sizeof(kChunkMagic));
  AppendU32(&chunk, kRecordSchemaVersion);
  AppendU32(&chunk, joint_count);
  AppendU32(&chunk, rows);
  AppendU32(&chunk, static_cast<std::uint32_t>(payload.size()));
  AppendU64(&chunk, buffer.front().stamp_us);
  AppendU64(&chunk, buffer.back().stamp_us);
  AppendU64(&chunk, buffer.front().state_seq);
  AppendU64(&chunk, buffer.back().state_seq);
  AppendU32(&chunk, ChunkCrc32(payload.data(), payload.size()));
  AppendU32(&chunk, ChunkCrc32(chunk.data(), chunk.size()));
  AppendBytes(&chunk, payload.data(), payload.size());

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
      ++write_failures;
      DropChunk(RecordGapReason::WriteFailed);
      return;
    }
    file.write(reinterpret_cast<const char *>(chunk.data()),
               static_cast<std::streamsize>(chunk.size()));
    file.flush();
    if (!file.good()) {
      ++write_failures;
      DropChunk(RecordGapReason::WriteFailed);
      return;
    }
  }
  std::error_code code;
  std::filesystem::rename(temporary, final_path, code);
  if (code) {
    ++write_failures;
    std::filesystem::remove(temporary, code);
    DropChunk(RecordGapReason::WriteFailed);
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
  buffer.clear();
}

std::size_t Recorder::Impl::DrainInto() {
  std::size_t taken = 0;
  RecordSample sample;
  // The queue is the FIFO the control thread pushes into; a take either hands a
  // sample over or reports that none is waiting, and it never blocks.
  //
  // A jump in the published sequence between the rows this takes is not a loss
  // and is not guessed into one. It used to be (a hole from `last_state_seq +
  // stride` with a stamp range that began at zero), which was wrong in both
  // directions: it re-reported losses the control thread had already counted
  // and the worker had already written down - one inferred gap per interrupted
  // write, covering the samples in between that had been recorded fine - and it
  // invented a stamp range for a stretch the recorder never saw. Every loss now
  // has one owner that names it where it happened: TryRecord for the refusals
  // of a full handoff, DropChunk for a chunk that could not be written, Stop()
  // for the tail. What is left here is a stretch the runtime never sampled,
  // which is not this component's loss to report.
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
    buffer.push_back(sample);
  }
  if (buffer.size() >= kRecordChunkRows) {
    WriteChunk();
  }
  return taken;
}

void Recorder::Impl::DrainSessionsInto() {
  RecordSessionEvent event;
  // Non-blocking, like the sample drain: take until the bounded channel is
  // empty. A session channel a control loop filled faster than this side could
  // drain has already counted the events it refused; what is here is what was
  // accepted.
  while (owner.session_queue_.TryPop(event)) {
    sessions.push_back(event);
  }
}

void Recorder::Impl::WriteSessions() const {
  Json array = Json::array();
  for (const RecordSessionEvent &event : sessions) {
    Json entry;
    entry["schema_version"] = event.schema_version;
    entry["runtime_id"] = IdHex(event.runtime_id);
    entry["session_id"] = IdHex(event.session_id);
    entry["producer_id"] = IdHex(event.producer_id);
    entry["epoch"] = event.epoch;
    entry["stamp_us"] = event.stamp_us;
    entry["state_seq"] = event.state_seq;
    entry["joint_count"] = event.joint_count;
    Json kp = Json::array();
    Json kd = Json::array();
    for (std::uint32_t joint = 0;
         joint < event.joint_count && joint < protocol::kMaxJoints; ++joint) {
      kp.push_back(event.kp[joint]);
      kd.push_back(event.kd[joint]);
    }
    entry["kp"] = std::move(kp);
    entry["kd"] = std::move(kd);
    entry["model_sha256"] = DigestText(event.model_sha256);
    array.push_back(std::move(entry));
  }
  const std::filesystem::path path =
      run_directory / std::string(kRecordSessionsFile);
  std::ofstream file(path, std::ios::trunc);
  if (!file) {
    return;
  }
  file << array.dump(1);
}

void Recorder::Impl::Run() {
  while (!stop_requested.load(std::memory_order_relaxed)) {
    {
      std::unique_lock<std::mutex> lock(done_mutex);
      done_cv.wait_for(lock, std::chrono::milliseconds(50),
                       [this] { return stop_requested.load(); });
    }
    static_cast<void>(DrainInto());
    DrainSessionsInto();
  }
  // Drain what is left inside the bound Stop() gives, then close the run. The
  // loop ends when the queue has nothing left, and the pass bound keeps a
  // producer that keeps pushing from holding the stop forever.
  for (int pass = 0; pass < 200; ++pass) {
    if (DrainInto() == 0) {
      break;
    }
  }
  DrainSessionsInto();
  if (!buffer.empty()) {
    WriteChunk();
  }
  // The run is not closed here: Stop() takes the tail out of the handoff and
  // writes the index, because the samples still queued at that moment are a
  // loss only Stop() can see, and the table it writes has to hold them.
  {
    const std::lock_guard<std::mutex> lock(done_mutex);
    finished = true;
  }
  done_cv.notify_all();
}

void Recorder::Impl::WriteIndex(const std::vector<RecordGap> &gaps) const {
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
  // The control thread's own copy of the run's sampling stride, so the refusal
  // path never reaches into the worker's record - it is set before the release
  // store of `started_` that lets the control thread in.
  stride_ = impl_->Stride();
  started_.store(true, std::memory_order_release);
  impl_->worker = std::thread([this] { impl_->Run(); });
  return true;
}

bool Recorder::TryRecord(const RecordSample &sample) noexcept {
  // The control thread announces itself around every offer so a Stop() on
  // another thread can tell when the refusal table below is quiescent and no
  // sample can still land behind the tail it takes. This side never waits for
  // anyone: the announcement is two lock-free increments.
  offering_.fetch_add(1, std::memory_order_acquire);
  const bool accepted = OfferSample(sample);
  offering_.fetch_sub(1, std::memory_order_release);
  return accepted;
}

bool Recorder::OfferSample(const RecordSample &sample) noexcept {
  // On the control thread: one copy into a fixed-capacity queue and nothing
  // else. A disabled or stopped recorder, and a full queue, refuse the sample
  // and count it - the tick that produced it does not wait for the disk.
  if (!started_.load(std::memory_order_acquire)) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!queue_.TryPush(sample)) {
    NoteRefusal(sample);
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  recorded_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool Recorder::TryRecordSession(const RecordSessionEvent &event) noexcept {
  // The session side of the control-thread contract: one copy into the bounded
  // session handoff and nothing else. Never blocks, allocates, formats or
  // throws; a stopped recorder or a full channel costs the event and is
  // counted, exactly like a sample.
  session_offering_.fetch_add(1, std::memory_order_acquire);
  bool accepted = false;
  if (started_.load(std::memory_order_acquire)) {
    accepted = session_queue_.TryPush(event);
  }
  session_offering_.fetch_sub(1, std::memory_order_release);
  if (!accepted) {
    session_dropped_.fetch_add(1, std::memory_order_relaxed);
  }
  return accepted;
}

void Recorder::NoteRefusal(const RecordSample &sample) noexcept {
  // A refusal extends the current stretch when it is the same sample offered
  // again - a caller that retries a tick costs itself the same loss twice, not
  // two losses - or the next sample of the stride the run is sampled on, and
  // nothing was recorded since the stretch opened. Anything else - a jump in
  // the published sequence, or a sample that did make it into the handoff -
  // ends the stretch and starts a new one, so two refusals a minute apart are
  // two losses rather than one gap spanning every sample that was recorded fine
  // between them.
  const bool same_sample = sample.state_seq == refusal_open_.last_seq;
  const bool next_sample = sample.state_seq == refusal_open_.last_seq + stride_;
  if (!refusal_open_valid_ || (!same_sample && !next_sample) ||
      recorded_.load(std::memory_order_relaxed) != refusal_open_recorded_) {
    CloseRefusal();
    refusal_open_.first_seq = sample.state_seq;
    refusal_open_.first_stamp_us = sample.stamp_us;
    refusal_open_valid_ = true;
    refusal_open_recorded_ = recorded_.load(std::memory_order_relaxed);
  }
  refusal_open_.last_seq = sample.state_seq;
  refusal_open_.last_stamp_us = sample.stamp_us;
}

void Recorder::PutRefusal(RefusalSpan *table, std::uint32_t *count,
                          const RefusalSpan &span) noexcept {
  if (*count < kRecordGapSegments) {
    table[*count] = span;
    ++*count;
    return;
  }
  // The table is full: the last stretch there absorbs this one instead of the
  // loss going unreported. Its range then covers samples that were recorded
  // fine, so a run that loses more than kRecordGapSegments separate stretches
  // trades their resolution for the guarantee that none of them is silent.
  RefusalSpan &last = table[kRecordGapSegments - 1];
  if (span.last_seq > last.last_seq) {
    last.last_seq = span.last_seq;
    last.last_stamp_us = span.last_stamp_us;
  }
}

void Recorder::CloseRefusal() noexcept {
  if (!refusal_open_valid_) {
    return;
  }
  refusal_open_valid_ = false;
  PutRefusal(refused_, &refused_count_, refusal_open_);
}

void Recorder::AppendRefusalGaps(std::vector<RecordGap> *gaps) const {
  // Called from Stop() once the control thread has left TryRecord, so the table
  // is still. The stretch that was open when the run was closed is a loss like
  // any other - its refusals were counted like any other - and is filed by the
  // same rule.
  RefusalSpan table[kRecordGapSegments];
  std::uint32_t count = refused_count_;
  for (std::uint32_t index = 0; index < count; ++index) {
    table[index] = refused_[index];
  }
  if (refusal_open_valid_) {
    PutRefusal(table, &count, refusal_open_);
  }
  for (std::uint32_t index = 0; index < count; ++index) {
    RecordGap gap;
    gap.first_seq = table[index].first_seq;
    gap.last_seq = table[index].last_seq;
    gap.first_stamp_us = table[index].first_stamp_us;
    gap.last_stamp_us = table[index].last_stamp_us;
    gap.dropped = MissingSamples(gap.first_seq, gap.last_seq, stride_);
    gap.reason = RecordGapReason::QueueFull;
    gaps->push_back(gap);
  }
}

bool Recorder::Stop() {
  if (!impl_) {
    started_.store(false, std::memory_order_release);
    return true;
  }
  // The worker drains what it can and closes the last chunk; it does not write
  // the index, because what follows is a loss only this call can see.
  impl_->Finish();
  // Nothing is accepted from here on, and a control thread that was already
  // inside TryRecord is waited out: that freezes the queue for the tail below
  // and leaves the refusal table still for the read below.
  started_.store(false, std::memory_order_release);
  while (offering_.load(std::memory_order_acquire) != 0) {
    std::this_thread::yield();
  }
  // The session channel is frozen the same way: a control thread that was
  // already inside TryRecordSession is waited out, and then the events still in
  // the channel - which no worker pass will see again - are drained and written
  // to the sidecar. Sessions are not samples: the tail here is not a loss, it
  // is simply what the run adopted after the worker's last pass.
  while (session_offering_.load(std::memory_order_acquire) != 0) {
    std::this_thread::yield();
  }
  impl_->DrainSessionsInto();
  impl_->WriteSessions();
  // The samples still in the handoff were accepted and never written. They are
  // a loss the run names - with their sequence and stamp range - rather than
  // only a number in dropped(): a reader has to be able to see where a run
  // stopped, not infer it from the samples that happen to surround it.
  RecordSample sample;
  RecordGap tail;
  bool have_tail = false;
  std::uint64_t left = 0;
  while (queue_.TryPop(sample)) {
    if (!have_tail) {
      have_tail = true;
      tail.first_seq = sample.state_seq;
      tail.first_stamp_us = sample.stamp_us;
      tail.reason = RecordGapReason::Stopped;
    }
    tail.last_seq = sample.state_seq;
    tail.last_stamp_us = sample.stamp_us;
    tail.dropped = MissingSamples(tail.first_seq, tail.last_seq, stride_);
    ++left;
  }
  if (have_tail) {
    impl_->gaps.push_back(tail);
  }
  // One table for the run: what the worker failed to write, what the handoff
  // refused, and the tail, ordered by the sequence each loss covers - which is
  // the order the losses happened for every run that is not still being offered
  // samples while it is stopped.
  AppendRefusalGaps(&impl_->gaps);
  std::stable_sort(impl_->gaps.begin(), impl_->gaps.end(),
                   [](const RecordGap &left_gap, const RecordGap &right_gap) {
                     return left_gap.first_seq < right_gap.first_seq;
                   });
  {
    const std::lock_guard<std::mutex> lock(gaps_mutex_);
    gaps_ = impl_->gaps;
  }
  // The index and gaps() are one list by construction: a caller reads what a
  // reader of the run reads.
  impl_->WriteIndex(gaps_);
  if (left != 0) {
    dropped_.fetch_add(left, std::memory_order_relaxed);
  }
  chunks_.store(static_cast<std::uint64_t>(impl_->chunks.size()),
                std::memory_order_relaxed);
  sessions_written_.store(static_cast<std::uint64_t>(impl_->sessions.size()),
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

std::uint64_t Recorder::session_dropped() const {
  return session_dropped_.load(std::memory_order_relaxed);
}

std::uint64_t Recorder::sessions_written() const {
  return sessions_written_.load(std::memory_order_relaxed);
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
  const std::lock_guard<std::mutex> lock(gaps_mutex_);
  return gaps_;
}

}  // namespace irmv::telemetry
