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

// Writes a JSON document under a temporary name and renames it into place, so a
// reader never observes a half-written index or sidecar. The rename is the
// publication point; a crash before it leaves the previous document intact.
bool WriteJsonAtomic(const std::filesystem::path &path, const Json &document) {
  const std::filesystem::path temporary = path.string() + ".tmp";
  {
    std::ofstream file(temporary, std::ios::trunc);
    if (!file) {
      return false;
    }
    file << document.dump(1);
    file.flush();
    if (!file.good()) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
      return false;
    }
  }
  std::error_code code;
  std::filesystem::rename(temporary, path, code);
  if (code) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    return false;
  }
  return true;
}

// Test seam for the lifetime ordering below. It is called after the producer
// count is decremented and touches no recorder state; a test can pause there
// while the worker finalizes and the wrapper is destroyed.
std::atomic<void (*)()> g_after_decrement_hook{nullptr};

void RunAfterDecrementHook() noexcept {
  void (*const hook)() = g_after_decrement_hook.load(std::memory_order_relaxed);
  if (hook != nullptr) {
    hook();
  }
}

}  // namespace

void SetProducerAfterDecrementHookForTest(void (*hook)()) noexcept {
  g_after_decrement_hook.store(hook, std::memory_order_relaxed);
}

struct Recorder::Impl {
  // One stretch of loss, in the sequence space the runtime published and the
  // stamps it published it with.
  struct RefusalSpan {
    protocol::Sequence first_seq{0};
    protocol::Sequence last_seq{0};
    protocol::TimeUs first_stamp_us{0};
    protocol::TimeUs last_stamp_us{0};
  };

  bool Prepare(const RecorderConfig &recorder_config, std::string meta_json,
               std::string *error);
  void Run();
  // Takes what the queue holds into the chunk buffer, and closes the chunk when
  // it is full. Returns how many samples it took.
  std::size_t DrainInto();
  void WriteChunk();
  // Records the rows the chunk buffer held as one loss of the given reason.
  void DropChunk(RecordGapReason reason);
  // Atomically replaces `index.json`. `complete` is false while the run is
  // still open and true only after the worker has finalized every queue.
  bool WriteIndex(const std::vector<RecordGap> &gaps, bool complete) const;
  // Takes what the session channel holds into the worker's session list.
  void DrainSessionsInto();
  // Atomically replaces `sessions.json`.
  bool WriteSessions();
  // Producer side, run by the control thread with `offering` announced.
  bool OfferSample(const RecordSample &sample) noexcept;
  bool OfferSession(const RecordSessionEvent &event) noexcept;
  void NoteRefusal(const RecordSample &sample) noexcept;
  void CloseRefusal() noexcept;
  static void PutRefusal(RefusalSpan *table, std::uint32_t *count,
                         const RefusalSpan &span) noexcept;
  void AppendRefusalGaps(std::vector<RecordGap> *gaps) const;
  // The sampling stride of the run, normalized.
  protocol::Sequence Stride() const noexcept {
    return config.stride == 0 ? 1 : config.stride;
  }

  RecorderConfig config;
  std::filesystem::path run_directory;
  std::string metadata;

  // Shared ownership: the worker thread captures this state, not the Recorder
  // object. After a timed-out Stop() the worker may finish on its own without
  // touching destroyed caller state.
  std::thread worker;
  std::atomic<bool> accepting{false};
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool stop_signalled{false};
  bool finished{false};

  // Queues and the control thread's refusal accounting. TryRecord announces
  // itself around every offer, so Stop() can wait for producers to leave before
  // the worker reads the table.
  transport::BoundedChannel<RecordSample, kRecordQueueCapacity> queue;
  transport::BoundedChannel<RecordSessionEvent, kRecordSessionQueueCapacity>
      session_queue;
  std::atomic<std::uint32_t> offering{0};
  std::atomic<std::uint32_t> session_offering{0};
  RefusalSpan refused[kRecordGapSegments]{};
  std::uint32_t refused_count{0};
  RefusalSpan refusal_open{};
  bool refusal_open_valid{false};
  std::uint64_t refusal_open_recorded{0};
  protocol::Sequence stride{1};

  // Counters are shared with the caller's accessors and may still advance
  // after Stop() timed out.
  std::atomic<std::uint64_t> dropped{0};
  std::atomic<std::uint64_t> session_dropped{0};
  std::atomic<std::uint64_t> sessions_written{0};
  std::atomic<std::uint64_t> recorded{0};
  std::atomic<std::uint64_t> chunks{0};
  std::atomic<std::uint64_t> write_failures{0};

  // Worker-owned after construction.
  std::vector<RecordSample> buffer;
  std::vector<RecordSessionEvent> sessions;
  std::vector<ChunkEntry> chunk_entries;
  std::vector<RecordGap> gaps;
  std::uint32_t joint_count{0};
  bool have_joint_count{false};
  std::uint64_t write_failure_count{0};
  bool sessions_dirty{false};

  // The published loss table, guarded because gaps() may be asked from another
  // thread while the worker finalizes.
  mutable std::mutex gaps_mutex;
  std::vector<RecordGap> published_gaps;
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
  // The metadata is authoritative for the run's joint count, so an index
  // published before the first chunk already describes the layout. A test or
  // caller whose metadata omits it keeps the worker's first-sample discovery.
  const Json document = Json::parse(metadata, nullptr, false);
  if (!document.is_discarded() && document.is_object()) {
    const std::uint32_t declared = document.value("joint_count", 0U);
    if (declared > 0 && declared <= protocol::kMaxJoints) {
      joint_count = declared;
      have_joint_count = true;
    }
  }
  buffer.reserve(kRecordChunkRows);
  // Publish an open run immediately: a reader can open it while it is still
  // recording, and a crash after any committed chunk is recoverable even if the
  // index replacement never happened.
  if (!WriteSessions() || !WriteIndex({}, false)) {
    *error = "the recorder could not write " + run_directory.string();
    return false;
  }
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

  const std::string name = std::string(kRecordChunkPrefix) +
                           std::to_string(chunk_entries.size()) + ".bin";
  const std::filesystem::path final_path = run_directory / name;
  // The chunk is written under a temporary name and renamed into place only
  // once every byte is on the device, so a run that ends abruptly leaves a
  // temporary file a reader ignores rather than a chunk it would have to judge
  // complete or not.
  const std::filesystem::path temporary = final_path.string() + ".part";
  {
    std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
    if (!file) {
      ++write_failure_count;
      write_failures.store(write_failure_count, std::memory_order_relaxed);
      DropChunk(RecordGapReason::WriteFailed);
      return;
    }
    file.write(reinterpret_cast<const char *>(chunk.data()),
               static_cast<std::streamsize>(chunk.size()));
    file.flush();
    if (!file.good()) {
      ++write_failure_count;
      write_failures.store(write_failure_count, std::memory_order_relaxed);
      DropChunk(RecordGapReason::WriteFailed);
      return;
    }
  }
  std::error_code code;
  std::filesystem::rename(temporary, final_path, code);
  if (code) {
    ++write_failure_count;
    write_failures.store(write_failure_count, std::memory_order_relaxed);
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
  chunk_entries.push_back(std::move(entry));
  chunks.store(static_cast<std::uint64_t>(chunk_entries.size()),
               std::memory_order_relaxed);
  buffer.clear();
  // The chunk is already committed under its final name. Replacing the index
  // now keeps a reader that opens the run during recording aware of it; a
  // crash between the rename and this write is covered by the reader's
  // recovery scan of committed chunks.
  static_cast<void>(WriteIndex(gaps, false));
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
  while (buffer.size() < kRecordChunkRows && queue.TryPop(sample)) {
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
  while (session_queue.TryPop(event)) {
    sessions.push_back(event);
    sessions_dirty = true;
  }
}

bool Recorder::Impl::WriteSessions() {
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
  if (!WriteJsonAtomic(path, array)) {
    return false;
  }
  sessions_written.store(static_cast<std::uint64_t>(sessions.size()),
                         std::memory_order_relaxed);
  sessions_dirty = false;
  return true;
}

void Recorder::Impl::Run() {
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(done_mutex);
      done_cv.wait_for(lock, std::chrono::milliseconds(50),
                       [this] { return stop_requested.load(); });
    }
    if (stop_requested.load(std::memory_order_relaxed)) {
      break;
    }
    static_cast<void>(DrainInto());
    DrainSessionsInto();
    if (sessions_dirty) {
      static_cast<void>(WriteSessions());
    }
  }
  // Stop() signals shutdown on every path, including one where its own
  // deadline expired before the producers left. Finalization waits here until
  // both producer counts are zero, so a paused producer that resumes sees the
  // refusal and leaves before the worker reads the queue or the refusal table.
  // The sequential-consistent handshake makes the zero observation final: a
  // producer that announces after it must load the refusal and not touch the
  // data.
  for (;;) {
    const bool quiet = offering.load(std::memory_order_seq_cst) == 0 &&
                       session_offering.load(std::memory_order_seq_cst) == 0;
    if (quiet) {
      break;
    }
    std::unique_lock<std::mutex> lock(done_mutex);
    done_cv.wait_for(lock, std::chrono::milliseconds(10));
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
  // The samples still in the handoff were accepted and never written. They are
  // a loss the run names - with their sequence and stamp range - rather than
  // only a number in dropped(): a reader has to be able to see where a run
  // stopped, not infer it from the samples that happen to surround it. Stop()
  // has already waited for every producer, so what is here is final.
  RecordSample sample;
  RecordGap tail;
  bool have_tail = false;
  std::uint64_t left = 0;
  while (queue.TryPop(sample)) {
    if (!have_tail) {
      have_tail = true;
      tail.first_seq = sample.state_seq;
      tail.first_stamp_us = sample.stamp_us;
      tail.reason = RecordGapReason::Stopped;
    }
    tail.last_seq = sample.state_seq;
    tail.last_stamp_us = sample.stamp_us;
    tail.dropped = MissingSamples(tail.first_seq, tail.last_seq, Stride());
    ++left;
  }
  if (have_tail) {
    gaps.push_back(tail);
  }
  if (left != 0) {
    dropped.fetch_add(left, std::memory_order_relaxed);
  }
  // One table for the run: what the worker failed to write, what the handoff
  // refused, and the tail, ordered by the sequence each loss covers.
  AppendRefusalGaps(&gaps);
  std::stable_sort(gaps.begin(), gaps.end(),
                   [](const RecordGap &left_gap, const RecordGap &right_gap) {
                     return left_gap.first_seq < right_gap.first_seq;
                   });
  {
    const std::lock_guard<std::mutex> lock(gaps_mutex);
    published_gaps = gaps;
  }
  if (sessions_dirty || !sessions.empty()) {
    static_cast<void>(WriteSessions());
  }
  static_cast<void>(WriteIndex(published_gaps, true));
  {
    const std::lock_guard<std::mutex> lock(done_mutex);
    finished = true;
  }
  done_cv.notify_all();
}

bool Recorder::Impl::WriteIndex(const std::vector<RecordGap> &gaps,
                                bool complete) const {
  Json index;
  index["schema_version"] = kRecordSchemaVersion;
  index["complete"] = complete;
  index["joint_count"] = joint_count;
  index["rows_recorded"] = 0;
  Json chunk_list = Json::array();
  std::uint64_t rows_total = 0;
  for (const ChunkEntry &entry : chunk_entries) {
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
  index["write_failures"] = write_failure_count;
  const std::filesystem::path path =
      run_directory / std::string(kRecordIndexFile);
  return WriteJsonAtomic(path, index);
}

Recorder::Recorder() = default;

Recorder::~Recorder() { Stop(); }

bool Recorder::Start(const RecorderConfig &config, std::string meta_json,
                     std::string *error) {
  if (impl_) {
    return true;
  }
  std::shared_ptr<Impl> impl = std::make_shared<Impl>();
  std::string message;
  if (!impl->Prepare(config, std::move(meta_json), &message)) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  }
  impl->stride = impl->Stride();
  impl_ = impl;
  // Sequential consistency is part of the stop handshake: Stop's refusal store
  // and a producer's acceptance load must be in one total order with the
  // producer-count operations.
  impl->accepting.store(true, std::memory_order_seq_cst);
  const std::shared_ptr<Impl> shared = impl;
  impl->worker = std::thread([shared] { shared->Run(); });
  return true;
}

bool Recorder::TryRecord(const RecordSample &sample) noexcept {
  if (!impl_) {
    unattached_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  Impl &impl = *impl_;
  // The fast path keeps a producer that runs after Stop from making the
  // quiescence wait longer on every call. It is not the safety argument: a
  // producer that observes acceptance here still announces and re-checks
  // below, which is what makes the handshake final.
  if (!impl.accepting.load(std::memory_order_seq_cst)) {
    impl.dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Announce first, then check acceptance. The count and `accepting` are
  // sequential-consistent on purpose, exactly like the logger's producer
  // scope: if Stop() publishes the refusal and then reads a zero producer
  // count, a producer that increments afterwards must see the refusal in its
  // own load and never touch the queue or the refusal table. Acquire/release on
  // two separate atomics would let both sides read the stale value.
  impl.offering.fetch_add(1, std::memory_order_seq_cst);
  if (!impl.accepting.load(std::memory_order_seq_cst)) {
    impl.dropped.fetch_add(1, std::memory_order_relaxed);
    // Notify before the decrement: the decrement is this producer's final
    // access to the shared state. If the worker observes zero, finalizes and
    // releases the last owner while this thread is preempted, a notify after
    // the decrement would touch freed storage. The worker polls with a bounded
    // wait, so a missed notification when another producer is still counted
    // delays it by at most one poll interval.
    if (impl.offering.load(std::memory_order_seq_cst) == 1) {
      impl.done_cv.notify_all();
    }
    impl.offering.fetch_sub(1, std::memory_order_seq_cst);
    RunAfterDecrementHook();
    return false;
  }
  const bool accepted = impl.OfferSample(sample);
  if (impl.offering.load(std::memory_order_seq_cst) == 1) {
    impl.done_cv.notify_all();
  }
  impl.offering.fetch_sub(1, std::memory_order_seq_cst);
  RunAfterDecrementHook();
  return accepted;
}

bool Recorder::Impl::OfferSample(const RecordSample &sample) noexcept {
  // On the control thread: one copy into a fixed-capacity queue and nothing
  // else. The caller already passed the sequential-consistent admission check
  // and is counted, so Stop() cannot finalize before this offer leaves. A full
  // queue is refused and counted - the tick that produced it does not wait for
  // the disk.
  if (!queue.TryPush(sample)) {
    NoteRefusal(sample);
    dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  recorded.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool Recorder::TryRecordSession(const RecordSessionEvent &event) noexcept {
  // The session side of the control-thread contract: one copy into the bounded
  // session handoff and nothing else. Never blocks, allocates, formats or
  // throws; a stopped recorder or a full channel costs the event and is
  // counted, exactly like a sample.
  if (!impl_) {
    unattached_session_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  Impl &impl = *impl_;
  if (!impl.accepting.load(std::memory_order_seq_cst)) {
    impl.session_dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  impl.session_offering.fetch_add(1, std::memory_order_seq_cst);
  if (!impl.accepting.load(std::memory_order_seq_cst)) {
    impl.session_dropped.fetch_add(1, std::memory_order_relaxed);
    // Same final-access ordering as the sample channel: notify (when this is
    // the last counted producer), then decrement and do not touch the shared
    // state again.
    if (impl.session_offering.load(std::memory_order_seq_cst) == 1) {
      impl.done_cv.notify_all();
    }
    impl.session_offering.fetch_sub(1, std::memory_order_seq_cst);
    RunAfterDecrementHook();
    return false;
  }
  const bool accepted = impl.OfferSession(event);
  if (impl.session_offering.load(std::memory_order_seq_cst) == 1) {
    impl.done_cv.notify_all();
  }
  impl.session_offering.fetch_sub(1, std::memory_order_seq_cst);
  RunAfterDecrementHook();
  return accepted;
}

bool Recorder::Impl::OfferSession(const RecordSessionEvent &event) noexcept {
  const bool accepted = session_queue.TryPush(event);
  if (!accepted) {
    session_dropped.fetch_add(1, std::memory_order_relaxed);
  }
  return accepted;
}

void Recorder::Impl::NoteRefusal(const RecordSample &sample) noexcept {
  // A refusal extends the current stretch when it is the same sample offered
  // again - a caller that retries a tick costs itself the same loss twice, not
  // two losses - or the next sample of the stride the run is sampled on, and
  // nothing was recorded since the stretch opened. Anything else - a jump in
  // the published sequence, or a sample that did make it into the handoff -
  // ends the stretch and starts a new one, so two refusals a minute apart are
  // two losses rather than one gap spanning every sample that was recorded fine
  // between them.
  const bool same_sample = sample.state_seq == refusal_open.last_seq;
  const bool next_sample = sample.state_seq == refusal_open.last_seq + stride;
  if (!refusal_open_valid || (!same_sample && !next_sample) ||
      recorded.load(std::memory_order_relaxed) != refusal_open_recorded) {
    CloseRefusal();
    refusal_open.first_seq = sample.state_seq;
    refusal_open.first_stamp_us = sample.stamp_us;
    refusal_open_valid = true;
    refusal_open_recorded = recorded.load(std::memory_order_relaxed);
  }
  refusal_open.last_seq = sample.state_seq;
  refusal_open.last_stamp_us = sample.stamp_us;
}

void Recorder::Impl::PutRefusal(RefusalSpan *table, std::uint32_t *count,
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

void Recorder::Impl::CloseRefusal() noexcept {
  if (!refusal_open_valid) {
    return;
  }
  refusal_open_valid = false;
  PutRefusal(refused, &refused_count, refusal_open);
}

void Recorder::Impl::AppendRefusalGaps(
    std::vector<RecordGap> *target_gaps) const {
  // Called by the worker after Stop() has waited for every control-thread
  // producer to leave, so the table is still.
  RefusalSpan table[kRecordGapSegments];
  std::uint32_t count = refused_count;
  for (std::uint32_t index = 0; index < count; ++index) {
    table[index] = refused[index];
  }
  if (refusal_open_valid) {
    PutRefusal(table, &count, refusal_open);
  }
  for (std::uint32_t index = 0; index < count; ++index) {
    RecordGap gap;
    gap.first_seq = table[index].first_seq;
    gap.last_seq = table[index].last_seq;
    gap.first_stamp_us = table[index].first_stamp_us;
    gap.last_stamp_us = table[index].last_stamp_us;
    gap.dropped = MissingSamples(gap.first_seq, gap.last_seq, stride);
    gap.reason = RecordGapReason::QueueFull;
    target_gaps->push_back(gap);
  }
}

bool Recorder::Stop() {
  if (!impl_) {
    return true;
  }
  const std::shared_ptr<Impl> impl = impl_;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kRecorderStopTimeoutMs);
  // Refuse every later sample first, with the sequentially consistent
  // handshake that makes a later zero producer count final. Shutdown is then
  // signalled unconditionally: the worker, not this deadline-limited caller,
  // waits for any producer that is still inside TryRecord to leave. If Stop()
  // times out with a paused producer, the worker keeps the shared state alive
  // until that producer resumes and the run can be finalized safely.
  impl->accepting.store(false, std::memory_order_seq_cst);
  {
    const std::lock_guard<std::mutex> lock(impl->done_mutex);
    if (!impl->stop_signalled) {
      impl->stop_signalled = true;
      impl->stop_requested.store(true, std::memory_order_release);
    }
  }
  impl->done_cv.notify_all();

  bool finished = false;
  {
    std::unique_lock<std::mutex> lock(impl->done_mutex);
    finished = impl->done_cv.wait_until(lock, deadline,
                                        [&impl] { return impl->finished; });
  }
  if (finished) {
    if (impl->worker.joinable()) {
      impl->worker.join();
    }
    return true;
  }
  // A blocked writer must not hold the control loop. The worker only references
  // this shared state, which it keeps alive itself, and it is the only writer
  // from here on; Stop() returns without joining or touching the filesystem.
  if (impl->worker.joinable()) {
    impl->worker.detach();
  }
  return false;
}

bool Recorder::started() const {
  return impl_ && impl_->accepting.load(std::memory_order_acquire);
}

const std::filesystem::path &Recorder::run_directory() const {
  static const std::filesystem::path kEmpty;
  return impl_ ? impl_->run_directory : kEmpty;
}

std::uint64_t Recorder::dropped() const {
  return unattached_dropped_.load(std::memory_order_relaxed) +
         (impl_ ? impl_->dropped.load(std::memory_order_relaxed) : 0);
}

std::uint64_t Recorder::session_dropped() const {
  return unattached_session_dropped_.load(std::memory_order_relaxed) +
         (impl_ ? impl_->session_dropped.load(std::memory_order_relaxed) : 0);
}

std::uint64_t Recorder::sessions_written() const {
  return impl_ ? impl_->sessions_written.load(std::memory_order_relaxed) : 0;
}

std::uint64_t Recorder::recorded() const {
  return impl_ ? impl_->recorded.load(std::memory_order_relaxed) : 0;
}

std::uint64_t Recorder::chunks() const {
  return impl_ ? impl_->chunks.load(std::memory_order_relaxed) : 0;
}

std::uint64_t Recorder::write_failures() const {
  return impl_ ? impl_->write_failures.load(std::memory_order_relaxed) : 0;
}

std::vector<RecordGap> Recorder::gaps() const {
  if (!impl_) {
    return {};
  }
  const std::lock_guard<std::mutex> lock(impl_->gaps_mutex);
  return impl_->published_gaps;
}

}  // namespace irmv::telemetry
