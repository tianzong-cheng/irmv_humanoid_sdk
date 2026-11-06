#include "irmv/telemetry/velocity_record.hpp"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <exception>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "irmv/transport/bounded_channel.hpp"

namespace irmv::telemetry {
namespace {

using Json = nlohmann::json;

constexpr char kHexDigits[] = "0123456789abcdef";

// The control thread may report this many separate queue-loss stretches. Past
// the bound a further stretch is folded into the last one, so a pathological
// run reports one coarser loss rather than silence. The worker's write losses
// use the same bound. Unlike the runtime recorder, neither table allocates.
constexpr std::size_t kVelocityLossSegments = 64;

std::string HexBytes(const std::uint8_t *bytes, std::size_t count) {
  std::string text(2 * count, '0');
  for (std::size_t index = 0; index < count; ++index) {
    text[2 * index] = kHexDigits[(bytes[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHexDigits[bytes[index] & 0x0f];
  }
  return text;
}

std::string HexId(const protocol::Id &id) {
  return HexBytes(id.data(), id.size());
}

// A 64-bit value as exactly sixteen lower-case hex digits. The width is part of
// the line schema, so a reader never has to guess whether a shorter value was
// truncated, and a consumer that loses integer precision above 2^53 cannot
// corrupt the value.
std::string HexSequence(std::uint64_t value) {
  std::uint8_t bytes[8] = {};
  for (int index = 0; index < 8; ++index) {
    bytes[index] =
        static_cast<std::uint8_t>((value >> (8 * (7 - index))) & 0xFFU);
  }
  return HexBytes(bytes, sizeof(bytes));
}

bool HexValue(std::string_view text, std::size_t digits, std::uint64_t *out) {
  if (text.size() != digits) {
    return false;
  }
  std::uint64_t value = 0;
  for (const char character : text) {
    std::uint32_t digit = 0;
    if (character >= '0' && character <= '9') {
      digit = static_cast<std::uint32_t>(character - '0');
    } else if (character >= 'a' && character <= 'f') {
      digit = static_cast<std::uint32_t>(character - 'a') + 10;
    } else if (character >= 'A' && character <= 'F') {
      digit = static_cast<std::uint32_t>(character - 'A') + 10;
    } else {
      return false;
    }
    value = (value << 4) | digit;
  }
  *out = value;
  return true;
}

bool HexIdValue(std::string_view text, protocol::Id *out) {
  if (text.size() != 2 * out->size()) {
    return false;
  }
  for (std::size_t index = 0; index < out->size(); ++index) {
    std::uint64_t byte = 0;
    if (!HexValue(text.substr(2 * index, 2), 2, &byte)) {
      return false;
    }
    (*out)[index] = static_cast<std::uint8_t>(byte);
  }
  return true;
}

// A non-finite double has no JSON number representation; null would lose the
// distinction between "no value" and "not a number". The writer uses the same
// explicit spellings as the fault sidecar and the reader restores them.
Json EncodeDouble(double value) {
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value > 0.0 ? "+Inf" : "-Inf";
  }
  return value;
}

bool DecodeDouble(const Json &value, double *out) {
  if (value.is_number()) {
    const double number = value.get<double>();
    if (!std::isfinite(number)) {
      return false;
    }
    *out = number;
    return true;
  }
  if (!value.is_string()) {
    return false;
  }
  const std::string text = value.get<std::string>();
  if (text == "NaN") {
    *out = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  if (text == "+Inf") {
    *out = std::numeric_limits<double>::infinity();
    return true;
  }
  if (text == "-Inf") {
    *out = -std::numeric_limits<double>::infinity();
    return true;
  }
  return false;
}

bool FieldU64(const Json &object, const char *key, std::uint64_t *out) {
  const auto found = object.find(key);
  if (found == object.end() || !found->is_number_unsigned()) {
    return false;
  }
  *out = found->get<std::uint64_t>();
  return true;
}

bool FieldU32(const Json &object, const char *key, std::uint32_t *out) {
  std::uint64_t value = 0;
  if (!FieldU64(object, key, &value) || value > 0xFFFFFFFFULL) {
    return false;
  }
  *out = static_cast<std::uint32_t>(value);
  return true;
}

bool FieldString(const Json &object, const char *key, std::string *out) {
  const auto found = object.find(key);
  if (found == object.end() || !found->is_string()) {
    return false;
  }
  *out = found->get<std::string>();
  return true;
}

bool SchemaMatches(const Json &document, std::string *why) {
  std::string schema;
  if (!FieldString(document, "schema", &schema)) {
    *why = "schema is missing or not a string";
    return false;
  }
  if (schema != kVelocitySchema) {
    *why = "the document declares schema " + schema;
    return false;
  }
  std::uint32_t version = 0;
  if (!FieldU32(document, "version", &version)) {
    *why = "version is missing or not a number";
    return false;
  }
  if (version != kVelocitySchemaVersion) {
    *why = "the document declares version " + std::to_string(version);
    return false;
  }
  return true;
}

std::uint64_t HostMicros() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

// Writes a JSON document under a temporary name and renames it into place, so a
// reader never observes a half-written index. The rename is the publication
// point.
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

// `YYYYMMDDTHHMMSS.ffffffZ`, the UTC instant a run directory is named after.
std::string UtcStamp() {
  const auto now = std::chrono::system_clock::now();
  const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count() %
                      1000000;
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm broken{};
  gmtime_r(&seconds, &broken);
  char date[32] = {};
  std::strftime(date, sizeof(date), "%Y%m%dT%H%M%S", &broken);
  char stamp[48] = {};
  std::snprintf(stamp, sizeof(stamp), "%s.%06lldZ", date,
                static_cast<long long>(micros));
  return stamp;
}

std::string UniqueSuffix() {
  static std::atomic<std::uint64_t> counter{0};
  return std::to_string(static_cast<long long>(::getpid())) + "-" +
         std::to_string(counter.fetch_add(1, std::memory_order_relaxed));
}

// The hook and its context are replaced as one pair under a mutex, so the
// worker never observes a function from one installation and the context of
// another. The mutex is leaked on purpose: a timed-out Stop() detaches the
// worker, which may still be finalizing during static destruction.
std::mutex &VelocityLineHookMutex() {
  static std::mutex *const mutex = new std::mutex;
  return *mutex;
}

int (*g_velocity_line_hook)(void *, std::string_view) = nullptr;
void *g_velocity_line_hook_context = nullptr;

bool DecodeLoss(const Json &entry, VelocityLossSpan *loss, std::string *why) {
  std::string seq;
  if (!FieldString(entry, "first_sample_seq", &seq) ||
      !HexValue(seq, 16, &loss->first_sample_seq)) {
    *why = "first_sample_seq is not a 16-digit hex sequence";
    return false;
  }
  if (!FieldString(entry, "last_sample_seq", &seq) ||
      !HexValue(seq, 16, &loss->last_sample_seq)) {
    *why = "last_sample_seq is not a 16-digit hex sequence";
    return false;
  }
  if (!FieldU64(entry, "first_stamp_us", &loss->first_stamp_us)) {
    *why = "first_stamp_us is missing or not an unsigned number";
    return false;
  }
  if (!FieldU64(entry, "last_stamp_us", &loss->last_stamp_us)) {
    *why = "last_stamp_us is missing or not an unsigned number";
    return false;
  }
  if (!FieldU64(entry, "dropped", &loss->dropped)) {
    *why = "dropped is missing or not an unsigned number";
    return false;
  }
  std::string reason;
  if (!FieldString(entry, "reason", &reason)) {
    *why = "reason is missing or not a string";
    return false;
  }
  if (reason == "queue_full") {
    loss->reason = VelocityLossReason::QueueFull;
  } else if (reason == "write_failed") {
    loss->reason = VelocityLossReason::WriteFailed;
  } else {
    *why = "reason is not a known loss reason";
    return false;
  }
  return true;
}

bool DecodeSample(const Json &line, VelocitySample *sample, std::string *why) {
  if (!line.is_object()) {
    *why = "the line is not a JSON object";
    return false;
  }
  if (!SchemaMatches(line, why)) {
    return false;
  }
  if (!FieldU64(line, "stamp_us", &sample->stamp_us)) {
    *why = "stamp_us is missing or not an unsigned number";
    return false;
  }
  std::string sequence;
  if (!FieldString(line, "sample_seq", &sequence) ||
      !HexValue(sequence, 16, &sample->sample_seq)) {
    *why = "sample_seq is not a 16-digit hex sequence";
    return false;
  }
  const auto identity = line.find("identity_known");
  if (identity == line.end() || !identity->is_boolean()) {
    *why = "identity_known is missing or not a boolean";
    return false;
  }
  sample->identity_known = identity->get<bool>();
  std::string runtime_id;
  std::string session_id;
  if (!FieldString(line, "runtime_id", &runtime_id) ||
      !FieldString(line, "session_id", &session_id)) {
    *why = "runtime_id/session_id are missing or not strings";
    return false;
  }
  if (sample->identity_known) {
    if (!HexIdValue(runtime_id, &sample->runtime_id) ||
        !HexIdValue(session_id, &sample->session_id)) {
      *why = "a known identity is not two 32-digit hex ids";
      return false;
    }
    // The writer never emits a "known" identity that is the protocol's
    // all-zero "none" value; a file claiming one is not a sample this build
    // produced, and accepting it would make the identity unusable as a join
    // key.
    if (sample->runtime_id == protocol::kNoId ||
        sample->session_id == protocol::kNoId) {
      *why = "a known identity is the all-zero id";
      return false;
    }
  } else {
    if (!runtime_id.empty() || !session_id.empty()) {
      *why = "an unknown identity carries a non-empty id";
      return false;
    }
    sample->runtime_id = protocol::kNoId;
    sample->session_id = protocol::kNoId;
  }
  if (!FieldString(line, "latest_state_seq", &sequence) ||
      !HexValue(sequence, 16, &sample->latest_state_seq)) {
    *why = "latest_state_seq is not a 16-digit hex sequence";
    return false;
  }
  const auto forward = line.find("forward");
  const auto lateral = line.find("lateral");
  const auto yaw = line.find("yaw");
  if (forward == line.end() || !DecodeDouble(*forward, &sample->forward)) {
    *why = "forward is missing or not a number";
    return false;
  }
  if (lateral == line.end() || !DecodeDouble(*lateral, &sample->lateral)) {
    *why = "lateral is missing or not a number";
    return false;
  }
  if (yaw == line.end() || !DecodeDouble(*yaw, &sample->yaw)) {
    *why = "yaw is missing or not a number";
    return false;
  }
  return true;
}

}  // namespace

const char *VelocityLossReasonName(VelocityLossReason reason) noexcept {
  switch (reason) {
    case VelocityLossReason::None:
      return "none";
    case VelocityLossReason::QueueFull:
      return "queue_full";
    case VelocityLossReason::WriteFailed:
      return "write_failed";
  }
  return "unknown";
}

void SetVelocityLineWriteHookForTest(int (*hook)(void *context,
                                                 std::string_view line),
                                     void *context) noexcept {
  const std::lock_guard<std::mutex> lock(VelocityLineHookMutex());
  g_velocity_line_hook = hook;
  g_velocity_line_hook_context = context;
}

struct VelocityRecorder::Impl {
  struct LossSpan {
    std::uint64_t first_seq{0};
    std::uint64_t last_seq{0};
    protocol::TimeUs first_stamp_us{0};
    protocol::TimeUs last_stamp_us{0};
    std::uint64_t dropped{0};
    VelocityLossReason reason{VelocityLossReason::None};
  };

  bool Prepare(const configuration::RecorderConfig &recorder_config,
               std::string meta_json, std::string *error,
               LiveTelemetryOptions live_options);
  void Run();
  void Drain();
  bool WriteLine(const VelocitySample &sample);
  bool WriteIndex(bool complete) const;
  // Control-thread loss accounting. Fixed storage: the control path may not
  // allocate.
  void NoteQueueRefusal(const VelocitySample &sample) noexcept;
  void CloseQueueLoss() noexcept;
  // Worker-side loss accounting for a line that was not persisted. Runs on the
  // worker only.
  void NoteWriteFailure(const VelocitySample &sample) noexcept;
  void CloseWriteLoss() noexcept;
  static void PutLoss(LossSpan *table, std::uint32_t *count,
                      const LossSpan &span) noexcept;
  void AppendLosses(Json *losses) const;
  void RollBack() noexcept;

  configuration::RecorderConfig config;
  std::filesystem::path run_directory;
  std::string error;
  LiveTelemetryOptions live_options;
  std::unique_ptr<LiveTelemetrySender> live;
  std::chrono::steady_clock::time_point last_live_status{};
  void SendLiveStatus(bool hello, bool complete = false);
  std::size_t capacity{kVelocityQueueCapacity};

  // Shared ownership: the worker captures this state, not the VelocityRecorder
  // object, so a timed-out Stop() leaves the worker with everything it needs.
  std::thread worker;
  std::atomic<bool> accepting{false};
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool stop_signalled{false};
  bool finished{false};

  transport::BoundedChannel<VelocitySample, kVelocityQueueCapacity> queue;
  std::atomic<std::uint32_t> offering{0};
  LossSpan queue_losses[kVelocityLossSegments]{};
  std::uint32_t queue_loss_count{0};
  LossSpan queue_loss_open{};
  bool queue_loss_open_valid{false};

  std::atomic<std::uint64_t> recorded{0};
  std::atomic<std::uint64_t> written{0};
  std::atomic<std::uint64_t> dropped{0};
  std::atomic<std::uint64_t> write_failures{0};

  // Worker-owned state.
  std::ofstream file;
  std::uint64_t committed_bytes{0};
  LossSpan write_losses[kVelocityLossSegments]{};
  std::uint32_t write_loss_count{0};
  LossSpan write_loss_open{};
  bool write_loss_open_valid{false};
  std::uint64_t write_failure_count{0};
};

bool VelocityRecorder::Impl::Prepare(
    const configuration::RecorderConfig &recorder_config, std::string meta_json,
    std::string *error_out, LiveTelemetryOptions live_options_in) {
  config = recorder_config;
  live_options = std::move(live_options_in);
  if (!config.enabled) {
    *error_out = "the velocity recorder is disabled";
    return false;
  }
  if (config.sample_hz == 0) {
    *error_out = "the velocity recorder sample_hz must be positive";
    return false;
  }
  if (config.queue_capacity == 0 ||
      config.queue_capacity > kVelocityQueueCapacity) {
    *error_out = "the velocity recorder queue capacity must be 1.." +
                 std::to_string(kVelocityQueueCapacity) + ", got " +
                 std::to_string(config.queue_capacity);
    return false;
  }
  capacity = config.queue_capacity;
  if (config.directory.empty()) {
    *error_out = "the velocity recorder has no directory";
    return false;
  }
  // The configured base may not exist yet: an application can start before the
  // logger or the runtime created its shared directory. Creating it here keeps
  // the run directory (below) unique while making Start independent of who
  // wrote the base first. A path that exists as a file, or a directory that
  // cannot be created, is still a clear startup error.
  std::error_code code;
  std::filesystem::create_directories(config.directory, code);
  if (code) {
    *error_out = "the velocity recorder could not create " +
                 config.directory.string() + ": " + code.message();
    return false;
  }
  if (!std::filesystem::is_directory(config.directory, code) || code) {
    *error_out = "the velocity recorder directory " +
                 config.directory.string() + " is not a directory";
    return false;
  }

  // The metadata is the run's own schema identity; a caller that hands over
  // another version is refused here instead of producing a directory no reader
  // of this build accepts.
  const Json metadata = Json::parse(meta_json, nullptr, false);
  if (metadata.is_discarded() || !metadata.is_object()) {
    *error_out = "the velocity metadata is not a JSON object";
    return false;
  }
  std::string why;
  if (!SchemaMatches(metadata, &why)) {
    *error_out = "the velocity metadata is refused: " + why;
    return false;
  }

  // One directory per process run. The name is the UTC instant plus an id that
  // is unique within the host, and the creation is the claim: an existing
  // directory is never written into.
  bool created = false;
  for (int attempt = 0; attempt < 64 && !created; ++attempt) {
    run_directory =
        config.directory / ("app_run_" + UtcStamp() + "_" + UniqueSuffix());
    created = std::filesystem::create_directory(run_directory, code);
    if (code) {
      *error_out = "the velocity recorder could not create " +
                   run_directory.string() + ": " + code.message();
      return false;
    }
  }
  if (!created) {
    *error_out =
        "the velocity recorder could not create a unique run "
        "directory under " +
        config.directory.string();
    run_directory.clear();
    return false;
  }

  const auto discard_run = [this] {
    std::error_code ignored;
    std::filesystem::remove_all(run_directory, ignored);
    run_directory.clear();
  };
  {
    std::ofstream metadata_file(run_directory /
                                std::string(kVelocityMetadataFile));
    metadata_file << meta_json;
    metadata_file.put('\n');
    metadata_file.flush();
    if (!metadata_file.good()) {
      *error_out =
          "the velocity recorder could not write " + run_directory.string();
      discard_run();
      return false;
    }
  }
  if (!live_options.socket_path.empty()) {
    Json live_metadata = Json::parse(meta_json, nullptr, false);
    if (live_metadata.is_object()) {
      live_metadata["source"] = "application";
      live_metadata["run_path"] = run_directory.string();
      live = std::make_unique<LiveTelemetrySender>(live_options,
                                                   live_metadata.dump());
    }
  }
  // The stream exists from the start, even when no sample is ever recorded: a
  // reader then distinguishes "this run recorded no velocity" from a file that
  // was never created.
  {
    std::ofstream stream(run_directory / std::string(kVelocityFile),
                         std::ios::trunc);
    stream.flush();
    if (!stream.good()) {
      *error_out =
          "the velocity recorder could not write " + run_directory.string();
      discard_run();
      return false;
    }
  }
  // Publish the open run immediately; the worker replaces the index when it
  // finalizes. A reader can open a run that is still recording.
  if (!WriteIndex(false)) {
    *error_out =
        "the velocity recorder could not write " + run_directory.string();
    discard_run();
    return false;
  }
  return true;
}

void VelocityRecorder::Impl::RollBack() noexcept {
  if (file.is_open()) {
    file.close();
  }
  file.clear();
  std::error_code code;
  const std::uintmax_t size = std::filesystem::file_size(
      run_directory / std::string(kVelocityFile), code);
  if (!code && size > committed_bytes) {
    std::error_code ignored;
    std::filesystem::resize_file(run_directory / std::string(kVelocityFile),
                                 committed_bytes, ignored);
  }
}

bool VelocityRecorder::Impl::WriteLine(const VelocitySample &sample) {
  Json line;
  line["schema"] = std::string(kVelocitySchema);
  line["version"] = kVelocitySchemaVersion;
  // Host wall time is informational only; `stamp_us` is the alignment key.
  line["host_us"] = HostMicros();
  line["stamp_us"] = sample.stamp_us;
  line["sample_seq"] = HexSequence(sample.sample_seq);
  line["identity_known"] = sample.identity_known;
  line["runtime_id"] =
      sample.identity_known ? HexId(sample.runtime_id) : std::string();
  line["session_id"] =
      sample.identity_known ? HexId(sample.session_id) : std::string();
  line["latest_state_seq"] = HexSequence(sample.latest_state_seq);
  line["forward"] = EncodeDouble(sample.forward);
  line["lateral"] = EncodeDouble(sample.lateral);
  line["yaw"] = EncodeDouble(sample.yaw);
  const std::string text =
      line.dump(-1, ' ', false, Json::error_handler_t::replace);

  int (*hook)(void *, std::string_view) = nullptr;
  void *hook_context = nullptr;
  {
    const std::lock_guard<std::mutex> lock(VelocityLineHookMutex());
    hook = g_velocity_line_hook;
    hook_context = g_velocity_line_hook_context;
  }
  const int action = hook == nullptr ? 0 : hook(hook_context, text);
  if (action == 1) {
    RollBack();
    return false;
  }

  if (!file.is_open()) {
    file.open(run_directory / std::string(kVelocityFile),
              std::ios::binary | std::ios::app);
    if (!file) {
      RollBack();
      return false;
    }
  }
  if (action == 2) {
    // A torn write: a prefix without its newline reaches the device, then the
    // write fails. The rollback removes it, so the next line starts at a
    // boundary and cannot merge with these bytes.
    const std::size_t prefix = text.empty() ? 0 : text.size() / 2;
    if (prefix != 0) {
      file.write(text.data(), static_cast<std::streamsize>(prefix));
    }
    RollBack();
    return false;
  }
  file.write(text.data(), static_cast<std::streamsize>(text.size()));
  file.put('\n');
  file.flush();
  if (!file.good()) {
    RollBack();
    return false;
  }
  committed_bytes += static_cast<std::uint64_t>(text.size() + 1);
  return true;
}

void VelocityRecorder::Impl::NoteQueueRefusal(
    const VelocitySample &sample) noexcept {
  if (queue_loss_open_valid) {
    queue_loss_open.last_seq = sample.sample_seq;
    queue_loss_open.last_stamp_us = sample.stamp_us;
    ++queue_loss_open.dropped;
    return;
  }
  queue_loss_open = LossSpan{};
  queue_loss_open.first_seq = sample.sample_seq;
  queue_loss_open.last_seq = sample.sample_seq;
  queue_loss_open.first_stamp_us = sample.stamp_us;
  queue_loss_open.last_stamp_us = sample.stamp_us;
  queue_loss_open.dropped = 1;
  queue_loss_open.reason = VelocityLossReason::QueueFull;
  queue_loss_open_valid = true;
}

void VelocityRecorder::Impl::CloseQueueLoss() noexcept {
  if (!queue_loss_open_valid) {
    return;
  }
  PutLoss(queue_losses, &queue_loss_count, queue_loss_open);
  queue_loss_open_valid = false;
}

void VelocityRecorder::Impl::NoteWriteFailure(
    const VelocitySample &sample) noexcept {
  if (write_loss_open_valid) {
    write_loss_open.last_seq = sample.sample_seq;
    write_loss_open.last_stamp_us = sample.stamp_us;
    ++write_loss_open.dropped;
    return;
  }
  write_loss_open = LossSpan{};
  write_loss_open.first_seq = sample.sample_seq;
  write_loss_open.last_seq = sample.sample_seq;
  write_loss_open.first_stamp_us = sample.stamp_us;
  write_loss_open.last_stamp_us = sample.stamp_us;
  write_loss_open.dropped = 1;
  write_loss_open.reason = VelocityLossReason::WriteFailed;
  write_loss_open_valid = true;
}

void VelocityRecorder::Impl::CloseWriteLoss() noexcept {
  if (!write_loss_open_valid) {
    return;
  }
  PutLoss(write_losses, &write_loss_count, write_loss_open);
  write_loss_open_valid = false;
}

void VelocityRecorder::Impl::PutLoss(LossSpan *table, std::uint32_t *count,
                                     const LossSpan &span) noexcept {
  if (*count < kVelocityLossSegments) {
    table[*count] = span;
    ++*count;
    return;
  }
  // Past the bound the last span absorbs the new one: the loss is still named,
  // at a coarser resolution.
  LossSpan &last = table[kVelocityLossSegments - 1];
  last.last_seq = span.last_seq;
  last.last_stamp_us = span.last_stamp_us;
  last.dropped += span.dropped;
}

void VelocityRecorder::Impl::AppendLosses(Json *losses) const {
  for (std::uint32_t index = 0; index < queue_loss_count; ++index) {
    const LossSpan &loss = queue_losses[index];
    losses->push_back({{"first_sample_seq", HexSequence(loss.first_seq)},
                       {"last_sample_seq", HexSequence(loss.last_seq)},
                       {"first_stamp_us", loss.first_stamp_us},
                       {"last_stamp_us", loss.last_stamp_us},
                       {"dropped", loss.dropped},
                       {"reason", VelocityLossReasonName(loss.reason)}});
  }
  for (std::uint32_t index = 0; index < write_loss_count; ++index) {
    const LossSpan &loss = write_losses[index];
    losses->push_back({{"first_sample_seq", HexSequence(loss.first_seq)},
                       {"last_sample_seq", HexSequence(loss.last_seq)},
                       {"first_stamp_us", loss.first_stamp_us},
                       {"last_stamp_us", loss.last_stamp_us},
                       {"dropped", loss.dropped},
                       {"reason", VelocityLossReasonName(loss.reason)}});
  }
}

bool VelocityRecorder::Impl::WriteIndex(bool complete) const {
  Json index;
  index["schema"] = std::string(kVelocitySchema);
  index["version"] = kVelocitySchemaVersion;
  index["complete"] = complete;
  index["sample_hz"] = config.sample_hz;
  index["rows_recorded"] = recorded.load(std::memory_order_relaxed);
  index["rows_written"] = written.load(std::memory_order_relaxed);
  index["samples_dropped"] = dropped.load(std::memory_order_relaxed);
  index["write_failures"] = write_failure_count;
  Json losses = Json::array();
  AppendLosses(&losses);
  index["losses"] = std::move(losses);
  return WriteJsonAtomic(run_directory / std::string(kVelocityIndexFile),
                         index);
}

void VelocityRecorder::Impl::Drain() {
  VelocitySample sample;
  while (queue.TryPop(sample)) {
    if (live) {
      static_cast<void>(live->TrySend("sample", EncodeLiveVelocity(sample)));
    }
    if (WriteLine(sample)) {
      written.fetch_add(1, std::memory_order_relaxed);
      CloseWriteLoss();
    } else {
      write_failures.fetch_add(1, std::memory_order_relaxed);
      ++write_failure_count;
      NoteWriteFailure(sample);
    }
  }
}

void VelocityRecorder::Impl::SendLiveStatus(bool hello, bool complete) {
  if (!live || !live->enabled()) {
    return;
  }
  Json payload;
  payload["kind"] = hello ? "hello" : "status";
  payload["run_path"] = run_directory.string();
  payload["record_kind"] = "application";
  payload["sample_hz"] = config.sample_hz;
  payload["recorded"] = std::to_string(recorded.load());
  payload["written"] = std::to_string(written.load());
  payload["dropped"] = std::to_string(dropped.load());
  payload["write_failures"] = std::to_string(write_failures.load());
  payload["complete"] = complete;
  static_cast<void>(live->TrySend(
      hello ? "hello" : "status",
      payload.dump(-1, ' ', false, Json::error_handler_t::replace)));
  last_live_status = std::chrono::steady_clock::now();
}

void VelocityRecorder::Impl::Run() {
  SendLiveStatus(true);
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(done_mutex);
      done_cv.wait_for(lock, std::chrono::milliseconds(50), [this] {
        return stop_requested.load(std::memory_order_relaxed);
      });
    }
    if (stop_requested.load(std::memory_order_relaxed)) {
      break;
    }
    Drain();
    if (live && std::chrono::steady_clock::now() - last_live_status >=
                    std::chrono::milliseconds(100)) {
      SendLiveStatus(false);
    }
  }
  // Wait for a producer already inside TryRecord to leave, so what is left in
  // the queue is exactly the accepted samples of the run. The
  // sequentially-consistent handshake makes the zero observation final.
  for (;;) {
    if (offering.load(std::memory_order_seq_cst) == 0) {
      break;
    }
    std::unique_lock<std::mutex> lock(done_mutex);
    done_cv.wait_for(lock, std::chrono::milliseconds(10));
  }
  Drain();
  CloseQueueLoss();
  CloseWriteLoss();
  // The completion marker is a statement about persistence, not about the
  // sampling: a failed line is a loss the index names, and the run cannot claim
  // to be intact.
  static_cast<void>(WriteIndex(write_failure_count == 0));
  SendLiveStatus(false, true);
  {
    const std::lock_guard<std::mutex> lock(done_mutex);
    finished = true;
  }
  done_cv.notify_all();
}

VelocityRecorder::VelocityRecorder() = default;

VelocityRecorder::~VelocityRecorder() { Stop(); }

bool VelocityRecorder::Start(const configuration::RecorderConfig &config,
                             std::string meta_json, std::string *error,
                             LiveTelemetryOptions live) {
  if (impl_) {
    return true;
  }
  std::shared_ptr<Impl> impl = std::make_shared<Impl>();
  std::string message;
  if (!impl->Prepare(config, std::move(meta_json), &message, std::move(live))) {
    if (error != nullptr) {
      *error = message;
    }
    start_error_ = std::move(message);
    return false;
  }
  start_error_.clear();
  impl->accepting.store(true, std::memory_order_seq_cst);
  try {
    const std::shared_ptr<Impl> shared = impl;
    impl->worker = std::thread([shared] { shared->Run(); });
  } catch (const std::exception &exception) {
    // A process that cannot create the worker has not started a run: no state
    // may claim it accepted samples, and the half-created directory is removed
    // rather than left behind for a reader to misinterpret.
    impl->accepting.store(false, std::memory_order_seq_cst);
    std::error_code ignored;
    std::filesystem::remove_all(impl->run_directory, ignored);
    impl->run_directory.clear();
    start_error_ = std::string(
                       "the velocity recorder could not start its "
                       "worker: ") +
                   exception.what();
    if (error != nullptr) {
      *error = start_error_;
    }
    return false;
  }
  start_error_.clear();
  impl_ = impl;
  return true;
}

bool VelocityRecorder::TryRecord(const VelocitySample &sample) noexcept {
  if (!impl_) {
    ++unattached_dropped_;
    return false;
  }
  Impl &impl = *impl_;
  if (!impl.accepting.load(std::memory_order_seq_cst)) {
    impl.dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Announce first, then re-check acceptance. The count and `accepting` are
  // sequentially consistent, so a producer that increments after Stop()
  // published the refusal must see it and never touch the queue.
  impl.offering.fetch_add(1, std::memory_order_seq_cst);
  if (!impl.accepting.load(std::memory_order_seq_cst)) {
    impl.dropped.fetch_add(1, std::memory_order_relaxed);
    if (impl.offering.load(std::memory_order_seq_cst) == 1) {
      impl.done_cv.notify_all();
    }
    impl.offering.fetch_sub(1, std::memory_order_seq_cst);
    return false;
  }
  bool accepted = false;
  if (impl.queue.size() < impl.capacity) {
    accepted = impl.queue.TryPush(sample);
  }
  if (accepted) {
    impl.recorded.fetch_add(1, std::memory_order_relaxed);
    impl.CloseQueueLoss();
  } else {
    impl.NoteQueueRefusal(sample);
    impl.dropped.fetch_add(1, std::memory_order_relaxed);
  }
  if (impl.offering.load(std::memory_order_seq_cst) == 1) {
    impl.done_cv.notify_all();
  }
  impl.offering.fetch_sub(1, std::memory_order_seq_cst);
  return accepted;
}

bool VelocityRecorder::Stop() {
  if (!impl_) {
    return true;
  }
  const std::shared_ptr<Impl> impl = impl_;
  {
    const std::lock_guard<std::mutex> lock(impl->done_mutex);
    if (impl->finished) {
      if (impl->worker.joinable()) {
        impl->worker.join();
      }
      return true;
    }
    // A previous Stop already timed out and left the worker detached. Waiting
    // again would give one exit path two full deadlines; this call reports the
    // same unfinished state at once, and a later call observes the worker's
    // own completion above.
    if (impl->stop_signalled && !impl->worker.joinable()) {
      return false;
    }
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kVelocityStopTimeoutMs);
  impl->accepting.store(false, std::memory_order_seq_cst);
  {
    const std::lock_guard<std::mutex> lock(impl->done_mutex);
    if (!impl->stop_signalled) {
      impl->stop_signalled = true;
      impl->stop_requested.store(true, std::memory_order_relaxed);
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

bool VelocityRecorder::started() const noexcept {
  return impl_ && impl_->accepting.load(std::memory_order_acquire);
}

const std::filesystem::path &VelocityRecorder::run_directory() const noexcept {
  static const std::filesystem::path kEmpty;
  return impl_ ? impl_->run_directory : kEmpty;
}

std::uint64_t VelocityRecorder::recorded() const noexcept {
  return impl_ ? impl_->recorded.load(std::memory_order_relaxed) : 0;
}

std::uint64_t VelocityRecorder::written() const noexcept {
  return impl_ ? impl_->written.load(std::memory_order_relaxed) : 0;
}

std::uint64_t VelocityRecorder::dropped() const noexcept {
  return impl_ ? impl_->dropped.load(std::memory_order_relaxed)
               : unattached_dropped_;
}

std::uint64_t VelocityRecorder::write_failures() const noexcept {
  return impl_ ? impl_->write_failures.load(std::memory_order_relaxed) : 0;
}

const std::string &VelocityRecorder::error() const noexcept {
  return impl_ ? impl_->error : start_error_;
}

bool VisitVelocityRange(const std::filesystem::path &run_directory,
                        protocol::TimeUs begin_us, protocol::TimeUs end_us,
                        const VelocitySampleVisitor &visitor,
                        VelocityRecordRange *summary) {
  if (summary == nullptr) {
    return false;
  }
  *summary = VelocityRecordRange{};
  std::error_code code;

  // The metadata owns the run's schema identity. A run whose metadata is
  // missing or of another version is not interpreted at all.
  const std::filesystem::path metadata_path =
      run_directory / std::string(kVelocityMetadataFile);
  std::ifstream metadata_file(metadata_path);
  if (!metadata_file) {
    summary->error = "cannot open " + metadata_path.string();
    return false;
  }
  const Json metadata = Json::parse(metadata_file, nullptr, false);
  if (metadata.is_discarded() || !metadata.is_object()) {
    summary->error = metadata_path.string() + " is not valid JSON";
    return false;
  }
  std::string why;
  if (!SchemaMatches(metadata, &why)) {
    summary->error = metadata_path.string() + ": " + why;
    return false;
  }

  // The index owns the loss table and the final completion marker. Without it
  // the run is readable but incomplete: the worker never published its final
  // state.
  const std::filesystem::path index_path =
      run_directory / std::string(kVelocityIndexFile);
  const bool index_exists = std::filesystem::exists(index_path, code);
  if (code) {
    summary->error = "cannot inspect " + index_path.string();
    return false;
  }
  if (index_exists) {
    std::ifstream file(index_path);
    if (!file) {
      summary->error = "cannot open " + index_path.string();
      return false;
    }
    const Json document = Json::parse(file, nullptr, false);
    if (document.is_discarded() || !document.is_object()) {
      summary->error = index_path.string() + " is not valid JSON";
      return false;
    }
    if (!SchemaMatches(document, &why)) {
      summary->error = index_path.string() + ": " + why;
      return false;
    }
    const auto complete = document.find("complete");
    if (complete != document.end()) {
      if (!complete->is_boolean()) {
        summary->error = index_path.string() + " has no completion marker";
        return false;
      }
      summary->complete = complete->get<bool>();
    }
    if (summary->complete) {
      // A finalized run with a failed write is not intact: the worker's final
      // marker already says false, but an index hand-edited to true must not
      // hide a loss that is still in the table.
      const auto failures = document.find("write_failures");
      if (failures != document.end() && failures->is_number_unsigned() &&
          failures->get<std::uint64_t>() != 0) {
        summary->complete = false;
      }
    }
    const auto losses = document.find("losses");
    if (losses != document.end()) {
      if (!losses->is_array()) {
        summary->error = index_path.string() + " has no loss table";
        return false;
      }
      for (const Json &entry : *losses) {
        VelocityLossSpan loss;
        if (!DecodeLoss(entry, &loss, &why)) {
          summary->error = index_path.string() + ": " + why;
          return false;
        }
        summary->losses.push_back(loss);
      }
    }
  }

  const std::filesystem::path stream_path =
      run_directory / std::string(kVelocityFile);
  std::ifstream file(stream_path, std::ios::binary);
  if (!file) {
    summary->error = "cannot open " + stream_path.string();
    return false;
  }
  // Read one complete line at a time. Only newline-terminated lines are
  // complete: a final line that reaches EOF without one is a write that did
  // not finish, and its bytes are never parsed as evidence of a sample.
  std::string line;
  std::uint64_t line_number = 0;
  bool keep_going = true;
  while (keep_going && std::getline(file, line)) {
    ++line_number;
    if (file.eof()) {
      summary->truncated = true;
      break;
    }
    const Json document = Json::parse(line, nullptr, false);
    if (document.is_discarded()) {
      summary->error = stream_path.string() + ":" +
                       std::to_string(line_number) + ": not valid JSON";
      return false;
    }
    VelocitySample sample;
    std::string why;
    if (!DecodeSample(document, &sample, &why)) {
      summary->error =
          stream_path.string() + ":" + std::to_string(line_number) + ": " + why;
      return false;
    }
    if (begin_us < end_us && sample.stamp_us >= begin_us &&
        sample.stamp_us < end_us) {
      if (visitor && !visitor(sample)) {
        keep_going = false;
      }
    }
  }
  if (file.bad()) {
    summary->error = "cannot read " + stream_path.string();
    return false;
  }
  summary->ok = true;
  return true;
}

bool ReadVelocityRange(const std::filesystem::path &run_directory,
                       protocol::TimeUs begin_us, protocol::TimeUs end_us,
                       VelocityRecordRange *range) {
  if (range == nullptr) {
    return false;
  }
  VelocityRecordRange summary;
  const bool ok = VisitVelocityRange(
      run_directory, begin_us, end_us,
      [&summary](const VelocitySample &sample) {
        summary.rows.push_back(sample);
        return true;
      },
      &summary);
  *range = std::move(summary);
  return ok;
}

}  // namespace irmv::telemetry
