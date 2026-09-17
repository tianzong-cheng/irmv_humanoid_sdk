#include "irmv/telemetry/logger.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace irmv::telemetry {
namespace {

// How long the worker waits for the producer before it looks at the queue
// again. The producer never takes the worker's mutex, so a wake-up can be
// missed; this bound is the safety net that keeps a line from waiting forever.
constexpr int kWorkerPollMs = 10;

// Severity order is the declaration order of configuration::LogLevel, which is
// documented as running from the most verbose to the most severe.
int LevelRank(configuration::LogLevel level) {
  return static_cast<int>(level);
}

// Appends to a fixed buffer and never writes past its end. An event that
// respects the fixed string capacities always fits (see kMaxLogLine), so a
// truncated line is a bug in the caller, not a lost log.
void Append(char *buffer, std::size_t capacity, std::size_t &used,
            std::string_view text) {
  const std::size_t room = used < capacity ? capacity - used : 0;
  const std::size_t count = std::min(room, text.size());
  std::memcpy(buffer + used, text.data(), count);
  used += count;
}

void AppendUint(char *buffer, std::size_t capacity, std::size_t &used,
                std::uint64_t value) {
  char digits[20];
  std::size_t length = 0;
  do {
    digits[length++] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0);
  while (length > 0) {
    const std::string_view digit(&digits[--length], 1);
    Append(buffer, capacity, used, digit);
  }
}

// Explicit bounded encoding: control bytes, backslashes and quotes are escaped
// so that one event is exactly one line and the structured tail can be split on
// spaces and quotes. Values are quoted; the message and the component are not,
// because they are the plain readable part of the line.
void AppendText(char *buffer, std::size_t capacity, std::size_t &used,
                std::string_view text, bool quoted) {
  constexpr char kHex[] = "0123456789abcdef";
  if (quoted) {
    Append(buffer, capacity, used, "\"");
  }
  for (const char character : text) {
    const unsigned char byte = static_cast<unsigned char>(character);
    if (character == '\\') {
      Append(buffer, capacity, used, "\\\\");
    } else if (character == '"') {
      Append(buffer, capacity, used, "\\\"");
    } else if (byte < 0x20 || byte == 0x7f) {
      const char escape[4] = {'\\', 'x', kHex[(byte >> 4) & 0x0f],
                              kHex[byte & 0x0f]};
      Append(buffer, capacity, used, std::string_view(escape, 4));
    } else {
      Append(buffer, capacity, used, std::string_view(&character, 1));
    }
  }
  if (quoted) {
    Append(buffer, capacity, used, "\"");
  }
}

// `LEVEL stamp_us tick [component] message {event=<code> name="value" ...}`.
// The terminator is always written: one event, one line.
std::size_t FormatEvent(const LogEvent &event, char *buffer,
                        std::size_t capacity) {
  if (capacity == 0) {
    return 0;
  }
  const std::size_t content = capacity - 1;
  std::size_t used = 0;
  Append(buffer, content, used, configuration::LogLevelName(event.level));
  Append(buffer, content, used, " ");
  AppendUint(buffer, content, used, event.stamp_us);
  Append(buffer, content, used, " ");
  AppendUint(buffer, content, used, event.tick);
  Append(buffer, content, used, " [");
  AppendText(buffer, content, used, event.component.View(), false);
  Append(buffer, content, used, "] ");
  AppendText(buffer, content, used, event.message.View(), false);

  const std::uint32_t field_count =
      std::min<std::uint32_t>(event.field_count, kMaxLogFields);
  if (event.event_code != 0 || field_count > 0) {
    Append(buffer, content, used, " {");
    bool first = true;
    if (event.event_code != 0) {
      Append(buffer, content, used, "event=");
      AppendUint(buffer, content, used, event.event_code);
      first = false;
    }
    for (std::uint32_t index = 0; index < field_count; ++index) {
      if (!first) {
        Append(buffer, content, used, " ");
      }
      first = false;
      AppendText(buffer, content, used, event.fields[index].name.View(), false);
      Append(buffer, content, used, "=");
      AppendText(buffer, content, used, event.fields[index].value.View(), true);
    }
    Append(buffer, content, used, "}");
  }
  buffer[used++] = '\n';
  return used;
}

std::string UtcStamp() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  char buffer[32] = {};
  const std::size_t length =
      std::strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%SZ", &utc);
  return std::string(buffer, length);
}

std::filesystem::path RotatedPath(const std::filesystem::path &path,
                                  std::uint32_t index) {
  return std::filesystem::path(path.string() + "." + std::to_string(index));
}

}  // namespace

RotatingFileSink::RotatingFileSink(const configuration::TelemetryConfig &config)
    : directory_(config.directory),
      rotate_bytes_(config.rotate_bytes),
      retained_files_(config.retained_files) {
  // Only a path is built here: the directory and the file are created on the
  // worker thread, on the first Write().
  path_ = directory_ / ("run_" + UtcStamp()) / "humanoid.log";
}

bool RotatingFileSink::ReportError(std::string_view what,
                                   std::error_code error) {
  if (error_reported_) {
    return false;
  }
  error_reported_ = true;
  const std::string message = error ? error.message() : std::string();
  std::fprintf(stderr, "irmv logger: %.*s: %s%s%s\n",
               static_cast<int>(what.size()), what.data(), path_.string().c_str(),
               error ? ": " : "", message.c_str());
  return false;
}

bool RotatingFileSink::OpenIfNeeded() {
  if (file_.is_open()) {
    return true;
  }
  std::error_code error;
  const std::filesystem::path parent = path_.parent_path();
  if (!std::filesystem::create_directories(parent, error) && error) {
    return ReportError("cannot create log directory", error);
  }
  file_.open(path_, std::ios::out | std::ios::binary | std::ios::app);
  if (!file_.is_open()) {
    return ReportError("cannot open log file", std::error_code());
  }
  error.clear();
  const std::uintmax_t existing = std::filesystem::file_size(path_, error);
  size_ = error ? 0 : static_cast<std::uint64_t>(existing);
  return true;
}

bool RotatingFileSink::RotateIfNeeded(std::size_t line_bytes) {
  if (rotate_bytes_ == 0 || size_ == 0 ||
      size_ + line_bytes <= rotate_bytes_) {
    return true;
  }
  file_.close();
  // The active file is about to be rotated away or truncated; its size is stale
  // from here on.
  size_ = 0;
  std::error_code error;
  if (retained_files_ == 0) {
    // No rotation history is kept: the finished file is discarded.
    std::filesystem::remove(path_, error);
  } else {
    // The oldest retained file makes room, the rest shift by one and the
    // finished file becomes the newest rotated one.
    std::filesystem::remove(RotatedPath(path_, retained_files_), error);
    error.clear();
    for (std::uint32_t index = retained_files_; index > 1; --index) {
      const std::filesystem::path from = RotatedPath(path_, index - 1);
      const std::filesystem::path to = RotatedPath(path_, index);
      error.clear();
      const bool present = std::filesystem::exists(from, error);
      if (error) {
        return ReportError("cannot inspect rotated log file", error);
      }
      if (!present) {
        continue;
      }
      error.clear();
      std::filesystem::rename(from, to, error);
      if (error) {
        return ReportError("cannot rotate log file", error);
      }
    }
    error.clear();
    std::filesystem::rename(path_, RotatedPath(path_, 1), error);
    if (error) {
      return ReportError("cannot rotate log file", error);
    }
  }
  size_ = 0;
  file_.open(path_, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!file_.is_open()) {
    return ReportError("cannot reopen log file", std::error_code());
  }
  return true;
}

bool RotatingFileSink::AppendLine(const char *line, std::size_t length) {
  file_.write(line, static_cast<std::streamsize>(length));
  file_.flush();
  if (!file_) {
    return ReportError("log write failed", std::error_code());
  }
  size_ += length;
  error_reported_ = false;
  return true;
}

bool RotatingFileSink::Write(const LogEvent &event) {
  char line[kMaxLogLine];
  const std::size_t length = FormatEvent(event, line, sizeof(line));
  // stderr is the sink that never rotates: a run keeps its diagnostics even
  // when the file backend is unusable.
  const bool reportable = std::fwrite(line, 1, length, stderr) == length;
  const bool stored = OpenIfNeeded() && RotateIfNeeded(length) &&
                      AppendLine(line, length);
  return reportable && stored;
}

bool RotatingFileSink::Flush() {
  if (!file_.is_open()) {
    // Nothing is buffered: every line was flushed by Write().
    return true;
  }
  file_.flush();
  if (!file_) {
    return ReportError("log flush failed", std::error_code());
  }
  error_reported_ = false;
  return true;
}

// Internal state of the logger. It is shared with the worker so that a worker
// left behind by a blocked sink keeps everything it touches alive after Stop().
struct Logger::Shared {
  Shared(const configuration::TelemetryConfig &config, LogSink *sink)
      : level_(config.level),
        capacity_(std::max<std::size_t>(config.queue_capacity, 1)),
        slots_(capacity_) {
    if (sink != nullptr) {
      sink_ = sink;
      return;
    }
    owned_sink_ = std::make_unique<RotatingFileSink>(config);
    sink_ = owned_sink_.get();
  }

  // Producer side. Returns false when the queue is full and leaves it
  // untouched.
  bool TryPush(const LogEvent &event) {
    const std::size_t write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) >= capacity_) {
      return false;
    }
    slots_[write % capacity_] = event;
    write_.store(write + 1, std::memory_order_release);
    return true;
  }

  // Consumer side. Returns false when the queue is empty.
  bool TryPop(LogEvent &out) {
    const std::size_t read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {
      return false;
    }
    out = slots_[read % capacity_];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }

  bool Empty() const {
    return read_.load(std::memory_order_relaxed) ==
           write_.load(std::memory_order_acquire);
  }

  // Fixed-capacity single-producer/single-consumer ring. Only the indices cross
  // the thread boundary: the producer writes write_ alone, the consumer reads_
  // alone, and both are published with release/acquire ordering. No lock, no
  // allocation and no waiting on either side.
  configuration::LogLevel level_;
  std::size_t capacity_;
  std::vector<LogEvent> slots_;
  std::atomic<std::size_t> write_{0};
  std::atomic<std::size_t> read_{0};

  std::unique_ptr<RotatingFileSink> owned_sink_;
  LogSink *sink_{nullptr};

  // Pending events are counted by the producer before they become visible to
  // the consumer, so the consumer can never decrement an entry it has not seen
  // and Stop() can read how much is still in flight at its deadline.
  std::atomic<std::uint64_t> pending_{0};
  std::atomic<std::uint64_t> written_{0};
  std::atomic<std::uint64_t> errors_{0};
  std::atomic<bool> accepting_{false};

  // Producer-side counters: only the thread calling TryLog() touches them.
  std::uint64_t dropped_{0};
  std::uint64_t filtered_{0};

  // Written by Stop() and Undrained(), read on the same thread.
  std::uint64_t undrained_{0};

  std::mutex mutex_;
  std::condition_variable cv_;
  bool started_once_{false};
  bool stopping_{false};
  bool stopped_{false};
  bool worker_done_{false};
  std::thread worker_;
};

Logger::Logger(const configuration::TelemetryConfig &config)
    : shared_(std::make_shared<Shared>(config, nullptr)) {}

Logger::Logger(const configuration::TelemetryConfig &config, LogSink *sink)
    : shared_(std::make_shared<Shared>(config, sink)) {}

Logger::~Logger() { Stop(); }

bool Logger::Start() {
  const std::shared_ptr<Shared> shared = shared_;
  std::lock_guard<std::mutex> lock(shared->mutex_);
  if (shared->started_once_) {
    return false;
  }
  shared->started_once_ = true;
  shared->accepting_.store(true, std::memory_order_release);
  shared->worker_ = std::thread([shared] { Logger::WorkerMain(shared); });
  return true;
}

void Logger::Stop() {
  const std::shared_ptr<Shared> shared = shared_;
  bool drained = false;
  {
    std::unique_lock<std::mutex> lock(shared->mutex_);
    if (!shared->started_once_ || shared->stopped_) {
      return;
    }
    shared->stopped_ = true;
    shared->accepting_.store(false, std::memory_order_release);
    shared->stopping_ = true;
    shared->cv_.notify_all();
    drained =
        shared->cv_.wait_for(lock, std::chrono::milliseconds(kStopDrainTimeoutMs),
                             [&shared] { return shared->worker_done_; });
    // Whatever is still in flight at the deadline is reported, not assumed
    // written.
    shared->undrained_ = shared->pending_.load(std::memory_order_acquire);
    if (!drained && shared->worker_.joinable()) {
      // A sink blocked inside Write() would make join() wait forever. The
      // worker owns its state through its shared_ptr, so it is left to finish on
      // its own instead of hanging the shutting-down control loop.
      shared->worker_.detach();
    }
  }
  if (drained && shared->worker_.joinable()) {
    shared->worker_.join();
  }
}

bool Logger::TryLog(const LogEvent &event) {
  Shared &shared = *shared_;
  if (!shared.accepting_.load(std::memory_order_acquire)) {
    return false;
  }
  if (LevelRank(event.level) < LevelRank(shared.level_)) {
    ++shared.filtered_;
    return false;
  }
  shared.pending_.fetch_add(1, std::memory_order_relaxed);
  if (!shared.TryPush(event)) {
    shared.pending_.fetch_sub(1, std::memory_order_relaxed);
    ++shared.dropped_;
    return false;
  }
  shared.cv_.notify_one();
  return true;
}

std::uint64_t Logger::Dropped() const { return shared_->dropped_; }

std::uint64_t Logger::Filtered() const { return shared_->filtered_; }

std::uint64_t Logger::Written() const {
  return shared_->written_.load(std::memory_order_relaxed);
}

std::uint64_t Logger::Undrained() const { return shared_->undrained_; }

std::uint64_t Logger::error_count() const {
  return shared_->errors_.load(std::memory_order_relaxed);
}

bool Logger::started() const {
  return shared_->accepting_.load(std::memory_order_acquire);
}

void Logger::WorkerMain(std::shared_ptr<Shared> shared) {
  LogEvent event;
  for (;;) {
    bool stopping = false;
    {
      std::lock_guard<std::mutex> lock(shared->mutex_);
      stopping = shared->stopping_;
    }
    while (shared->TryPop(event)) {
      bool ok = false;
      try {
        ok = shared->sink_->Write(event);
      } catch (const std::exception &) {
        // A throwing sink is an error to count, not a reason to end the process
        // from inside the logging thread.
        ok = false;
      }
      if (ok) {
        shared->written_.fetch_add(1, std::memory_order_relaxed);
      } else {
        shared->errors_.fetch_add(1, std::memory_order_relaxed);
      }
      shared->pending_.fetch_sub(1, std::memory_order_relaxed);
    }
    if (stopping) {
      break;
    }
    std::unique_lock<std::mutex> lock(shared->mutex_);
    shared->cv_.wait_for(lock, std::chrono::milliseconds(kWorkerPollMs),
                         [&shared] {
                           return shared->stopping_ || !shared->Empty();
                         });
  }
  try {
    if (!shared->sink_->Flush()) {
      shared->errors_.fetch_add(1, std::memory_order_relaxed);
    }
  } catch (const std::exception &) {
    shared->errors_.fetch_add(1, std::memory_order_relaxed);
  }
  {
    std::lock_guard<std::mutex> lock(shared->mutex_);
    shared->worker_done_ = true;
  }
  shared->cv_.notify_all();
}

}  // namespace irmv::telemetry
