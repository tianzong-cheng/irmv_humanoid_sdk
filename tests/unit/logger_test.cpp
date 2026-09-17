// Tests for the bounded asynchronous logger: the queue bound and its drop
// accounting, level filtering, FIFO order, the drain on Stop(), rotation of the
// default sink, the formatted line and the error reporting path.
//
// Waits are bounded and explicit: the blocking sink signals when the worker is
// inside Write(), so filling a bounded queue is deterministic instead of
// timing-based. The only wall-clock assertions are hang guards.

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/telemetry/logger.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::LogLevel;
using irmv::configuration::TelemetryConfig;
using irmv::telemetry::kMaxLogFields;
using irmv::telemetry::kStopDrainTimeoutMs;
using irmv::telemetry::LogEvent;
using irmv::telemetry::Logger;
using irmv::telemetry::LogSink;

// Generous bound for a wait that is expected to complete in microseconds; only
// a stalled implementation reaches it, and reaching it fails a CHECK.
constexpr std::chrono::milliseconds kWait{5000};

TelemetryConfig BaseConfig(const std::filesystem::path &directory) {
  TelemetryConfig config;
  config.level = LogLevel::Trace;
  config.directory = directory;
  config.rotate_bytes = 1U << 20;
  config.retained_files = 3;
  config.queue_capacity = 1024;
  return config;
}

class TemporaryDirectory {
 public:
  explicit TemporaryDirectory(std::string_view name) {
    static unsigned int next_id = 0;
    path_ = std::filesystem::temp_directory_path() /
            ("irmv-logger-test-" + std::string(name) + "-" +
             std::to_string(next_id++));
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path &path() const { return path_; }

 private:
  std::filesystem::path path_;
};

// Records every event it is handed. Block() holds the worker inside Write()
// until Release(), which is what makes filling the bounded queue deterministic.
class RecordingSink final : public LogSink {
 public:
  bool Write(const LogEvent &event) override {
    std::unique_lock<std::mutex> lock(mutex_);
    ++in_write_;
    cv_.notify_all();
    while (blocked_ && !released_) {
      cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    events_.push_back(event);
    --in_write_;
    cv_.notify_all();
    return !fail_;
  }

  bool Flush() override {
    std::lock_guard<std::mutex> lock(mutex_);
    ++flush_calls_;
    cv_.notify_all();
    return !fail_;
  }

  void Block() {
    std::lock_guard<std::mutex> lock(mutex_);
    blocked_ = true;
  }

  void Release() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    cv_.notify_all();
  }

  void set_fail(bool fail) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail_ = fail;
  }

  // True once the worker sits inside Write(), that is once it owns an event and
  // will not touch the queue again until Release().
  bool WaitForWriteEntry(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this] { return in_write_ > 0; });
  }

  bool WaitForEvents(std::size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout,
                        [this, count] { return events_.size() >= count; });
  }

  // The worker flushes only after it drained the queue, so this wait also
  // proves that a detached worker no longer touches the sink.
  bool WaitForFlushes(std::size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout,
                        [this, count] { return flush_calls_ >= count; });
  }

  std::vector<LogEvent> events() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  std::size_t event_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_.size();
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<LogEvent> events_;
  std::size_t in_write_{0};
  std::size_t flush_calls_{0};
  bool blocked_{false};
  bool released_{false};
  bool fail_{false};
};

LogEvent MakeEvent(LogLevel level, std::string_view message, std::uint64_t tick) {
  LogEvent event;
  event.level = level;
  event.stamp_us = 1000 + tick;
  event.tick = tick;
  static_cast<void>(event.component.Assign("test"));
  static_cast<void>(event.message.Assign(message));
  return event;
}

void AddField(LogEvent &event, std::string_view name, std::string_view value) {
  if (event.field_count >= kMaxLogFields) {
    return;
  }
  static_cast<void>(event.fields[event.field_count].name.Assign(name));
  static_cast<void>(event.fields[event.field_count].value.Assign(value));
  ++event.field_count;
}

std::vector<std::string> ReadLines(const std::filesystem::path &path) {
  std::vector<std::string> lines;
  std::ifstream input(path);
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  return lines;
}

// Every file the default sink produced, oldest rotated file first, the active
// file last.
std::vector<std::filesystem::path> LogFiles(
    const std::filesystem::path &directory) {
  std::vector<std::filesystem::path> files;
  std::error_code error;
  for (const auto &run :
       std::filesystem::directory_iterator(directory, error)) {
    if (!run.is_directory()) {
      continue;
    }
    for (const auto &file :
         std::filesystem::directory_iterator(run.path(), error)) {
      files.push_back(file.path());
    }
  }
  std::sort(files.begin(), files.end(),
            [](const std::filesystem::path &left,
               const std::filesystem::path &right) {
              // The plain name is the active file and therefore the newest; a
              // higher rotation index is older.
              if (left.filename() == "humanoid.log") {
                return false;
              }
              if (right.filename() == "humanoid.log") {
                return true;
              }
              return left.filename().string() > right.filename().string();
            });
  return files;
}

std::uint64_t LineTick(const std::string &line) {
  std::istringstream stream(line);
  std::string level;
  std::uint64_t stamp = 0;
  std::uint64_t tick = 0;
  stream >> level >> stamp >> tick;
  return tick;
}

}  // namespace

int main() {
  // A blocked sink fills the bounded queue: TryLog then reports the drop
  // instead of waiting for the worker or blocking the caller.
  {
    const TemporaryDirectory directory("drop");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 1024;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    sink->Block();
    Logger logger(config, sink);
    CHECK(logger.Start());
    CHECK(logger.started());
    CHECK(logger.TryLog(MakeEvent(LogLevel::Info, "in flight", 0)));
    CHECK(sink->WaitForWriteEntry(kWait));

    const auto begin = std::chrono::steady_clock::now();
    int accepted = 0;
    int rejected = 0;
    for (int index = 0; index < 2048; ++index) {
      if (logger.TryLog(MakeEvent(LogLevel::Info, "flood",
                                  static_cast<std::uint64_t>(index)))) {
        ++accepted;
      } else {
        ++rejected;
      }
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - begin);
    CHECK(logger.Dropped() > 0);
    // One event is inside the blocked Write(), the queue holds exactly its
    // capacity, everything beyond that is refused without waiting.
    CHECK(accepted == 1024);
    CHECK(rejected == 1024);
    CHECK(logger.Dropped() == 1024);
    CHECK(elapsed < std::chrono::milliseconds(2000));
    CHECK(logger.Filtered() == 0);

    // While the worker stays blocked the queue stays full: every further call is
    // refused, and it is refused immediately rather than after a wait.
    const auto refused_begin = std::chrono::steady_clock::now();
    for (int index = 0; index < 1024; ++index) {
      CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "refused", 0)));
    }
    const auto refused_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - refused_begin);
    CHECK(logger.Dropped() == 2048);
    CHECK(refused_elapsed < std::chrono::milliseconds(500));

    sink->Release();
    logger.Stop();
    CHECK(!logger.started());
    CHECK(logger.Written() == 1025);
    CHECK(logger.Undrained() == 0);
  }

  // Stop() must not hang on a blocked sink: it returns after its bounded drain
  // deadline and reports the events it could not drain instead of pretending
  // the log is complete.
  {
    const TemporaryDirectory directory("stop-blocked");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 4;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    sink->Block();
    Logger logger(config, sink);
    CHECK(logger.Start());
    int accepted = 0;
    for (int index = 0; index < 8; ++index) {
      if (logger.TryLog(MakeEvent(LogLevel::Info, "queued",
                                  static_cast<std::uint64_t>(index)))) {
        ++accepted;
      }
    }
    CHECK(accepted >= 4);
    CHECK(logger.Dropped() == static_cast<std::uint64_t>(8 - accepted));

    const auto begin = std::chrono::steady_clock::now();
    logger.Stop();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - begin);
    CHECK(elapsed <
          std::chrono::milliseconds(4 * kStopDrainTimeoutMs + 1000));
    CHECK(!logger.started());
    CHECK(logger.Undrained() == static_cast<std::uint64_t>(accepted));

    // A stopped logger accepts nothing, and a refused event is not a drop.
    const std::uint64_t dropped = logger.Dropped();
    CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "late", 99)));
    CHECK(logger.Dropped() == dropped);

    // The detached worker still finishes the events it owns.
    sink->Release();
    CHECK(sink->WaitForFlushes(1, kWait));
    CHECK(sink->event_count() == static_cast<std::size_t>(accepted));
    CHECK(logger.Written() == static_cast<std::uint64_t>(accepted));
  }

  // Level filtering: an event below the configured level is counted and never
  // reaches the sink.
  {
    const TemporaryDirectory directory("filter");
    TelemetryConfig config = BaseConfig(directory.path());
    config.level = LogLevel::Warning;
    config.queue_capacity = 8;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    Logger logger(config, sink);
    CHECK(logger.Start());
    CHECK(!logger.TryLog(MakeEvent(LogLevel::Trace, "trace", 0)));
    CHECK(!logger.TryLog(MakeEvent(LogLevel::Debug, "debug", 1)));
    CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "info", 2)));
    CHECK(logger.TryLog(MakeEvent(LogLevel::Warning, "warning", 3)));
    CHECK(logger.TryLog(MakeEvent(LogLevel::Error, "error", 4)));
    CHECK(logger.TryLog(MakeEvent(LogLevel::Fatal, "fatal", 5)));
    CHECK(logger.Filtered() == 3);
    CHECK(logger.Dropped() == 0);
    logger.Stop();

    const auto events = sink->events();
    CHECK(events.size() == 3);
    CHECK(events[0].level == LogLevel::Warning);
    CHECK(events[1].level == LogLevel::Error);
    CHECK(events[2].level == LogLevel::Fatal);
    CHECK(events[0].tick == 3);
    CHECK(events[1].tick == 4);
    CHECK(events[2].tick == 5);
    CHECK(logger.Written() == 3);
  }

  // Accepted events reach the sink in order and Stop() drains the queue.
  {
    const TemporaryDirectory directory("drain");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 8;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    Logger logger(config, sink);
    CHECK(logger.Start());
    std::vector<std::uint64_t> accepted_ticks;
    for (std::uint64_t tick = 0; tick < 40; ++tick) {
      if (logger.TryLog(MakeEvent(LogLevel::Info, "ordered", tick))) {
        accepted_ticks.push_back(tick);
      }
    }
    // The queue absorbs its whole capacity even before the worker wakes up.
    CHECK(accepted_ticks.size() >= 8);
    logger.Stop();
    CHECK(logger.Undrained() == 0);
    CHECK(logger.Written() == accepted_ticks.size());
    const auto events = sink->events();
    CHECK(events.size() == accepted_ticks.size());
    for (std::size_t index = 0; index < events.size(); ++index) {
      CHECK(events[index].tick == accepted_ticks[index]);
    }
  }

  // Started once, stopped idempotently, and nothing reaches the sink or the
  // counters after Stop().
  {
    const TemporaryDirectory directory("stopped");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 8;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    Logger logger(config, sink);
    CHECK(!logger.started());
    CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "before start", 0)));
    CHECK(logger.Start());
    CHECK(!logger.Start());
    CHECK(logger.TryLog(MakeEvent(LogLevel::Info, "written", 0)));
    logger.Stop();
    CHECK(!logger.started());
    const std::size_t written = sink->event_count();
    CHECK(written == 1);
    for (int index = 0; index < 8; ++index) {
      CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "after stop",
                                     static_cast<std::uint64_t>(index))));
    }
    logger.Stop();
    CHECK(logger.Dropped() == 0);
    CHECK(logger.Filtered() == 0);
    CHECK(sink->event_count() == written);
    CHECK(logger.Written() == written);
  }

  // A worker left behind by a timed-out Stop() must not outlive the sink: the
  // logger owns its own reference to the injected sink until that worker has
  // finished, so releasing the caller's reference and destroying the logger
  // right after Stop() is safe, and the events the worker still owns still land
  // in the sink.
  {
    const TemporaryDirectory directory("sink-lifetime");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 4;
    std::weak_ptr<RecordingSink> reference;
    std::size_t accepted = 0;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    sink->Block();
    {
      Logger logger(config, sink);
      CHECK(logger.Start());
      for (int index = 0; index < 8; ++index) {
        if (logger.TryLog(MakeEvent(LogLevel::Info, "detached",
                                    static_cast<std::uint64_t>(index)))) {
          ++accepted;
        }
      }
      CHECK(accepted >= 4);
      logger.Stop();
      // The worker is detached inside Write() and still owns accepted events.
      CHECK(logger.Undrained() == accepted);
      reference = sink;
    }
    // Both the logger and the caller release the sink while that worker is still
    // blocked. Only the logger's own reference keeps it alive now.
    sink.reset();
    const std::shared_ptr<RecordingSink> revived = reference.lock();
    CHECK(revived != nullptr);
    revived->Release();
    CHECK(revived->WaitForFlushes(1, kWait));
    CHECK(revived->event_count() == accepted);
  }

  // The default sink writes `LEVEL stamp tick [component] message` plus the
  // structured fields, with control bytes and quotes escaped.
  {
    const TemporaryDirectory directory("format");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 8;
    {
      Logger logger(config);
      CHECK(logger.Start());
      LogEvent state = MakeEvent(LogLevel::Warning, "tracking entered", 42);
      state.stamp_us = 1234567;
      CHECK(state.component.Assign("fsm"));
      state.event_code = 3;
      AddField(state, "from", "damping");
      AddField(state, "to", "tracking");
      CHECK(logger.TryLog(state));

      LogEvent escaped = MakeEvent(LogLevel::Error, "reason\tquoted", 43);
      escaped.event_code = 12;
      AddField(escaped, "detail", "a\\b\"c");
      CHECK(logger.TryLog(escaped));
      logger.Stop();
      CHECK(logger.Written() == 2);
      CHECK(logger.error_count() == 0);
      CHECK(logger.Undrained() == 0);
    }

    const auto files = LogFiles(directory.path());
    CHECK(files.size() == 1);
    const auto lines = ReadLines(files.at(0));
    CHECK(lines.size() == 2);
    CHECK(lines[0] ==
          "WARNING 1234567 42 [fsm] tracking entered "
          "{event=3 from=\"damping\" to=\"tracking\"}");
    CHECK(lines[1] ==
          "ERROR 1043 43 [test] reason\\x09quoted {event=12 detail=\"a\\\\b\\\"c\"}");
  }

  // The default sink rotates by size and keeps only the retained files.
  {
    const TemporaryDirectory directory("rotate");
    TelemetryConfig config = BaseConfig(directory.path());
    config.rotate_bytes = 256;
    config.retained_files = 2;
    config.queue_capacity = 64;
    const std::string message(60, 'x');
    {
      Logger logger(config);
      CHECK(logger.Start());
      for (std::uint64_t tick = 0; tick < 60; ++tick) {
        CHECK(logger.TryLog(MakeEvent(LogLevel::Info, message, tick)));
      }
      logger.Stop();
      CHECK(logger.Written() == 60);
      CHECK(logger.error_count() == 0);
    }

    const auto files = LogFiles(directory.path());
    CHECK(files.size() == 3);
    CHECK(files.back().filename() == "humanoid.log");
    CHECK(files[0].filename() == "humanoid.log.2");
    CHECK(files[1].filename() == "humanoid.log.1");

    std::vector<std::uint64_t> ticks;
    for (const auto &file : files) {
      const auto size = std::filesystem::file_size(file);
      CHECK(size > 0);
      CHECK(size <= 256);
      for (const auto &line : ReadLines(file)) {
        ticks.push_back(LineTick(line));
      }
    }
    // The retained files hold the newest events, complete and in order; the
    // older ones were rotated away instead of growing the run forever.
    CHECK(ticks.size() >= 3);
    CHECK(ticks.front() > 0);
    for (std::size_t index = 1; index < ticks.size(); ++index) {
      CHECK(ticks[index] == ticks[index - 1] + 1);
    }
    CHECK(ticks.back() == 59);
  }

  // A sink that fails is counted and the worker keeps running; the failed run
  // still ends through Stop().
  {
    const TemporaryDirectory directory("failure");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 8;
    std::shared_ptr<RecordingSink> sink = std::make_shared<RecordingSink>();
    sink->set_fail(true);
    Logger logger(config, sink);
    CHECK(logger.Start());
    for (std::uint64_t tick = 0; tick < 4; ++tick) {
      CHECK(logger.TryLog(MakeEvent(LogLevel::Info, "failed", tick)));
    }
    logger.Stop();
    CHECK(sink->event_count() == 4);
    CHECK(logger.Written() == 0);
    // Four failed writes plus the failed flush the worker performs on exit.
    CHECK(logger.error_count() == 5);
    CHECK(logger.Undrained() == 0);
  }

  // An unusable log directory is an observable error, not a crash out of the
  // worker thread: the file never opens, the write is counted and the line is
  // still reported on stderr.
  {
    const TemporaryDirectory directory("open-failure");
    std::filesystem::create_directories(directory.path());
    const std::filesystem::path file = directory.path() / "not-a-directory";
    {
      std::ofstream output(file);
      output << "occupied";
    }
    CHECK(std::filesystem::is_regular_file(file));
    TelemetryConfig config = BaseConfig(file / "logs");
    config.queue_capacity = 8;
    Logger logger(config);
    CHECK(logger.Start());
    CHECK(logger.TryLog(MakeEvent(LogLevel::Info, "no directory", 0)));
    logger.Stop();
    CHECK(logger.Written() == 0);
    CHECK(logger.error_count() == 1);
    CHECK(!std::filesystem::exists(file / "logs"));
  }

  return 0;
}
