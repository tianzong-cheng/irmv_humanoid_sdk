// Tests for the bounded asynchronous logger: the queue bound and its drop
// accounting, level filtering, FIFO order, the drain on Stop(), the accounting
// of an event that races with Stop(), the 256-byte message bound, rotation of
// the default sink, the formatted line and the error reporting path.
//
// Waits are bounded and explicit: the blocking sink signals when the worker is
// inside Write(), so filling a bounded queue is deterministic instead of
// timing-based, and assertions after Stop() wait for the sink or for a counter
// instead of assuming Stop() met its drain deadline. The only wall-clock
// assertions are hang guards.

#include "irmv/telemetry/logger.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
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
#include <thread>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/telemetry/diagnostic_write.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::LogLevel;
using irmv::configuration::TelemetryConfig;
using irmv::telemetry::kMaxBoundedDiagnosticBytes;
using irmv::telemetry::kMaxLogFields;
using irmv::telemetry::kMaxLogLine;
using irmv::telemetry::kStopDrainTimeoutMs;
using irmv::telemetry::LogEvent;
using irmv::telemetry::Logger;
using irmv::telemetry::LogSink;
using irmv::telemetry::WriteDiagnosticBlocking;
using irmv::telemetry::WriteDiagnosticBounded;

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

// Accepts everything immediately. The totals in the race test come from the
// logger itself, and a recording sink would make the producer's window
// unrealistically slow.
class FastSink final : public LogSink {
 public:
  bool Write(const LogEvent &) override { return true; }
  bool Flush() override { return true; }
};

// Waits for a condition the worker reaches asynchronously, so that assertions
// after Stop() do not depend on Stop()'s drain deadline: a loaded machine may
// leave the worker detached past it.
template <typename Predicate>
bool WaitFor(Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::yield();
  }
  return true;
}

LogEvent MakeEvent(LogLevel level, std::string_view message,
                   std::uint64_t tick) {
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
    CHECK(logger.Started());
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

    // While the worker stays blocked the queue stays full: every further call
    // is refused, and it is refused immediately rather than after a wait.
    const auto refused_begin = std::chrono::steady_clock::now();
    for (int index = 0; index < 1024; ++index) {
      CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "refused", 0)));
    }
    const auto refused_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - refused_begin);
    CHECK(logger.Dropped() == 2048);
    CHECK(refused_elapsed < std::chrono::milliseconds(500));

    // The worker takes every accepted event; the totals below are compared
    // after the sink has them, so a machine that misses Stop()'s drain deadline
    // cannot fail against a correct logger.
    sink->Release();
    logger.Stop();
    CHECK(!logger.Started());
    CHECK(WaitFor([&logger] { return logger.Written() == 1025; }, kWait));
    CHECK(logger.Dropped() == 2048);
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
    CHECK(elapsed < std::chrono::milliseconds(4 * kStopDrainTimeoutMs + 1000));
    CHECK(!logger.Started());
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
    CHECK(sink->WaitForEvents(3, kWait));
    CHECK(sink->WaitForFlushes(1, kWait));

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
    // Stop()'s worker may have been left detached by a loaded machine, so the
    // accepted events are awaited through the sink instead of being assumed
    // already written.
    CHECK(sink->WaitForEvents(accepted_ticks.size(), kWait));
    CHECK(sink->WaitForFlushes(1, kWait));
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
    CHECK(!logger.Started());
    CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "before start", 0)));
    CHECK(logger.Start());
    CHECK(!logger.Start());
    CHECK(logger.TryLog(MakeEvent(LogLevel::Info, "written", 0)));
    logger.Stop();
    CHECK(!logger.Started());
    CHECK(sink->WaitForEvents(1, kWait));
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
    // Both the logger and the caller release the sink while that worker is
    // still blocked. Only the logger's own reference keeps it alive now.
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
      // The sink is invisible to the test, so the two writes are awaited rather
      // than assumed to be finished before Stop()'s deadline.
      CHECK(WaitFor([&logger] { return logger.Written() == 2; }, kWait));
      CHECK(logger.ErrorCount() == 0);
    }

    const auto files = LogFiles(directory.path());
    CHECK(files.size() == 1);
    const auto lines = ReadLines(files.at(0));
    CHECK(lines.size() == 2);
    CHECK(lines[0] ==
          "WARNING 1234567 42 [fsm] tracking entered "
          "{event=3 from=\"damping\" to=\"tracking\"}");
    CHECK(lines[1] ==
          "ERROR 1043 43 [test] reason\\x09quoted {event=12 "
          "detail=\"a\\\\b\\\"c\"}");
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
      CHECK(WaitFor([&logger] { return logger.Written() == 60; }, kWait));
      CHECK(logger.ErrorCount() == 0);
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
    // The sink took every event and the worker flushed once before it exited;
    // on a machine slow enough to miss the drain deadline the bounded flush
    // wait covers the detached worker instead of assuming Stop() finished it.
    CHECK(sink->WaitForFlushes(1, kWait));
    CHECK(sink->event_count() == 4);
    CHECK(logger.Written() == 0);
    // Four failed writes plus the failed flush the worker performs on exit.
    CHECK(logger.ErrorCount() == 5);
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
    CHECK(WaitFor([&logger] { return logger.ErrorCount() == 1; }, kWait));
    CHECK(logger.Written() == 0);
    CHECK(!std::filesystem::exists(file / "logs"));
  }

  // The message carries the full 256-byte diagnostic text the design freezes
  // for fault and rejection reasons, and a longer value is refused without
  // touching the event: a caller copying a long reason never logs an empty
  // message.
  {
    const TemporaryDirectory directory("message-bound");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 8;
    LogEvent event = MakeEvent(LogLevel::Error, "placeholder", 7);
    // 253 printable bytes plus a backslash, a quote and a tab: 256 bytes.
    const std::string longest = std::string(253, 'm') + "\\\"\t";
    CHECK(longest.size() == 256);
    CHECK(event.message.Assign(longest));
    CHECK(event.message.View() == longest);
    const std::string too_long(257, 'm');
    CHECK(!event.message.Assign(too_long));
    CHECK(event.message.View() == longest);

    {
      Logger logger(config);
      CHECK(logger.Start());
      CHECK(logger.TryLog(event));
      logger.Stop();
      CHECK(WaitFor([&logger] { return logger.Written() == 1; }, kWait));
      CHECK(logger.ErrorCount() == 0);
    }

    const auto files = LogFiles(directory.path());
    CHECK(files.size() == 1);
    const auto lines = ReadLines(files.at(0));
    CHECK(lines.size() == 1);
    std::string expected = "ERROR 1007 7 [test] " + std::string(253, 'm');
    expected += "\\\\";   // the escaped backslash
    expected += "\\\"";   // the escaped quote
    expected += "\\x09";  // the escaped tab
    CHECK(lines[0] == expected);
  }

  // A producer thread that keeps logging while another thread stops the logger:
  // every event TryLog() accepted is either written or reported, so none of
  // them disappears silently, and nothing is accepted once Stop() has returned.
  {
    const TemporaryDirectory directory("stop-race");
    TelemetryConfig config = BaseConfig(directory.path());
    config.queue_capacity = 8;
    constexpr std::uint64_t kAttempts = 2048;
    for (int round = 0; round < 128; ++round) {
      std::shared_ptr<FastSink> sink = std::make_shared<FastSink>();
      Logger logger(config, sink);
      CHECK(logger.Start());
      std::atomic<std::uint64_t> attempts{0};
      std::atomic<std::uint64_t> accepted{0};
      std::mutex done_mutex;
      std::condition_variable done_cv;
      bool done = false;
      std::thread producer([&] {
        for (std::uint64_t tick = 0; tick < kAttempts; ++tick) {
          attempts.fetch_add(1, std::memory_order_relaxed);
          if (logger.TryLog(MakeEvent(LogLevel::Info, "race", tick))) {
            accepted.fetch_add(1, std::memory_order_relaxed);
          }
        }
        {
          std::lock_guard<std::mutex> lock(done_mutex);
          done = true;
        }
        done_cv.notify_all();
      });

      logger.Stop();
      // Stop() refuses every later event, so the producer's loop is bounded;
      // the wait only guards against a stalled thread.
      bool finished = false;
      {
        std::unique_lock<std::mutex> lock(done_mutex);
        finished = done_cv.wait_for(lock, kWait, [&] { return done; });
      }
      CHECK(finished);
      producer.join();

      const std::uint64_t attempted = attempts.load(std::memory_order_relaxed);
      const std::uint64_t granted = accepted.load(std::memory_order_relaxed);
      CHECK(attempted == kAttempts);
      // A stopped logger refuses a later event without counting it as a drop.
      const std::uint64_t dropped = logger.Dropped();
      CHECK(!logger.TryLog(MakeEvent(LogLevel::Info, "after stop", 0)));
      CHECK(logger.Dropped() == dropped);
      CHECK(logger.Filtered() == 0);
      CHECK(dropped <= attempted - granted);
      // Written is read before Undrained: the worker may still be writing
      // events that the stop already reported, so this order can only
      // over-count the two together, never under-count them.
      const std::uint64_t written = logger.Written();
      const std::uint64_t undrained = logger.Undrained();
      CHECK(written + undrained >= granted);
    }
  }

  // The diagnostic writer of the control thread, the trace worker and the exit
  // path is bounded: a pipe nobody drains must not park it, and a line that
  // does not fit must be reported as not written instead of waited for. The
  // log worker keeps the blocking behaviour of its own writer, pinned on the
  // same pipe.
  {
    int pipe_fds[2] = {-1, -1};
    CHECK(::pipe(pipe_fds) == 0);
    // The filling writer never blocks the test process itself.
    const int flags = ::fcntl(pipe_fds[1], F_GETFL, 0);
    CHECK(flags >= 0);
    CHECK(::fcntl(pipe_fds[1], F_SETFL, flags | O_NONBLOCK) == 0);
    std::array<char, 4096> filler{};
    std::size_t filled = 0;
    while (::write(pipe_fds[1], filler.data(), filler.size()) > 0) {
      filled += filler.size();
    }
    CHECK(filled > 0);

    // Full pipe: the bounded writer returns inside its budget and reports the
    // line as not written.
    const auto start = std::chrono::steady_clock::now();
    CHECK(!WriteDiagnosticBounded(pipe_fds[1], "dropped line\n"));
    CHECK(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));

    // Draining shows that the dropped line really did not arrive: the pipe
    // holds exactly what the filling writer put in it. The read is bounded by
    // that byte count - the write end is still open, so a read past it would
    // park the test.
    std::array<char, 4096> buffer{};
    std::size_t drained = 0;
    while (drained < filled) {
      const ssize_t count = ::read(pipe_fds[0], buffer.data(), buffer.size());
      CHECK(count > 0);
      drained += static_cast<std::size_t>(count);
    }
    CHECK(drained == filled);

    // With room again the whole line arrives, byte for byte.
    CHECK(WriteDiagnosticBounded(pipe_fds[1], "kept line\n"));
    const ssize_t kept = ::read(pipe_fds[0], buffer.data(), buffer.size());
    CHECK(kept == 10);
    CHECK(std::string_view(buffer.data(), 10) == "kept line\n");

    // A line above the bound is refused as a whole: the caller can tell that
    // its diagnostic was not delivered.
    const std::string too_long(kMaxBoundedDiagnosticBytes + 1, 'x');
    CHECK(!WriteDiagnosticBounded(pipe_fds[1], too_long));
    CHECK(::read(pipe_fds[0], buffer.data(), buffer.size()) ==
          static_cast<ssize_t>(kMaxBoundedDiagnosticBytes));

    // The log worker's writer still delivers arbitrary lengths, waiting for
    // room rather than dropping the line.
    const std::string worker_line(kMaxLogLine / 2, 'w');
    CHECK(WriteDiagnosticBlocking(pipe_fds[1], worker_line));
    std::string received;
    while (received.size() < worker_line.size()) {
      const ssize_t part = ::read(pipe_fds[0], buffer.data(), buffer.size());
      CHECK(part > 0);
      received.append(buffer.data(), static_cast<std::size_t>(part));
    }
    CHECK(received == worker_line);

    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
  }

  return 0;
}
