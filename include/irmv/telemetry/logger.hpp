#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string_view>
#include <system_error>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::telemetry {

// Structured fields one event may carry. The array is part of the event, so
// building and enqueueing an event never allocates.
inline constexpr std::size_t kMaxLogFields = 8;

// Longest line the default sink builds, on the worker stack. It fits any event
// that respects the fixed string capacities, including the worst case that
// escapes every byte of a 256-byte message and eight fields into four bytes.
inline constexpr std::size_t kMaxLogLine = 16 * 1024;

// Bounded time Stop() gives the worker to drain the queue before it reports
// what is left instead of claiming the run was logged completely.
inline constexpr std::uint32_t kStopDrainTimeoutMs = 500;

// One key/value pair of an event, for example the states of a transition or the
// reason of a rejection. Both parts are fixed capacity: a longer value is
// refused by Assign() instead of being allocated.
struct LogField {
  protocol::Text64 name;
  protocol::Text64 value;
};

// One log record. Fixed capacity and trivially copyable: it moves through the
// logger queue by copy and every string it holds is bounded.
struct LogEvent {
  configuration::LogLevel level{configuration::LogLevel::Info};
  protocol::TimeUs stamp_us{0};
  protocol::Sequence tick{0};
  protocol::Text64 component;
  protocol::Text64 message;
  std::uint32_t event_code{0};
  std::uint32_t field_count{0};
  std::array<LogField, kMaxLogFields> fields{};
};

// Consumer of structured events. Every call runs on the logger worker thread:
// Write() may block for as long as the backing device does, which delays the
// worker and never the producer, and Flush() is called once when the worker
// stops. Both return false to report a failure; a sink must not throw.
class LogSink {
 public:
  virtual ~LogSink() = default;

  virtual bool Write(const LogEvent &event) = 0;
  virtual bool Flush() = 0;
};

// Default sink: one line per event, both on stderr and in a rotating file.
//
//   LEVEL stamp_us tick [component] message {event=<code> name="value" ...}
//
// The structured tail appears only when the event carries an event code or
// fields; field values are quoted and control bytes, backslashes and quotes are
// escaped so that one event is always exactly one line. Files live in
// <directory>/run_<UTC timestamp>/humanoid.log. The file is created, rotated and
// flushed only inside Write()/Flush(), that is only on the worker thread, and a
// failing file system turns into a false return plus a one-time stderr
// diagnostic instead of an exception.
class RotatingFileSink final : public LogSink {
 public:
  explicit RotatingFileSink(const configuration::TelemetryConfig &config);

  RotatingFileSink(const RotatingFileSink &) = delete;
  RotatingFileSink &operator=(const RotatingFileSink &) = delete;

  bool Write(const LogEvent &event) override;
  bool Flush() override;

 private:
  bool OpenIfNeeded();
  bool RotateIfNeeded(std::size_t line_bytes);
  bool AppendLine(const char *line, std::size_t length);
  bool ReportError(std::string_view what, std::error_code error);

  std::filesystem::path directory_;
  std::filesystem::path path_;
  std::ofstream file_;
  std::uint64_t rotate_bytes_{0};
  std::uint32_t retained_files_{0};
  std::uint64_t size_{0};
  // Throttles the stderr diagnostic to one message per failure streak.
  bool error_reported_{false};
};

// Bounded asynchronous logger. The producing thread (the control loop) hands a
// fully built event to a fixed-capacity queue and returns immediately: TryLog
// never waits, allocates, formats or touches a file. One worker thread owns
// everything else - formatting, the sink, rotation - and reads events in order.
//
// The queue is bounded by configuration::TelemetryConfig::queue_capacity, so a
// slow or blocked consumer cannot grow memory or delay the producer: an event
// that does not fit is counted in Dropped() instead. Events below the configured
// level are counted in Filtered() and never reach the queue. The logger reports
// what it did not write, so a suffix of a run is never mistaken for a complete
// log.
//
// Construction allocates the queue storage once; it performs no file I/O. The
// injected sink is caller-owned and must outlive the logger, including the case
// where Stop() leaves a blocked worker behind (see Stop()).
class Logger {
 public:
  explicit Logger(const configuration::TelemetryConfig &config);

  // `sink` may be null to select the built-in RotatingFileSink.
  Logger(const configuration::TelemetryConfig &config, LogSink *sink);
  ~Logger();

  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;

  // Starts the worker. Returns false when the logger was already started: one
  // Start() per logger lifetime.
  bool Start();

  // Idempotent. Asks the worker to drain the queue within kStopDrainTimeoutMs
  // and joins it. A sink blocked inside Write() cannot be waited for forever, so
  // past that deadline the worker is left detached to finish on its own and the
  // events that were still queued are reported by Undrained(). No event is
  // accepted after Stop() returns.
  void Stop();

  // Producer side, non-blocking. Returns false without waiting when the logger
  // is not running, when the event is below the configured level (Filtered()) or
  // when the queue is full (Dropped()).
  bool TryLog(const LogEvent &event);

  // Events the producer refused because the queue was full. Producer side.
  std::uint64_t Dropped() const;

  // Events the producer refused because they were below the configured level.
  // Producer side.
  std::uint64_t Filtered() const;

  // Events the worker handed to the sink. Worker side.
  std::uint64_t Written() const;

  // Events that were still queued when the Stop() deadline expired.
  std::uint64_t Undrained() const;

  // Sink Write()/Flush() calls that failed. Worker side.
  std::uint64_t error_count() const;

  // True between a successful Start() and Stop().
  bool started() const;

 private:
  struct Shared;

  static void WorkerMain(std::shared_ptr<Shared> shared);

  std::shared_ptr<Shared> shared_;
};

}  // namespace irmv::telemetry
