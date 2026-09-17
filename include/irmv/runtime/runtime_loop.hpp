#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/fake_backend.hpp"
#include "irmv/protocol/types.hpp"

// Standalone fake deployment: `robot_runtime` runs the guarded execution state
// machine against the fake hardware backend, with the bounded DDS transport and
// the asynchronous structured logger of this phase. No other backend exists in
// this phase, so the binary refuses every configuration that does not select
// the fake backend before it touches a device or DDS.
//
//   robot_runtime --config PATH [--domain N] [--fake-scenario PATH]
//                 [--output-trace PATH]
//
// `--config` is mandatory. `--domain` overrides the configured DDS domain and
// accepts the same inclusive 0..230 range the configuration parser enforces for
// `dds.domain`. `--fake-scenario` is only meaningful for the fake backend and
// injects scripted feedback and write acceptance into the backend; it never
// adds a protocol topic. `--output-trace` is the test-only per-tick output
// trace: one JSON-lines record per applied torque write, rendered off the
// control thread. It belongs to the fake deployment like every other option of
// this binary and is skipped entirely when it is absent.
//
// Startup order is fixed: parse the command line, load and validate the
// configuration, build the runtime identity, initialize the fake backend,
// construct the state machine and the logger, start the logger, then create and
// start the transport. Every failure before the loop reports on stderr and
// exits non-zero without starting DDS.
//
// The control loop samples a monotonic clock, drains the transport input, reads
// one feedback snapshot, ticks the state machine, publishes the tick result and
// then applies the fake scenario entries that became due. It sleeps against an
// absolute deadline that advances by the configured control period, so the
// scheduling phase does not drift, and it counts every tick whose work exceeded
// the period. The state machine itself reads no clock and never sleeps.

namespace irmv::runtime {

// Exit codes of the standalone runtime.
inline constexpr int kExitOk = 0;
// Invalid configuration, invalid scenario or a failure of the startup sequence.
inline constexpr int kExitStartupError = 1;
// Unknown or malformed command line argument.
inline constexpr int kExitUsageError = 2;

// Inclusive upper bound of the accepted DDS domain, the same range the
// configuration parser enforces for `dds.domain`.
inline constexpr std::uint32_t kMaxDdsDomain = 230;

// Host monotonic clock in microseconds. The value only ever moves forward and
// is the single time base of the control loop and of the fake scenario.
protocol::TimeUs MonotonicUs();

struct RuntimeOptions {
  std::filesystem::path config_path;
  std::optional<std::uint32_t> dds_domain;
  std::optional<std::filesystem::path> fake_scenario_path;
  // Test-only per-tick output trace. The file is created before the control
  // loop starts and is refused when it cannot be opened; nothing else about the
  // runtime changes when the option is absent.
  std::optional<std::filesystem::path> output_trace_path;
};

// Result of parsing argv. `error` is set and `ok` is false for every rejected
// command line; the caller prints it together with UsageLine().
struct CommandLine {
  bool ok{false};
  int exit_code{kExitUsageError};
  std::string error;
  RuntimeOptions options;
};

const char *UsageLine();

CommandLine ParseCommandLine(int argc, char **argv);

// ---------------------------------------------------------------------------
// Fake scenario
// ---------------------------------------------------------------------------
//
// The scenario is a JSON object with an `entries` array; an optional top level
// `$comment` string documents the file:
//
//   {
//     "$comment": "free text",
//     "entries": [
//       {
//         "elapsed_us": 0,
//         "paused": false,
//         "feedback": {
//           "count": 2,
//           "joints": [
//             {"q": 0.0, "dq": 0.0, "tau_est": 0.0, "valid": true,
//              "connected": true, "freq_hz": 500.0, "error_code": 0}
//           ],
//           "imu": {"valid": false, "rx_stamp_us": 0}
//         },
//         "write_result": {"accepted": true}
//       }
//     ]
//   }
//
// `elapsed_us` is monotonic from process start and mandatory; entries must be
// strictly increasing. An entry applies from its elapsed time onwards and holds
// until the next entry. `feedback.count` and every joint or IMU field are
// optional: what an entry leaves out keeps the value of the previous entry,
// ultimately the initial snapshot (every configured joint valid, connected and
// reporting the configured control frequency, all values zero, IMU invalid).
// `joints[i]` addresses joint `i` and must stay inside the effective count, so
// an entry that cannot be applied is a startup error with a field path.
//
// Every tick refreshes the receive stamp of the injected samples to the current
// monotonic time, which keeps the snapshot fresh for the configured feedback
// timeout; `imu.rx_stamp_us` is an offset from the current monotonic time for
// the IMU stamp only (0 keeps it fresh, a negative offset ages it). An entry
// with `paused: true` stops the refresh, so the held samples age and can lapse;
// the next entry without `paused` refreshes again.
//
// `write_result.accepted` drives FakeBackend::SetWriteAccepted and holds until
// the next entry that sets it. `write_result.error_code` is refused: the fake
// backend reports its own rejection code and exposes no way to set one.

// One optional per-joint override of a scenario entry.
struct ScenarioJointOverride {
  std::optional<double> q;
  std::optional<double> dq;
  std::optional<double> tau_est;
  std::optional<bool> valid;
  std::optional<bool> connected;
  std::optional<double> freq_hz;
  std::optional<std::uint32_t> error_code;
};

struct ScenarioImuOverride {
  std::optional<bool> valid;
  // Offset of the IMU receive stamp from the current monotonic time.
  std::optional<std::int64_t> rx_stamp_offset_us;
};

struct ScenarioFeedbackEntry {
  std::optional<std::uint32_t> count;
  std::vector<ScenarioJointOverride> joints;
  std::optional<ScenarioImuOverride> imu;
};

struct ScenarioEntry {
  protocol::TimeUs elapsed_us{0};
  bool paused{false};
  std::optional<ScenarioFeedbackEntry> feedback;
  std::optional<bool> write_accepted;
};

struct FakeScenario {
  std::vector<ScenarioEntry> entries;
};

// Invalid scenario file: `field_path()` names the offending field, "$" for the
// document itself. The message of the exception already carries the path.
class ScenarioError : public std::runtime_error {
 public:
  ScenarioError(std::string field_path, std::string message);

  const std::string &field_path() const noexcept { return field_path_; }

 private:
  std::string field_path_;
};

// Loads and validates a scenario against the configuration the runtime is about
// to use: the configured joint count is the count every entry starts from.
FakeScenario LoadFakeScenario(const std::filesystem::path &path,
                              const configuration::RuntimeConfig &config);

// Control-thread owner of the injected snapshot. ApplyDue() is the only writer
// of the fake backend's feedback and write acceptance, so no other thread ever
// touches them; it allocates nothing.
class ScenarioApplier {
 public:
  ScenarioApplier(FakeScenario scenario,
                  const configuration::RuntimeConfig &config);

  // Applies every entry whose elapsed time has been reached, refreshes the
  // receive stamps of the samples that are not paused, injects the resulting
  // snapshot into the backend and applies the write acceptance of the entries
  // that became due. `elapsed_us` is the time since process start, `now_us` the
  // current monotonic time. Safe to call with a monotonically non-decreasing
  // elapsed time; an earlier value applies nothing.
  void ApplyDue(protocol::TimeUs elapsed_us, protocol::TimeUs now_us,
                hardware::FakeBackend &backend);

  const protocol::FeedbackSnapshot &snapshot() const { return snapshot_; }

  bool paused() const { return paused_; }

  std::size_t applied_entries() const { return applied_entries_; }

  std::size_t entry_count() const { return scenario_.entries.size(); }

 private:
  void ApplyFeedback(const ScenarioFeedbackEntry &entry);
  void RefreshStamps(protocol::TimeUs now_us);

  FakeScenario scenario_;
  protocol::FeedbackSnapshot snapshot_;
  std::int64_t imu_stamp_offset_us_{0};
  bool paused_{false};
  std::size_t applied_entries_{0};
};

// Runs the standalone fake runtime described by `options` until a SIGINT or
// SIGTERM asks it to stop. Returns the process exit code: kExitOk only after a
// clean stop, that is only from ZeroTorque with an accepted torque write.
int RunStandaloneRuntime(const RuntimeOptions &options);

}  // namespace irmv::runtime
