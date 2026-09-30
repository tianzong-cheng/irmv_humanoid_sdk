#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "irmv/protocol/types.hpp"

namespace irmv::configuration {

class ConfigError : public std::runtime_error {
 public:
  ConfigError(std::string field_path, std::string message);

  const std::string &field_path() const noexcept { return field_path_; }

 private:
  std::string field_path_;
};

using JointArray = std::array<double, protocol::kMaxJoints>;

struct TimeoutConfig {
  protocol::TimeUs command_us{0};
  protocol::TimeUs source_state_us{0};
  protocol::TimeUs feedback_us{0};
  protocol::TimeUs imu_us{0};
  protocol::TimeUs supervisor_us{0};
};

struct BootConfig {
  protocol::TimeUs frequency_grace_us{0};
  protocol::TimeUs healthy_window_us{0};
  protocol::TimeUs deadline_us{0};
  double minimum_frequency_hz{0.0};
};

struct RecoveryConfig {
  protocol::TimeUs healthy_window_us{0};
  double minimum_frequency_hz{0.0};
};

struct ContinuityConfig {
  JointArray position_delta{};
  JointArray velocity_delta{};
  JointArray torque_delta{};
};

// Severity of a log event, declared from the most verbose to the most severe:
// an event is written when its level is at least the configured one.
enum class LogLevel {
  Trace,
  Debug,
  Info,
  Warning,
  Error,
  Fatal,
};

// Canonical upper-case token of a level: the spelling the JSON parser accepts
// and the logger prints. Returns "UNKNOWN" for a value outside the enum.
const char *LogLevelName(LogLevel level) noexcept;

// The run recorder's own configuration. It is separate from the logger's: the
// two write different things at different rates into the same tree, and a
// deployment may well want one without the other.
struct RecorderConfig {
  // Whether this run records at all. A disabled recorder opens no file and
  // accepts no sample.
  bool enabled{false};
  // The sampling rate of the recorder, independent of the control rate: the
  // runtime offers every control tick, and the recorder is asked for the ones
  // that fall on this period. 100 Hz is the shipped default; the contract
  // allows the full control rate as well.
  std::uint32_t sample_hz{100};
  // Capacity of the handoff between the control thread and the writer. At the
  // sampling rate this bounds how long a slow disk may fall behind before
  // samples are dropped - and each drop is counted and reported as a gap.
  std::size_t queue_capacity{256};
  // Where one directory per run is created. It must exist; the recorder never
  // creates a path it was not told to use.
  std::filesystem::path directory;
};

struct TelemetryConfig {
  LogLevel level{LogLevel::Info};
  std::filesystem::path directory;
  std::uint64_t rotate_bytes{0};
  std::uint32_t retained_files{0};
  std::size_t queue_capacity{0};
  RecorderConfig recorder;
};

struct DdsConfig {
  std::uint32_t domain{0};
  std::string interface;
};

struct RuntimeConfig {
  std::uint32_t schema_version{0};
  std::string deployment;
  std::string backend;

  std::vector<std::string> joint_names;
  JointArray position_min{};
  JointArray position_max{};

  double control_hz{0.0};
  protocol::TimeUs control_period_us{0};
  JointArray damping_kd{};
  JointArray joint_velocity_limits{};
  JointArray torque_limits{};
  JointArray target_velocity_limits{};
  TimeoutConfig timeouts;
  BootConfig boot;
  RecoveryConfig recovery;
  bool imu_required{false};
  ContinuityConfig continuity;
  std::size_t state_window_capacity{0};

  TelemetryConfig telemetry;
  DdsConfig dds;
  std::string config_id;
  std::string layout_id;

  std::size_t joint_count() const noexcept { return joint_names.size(); }
};

// Inclusive range of the DDS domain id this build accepts. The bound is a
// deployment property, so it has exactly one definition: the parser enforces it
// for `dds.domain` and every command line that can override the domain uses the
// same value.
inline constexpr std::uint32_t kMaxDdsDomain = 230;

// Command line overrides of a configuration file. They are applied to the
// parsed configuration before its identity digest is computed, so `config_id`
// always describes the configuration that is actually in effect.
struct ConfigOverrides {
  std::optional<std::uint32_t> dds_domain;
};

RuntimeConfig LoadConfig(const std::filesystem::path &path);

RuntimeConfig LoadConfig(const std::filesystem::path &path,
                         const ConfigOverrides &overrides);

}  // namespace irmv::configuration
