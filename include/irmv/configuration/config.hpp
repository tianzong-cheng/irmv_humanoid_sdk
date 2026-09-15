#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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

enum class LogLevel {
  Debug,
  Info,
  Warning,
  Error,
};

struct TelemetryConfig {
  LogLevel level{LogLevel::Info};
  std::filesystem::path directory;
  std::uint64_t rotate_bytes{0};
  std::uint32_t retained_files{0};
  std::size_t queue_capacity{0};
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

RuntimeConfig LoadConfig(const std::filesystem::path &path);

}  // namespace irmv::configuration
