#include "irmv/configuration/config.hpp"

#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <unordered_set>

#include "irmv/configuration/sha256.hpp"
#include "parse_helpers.hpp"

namespace irmv::configuration {
namespace {

using Json = detail::Json;
using detail::BoolMember;
using detail::Fail;
using detail::FiniteMember;
using detail::IntegerInRange;
using detail::JointValueRule;
using detail::Member;
using detail::ObjectMember;
using detail::PositiveInteger;
using detail::PositiveIntegerMember;
using detail::ReadJointMap;
using detail::StringMember;
using detail::ValidateJointName;

// The level names live in one place: parsing accepts exactly the tokens
// LogLevelName() produces.
LogLevel ParseLogLevel(const std::string &level) {
  for (const LogLevel candidate :
       {LogLevel::Trace, LogLevel::Debug, LogLevel::Info, LogLevel::Warning,
        LogLevel::Error, LogLevel::Fatal}) {
    if (level == LogLevelName(candidate)) {
      return candidate;
    }
  }
  Fail("telemetry.level",
       "must be TRACE, DEBUG, INFO, WARNING, ERROR, or FATAL");
}

Json JointMap(const std::vector<std::string> &names, const JointArray &values) {
  Json mapping = Json::object();
  for (std::size_t index = 0; index < names.size(); ++index) {
    mapping[names[index]] = values[index];
  }
  return mapping;
}

Json CanonicalConfig(const RuntimeConfig &config) {
  Json joints = Json::array();
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    joints.push_back({{"name", config.joint_names[index]},
                      {"position_min", config.position_min[index]},
                      {"position_max", config.position_max[index]}});
  }
  return {
      {"schema_version", config.schema_version},
      {"deployment", config.deployment},
      {"backend", config.backend},
      {"robot",
       {{"units",
         {{"position", "rad"}, {"velocity", "rad_s"}, {"torque", "Nm"}}},
        {"joints", joints}}},
      {"runtime",
       {{"control_hz", config.control_hz},
        {"damping", {{"kd", JointMap(config.joint_names, config.damping_kd)}}},
        {"safety",
         {{"joint_velocity_limits",
           JointMap(config.joint_names, config.joint_velocity_limits)},
          {"torque_limits", JointMap(config.joint_names, config.torque_limits)},
          {"target_velocity_limits",
           JointMap(config.joint_names, config.target_velocity_limits)},
          {"timeouts",
           {{"command_us", config.timeouts.command_us},
            {"source_state_us", config.timeouts.source_state_us},
            {"feedback_us", config.timeouts.feedback_us},
            {"imu_us", config.timeouts.imu_us},
            {"supervisor_us", config.timeouts.supervisor_us}}}}},
        {"boot",
         {{"frequency_grace_us", config.boot.frequency_grace_us},
          {"healthy_window_us", config.boot.healthy_window_us},
          {"deadline_us", config.boot.deadline_us},
          {"minimum_frequency_hz", config.boot.minimum_frequency_hz}}},
        {"recovery",
         {{"healthy_window_us", config.recovery.healthy_window_us},
          {"minimum_frequency_hz", config.recovery.minimum_frequency_hz}}},
        {"imu_required", config.imu_required},
        {"continuity",
         {{"position_delta",
           JointMap(config.joint_names, config.continuity.position_delta)},
          {"velocity_delta",
           JointMap(config.joint_names, config.continuity.velocity_delta)},
          {"torque_delta",
           JointMap(config.joint_names, config.continuity.torque_delta)}}},
        {"state_window_capacity", config.state_window_capacity}}},
      {"telemetry",
       {{"level", LogLevelName(config.telemetry.level)},
        {"directory", config.telemetry.directory.string()},
        {"rotate_bytes", config.telemetry.rotate_bytes},
        {"retained_files", config.telemetry.retained_files},
        {"queue_capacity", config.telemetry.queue_capacity}}},
      {"dds",
       {{"domain", config.dds.domain}, {"interface", config.dds.interface}}}};
}

}  // namespace

const char *LogLevelName(LogLevel level) noexcept {
  switch (level) {
    case LogLevel::Trace:
      return "TRACE";
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warning:
      return "WARNING";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Fatal:
      return "FATAL";
  }
  return "UNKNOWN";
}

ConfigError::ConfigError(std::string field_path, std::string message)
    : std::runtime_error(field_path + ": " + message),
      field_path_(std::move(field_path)) {}

RuntimeConfig LoadConfig(const std::filesystem::path &path) {
  return LoadConfig(path, ConfigOverrides{});
}

RuntimeConfig LoadConfig(const std::filesystem::path &path,
                         const ConfigOverrides &overrides) {
  const Json document = detail::LoadDocument(path);

  RuntimeConfig config;
  const std::uint64_t schema_version =
      PositiveInteger(Member(document, "schema_version", ""), "schema_version",
                      std::numeric_limits<std::uint32_t>::max());
  if (schema_version != 1U) {
    Fail("schema_version", "must equal 1");
  }
  config.schema_version = static_cast<std::uint32_t>(schema_version);
  config.deployment = StringMember(document, "deployment", "");
  if (config.deployment != "fake_only") {
    Fail("deployment", "must equal fake_only");
  }
  config.backend = StringMember(document, "backend", "");
  if (config.backend != "fake") {
    Fail("backend", "only the fake backend is supported");
  }

  const Json &robot = ObjectMember(document, "robot", "");
  const Json &units = ObjectMember(robot, "units", "robot");
  if (StringMember(units, "position", "robot.units") != "rad") {
    Fail("robot.units.position", "must equal rad");
  }
  if (StringMember(units, "velocity", "robot.units") != "rad_s") {
    Fail("robot.units.velocity", "must equal rad_s");
  }
  if (StringMember(units, "torque", "robot.units") != "Nm") {
    Fail("robot.units.torque", "must equal Nm");
  }
  const Json &joints = Member(robot, "joints", "robot");
  if (!joints.is_array() || joints.empty() ||
      joints.size() > protocol::kMaxJoints) {
    Fail("robot.joints", "must contain between 1 and 64 joints");
  }
  std::unordered_set<std::string> seen_names;
  config.joint_names.reserve(joints.size());
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const std::string joint_path =
        "robot.joints[" + std::to_string(index) + "]";
    const Json &joint = joints[index];
    if (!joint.is_object()) {
      Fail(joint_path, "must be an object");
    }
    std::string name = StringMember(joint, "name", joint_path);
    ValidateJointName(name, joint_path + ".name");
    if (!seen_names.insert(name).second) {
      Fail(joint_path + ".name", "must be unique");
    }
    config.joint_names.push_back(std::move(name));
    config.position_min[index] =
        FiniteMember(joint, "position_min", joint_path);
    config.position_max[index] =
        FiniteMember(joint, "position_max", joint_path);
    if (config.position_min[index] >= config.position_max[index]) {
      Fail(joint_path + ".position_min", "must be less than position_max");
    }
  }

  const Json &runtime = ObjectMember(document, "runtime", "");
  config.control_hz = FiniteMember(runtime, "control_hz", "runtime");
  if (config.control_hz <= 0.0 || config.control_hz > 1'000'000.0) {
    Fail("runtime.control_hz", "must be in (0, 1000000]");
  }
  const double control_period_us = 1'000'000.0 / config.control_hz;
  if (!std::isfinite(control_period_us) ||
      control_period_us >= std::ldexp(1.0, 64)) {
    Fail("runtime.control_hz", "produces an out-of-range control period");
  }
  config.control_period_us =
      static_cast<protocol::TimeUs>(std::round(control_period_us));
  if (config.control_period_us == 0U) {
    Fail("runtime.control_hz", "produces a zero control period");
  }

  const Json &damping = ObjectMember(runtime, "damping", "runtime");
  ReadJointMap(Member(damping, "kd", "runtime.damping"), "runtime.damping.kd",
               config.joint_names, JointValueRule::Positive, config.damping_kd);
  const Json &safety = ObjectMember(runtime, "safety", "runtime");
  ReadJointMap(Member(safety, "joint_velocity_limits", "runtime.safety"),
               "runtime.safety.joint_velocity_limits", config.joint_names,
               JointValueRule::Positive, config.joint_velocity_limits);
  ReadJointMap(Member(safety, "torque_limits", "runtime.safety"),
               "runtime.safety.torque_limits", config.joint_names,
               JointValueRule::Positive, config.torque_limits);
  ReadJointMap(Member(safety, "target_velocity_limits", "runtime.safety"),
               "runtime.safety.target_velocity_limits", config.joint_names,
               JointValueRule::Positive, config.target_velocity_limits);

  const Json &timeouts = ObjectMember(safety, "timeouts", "runtime.safety");
  config.timeouts.command_us =
      PositiveIntegerMember(timeouts, "command_us", "runtime.safety.timeouts");
  config.timeouts.source_state_us = PositiveIntegerMember(
      timeouts, "source_state_us", "runtime.safety.timeouts");
  config.timeouts.feedback_us =
      PositiveIntegerMember(timeouts, "feedback_us", "runtime.safety.timeouts");
  config.timeouts.imu_us =
      PositiveIntegerMember(timeouts, "imu_us", "runtime.safety.timeouts");
  config.timeouts.supervisor_us = PositiveIntegerMember(
      timeouts, "supervisor_us", "runtime.safety.timeouts");

  const Json &boot = ObjectMember(runtime, "boot", "runtime");
  config.boot.frequency_grace_us =
      PositiveIntegerMember(boot, "frequency_grace_us", "runtime.boot");
  config.boot.healthy_window_us =
      PositiveIntegerMember(boot, "healthy_window_us", "runtime.boot");
  config.boot.deadline_us =
      PositiveIntegerMember(boot, "deadline_us", "runtime.boot");
  config.boot.minimum_frequency_hz =
      FiniteMember(boot, "minimum_frequency_hz", "runtime.boot");
  if (config.boot.minimum_frequency_hz <= 0.0 ||
      config.boot.minimum_frequency_hz > config.control_hz) {
    Fail("runtime.boot.minimum_frequency_hz",
         "must be positive and no greater than control_hz");
  }
  if (config.boot.frequency_grace_us >
          std::numeric_limits<protocol::TimeUs>::max() -
              config.boot.healthy_window_us ||
      config.boot.deadline_us <=
          config.boot.frequency_grace_us + config.boot.healthy_window_us) {
    Fail("runtime.boot.deadline_us",
         "must exceed frequency_grace_us plus healthy_window_us");
  }

  const Json &recovery = ObjectMember(runtime, "recovery", "runtime");
  config.recovery.healthy_window_us =
      PositiveIntegerMember(recovery, "healthy_window_us", "runtime.recovery");
  config.recovery.minimum_frequency_hz =
      FiniteMember(recovery, "minimum_frequency_hz", "runtime.recovery");
  if (config.recovery.minimum_frequency_hz <= 0.0 ||
      config.recovery.minimum_frequency_hz > config.control_hz) {
    Fail("runtime.recovery.minimum_frequency_hz",
         "must be positive and no greater than control_hz");
  }
  config.imu_required = BoolMember(runtime, "imu_required", "runtime");

  const Json &continuity = ObjectMember(runtime, "continuity", "runtime");
  ReadJointMap(Member(continuity, "position_delta", "runtime.continuity"),
               "runtime.continuity.position_delta", config.joint_names,
               JointValueRule::NonNegative, config.continuity.position_delta);
  ReadJointMap(Member(continuity, "velocity_delta", "runtime.continuity"),
               "runtime.continuity.velocity_delta", config.joint_names,
               JointValueRule::NonNegative, config.continuity.velocity_delta);
  ReadJointMap(Member(continuity, "torque_delta", "runtime.continuity"),
               "runtime.continuity.torque_delta", config.joint_names,
               JointValueRule::NonNegative, config.continuity.torque_delta);
  config.state_window_capacity = static_cast<std::size_t>(PositiveIntegerMember(
      runtime, "state_window_capacity", "runtime", 8192U));
  const auto required_samples =
      config.timeouts.source_state_us / config.control_period_us +
      (config.timeouts.source_state_us % config.control_period_us != 0U ? 1U
                                                                        : 0U);
  if (config.state_window_capacity < required_samples) {
    Fail("runtime.state_window_capacity",
         "must cover the source-state timeout at control_hz");
  }

  const Json &telemetry = ObjectMember(document, "telemetry", "");
  config.telemetry.level =
      ParseLogLevel(StringMember(telemetry, "level", "telemetry"));
  config.telemetry.directory =
      StringMember(telemetry, "directory", "telemetry");
  if (config.telemetry.directory.empty()) {
    Fail("telemetry.directory", "must not be empty");
  }
  config.telemetry.rotate_bytes =
      PositiveIntegerMember(telemetry, "rotate_bytes", "telemetry");
  config.telemetry.retained_files = static_cast<std::uint32_t>(
      PositiveIntegerMember(telemetry, "retained_files", "telemetry",
                            std::numeric_limits<std::uint32_t>::max()));
  config.telemetry.queue_capacity = static_cast<std::size_t>(
      PositiveIntegerMember(telemetry, "queue_capacity", "telemetry",
                            std::numeric_limits<std::uint32_t>::max()));

  const Json &dds = ObjectMember(document, "dds", "");
  const Json &domain = Member(dds, "domain", "dds");
  config.dds.domain = static_cast<std::uint32_t>(
      IntegerInRange(domain, "dds.domain", 0U, kMaxDdsDomain));
  config.dds.interface = StringMember(dds, "interface", "dds");
  if (config.dds.interface.empty()) {
    Fail("dds.interface", "must not be empty");
  }
  if (overrides.dds_domain) {
    // An override is held to the same bound as the file, and it is applied here
    // so that the digest below covers the configuration in effect instead of
    // the one on disk.
    if (*overrides.dds_domain > kMaxDdsDomain) {
      Fail("dds.domain", "is outside the supported range");
    }
    config.dds.domain = *overrides.dds_domain;
  }

  const Json canonical = CanonicalConfig(config);
  config.config_id = Sha256(canonical.dump());
  config.layout_id = Sha256(Json(config.joint_names).dump());
  return config;
}

}  // namespace irmv::configuration
