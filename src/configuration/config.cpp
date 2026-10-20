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
    Json joint = {{"name", config.joint_names[index]},
                  {"position_min", config.position_min[index]},
                  {"position_max", config.position_max[index]}};
    if (config.hardware && index < config.hardware->joints.size()) {
      const HardwareJointConfig &mapping = config.hardware->joints[index];
      joint["bus"] = mapping.bus;
      joint["motor_id"] = mapping.motor_id;
      joint["direction"] = mapping.direction;
      joint["reduction_ratio"] = mapping.reduction_ratio;
      joint["encoder_offset"] = mapping.encoder_offset;
      joint["torque_constant"] = mapping.torque_constant;
    }
    joints.push_back(std::move(joint));
  }
  Json robot = {
      {"units", {{"position", "rad"}, {"velocity", "rad_s"}, {"torque", "Nm"}}},
      {"joints", std::move(joints)}};
  if (config.hardware) {
    Json buses = Json::array();
    for (const BusConfig &bus : config.hardware->buses) {
      buses.push_back({{"name", bus.name},
                       {"device_type", bus.device_type},
                       {"device_index", bus.device_index},
                       {"channel", bus.channel},
                       {"arbitration_bps", bus.arbitration_bps},
                       {"data_bps", bus.data_bps},
                       {"iso_mode", bus.iso_mode},
                       {"resistance_enabled", bus.resistance_enabled}});
    }
    robot["buses"] = std::move(buses);
    if (config.hardware->imu) {
      robot["imu"] = {{"hardware_id", config.hardware->imu->hardware_id},
                      {"baudrate", config.hardware->imu->baudrate}};
    }
  }
  return {
      {"schema_version", config.schema_version},
      {"deployment", config.deployment},
      {"backend", config.backend},
      {"robot", std::move(robot)},
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
        {"queue_capacity", config.telemetry.queue_capacity},
        {"recorder",
         {{"enabled", config.telemetry.recorder.enabled},
          {"sample_hz", config.telemetry.recorder.sample_hz},
          {"queue_capacity", config.telemetry.recorder.queue_capacity},
          {"directory", config.telemetry.recorder.directory.string()}}}}},
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
  const bool real_deployment = config.deployment == "real";
  if (!real_deployment && config.deployment != "fake_only") {
    Fail("deployment", "must equal fake_only or real");
  }
  config.backend = StringMember(document, "backend", "");
  if (config.backend != (real_deployment ? "real" : "fake")) {
    Fail("backend", real_deployment
                        ? "must equal real when deployment is real"
                        : "must equal fake when deployment is fake_only");
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

  // A real deployment names its hardware explicitly. The tables are parsed here
  // with the joint order they belong to, so the runtime and the tools consume
  // one definition of the machine instead of two.
  std::optional<HardwareConfig> hardware;
  std::unordered_set<std::string> seen_bus_names;
  std::unordered_set<std::string> seen_endpoints;
  const auto buses_member = robot.find("buses");
  if (real_deployment && buses_member == robot.end()) {
    Fail("robot.buses", "is required for a real deployment");
  }
  if (buses_member != robot.end()) {
    if (!buses_member->is_array() || buses_member->empty()) {
      Fail("robot.buses", "must be a non-empty array");
    }
    hardware.emplace();
    for (std::size_t index = 0; index < buses_member->size(); ++index) {
      const std::string path = "robot.buses[" + std::to_string(index) + "]";
      const Json &bus = (*buses_member)[index];
      if (!bus.is_object()) {
        Fail(path, "must be an object");
      }
      BusConfig parsed;
      parsed.name = StringMember(bus, "name", path);
      if (parsed.name.empty() || parsed.name.size() > protocol::kMaxNameBytes) {
        Fail(path + ".name", "must contain 1 to 64 bytes");
      }
      if (!seen_bus_names.insert(parsed.name).second) {
        Fail(path + ".name", "must be unique");
      }
      parsed.device_type = static_cast<std::uint32_t>(PositiveInteger(
          Member(bus, "device_type", path), path + ".device_type",
          std::numeric_limits<std::uint32_t>::max()));
      parsed.device_index = static_cast<std::uint32_t>(IntegerInRange(
          Member(bus, "device_index", path), path + ".device_index", 0,
          std::numeric_limits<std::uint32_t>::max()));
      parsed.channel = static_cast<std::uint32_t>(IntegerInRange(
          Member(bus, "channel", path), path + ".channel", 0, 1));
      parsed.arbitration_bps = static_cast<std::uint32_t>(PositiveInteger(
          Member(bus, "arbitration_bps", path), path + ".arbitration_bps",
          std::numeric_limits<std::uint32_t>::max()));
      parsed.data_bps = static_cast<std::uint32_t>(
          PositiveInteger(Member(bus, "data_bps", path), path + ".data_bps",
                          std::numeric_limits<std::uint32_t>::max()));
      parsed.iso_mode = BoolMember(bus, "iso_mode", path);
      parsed.resistance_enabled = BoolMember(bus, "resistance_enabled", path);
      const std::string endpoint = std::to_string(parsed.device_type) + ":" +
                                   std::to_string(parsed.device_index) + ":" +
                                   std::to_string(parsed.channel);
      if (!seen_endpoints.insert(endpoint).second) {
        Fail(path, "duplicates the device/channel of another bus");
      }
      hardware->buses.push_back(std::move(parsed));
    }
  }
  std::unordered_set<std::string> seen_motor_addresses;
  std::vector<HardwareJointConfig> joint_mappings;
  bool joint_hardware_present = false;
  joint_mappings.reserve(joints.size());
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

    const bool require_mapping = real_deployment || buses_member != robot.end();
    static constexpr const char *kMappingFields[] = {"bus",
                                                     "motor_id",
                                                     "direction",
                                                     "reduction_ratio",
                                                     "encoder_offset",
                                                     "torque_constant"};
    bool has_mapping = false;
    for (const char *field : kMappingFields) {
      has_mapping = has_mapping || joint.contains(field);
    }
    if (require_mapping && !has_mapping) {
      Fail(joint_path + ".bus", "is required for a real hardware mapping");
    }
    if (has_mapping) {
      joint_hardware_present = true;
      HardwareJointConfig mapping;
      mapping.name = config.joint_names.back();
      mapping.bus = StringMember(joint, "bus", joint_path);
      if (mapping.bus.empty()) {
        Fail(joint_path + ".bus", "must not be empty");
      }
      mapping.motor_id = static_cast<std::uint32_t>(
          IntegerInRange(Member(joint, "motor_id", joint_path),
                         joint_path + ".motor_id", 1, 31));
      const double direction = FiniteMember(joint, "direction", joint_path);
      if (direction != 1.0 && direction != -1.0) {
        Fail(joint_path + ".direction", "must equal -1 or 1");
      }
      mapping.direction = direction > 0.0 ? 1 : -1;
      mapping.reduction_ratio =
          FiniteMember(joint, "reduction_ratio", joint_path);
      if (mapping.reduction_ratio <= 0.0) {
        Fail(joint_path + ".reduction_ratio", "must be positive");
      }
      mapping.encoder_offset =
          FiniteMember(joint, "encoder_offset", joint_path);
      mapping.torque_constant =
          FiniteMember(joint, "torque_constant", joint_path);
      if (mapping.torque_constant <= 0.0) {
        Fail(joint_path + ".torque_constant", "must be positive");
      }
      if (!seen_bus_names.count(mapping.bus)) {
        Fail(joint_path + ".bus", "does not name a configured bus");
      }
      const std::string address =
          mapping.bus + ":" + std::to_string(mapping.motor_id);
      if (!seen_motor_addresses.insert(address).second) {
        Fail(joint_path + ".motor_id",
             "duplicates another joint on the same bus");
      }
      joint_mappings.push_back(std::move(mapping));
    }
  }

  if (buses_member != robot.end() || joint_hardware_present) {
    if (!hardware) {
      hardware.emplace();
    }
    hardware->joints = std::move(joint_mappings);
  }

  const auto imu_member = robot.find("imu");
  if (imu_member != robot.end()) {
    if (!imu_member->is_object()) {
      Fail("robot.imu", "must be an object");
    }
    if (!hardware) {
      hardware.emplace();
    }
    ImuConfig imu;
    imu.hardware_id = StringMember(*imu_member, "hardware_id", "robot.imu");
    if (imu.hardware_id.empty() || imu.hardware_id.size() > 128) {
      Fail("robot.imu.hardware_id", "must contain 1 to 128 bytes");
    }
    imu.baudrate = static_cast<std::uint32_t>(PositiveInteger(
        Member(*imu_member, "baudrate", "robot.imu"), "robot.imu.baudrate",
        std::numeric_limits<std::uint32_t>::max()));
    hardware->imu = std::move(imu);
  }
  config.hardware = std::move(hardware);

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
  if (real_deployment && config.imu_required &&
      (!config.hardware || !config.hardware->imu)) {
    Fail("robot.imu", "is required when runtime.imu_required is true");
  }

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

  // The recorder's own section. Its directory defaults to the log directory of
  // the same document, because a deployment that keeps one keeps the other next
  // to it; an explicit value overrides that.
  // The section is optional: a document that does not carry it describes a
  // deployment that does not record, with the sample rate, queue capacity and
  // directory the schema states as its defaults, and an explicit
  // `{"enabled": false}` means the same thing.
  config.telemetry.recorder.directory = config.telemetry.directory;
  const auto recorder_entry = telemetry.find("recorder");
  if (recorder_entry != telemetry.end()) {
    const Json &recorder = *recorder_entry;
    const Json &recorder_enabled =
        Member(recorder, "enabled", "telemetry.recorder");
    if (!recorder_enabled.is_boolean()) {
      Fail("telemetry.recorder.enabled", "must be a boolean");
    }
    config.telemetry.recorder.enabled = recorder_enabled.get<bool>();
    config.telemetry.recorder.sample_hz = static_cast<std::uint32_t>(
        PositiveIntegerMember(recorder, "sample_hz", "telemetry.recorder",
                              std::numeric_limits<std::uint32_t>::max()));
    if (static_cast<double>(config.telemetry.recorder.sample_hz) >
        config.control_hz) {
      // A sampling rate above the control rate would ask for ticks that do not
      // exist; the recorder's period is derived from this and the control
      // period.
      Fail("telemetry.recorder.sample_hz",
           "must not exceed runtime.control_hz");
    }
    // The recorder samples a whole number of control ticks, so the two rates
    // have to divide each other exactly. A rate that would need a fraction of a
    // tick - 200 Hz on a 500 Hz loop is one sample per 2.5 ticks - is refused
    // here instead of being rounded into a run that records another rate than
    // the one the file declares, with the requested sampling phase drifting
    // against the loop.
    const auto sample_period_us = static_cast<protocol::TimeUs>(
        std::llround(1'000'000.0 /
                     static_cast<double>(config.telemetry.recorder.sample_hz)));
    if (sample_period_us == 0 ||
        sample_period_us % config.control_period_us != 0) {
      Fail("telemetry.recorder.sample_hz",
           "must divide runtime.control_hz exactly: the recorder samples a "
           "whole number of control ticks");
    }
    config.telemetry.recorder.queue_capacity = static_cast<std::size_t>(
        PositiveIntegerMember(recorder, "queue_capacity", "telemetry.recorder",
                              std::numeric_limits<std::uint32_t>::max()));
    const Json &recorder_directory =
        Member(recorder, "directory", "telemetry.recorder");
    if (recorder_directory.is_null()) {
      config.telemetry.recorder.directory = config.telemetry.directory;
    } else if (recorder_directory.is_string() &&
               !recorder_directory.get<std::string>().empty()) {
      config.telemetry.recorder.directory =
          recorder_directory.get<std::string>();
    } else {
      Fail("telemetry.recorder.directory",
           "must be a non-empty string or null");
    }
  }

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
