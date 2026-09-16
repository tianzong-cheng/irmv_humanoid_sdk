#include "irmv/configuration/config.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <optional>
#include <string_view>
#include <unordered_set>

#include "irmv/configuration/sha256.hpp"

namespace irmv::configuration {
namespace {

using Json = nlohmann::json;

std::optional<std::uint64_t> ExactNonnegativeInteger(std::string_view token) {
  std::size_t position = 0;
  bool negative = false;
  if (position < token.size() &&
      (token[position] == '-' || token[position] == '+')) {
    negative = token[position] == '-';
    ++position;
  }

  const std::size_t exponent_position = token.find_first_of("eE", position);
  const std::size_t significand_end =
      exponent_position == std::string_view::npos ? token.size()
                                                  : exponent_position;
  const std::size_t decimal_position = token.find('.', position);
  const std::size_t fractional_digits =
      decimal_position == std::string_view::npos
          ? 0U
          : significand_end - decimal_position - 1U;

  std::string digits;
  digits.reserve(significand_end - position);
  for (std::size_t index = position; index < significand_end; ++index) {
    if (token[index] != '.') {
      digits.push_back(token[index]);
    }
  }
  const std::size_t first_nonzero = digits.find_first_not_of('0');
  if (first_nonzero == std::string::npos) {
    return 0U;
  }
  if (negative) {
    return std::nullopt;
  }
  digits.erase(0, first_nonzero);

  bool exponent_negative = false;
  std::size_t exponent_magnitude = 0;
  if (exponent_position != std::string_view::npos) {
    position = exponent_position + 1U;
    if (position < token.size() &&
        (token[position] == '-' || token[position] == '+')) {
      exponent_negative = token[position] == '-';
      ++position;
    }
    const std::size_t exponent_cap =
        token.size() > std::numeric_limits<std::size_t>::max() - 64U
            ? std::numeric_limits<std::size_t>::max()
            : token.size() + 64U;
    for (; position < token.size(); ++position) {
      const std::size_t digit = static_cast<std::size_t>(token[position] - '0');
      if (exponent_magnitude > (exponent_cap - digit) / 10U) {
        exponent_magnitude = exponent_cap;
        break;
      }
      exponent_magnitude = exponent_magnitude * 10U + digit;
    }
  }

  bool negative_shift = false;
  std::size_t shift = 0;
  if (exponent_negative) {
    negative_shift = true;
    shift = exponent_magnitude >
                    std::numeric_limits<std::size_t>::max() - fractional_digits
                ? std::numeric_limits<std::size_t>::max()
                : exponent_magnitude + fractional_digits;
  } else if (exponent_magnitude >= fractional_digits) {
    shift = exponent_magnitude - fractional_digits;
  } else {
    negative_shift = true;
    shift = fractional_digits - exponent_magnitude;
  }

  if (negative_shift) {
    if (shift > digits.size()) {
      return std::nullopt;
    }
    for (std::size_t index = digits.size() - shift; index < digits.size();
         ++index) {
      if (digits[index] != '0') {
        return std::nullopt;
      }
    }
    digits.resize(digits.size() - shift);
    if (digits.empty()) {
      return 0U;
    }
  } else {
    if (shift > 20U || digits.size() > 20U - shift) {
      return std::nullopt;
    }
    digits.append(shift, '0');
  }

  std::uint64_t result = 0;
  for (const char digit_character : digits) {
    const std::uint64_t digit =
        static_cast<std::uint64_t>(digit_character - '0');
    if (result > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
      return std::nullopt;
    }
    result = result * 10U + digit;
  }
  return result;
}

class ExactIntegerSax {
 public:
  using number_integer_t = Json::number_integer_t;
  using number_unsigned_t = Json::number_unsigned_t;
  using number_float_t = Json::number_float_t;
  using string_t = Json::string_t;
  using binary_t = Json::binary_t;

  explicit ExactIntegerSax(Json &result) : dom_(result) {}

  bool null() { return dom_.null(); }
  bool boolean(bool value) { return dom_.boolean(value); }
  bool number_integer(number_integer_t value) {
    return dom_.number_integer(value);
  }
  bool number_unsigned(number_unsigned_t value) {
    return dom_.number_unsigned(value);
  }
  bool number_float(number_float_t value, const string_t &token) {
    const auto integer = ExactNonnegativeInteger(token);
    return integer.has_value() ? dom_.number_unsigned(*integer)
                               : dom_.number_float(value, token);
  }
  bool string(string_t &value) { return dom_.string(value); }
  bool binary(binary_t &value) { return dom_.binary(value); }
  bool start_object(std::size_t elements) {
    return dom_.start_object(elements);
  }
  bool key(string_t &value) { return dom_.key(value); }
  bool end_object() { return dom_.end_object(); }
  bool start_array(std::size_t elements) { return dom_.start_array(elements); }
  bool end_array() { return dom_.end_array(); }
  bool parse_error(std::size_t position, const std::string &last_token,
                   const nlohmann::detail::exception &exception) {
    return dom_.parse_error(position, last_token, exception);
  }

 private:
  nlohmann::detail::json_sax_dom_parser<Json> dom_;
};

[[noreturn]] void Fail(std::string path, std::string message) {
  throw ConfigError(std::move(path), std::move(message));
}

const Json &Member(const Json &object, std::string_view key,
                   const std::string &path) {
  if (!object.is_object()) {
    Fail(path, "must be an object");
  }
  const auto member = object.find(key);
  if (member == object.end()) {
    Fail(path.empty() ? std::string(key) : path + "." + std::string(key),
         "is required");
  }
  return *member;
}

const Json &ObjectMember(const Json &object, std::string_view key,
                         const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  const Json &value = Member(object, key, path);
  if (!value.is_object()) {
    Fail(member_path, "must be an object");
  }
  return value;
}

std::string StringMember(const Json &object, std::string_view key,
                         const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  const Json &value = Member(object, key, path);
  if (!value.is_string()) {
    Fail(member_path, "must be a string");
  }
  return value.get<std::string>();
}

double FiniteNumber(const Json &value, const std::string &path) {
  if (!value.is_number()) {
    Fail(path, "must be a number");
  }
  const double result = value.get<double>();
  if (!std::isfinite(result)) {
    Fail(path, "must be finite");
  }
  return result == 0.0 ? 0.0 : result;
}

double FiniteMember(const Json &object, std::string_view key,
                    const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  return FiniteNumber(Member(object, key, path), member_path);
}

std::uint64_t IntegerInRange(const Json &value, const std::string &path,
                             std::uint64_t minimum, std::uint64_t maximum) {
  std::uint64_t result = 0;
  if (value.is_number_unsigned()) {
    result = value.get<std::uint64_t>();
  } else if (value.is_number_integer()) {
    const auto signed_result = value.get<std::int64_t>();
    if (signed_result < 0) {
      Fail(path, "is outside the supported range");
    }
    result = static_cast<std::uint64_t>(signed_result);
  } else {
    Fail(path, "must be a mathematical integer");
  }
  if (result < minimum || result > maximum) {
    Fail(path, "is outside the supported range");
  }
  return result;
}

std::uint64_t PositiveInteger(
    const Json &value, const std::string &path,
    std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
  return IntegerInRange(value, path, 1U, maximum);
}

std::uint64_t PositiveIntegerMember(
    const Json &object, std::string_view key, const std::string &path,
    std::uint64_t maximum = std::numeric_limits<std::uint64_t>::max()) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  return PositiveInteger(Member(object, key, path), member_path, maximum);
}

bool BoolMember(const Json &object, std::string_view key,
                const std::string &path) {
  const std::string member_path =
      path.empty() ? std::string(key) : path + "." + std::string(key);
  const Json &value = Member(object, key, path);
  if (!value.is_boolean()) {
    Fail(member_path, "must be a boolean");
  }
  return value.get<bool>();
}

void ReadJointMap(const Json &mapping, const std::string &path,
                  const std::vector<std::string> &joint_names,
                  bool require_positive, JointArray &destination) {
  if (!mapping.is_object()) {
    Fail(path, "must be an object keyed by joint name");
  }
  std::unordered_set<std::string> expected(joint_names.begin(),
                                           joint_names.end());
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const std::string &name = joint_names[index];
    const auto value = mapping.find(name);
    if (value == mapping.end()) {
      Fail(path + "." + name, "is required");
    }
    destination[index] = FiniteNumber(*value, path + "." + name);
    if (require_positive && destination[index] <= 0.0) {
      Fail(path + "." + name, "must be positive");
    }
    if (!require_positive && destination[index] < 0.0) {
      Fail(path + "." + name, "must be non-negative");
    }
  }
  for (auto item = mapping.begin(); item != mapping.end(); ++item) {
    if (expected.count(item.key()) == 0U) {
      Fail(path + "." + item.key(), "does not name a configured joint");
    }
  }
}

LogLevel ParseLogLevel(const std::string &level) {
  if (level == "DEBUG") {
    return LogLevel::Debug;
  }
  if (level == "INFO") {
    return LogLevel::Info;
  }
  if (level == "WARNING") {
    return LogLevel::Warning;
  }
  if (level == "ERROR") {
    return LogLevel::Error;
  }
  Fail("telemetry.level", "must be DEBUG, INFO, WARNING, or ERROR");
}

std::string LogLevelName(LogLevel level) {
  switch (level) {
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warning:
      return "WARNING";
    case LogLevel::Error:
      return "ERROR";
  }
  return {};
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

ConfigError::ConfigError(std::string field_path, std::string message)
    : std::runtime_error(field_path + ": " + message),
      field_path_(std::move(field_path)) {}

RuntimeConfig LoadConfig(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) {
    Fail("$", "cannot open configuration file " + path.string());
  }

  Json document;
  try {
    ExactIntegerSax sax(document);
    if (!Json::sax_parse(input, &sax)) {
      Fail("$", "invalid JSON");
    }
  } catch (const Json::exception &error) {
    Fail("$", std::string("invalid JSON: ") + error.what());
  }
  if (!document.is_object()) {
    Fail("$", "must be an object");
  }

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
    if (name.empty() || name.size() > protocol::kMaxNameBytes) {
      Fail(joint_path + ".name", "must contain 1 to 64 UTF-8 bytes");
    }
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
               config.joint_names, true, config.damping_kd);
  const Json &safety = ObjectMember(runtime, "safety", "runtime");
  ReadJointMap(Member(safety, "joint_velocity_limits", "runtime.safety"),
               "runtime.safety.joint_velocity_limits", config.joint_names, true,
               config.joint_velocity_limits);
  ReadJointMap(Member(safety, "torque_limits", "runtime.safety"),
               "runtime.safety.torque_limits", config.joint_names, true,
               config.torque_limits);
  ReadJointMap(Member(safety, "target_velocity_limits", "runtime.safety"),
               "runtime.safety.target_velocity_limits", config.joint_names,
               true, config.target_velocity_limits);

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
               "runtime.continuity.position_delta", config.joint_names, false,
               config.continuity.position_delta);
  ReadJointMap(Member(continuity, "velocity_delta", "runtime.continuity"),
               "runtime.continuity.velocity_delta", config.joint_names, false,
               config.continuity.velocity_delta);
  ReadJointMap(Member(continuity, "torque_delta", "runtime.continuity"),
               "runtime.continuity.torque_delta", config.joint_names, false,
               config.continuity.torque_delta);
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
      IntegerInRange(domain, "dds.domain", 0U, 230U));
  config.dds.interface = StringMember(dds, "interface", "dds");
  if (config.dds.interface.empty()) {
    Fail("dds.interface", "must not be empty");
  }

  const Json canonical = CanonicalConfig(config);
  config.config_id = Sha256(canonical.dump());
  config.layout_id = Sha256(Json(config.joint_names).dump());
  return config;
}

}  // namespace irmv::configuration
