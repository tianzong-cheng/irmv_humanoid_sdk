#include "irmv/configuration/config.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "irmv/configuration/sha256.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::ConfigError;
using irmv::configuration::ConfigOverrides;
using irmv::configuration::kMaxDdsDomain;
using irmv::configuration::LoadConfig;

class GroupingPunctuation : public std::numpunct<char> {
 protected:
  char do_thousands_sep() const override { return '_'; }
  std::string do_grouping() const override { return "\1"; }
};

class TemporaryConfig {
 public:
  explicit TemporaryConfig(std::string_view contents) {
    static unsigned int next_id = 0;
    path_ = std::filesystem::temp_directory_path() /
            ("irmv-config-test-" + std::to_string(next_id++) + ".json");
    std::ofstream output(path_);
    output << contents;
  }

  ~TemporaryConfig() { std::filesystem::remove(path_); }

  const std::filesystem::path &path() const { return path_; }

 private:
  std::filesystem::path path_;
};

std::string ValidConfig() {
  return R"({
    "schema_version": 1,
    "deployment": "fake_only",
    "backend": "fake",
    "robot": {
      "units": {"position": "rad", "velocity": "rad_s", "torque": "Nm"},
      "joints": [
        {"name": "test_left", "position_min": -2, "position_max": 2},
        {"name": "test_right", "position_min": -2, "position_max": 2}
      ]
    },
    "runtime": {
      "control_hz": 500,
      "damping": {"kd": {"test_right": 2, "test_left": 2}},
      "safety": {
        "joint_velocity_limits": {"test_right": 10, "test_left": 10},
        "torque_limits": {"test_left": 20, "test_right": 20},
        "target_velocity_limits": {"test_left": 10, "test_right": 10},
        "timeouts": {"command_us": 100000, "source_state_us": 150000,
          "feedback_us": 20000, "imu_us": 20000, "supervisor_us": 300000}
      },
      "boot": {"frequency_grace_us": 1000000, "healthy_window_us": 100000,
        "deadline_us": 2000000, "minimum_frequency_hz": 400},
      "recovery": {"healthy_window_us": 100000, "minimum_frequency_hz": 400},
      "imu_required": false,
      "continuity": {
        "position_delta": {"test_left": 0.1, "test_right": 0.1},
        "velocity_delta": {"test_left": 0.5, "test_right": 0.5},
        "torque_delta": {"test_left": 5, "test_right": 5}
      },
      "state_window_capacity": 1024
    },
    "telemetry": {"level": "INFO", "directory": "logs", "rotate_bytes": 1048576,
      "retained_files": 3, "queue_capacity": 1024},
    "dds": {"domain": 1, "interface": "lo"}
  })";
}

std::string ReplaceOnce(std::string value, std::string_view from,
                        std::string_view to) {
  const auto position = value.find(from);
  if (position == std::string::npos) {
    return {};
  }
  value.replace(position, from.size(), to);
  return value;
}

std::string ReplaceAll(std::string value, std::string_view from,
                       std::string_view to) {
  std::size_t position = 0;
  while ((position = value.find(from, position)) != std::string::npos) {
    value.replace(position, from.size(), to);
    position += to.size();
  }
  return value;
}

std::string Repeat(std::string_view value, std::size_t count) {
  std::string result;
  result.reserve(value.size() * count);
  for (std::size_t index = 0; index < count; ++index) {
    result.append(value);
  }
  return result;
}

bool ThrowsAt(std::string_view contents, std::string_view field_path) {
  const TemporaryConfig file(contents);
  try {
    static_cast<void>(LoadConfig(file.path()));
  } catch (const ConfigError &error) {
    return error.field_path() == field_path &&
           std::string_view(error.what()).find(field_path) !=
               std::string_view::npos;
  }
  return false;
}

ConfigOverrides OverrideDomain(std::uint32_t domain) {
  ConfigOverrides overrides;
  overrides.dds_domain = domain;
  return overrides;
}

bool OverrideThrowsAt(const std::filesystem::path &path, std::uint32_t domain,
                      std::string_view field_path) {
  try {
    static_cast<void>(LoadConfig(path, OverrideDomain(domain)));
  } catch (const ConfigError &error) {
    return error.field_path() == field_path;
  }
  return false;
}

std::string TwelveJointConfig() {
  std::ostringstream joints;
  std::ostringstream values;
  for (int index = 0; index < 12; ++index) {
    if (index != 0) {
      joints << ',';
      values << ',';
    }
    joints << "{\"name\":\"joint_" << index
           << "\",\"position_min\":-2,\"position_max\":2}";
    values << "\"joint_" << index << "\":" << (index + 1);
  }
  const std::string mapping = values.str();
  std::ostringstream config;
  config << R"({"schema_version":1,"deployment":"fake_only","backend":"fake",
    "robot":{"units":{"position":"rad","velocity":"rad_s","torque":"Nm"},"joints":[)"
         << joints.str()
         << R"(]},"runtime":{"control_hz":500,"damping":{"kd":{)" << mapping
         << R"(}},"safety":{"joint_velocity_limits":{)" << mapping
         << R"(},"torque_limits":{)" << mapping
         << R"(},"target_velocity_limits":{)" << mapping
         << R"(},"timeouts":{"command_us":100000,"source_state_us":150000,
         "feedback_us":20000,"imu_us":20000,"supervisor_us":300000}},
         "boot":{"frequency_grace_us":1000000,"healthy_window_us":100000,
         "deadline_us":2000000,"minimum_frequency_hz":400},
         "recovery":{"healthy_window_us":100000,"minimum_frequency_hz":400},
         "imu_required":false,"continuity":{"position_delta":{)"
         << mapping << R"(},"velocity_delta":{)" << mapping
         << R"(},"torque_delta":{)" << mapping
         << R"(}},"state_window_capacity":1024},
         "telemetry":{"level":"INFO","directory":"logs","rotate_bytes":1048576,
         "retained_files":3,"queue_capacity":1024},
         "dds":{"domain":1,"interface":"lo"}})";
  return config.str();
}

}  // namespace

int main() {
  using irmv::configuration::LogLevel;
  using irmv::configuration::Sha256;

  CHECK(Sha256("") ==
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  CHECK(Sha256("abc") ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  CHECK(Sha256("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  CHECK(Sha256(std::string(1'000'000, 'a')) ==
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  const std::locale original_locale;
  std::locale::global(std::locale(original_locale, new GroupingPunctuation));
  CHECK(Sha256("abc") ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  std::locale::global(original_locale);

  const TemporaryConfig valid(ValidConfig());
  const auto config = LoadConfig(valid.path());
  const auto fixture = LoadConfig("config/sdk.json");
  CHECK(config.schema_version == 1);
  CHECK(config.deployment == "fake_only");
  CHECK(config.backend == "fake");
  CHECK(config.joint_names.size() == 2);
  CHECK(config.joint_names[0] == "test_left");
  CHECK(config.joint_names[1] == "test_right");
  CHECK(config.position_min[0] == -2.0);
  CHECK(config.position_max[1] == 2.0);
  CHECK(config.control_hz == 500.0);
  CHECK(config.control_period_us == 2000);
  CHECK(config.damping_kd[0] == 2.0);
  CHECK(config.joint_velocity_limits[1] == 10.0);
  CHECK(config.torque_limits[0] == 20.0);
  CHECK(config.target_velocity_limits[1] == 10.0);
  CHECK(config.timeouts.command_us == 100000);
  CHECK(config.timeouts.source_state_us == 150000);
  CHECK(config.timeouts.feedback_us == 20000);
  CHECK(config.timeouts.imu_us == 20000);
  CHECK(config.timeouts.supervisor_us == 300000);
  CHECK(config.boot.frequency_grace_us == 1000000);
  CHECK(config.boot.healthy_window_us == 100000);
  CHECK(config.boot.deadline_us == 2000000);
  CHECK(config.boot.minimum_frequency_hz == 400.0);
  CHECK(config.recovery.healthy_window_us == 100000);
  CHECK(config.recovery.minimum_frequency_hz == 400.0);
  CHECK(!config.imu_required);
  CHECK(config.continuity.position_delta[0] == 0.1);
  CHECK(config.continuity.velocity_delta[1] == 0.5);
  CHECK(config.continuity.torque_delta[0] == 5.0);
  CHECK(config.state_window_capacity == 1024);
  CHECK(config.telemetry.level == LogLevel::Info);
  CHECK(config.telemetry.directory == "logs");
  CHECK(config.telemetry.rotate_bytes == 1048576);
  CHECK(config.telemetry.retained_files == 3);
  CHECK(config.telemetry.queue_capacity == 1024);

  // The logging level covers the full report range: every level parses, the
  // level stays part of the configuration identity, and anything unknown is
  // still rejected.
  const std::pair<const char *, LogLevel> levels[] = {
      {"TRACE", LogLevel::Trace}, {"DEBUG", LogLevel::Debug},
      {"INFO", LogLevel::Info},   {"WARNING", LogLevel::Warning},
      {"ERROR", LogLevel::Error}, {"FATAL", LogLevel::Fatal},
  };
  for (const auto &level : levels) {
    const TemporaryConfig level_file(
        ReplaceOnce(ValidConfig(), "\"level\": \"INFO\"",
                    std::string("\"level\": \"") + level.first + "\""));
    const auto level_config = LoadConfig(level_file.path());
    CHECK(level_config.telemetry.level == level.second);
    CHECK((level_config.config_id == config.config_id) ==
          (level.second == LogLevel::Info));
  }
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"level\": \"INFO\"",
                             "\"level\": \"VERBOSE\""),
                 "telemetry.level"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"level\": \"INFO\"",
                             "\"level\": \"info\""),
                 "telemetry.level"));

  CHECK(config.dds.domain == 1);
  CHECK(config.dds.interface == "lo");
  CHECK(config.config_id.size() == 64);
  CHECK(config.layout_id.size() == 64);
  CHECK(fixture.config_id == config.config_id);
  CHECK(fixture.layout_id == config.layout_id);

  // A command line domain override is part of the configuration in effect, so
  // the identity digest covers it: two runs that differ only in the overridden
  // domain do not publish the same config_id, and an override of the value the
  // file already carries changes nothing.
  const auto overridden = LoadConfig(valid.path(), OverrideDomain(3));
  const auto other_override = LoadConfig(valid.path(), OverrideDomain(4));
  CHECK(overridden.dds.domain == 3);
  CHECK(other_override.dds.domain == 4);
  CHECK(overridden.config_id != config.config_id);
  CHECK(other_override.config_id != config.config_id);
  CHECK(other_override.config_id != overridden.config_id);
  CHECK(overridden.layout_id == config.layout_id);
  CHECK(LoadConfig(valid.path(), OverrideDomain(1)).config_id ==
        config.config_id);
  CHECK(LoadConfig(valid.path(), OverrideDomain(kMaxDdsDomain)).dds.domain ==
        kMaxDdsDomain);
  CHECK(OverrideThrowsAt(valid.path(), kMaxDdsDomain + 1, "dds.domain"));

  const std::string reordered = R"({"dds":{"interface":"lo","domain":1},
    "telemetry":{"queue_capacity":1024,"retained_files":3,"rotate_bytes":1048576,
      "directory":"logs","level":"INFO"},"backend":"fake","deployment":"fake_only",
    "schema_version":1,"runtime":{"state_window_capacity":1024,
      "continuity":{"torque_delta":{"test_right":5,"test_left":5},
        "velocity_delta":{"test_right":0.5,"test_left":0.5},
        "position_delta":{"test_right":0.1,"test_left":0.1}},"imu_required":false,
      "recovery":{"minimum_frequency_hz":400,"healthy_window_us":100000},
      "boot":{"minimum_frequency_hz":400,"deadline_us":2000000,
        "healthy_window_us":100000,"frequency_grace_us":1000000},
      "safety":{"timeouts":{"supervisor_us":300000,"imu_us":20000,
          "feedback_us":20000,"source_state_us":150000,"command_us":100000},
        "target_velocity_limits":{"test_right":10,"test_left":10},
        "torque_limits":{"test_right":20,"test_left":20},
        "joint_velocity_limits":{"test_left":10,"test_right":10}},
      "damping":{"kd":{"test_left":2,"test_right":2}},"control_hz":500},
    "robot":{"joints":[{"position_max":2,"position_min":-2,"name":"test_left"},
      {"position_max":2,"position_min":-2,"name":"test_right"}],
      "units":{"torque":"Nm","velocity":"rad_s","position":"rad"}}})";
  const TemporaryConfig reordered_file(reordered);
  const auto reordered_config = LoadConfig(reordered_file.path());
  CHECK(reordered_config.config_id == config.config_id);
  CHECK(reordered_config.layout_id == config.layout_id);

  const TemporaryConfig positive_zero_file(
      ReplaceOnce(ValidConfig(), "\"test_left\": 0.1", "\"test_left\": 0.0"));
  const TemporaryConfig negative_zero_file(
      ReplaceOnce(ValidConfig(), "\"test_left\": 0.1", "\"test_left\": -0.0"));
  const auto positive_zero = LoadConfig(positive_zero_file.path());
  const auto negative_zero = LoadConfig(negative_zero_file.path());
  CHECK(positive_zero.config_id == negative_zero.config_id);
  CHECK(!std::signbit(negative_zero.continuity.position_delta[0]));

  std::string mathematical_integers = ValidConfig();
  mathematical_integers =
      ReplaceOnce(mathematical_integers, "\"schema_version\": 1",
                  "\"schema_version\": 1.0");
  mathematical_integers = ReplaceOnce(
      mathematical_integers, "\"command_us\": 100000", "\"command_us\": 1e5");
  mathematical_integers =
      ReplaceOnce(mathematical_integers, "\"state_window_capacity\": 1024",
                  "\"state_window_capacity\": 1024.0");
  mathematical_integers =
      ReplaceOnce(mathematical_integers, "\"retained_files\": 3",
                  "\"retained_files\": 3e0");
  mathematical_integers =
      ReplaceOnce(mathematical_integers, "\"domain\": 1", "\"domain\": 1.0");
  const TemporaryConfig mathematical_integer_file(mathematical_integers);
  const auto mathematical_integer_config =
      LoadConfig(mathematical_integer_file.path());
  CHECK(mathematical_integer_config.config_id == config.config_id);

  const std::string unicode_name = Repeat("界", 21);
  const TemporaryConfig unicode_file(
      ReplaceAll(ValidConfig(), "test_left", unicode_name));
  const auto unicode_config = LoadConfig(unicode_file.path());
  CHECK(unicode_config.joint_names[0] == unicode_name);
  const std::string oversized_unicode_name = Repeat("界", 22);
  CHECK(ThrowsAt(ReplaceAll(ValidConfig(), "test_left", oversized_unicode_name),
                 "robot.joints[0].name"));

  CHECK(
      ThrowsAt(ReplaceOnce(ValidConfig(), "\"test_right\": 2, \"test_left\": 2",
                           "\"test_left\": 2"),
               "runtime.damping.kd.test_right"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"test_right\", \"position_min\"",
                             "\"test_left\", \"position_min\""),
                 "robot.joints[1].name"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"command_us\": 100000",
                             "\"command_us\": -1"),
                 "runtime.safety.timeouts.command_us"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"backend\": \"fake\"",
                             "\"backend\": \"real\""),
                 "backend"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"control_hz\": 500",
                             "\"control_hz\": 1e400"),
                 "$"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"position_min\": -2",
                             "\"position_min\": NaN"),
                 "$"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"state_window_capacity\": 1024",
                             "\"state_window_capacity\": 8193"),
                 "runtime.state_window_capacity"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(),
                             "\"target_velocity_limits\": {\"test_left\": 10, "
                             "\"test_right\": 10},",
                             ""),
                 "runtime.safety.target_velocity_limits"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"deadline_us\": 2000000",
                             "\"deadline_us\": 1100000"),
                 "runtime.boot.deadline_us"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"domain\": 1",
                             "\"domain\": 18446744073709551615"),
                 "dds.domain"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"control_hz\": 500",
                             "\"control_hz\": 1e-300"),
                 "runtime.control_hz"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"command_us\": 100000",
                             "\"command_us\": 100000.5"),
                 "runtime.safety.timeouts.command_us"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"schema_version\": 1",
                             "\"schema_version\": 1.00000000000000001"),
                 "schema_version"));
  CHECK(ThrowsAt(ReplaceOnce(ValidConfig(), "\"rotate_bytes\": 1048576",
                             "\"rotate_bytes\": 1.8446744073709552e19"),
                 "telemetry.rotate_bytes"));
  const TemporaryConfig maximum_decimal_file(
      ReplaceOnce(ValidConfig(), "\"rotate_bytes\": 1048576",
                  "\"rotate_bytes\": 18446744073709551615.0"));
  CHECK(LoadConfig(maximum_decimal_file.path()).telemetry.rotate_bytes ==
        std::numeric_limits<std::uint64_t>::max());
  const TemporaryConfig maximum_exponent_file(
      ReplaceOnce(ValidConfig(), "\"rotate_bytes\": 1048576",
                  "\"rotate_bytes\": 1.8446744073709551615e19"));
  CHECK(LoadConfig(maximum_exponent_file.path()).telemetry.rotate_bytes ==
        std::numeric_limits<std::uint64_t>::max());

  const TemporaryConfig twelve_file(TwelveJointConfig());
  const auto twelve = LoadConfig(twelve_file.path());
  CHECK(twelve.joint_names.size() == 12);
  CHECK(twelve.joint_names[11] == "joint_11");
  CHECK(twelve.damping_kd[0] == 1.0);
  CHECK(twelve.damping_kd[11] == 12.0);
  CHECK(twelve.target_velocity_limits[7] == 8.0);

  CHECK(ThrowsAt("{}", "schema_version"));
  return 0;
}
