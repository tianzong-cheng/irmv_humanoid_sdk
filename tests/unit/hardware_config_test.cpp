#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

#include "irmv/configuration/config.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::ConfigError;
using irmv::configuration::LoadConfig;
using Json = nlohmann::json;

class TemporaryConfig {
 public:
  explicit TemporaryConfig(const Json &document) {
    static unsigned int next_id = 0;
    path_ = std::filesystem::temp_directory_path() /
            ("irmv-hardware-config-" + std::to_string(next_id++) + ".json");
    std::ofstream output(path_);
    output << document.dump(2);
  }
  ~TemporaryConfig() { std::filesystem::remove(path_); }
  const std::filesystem::path &path() const { return path_; }

 private:
  std::filesystem::path path_;
};

Json BaseConfig() {
  return {
      {"schema_version", 1},
      {"deployment", "fake_only"},
      {"backend", "fake"},
      {"robot",
       {{"units",
         {{"position", "rad"}, {"velocity", "rad_s"}, {"torque", "Nm"}}},
        {"joints",
         {{{"name", "test_left"}, {"position_min", -2}, {"position_max", 2}},
          {{"name", "test_right"},
           {"position_min", -2},
           {"position_max", 2}}}}}},
      {"runtime",
       {{"control_hz", 500},
        {"damping", {{"kd", {{"test_left", 2}, {"test_right", 2}}}}},
        {"safety",
         {{"joint_velocity_limits", {{"test_left", 10}, {"test_right", 10}}},
          {"torque_limits", {{"test_left", 20}, {"test_right", 20}}},
          {"target_velocity_limits", {{"test_left", 10}, {"test_right", 10}}},
          {"timeouts",
           {{"command_us", 100000},
            {"source_state_us", 150000},
            {"feedback_us", 20000},
            {"imu_us", 20000},
            {"supervisor_us", 300000}}}}},
        {"boot",
         {{"frequency_grace_us", 1000000},
          {"healthy_window_us", 100000},
          {"deadline_us", 2000000},
          {"minimum_frequency_hz", 400}}},
        {"recovery",
         {{"healthy_window_us", 100000}, {"minimum_frequency_hz", 400}}},
        {"imu_required", false},
        {"continuity",
         {{"position_delta", {{"test_left", 0.1}, {"test_right", 0.1}}},
          {"velocity_delta", {{"test_left", 0.5}, {"test_right", 0.5}}},
          {"torque_delta", {{"test_left", 5}, {"test_right", 5}}}}},
        {"state_window_capacity", 1024}}},
      {"telemetry",
       {{"level", "INFO"},
        {"directory", "logs"},
        {"rotate_bytes", 1048576},
        {"retained_files", 3},
        {"queue_capacity", 1024}}},
      {"dds", {{"domain", 1}, {"interface", "lo"}}}};
}

Json RealConfig() {
  Json document = BaseConfig();
  document["deployment"] = "real";
  document["backend"] = "real";
  document["robot"]["buses"] = {{{"name", "can0"},
                                 {"device_type", 41},
                                 {"device_index", 0},
                                 {"channel", 0},
                                 {"arbitration_bps", 1000000},
                                 {"data_bps", 5000000},
                                 {"iso_mode", true},
                                 {"resistance_enabled", false}},
                                {{"name", "can1"},
                                 {"device_type", 41},
                                 {"device_index", 0},
                                 {"channel", 1},
                                 {"arbitration_bps", 1000000},
                                 {"data_bps", 5000000},
                                 {"iso_mode", true},
                                 {"resistance_enabled", false}}};
  document["robot"]["joints"][0]["bus"] = "can0";
  document["robot"]["joints"][0]["motor_id"] = 1;
  document["robot"]["joints"][0]["direction"] = 1;
  document["robot"]["joints"][0]["reduction_ratio"] = 36.0;
  document["robot"]["joints"][0]["encoder_offset"] = 0.0;
  document["robot"]["joints"][0]["torque_constant"] = 0.1;
  document["robot"]["joints"][1]["bus"] = "can1";
  document["robot"]["joints"][1]["motor_id"] = 7;
  document["robot"]["joints"][1]["direction"] = -1;
  document["robot"]["joints"][1]["reduction_ratio"] = 10.0;
  document["robot"]["joints"][1]["encoder_offset"] = 0.25;
  document["robot"]["joints"][1]["torque_constant"] = 0.13;
  return document;
}

bool ThrowsAt(const Json &document, std::string_view field_path) {
  const TemporaryConfig file(document);
  try {
    static_cast<void>(LoadConfig(file.path()));
  } catch (const ConfigError &error) {
    return error.field_path() == field_path &&
           std::string_view(error.what()).find(field_path) !=
               std::string_view::npos;
  }
  return false;
}

}  // namespace

int main() {
  // The shipped fake document carries no hardware table and keeps working.
  const auto fake = LoadConfig("config/sdk.json");
  CHECK(!fake.hardware.has_value());

  const TemporaryConfig real_file(RealConfig());
  const auto real = LoadConfig(real_file.path());
  CHECK(real.deployment == "real");
  CHECK(real.backend == "real");
  CHECK(real.hardware.has_value());
  CHECK(real.hardware->buses.size() == 2);
  CHECK(real.hardware->joints.size() == 2);
  CHECK(real.hardware->joints[0].name == "test_left");
  CHECK(real.hardware->joints[0].bus == "can0");
  CHECK(real.hardware->joints[0].motor_id == 1);
  CHECK(real.hardware->joints[0].direction == 1);
  CHECK(real.hardware->joints[0].reduction_ratio == 36.0);
  CHECK(real.hardware->joints[1].name == "test_right");
  CHECK(real.hardware->joints[1].motor_id == 7);
  CHECK(real.hardware->joints[1].direction == -1);
  CHECK(real.hardware->joints[1].encoder_offset == 0.25);

  // Reordering the joint and bus arrays is not a redefinition: the mapping is
  // still found by joint name, and the referenced bus by its name.
  {
    Json shuffled = RealConfig();
    std::swap(shuffled["robot"]["joints"][0], shuffled["robot"]["joints"][1]);
    std::swap(shuffled["robot"]["buses"][0], shuffled["robot"]["buses"][1]);
    const TemporaryConfig file(shuffled);
    const auto config = LoadConfig(file.path());
    CHECK(config.joint_names[0] == "test_right");
    CHECK(config.hardware->joints[0].name == "test_right");
    CHECK(config.hardware->joints[0].bus == "can1");
    CHECK(config.hardware->joints[1].name == "test_left");
    CHECK(config.hardware->joints[1].bus == "can0");
  }

  // A mapping change is part of the configuration identity.
  {
    Json changed = RealConfig();
    changed["robot"]["joints"][0]["reduction_ratio"] = 35.0;
    const TemporaryConfig file(changed);
    CHECK(LoadConfig(file.path()).config_id != real.config_id);
  }

  // Conditionally required fields and cross-field facts.
  {
    Json missing = RealConfig();
    missing["robot"]["joints"][0].erase("bus");
    CHECK(ThrowsAt(missing, "robot.joints[0].bus"));
  }
  {
    Json missing = RealConfig();
    missing["robot"]["joints"][0].erase("torque_constant");
    CHECK(ThrowsAt(missing, "robot.joints[0].torque_constant"));
  }
  {
    Json missing = RealConfig();
    missing["robot"].erase("buses");
    CHECK(ThrowsAt(missing, "robot.buses"));
  }
  {
    Json duplicate = RealConfig();
    duplicate["robot"]["joints"][1]["motor_id"] = 1;
    duplicate["robot"]["joints"][1]["bus"] = "can0";
    CHECK(ThrowsAt(duplicate, "robot.joints[1].motor_id"));
  }
  {
    Json unknown = RealConfig();
    unknown["robot"]["joints"][0]["bus"] = "can9";
    CHECK(ThrowsAt(unknown, "robot.joints[0].bus"));
  }
  {
    Json direction = RealConfig();
    direction["robot"]["joints"][0]["direction"] = 0;
    CHECK(ThrowsAt(direction, "robot.joints[0].direction"));
  }
  {
    Json ratio = RealConfig();
    ratio["robot"]["joints"][0]["reduction_ratio"] = 0.0;
    CHECK(ThrowsAt(ratio, "robot.joints[0].reduction_ratio"));
  }
  {
    Json ratio = RealConfig();
    ratio["robot"]["joints"][0]["reduction_ratio"] = -36.0;
    CHECK(ThrowsAt(ratio, "robot.joints[0].reduction_ratio"));
  }
  {
    Json gain = RealConfig();
    gain["robot"]["joints"][0]["torque_constant"] = 0.0;
    CHECK(ThrowsAt(gain, "robot.joints[0].torque_constant"));
  }
  {
    Json non_finite = RealConfig();
    non_finite["robot"]["joints"][0]["encoder_offset"] =
        std::numeric_limits<double>::quiet_NaN();
    CHECK(ThrowsAt(non_finite, "robot.joints[0].encoder_offset"));
  }
  {
    Json channel = RealConfig();
    channel["robot"]["buses"][0]["channel"] = 2;
    CHECK(ThrowsAt(channel, "robot.buses[0].channel"));
  }
  {
    Json endpoint = RealConfig();
    endpoint["robot"]["buses"][1]["channel"] = 0;
    CHECK(ThrowsAt(endpoint, "robot.buses[1]"));
  }
  {
    Json pair = BaseConfig();
    pair["backend"] = "real";
    CHECK(ThrowsAt(pair, "backend"));
  }
  {
    Json pair = RealConfig();
    pair["deployment"] = "fake_only";
    CHECK(ThrowsAt(pair, "backend"));
  }
  {
    Json imu = RealConfig();
    imu["runtime"]["imu_required"] = true;
    CHECK(ThrowsAt(imu, "robot.imu"));
  }
  {
    Json imu = RealConfig();
    imu["runtime"]["imu_required"] = true;
    imu["robot"]["imu"] = {{"hardware_id", "yis-0"}, {"baudrate", 921600}};
    const TemporaryConfig file(imu);
    const auto config = LoadConfig(file.path());
    CHECK(config.hardware->imu.has_value());
    CHECK(config.hardware->imu->baudrate == 921600);
  }
  return 0;
}
