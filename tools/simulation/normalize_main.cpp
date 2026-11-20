// Startup-only adapter: authoritative SDK loaders own canonical identities.
#include <cstdint>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "irmv/configuration/application_config.hpp"

int main(int argc, char** argv) {
  try {
    if (argc != 2 && argc != 4)
      throw std::runtime_error(
          "usage: simulation_normalize CONFIG [--domain N]");
    irmv::configuration::ConfigOverrides overrides;
    if (argc == 4) {
      if (std::string(argv[2]) != "--domain")
        throw std::runtime_error(
            "usage: simulation_normalize CONFIG [--domain N]");
      std::size_t consumed = 0;
      std::uint64_t domain = 0;
      try {
        domain = std::stoull(argv[3], &consumed);
      } catch (const std::exception&) {
        throw std::runtime_error("--domain must be an integer");
      }
      if (consumed != std::string(argv[3]).size() ||
          domain > irmv::configuration::kMaxDdsDomain)
        throw std::runtime_error(
            "--domain must be between 0 and " +
            std::to_string(irmv::configuration::kMaxDdsDomain));
      overrides.dds_domain = static_cast<std::uint32_t>(domain);
    }
    const auto a =
        irmv::configuration::LoadApplicationConfig(argv[1], overrides);
    const auto& c = a.runtime;
    if (!c.simulation) throw std::runtime_error("simulation config required");
    const auto& s = *c.simulation;
    nlohmann::json joints = nlohmann::json::array();
    nlohmann::json actuators = nlohmann::json::array();
    nlohmann::json q = nlohmann::json::array();
    for (std::size_t i = 0; i < c.joint_count(); ++i) {
      joints.push_back(s.joint_mapping[i].joint);
      actuators.push_back(s.joint_mapping[i].actuator);
      q.push_back(s.initial_joint_position[i]);
    }
    nlohmann::json out = {
        {"config_digest", c.config_id},
        {"application_digest", a.application_config_id},
        {"engine_digest", s.engine_sha256},
        {"physics_dt_us", s.physics_dt_us},
        {"control_dt_us", c.control_period_us},
        {"startup_timeout_us", s.host_startup_timeout_us},
        {"phase_timeout_us", s.host_phase_timeout_us},
        {"socket_directory", s.socket_directory.string()},
        {"asset_digest", s.asset_sha256},
        {"policy_path", a.policy.model_path.string()},
        {"engine_config",
         {{"model_path", s.model_path.string()},
          {"joint_names", joints},
          {"actuator_names", actuators},
          {"base_body", s.base_body},
          {"imu_site", s.imu_site},
          {"weld_name", s.weld_name},
          {"initial_base_position", s.initial_base_position},
          {"initial_base_quaternion", s.initial_base_quaternion},
          {"initial_joint_position", q}}}};
    std::cout << out.dump() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
