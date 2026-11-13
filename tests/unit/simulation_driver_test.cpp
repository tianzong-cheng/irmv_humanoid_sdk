#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "irmv/configuration/application_config.hpp"
#include "irmv/runtime/shutdown_authorization.hpp"
#include "irmv/simulation/application_driver.hpp"
#include "irmv/simulation/bootstrap.hpp"
#include "irmv/simulation/runtime_driver.hpp"
#include "support/check.hpp"
int main(int argc, char** argv) {
  if (argc == 2) {
    const auto c = irmv::configuration::LoadApplicationConfig(argv[1]);
    std::cout << c.runtime.config_id << " " << c.application_config_id << "\n";
    return 0;
  }
  using irmv::protocol::ExecutionState;
  using irmv::runtime::MayPlanLocalStop;
  CHECK(!MayPlanLocalStop(ExecutionState::Damping, false));
  CHECK(MayPlanLocalStop(ExecutionState::Damping, false, true));
  CHECK(MayPlanLocalStop(ExecutionState::Tracking, true));
  CHECK(!MayPlanLocalStop(ExecutionState::Fault, true, true));
  CHECK(!MayPlanLocalStop(ExecutionState::Boot, true, true));
  char name[] = "/tmp/irmv-driver-XXXXXX";
  CHECK(mkdtemp(name) != nullptr);
  const std::filesystem::path directory(name);
  irmv::configuration::RuntimeConfig config;
  config.backend = "sim";
  config.config_id = std::string(64, 'a');
  config.control_period_us = 2000;
  config.simulation.emplace();
  config.simulation->socket_directory = directory;
  config.simulation->engine_sha256 = std::string(64, 'e');
  config.simulation->physics_dt_us = 1000;
  nlohmann::json artifact = {{"run_id", std::string(32, '1')},
                             {"runtime_id", std::string(32, '2')},
                             {"config_digest", std::string(64, 'a')},
                             {"application_digest", std::string(64, 'b')},
                             {"engine_digest", std::string(64, 'e')},
                             {"physics_dt_us", 1000},
                             {"control_dt_us", 2000}};
  auto write = [&] {
    std::ofstream out(directory / "bootstrap.json");
    out << artifact;
    out.close();
    chmod((directory / "bootstrap.json").c_str(), 0600);
  };
  auto refuses = [&] {
    try {
      irmv::simulation::LoadBootstrap(
          config, irmv::simulation::Role::Application, std::string(64, 'b'));
    } catch (const std::exception&) {
      return true;
    }
    return false;
  };
  write();
  CHECK(!refuses());
  artifact["config_digest"] = std::string(64, 'c');
  write();
  CHECK(refuses());
  artifact["config_digest"] = std::string(64, 'a');
  artifact["control_dt_us"] = 4000;
  write();
  CHECK(refuses());
  artifact["control_dt_us"] = 2000;
  artifact["run_id"] = std::string(32, '2');
  write();
  CHECK(refuses());
  artifact["run_id"] = std::string(32, '1');
  write();
  chmod(directory.c_str(), 0755);
  CHECK(refuses());
  chmod(directory.c_str(), 0700);
  std::filesystem::rename(directory / "bootstrap.json",
                          directory / "real.json");
  std::filesystem::create_symlink(directory / "real.json",
                                  directory / "bootstrap.json");
  CHECK(refuses());
  std::filesystem::remove_all(directory);
  irmv::simulation::StepPermit permit;
  CHECK(permit.identity.logical_us == 1);
  irmv::simulation::MessageIdentity inactive;
  inactive = irmv::simulation::InactiveCommand({1});
  CHECK(inactive.kind == irmv::simulation::MessageKind::Command);
  CHECK(inactive.runtime_id[0] == 1);
  CHECK(inactive.content_digest == irmv::simulation::Digest{});
  CHECK(inactive.sequence == 0);
  // Host stalls supply no logical advance. Only a matching completion clears a
  // submission.
  irmv::simulation::InferencePhase inference;
  CHECK(inference.Begin(1, 7, 3, 20000));
  CHECK(!inference.Begin(1, 7, 3, 20000));
  CHECK(!inference.Complete(1, 7, 2));
  CHECK(inference.pending());
  CHECK(inference.Complete(1, 7, 3));
  CHECK(!inference.Begin(1, 7, 3, 20000));
  CHECK(!inference.Begin(18001, 7, 3, 20000));
  CHECK(inference.Begin(20001, 7, 3, 20000));
  inference.Cancel();
  CHECK(!inference.Complete(20001, 7, 3));
  CHECK(inference.Begin(20001, 8, 4, 20000));
  CHECK(!inference.Complete(20001, 7, 3));
  CHECK(inference.pending());
  return 0;
}
