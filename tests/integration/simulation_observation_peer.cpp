// Test peer for GPU -> SimBackend -> real DDS -> RobotClient -> observation.
// The coordinator owns physics. This peer replaces only the two business loops;
// production drivers, backend, transports and observation math are unchanged.
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <thread>

#include "irmv/application/robot_client.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/hardware/sim_backend.hpp"
#include "irmv/policy/action.hpp"
#include "irmv/policy/deployment.hpp"
#include "irmv/policy/observation.hpp"
#include "irmv/simulation/application_driver.hpp"
#include "irmv/simulation/bootstrap.hpp"
#include "irmv/simulation/runtime_driver.hpp"

namespace {
using namespace irmv;

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int Run(const char* config_path, const char* deployment_path) {
  const auto config = configuration::LoadApplicationConfig(config_path);
  const auto& lower_config = config.runtime;
  Require(lower_config.dds.domain == 225, "requires serial reserved domain225");
  std::ifstream input(deployment_path);
  const std::string document{std::istreambuf_iterator<char>(input), {}};
  const auto contract = policy::ParseDeployment(document);
  const auto mapping =
      policy::BuildJointMapping(contract, lower_config.joint_names);
  std::array<float, protocol::kMaxJoints> last_action{};
  policy::ResetLastAction(contract.action, &last_action);
  const policy::VelocityCommand intent{0.35, -0.2, 0.55};

  const auto lower_endpoint =
      simulation::LoadBootstrap(lower_config, simulation::Role::Runtime);
  const auto upper_endpoint =
      simulation::LoadBootstrap(lower_config, simulation::Role::Application,
                                config.application_config_id);
  simulation::SimulationClock lower_clock(lower_endpoint.run_id,
                                          lower_config.control_period_us);
  simulation::SimulationClock upper_clock(upper_endpoint.run_id,
                                          lower_config.control_period_us);
  simulation::SimulationExchange exchange(lower_config.joint_count());
  hardware::SimBackend backend(lower_config, exchange);
  backend.Initialize();
  transport::DdsTransport lower(lower_config);
  transport::ClientTransport upper(config);
  Require(lower.ConfigureSimulation(lower_clock) &&
              upper.ConfigureSimulation(upper_clock),
          "configure simulation clocks");
  lower.Start();
  upper.Start();
  simulation::RuntimeDriver runtime(lower_endpoint, lower_clock, exchange,
                                    lower);
  simulation::ApplicationDriver application(upper_endpoint, upper_clock, upper);
  application::RobotClient client(application, config,
                                  application::MakeSupervisorId());
  runtime.Start();
  application.Start();

  constexpr unsigned kTicks = 6;
  unsigned observed = 0;
  bool processing = false;
  simulation::MessageIdentity consumed;
  const auto deadline = simulation::DriverHostUs() + 30'000'000;
  while (observed < kTicks) {
    Require(simulation::DriverHostUs() < deadline, "observation host deadline");
    Require(!runtime.Failed() && !application.Failed(), "driver failed");
    simulation::StepPermit permit;
    // Poll both roles on the same thread, preserving each channel's sole owner.
    if (runtime.TakePermit(&permit)) {
      const auto snapshot = backend.Read(permit.identity.logical_us);
      Require(snapshot.count == lower_config.joint_count() &&
                  snapshot.stamp_us == permit.identity.logical_us &&
                  snapshot.imu.valid,
              "backend feedback shape/stamp/IMU");
      runtime::TickResult tick;
      auto& state = tick.state;
      state.runtime_id = lower_endpoint.local_hello.runtime_id;
      state.state_seq = snapshot.snapshot_seq;
      state.stamp_us = snapshot.stamp_us;
      state.execution_state = protocol::ExecutionState::Damping;
      state.output_action = protocol::OutputAction::Damping;
      Require(state.config_id.Assign(lower_config.config_id) &&
                  state.layout_id.Assign(lower_config.layout_id),
              "state digests");
      state.feedback = snapshot;
      hardware::TorqueOutput torque;
      torque.count = static_cast<std::uint32_t>(lower_config.joint_count());
      Require(torque.count == 2, "requires two distinct test joints");
      torque.torque[0] = 0.5;
      torque.torque[1] = -0.35;
      Require(backend.Write(permit.identity.logical_us, torque).accepted,
              "backend write");
      Require(runtime.CompleteTick({tick, nullptr}), "runtime completion");
    }
    if (!processing && application.TakePermit(&permit)) {
      const auto& events = application.Events();
      Require(events.has_state && events.result_count == 0,
              "unexpected application input");
      client.ObserveState(events.state, events.state_rx_us);
      const auto view = client.Poll(upper_clock.NowUs());
      Require(view.state_known && view.state_fresh, "client state rejected");
      policy::Observation observation;
      Require(policy::BuildObservation(contract, mapping, view.state, intent,
                                       last_action, &observation),
              "observation rejected");
      nlohmann::json line{{"stamp_us", view.state.stamp_us},
                          {"state_seq", view.state.state_seq},
                          {"rpy", view.state.feedback.imu.orientation_rpy},
                          {"gyro", view.state.feedback.imu.angular_velocity},
                          {"obs", nlohmann::json::array()},
                          {"q", nlohmann::json::array()},
                          {"dq", nlohmann::json::array()}};
      for (std::uint32_t i = 0; i < view.state.feedback.count; ++i) {
        Require(view.state.feedback.joints[i].valid, "invalid joint");
        line["q"].push_back(view.state.feedback.joints[i].q);
        line["dq"].push_back(view.state.feedback.joints[i].dq);
      }
      for (std::uint32_t i = 0; i < observation.count; ++i)
        line["obs"].push_back(observation.values[i]);
      std::cout << "OBS " << line.dump() << std::endl;
      Require(simulation::Identify(events.state, &consumed), "state identity");
      processing = true;
    }
    if (processing) {
      Require(application.Poll(), "application polling");
      // DDS dequeue is not consumption. Report after RobotClient and the
      // observation builder consumed this exact sample, then seal phase two.
      if (application.Processed(consumed) &&
          application.CompletePhase(application.Outputs())) {
        processing = false;
        ++observed;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // This feedback witness is not a motion-lifecycle experiment. Its disconnect
  // is intentionally unproven shutdown; Python must still close the real
  // engine.
  lower.Stop();
  upper.Stop();
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  try {
    return Run(argv[1], argv[2]);
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
