// Test-only runtime process for the RealBackend contract.
//
// It is the real robot_runtime loop, the real DDS transport and the real
// RealBackend lifecycle. Only the device port is scripted: it produces coherent
// feedback with host receive stamps and records every torque frame. Nothing
// here is installed or selected by the production factory.
//
//   real_backend_runtime --config PATH [--domain N]
//
// The behaviour comes from IRMV_TEST_SCRIPT: normal, near-future, far-future,
// write-refused, driver-error, overspeed, imu-missing.
// IRMV_TEST_SEND_LOG names a JSON-lines file of accepted send attempts.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend_factory.hpp"
#include "irmv/hardware/driver_port.hpp"
#include "irmv/hardware/real_backend.hpp"
#include "irmv/runtime/runtime_loop.hpp"

namespace {

std::uint64_t NowUs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::string Script() {
  const char *value = std::getenv("IRMV_TEST_SCRIPT");
  return value == nullptr ? "normal" : value;
}

class ScriptedPort final : public irmv::hardware::DriverPort {
 public:
  explicit ScriptedPort(std::string script) : script_(std::move(script)) {
    const char *log = std::getenv("IRMV_TEST_SEND_LOG");
    if (log != nullptr) {
      log_.open(log, std::ios::app);
    }
  }

  void Open(const irmv::configuration::HardwareConfig &hardware) override {
    joint_count_ = hardware.joints.size();
    opened_ = true;
  }

  irmv::protocol::FeedbackSnapshot ReadLatest() override {
    irmv::protocol::FeedbackSnapshot snapshot;
    snapshot.count = static_cast<std::uint32_t>(joint_count_);
    std::uint64_t stamp = NowUs();
    if (script_ == "near-future") {
      // Arrives after the loop read its clock but before the tick evaluation.
      stamp += 1000;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } else if (script_ == "far-future") {
      stamp += 1'000'000;
    }
    snapshot.stamp_us = stamp;
    snapshot.snapshot_seq = ++sequence_;
    for (std::size_t index = 0; index < joint_count_; ++index) {
      auto &joint = snapshot.joints[index];
      joint.valid = true;
      joint.connected = true;
      joint.rx_stamp_us = stamp;
      joint.freq_hz = 500.0;
      joint.q = 0.0;
      joint.dq = script_ == "overspeed" ? 1000.0 : 0.0;
      joint.tau_est = 0.0;
      joint.error_code = script_ == "driver-error" ? 0x80000000u : 0u;
    }
    const bool imu_missing = script_ == "imu-missing";
    snapshot.imu.valid = !imu_missing;
    snapshot.imu.rx_stamp_us = imu_missing ? 0 : stamp;
    snapshot.imu.orientation_rpy = {0.0, 0.0, 0.0};
    snapshot.imu.angular_velocity = {0.0, 0.0, 0.0};
    snapshot.imu.linear_acceleration = {0.0, 0.0, 9.81};
    return snapshot;
  }

  irmv::hardware::WriteResult SendTorque(
      const irmv::hardware::TorqueOutput &output) override {
    irmv::hardware::WriteResult result;
    ++send_count_;
    const bool refused = script_ == "write-refused" && send_count_ > 5;
    result.accepted = !refused;
    if (refused) {
      result.error_code = 0x80000001u;
    }
    if (log_) {
      log_ << "{\"send\":" << send_count_ << ",\"time_us\":" << NowUs()
           << ",\"count\":" << output.count
           << ",\"accepted\":" << (result.accepted ? "true" : "false")
           << ",\"torque0\":" << output.torque[0]
           << ",\"torque1\":" << output.torque[1] << "}\n";
      log_.flush();
    }
    return result;
  }

  irmv::hardware::WriteResult EnableMotor(const std::string &,
                                          std::uint32_t) override {
    return irmv::hardware::WriteResult{true, 0};
  }

  irmv::hardware::WriteResult SendMotorCommand(const std::string &,
                                               std::uint32_t,
                                               std::uint8_t) override {
    return irmv::hardware::WriteResult{true, 0};
  }

  void Close() noexcept override { opened_ = false; }

 private:
  std::string script_;
  std::size_t joint_count_{0};
  std::uint64_t sequence_{0};
  std::uint64_t send_count_{0};
  bool opened_{false};
  std::ofstream log_;
};

}  // namespace

namespace irmv::hardware {

std::unique_ptr<Backend> MakeBackend(
    const configuration::RuntimeConfig &config) {
  return std::make_unique<RealBackend>(
      config, std::make_unique<ScriptedPort>(Script()));
}

}  // namespace irmv::hardware

int main(int argc, char **argv) {
  irmv::runtime::RuntimeOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--config" && index + 1 < argc) {
      options.config_path = argv[++index];
    } else if (argument == "--domain" && index + 1 < argc) {
      options.dds_domain =
          static_cast<std::uint32_t>(std::stoul(argv[++index]));
    } else {
      return 2;
    }
  }
  if (options.config_path.empty()) {
    return 2;
  }
  return irmv::runtime::RunStandaloneRuntime(options);
}
