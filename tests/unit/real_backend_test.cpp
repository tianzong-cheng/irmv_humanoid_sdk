#include "irmv/hardware/real_backend.hpp"

#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>

#include "irmv/hardware/device_lease.hpp"
#include "support/check.hpp"
#include "support/scripted_driver_port.hpp"

namespace {

using irmv::configuration::BusConfig;
using irmv::configuration::HardwareConfig;
using irmv::configuration::HardwareJointConfig;
using irmv::configuration::RuntimeConfig;
using irmv::hardware::DeviceLease;
using irmv::hardware::RealBackend;
using irmv::hardware::ScriptedDriverPort;
using irmv::hardware::TorqueOutput;
using irmv::hardware::WriteResult;

class TemporaryLockDirectory {
 public:
  TemporaryLockDirectory() {
    static unsigned int next_id = 0;
    path_ = std::filesystem::temp_directory_path() /
            ("irmv-real-backend-test-" + std::to_string(next_id++));
    std::filesystem::create_directories(path_);
    ::setenv("IRMV_LOCK_DIR", path_.c_str(), 1);
  }
  ~TemporaryLockDirectory() {
    ::unsetenv("IRMV_LOCK_DIR");
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

 private:
  std::filesystem::path path_;
};

RuntimeConfig TwoJointConfig() {
  RuntimeConfig config;
  config.deployment = "real";
  config.backend = "real";
  config.joint_names = {"test_left", "test_right"};
  HardwareConfig hardware;
  BusConfig bus;
  bus.name = "can0";
  bus.device_type = 41;
  bus.device_index = 0;
  bus.channel = 0;
  bus.arbitration_bps = 1000000;
  bus.data_bps = 5000000;
  hardware.buses.push_back(bus);
  for (const auto &[name, motor_id] :
       {std::pair<const char *, std::uint32_t>{"test_left", 1},
        std::pair<const char *, std::uint32_t>{"test_right", 7}}) {
    HardwareJointConfig joint;
    joint.name = name;
    joint.bus = "can0";
    joint.motor_id = motor_id;
    joint.direction = 1;
    joint.reduction_ratio = 1.0;
    joint.encoder_offset = 0.0;
    joint.torque_constant = 1.0;
    hardware.joints.push_back(joint);
  }
  config.hardware = hardware;
  return config;
}

bool InitializeThrows(RealBackend &backend, std::string_view expected) {
  try {
    backend.Initialize();
  } catch (const std::exception &error) {
    return std::string_view(error.what()).find(expected) !=
           std::string_view::npos;
  }
  return false;
}

}  // namespace

int main() {
  const TemporaryLockDirectory locks;
  const RuntimeConfig config = TwoJointConfig();

  // Nothing observed yet: the documented empty shape, and a write that does not
  // reach the port.
  {
    auto port = std::make_unique<ScriptedDriverPort>();
    ScriptedDriverPort *script = port.get();
    RealBackend backend(config, std::move(port));
    const auto before = backend.Read(1000);
    CHECK(before.count == 2);
    CHECK(!before.joints[0].valid);
    CHECK(before.stamp_us == 0);
    const TorqueOutput output{2, {1.0, 2.0}};
    const WriteResult result = backend.Write(1000, output);
    CHECK(!result.accepted);
    CHECK(script->send_calls == 0);
  }

  // Startup opens the port once; read does not rewrite a received stamp.
  {
    auto port = std::make_unique<ScriptedDriverPort>();
    ScriptedDriverPort *script = port.get();
    script->snapshot.count = 2;
    script->snapshot.stamp_us = 1234;
    script->snapshot.joints[0].valid = true;
    script->snapshot.joints[0].connected = true;
    script->snapshot.joints[0].q = 0.5;
    script->snapshot.joints[0].rx_stamp_us = 1200;
    script->snapshot.joints[0].error_code = 0x80010000u;
    script->snapshot.joints[1].valid = true;
    script->snapshot.joints[1].rx_stamp_us = 1210;
    script->snapshot.imu.valid = true;
    script->snapshot.imu.rx_stamp_us = 1220;
    script->snapshot.imu.orientation_rpy[2] = 0.25;

    RealBackend backend(config, std::move(port));
    backend.Initialize();
    CHECK(script->open_calls == 1);
    CHECK(script->opened_hardware.buses.size() == 1);
    for (const irmv::protocol::TimeUs now : {1u, 50u, 5000u}) {
      const auto snapshot = backend.Read(now);
      CHECK(snapshot.joints[0].rx_stamp_us == 1200);
      CHECK(snapshot.joints[1].rx_stamp_us == 1210);
      CHECK(snapshot.imu.rx_stamp_us == 1220);
      CHECK(snapshot.joints[0].error_code == 0x80010000u);
      CHECK(snapshot.joints[0].q == 0.5);
      CHECK(snapshot.imu.orientation_rpy[2] == 0.25);
    }
    CHECK(script->read_calls == 3);
    backend.Close();
    CHECK(script->close_calls == 1);
    backend.Close();
    CHECK(script->close_calls == 1);
  }

  // Canonical order and refusal: a short or non-finite request never reaches
  // the port, so no partial non-zero command can be submitted.
  {
    auto port = std::make_unique<ScriptedDriverPort>();
    ScriptedDriverPort *script = port.get();
    RealBackend backend(config, std::move(port));
    backend.Initialize();

    TorqueOutput output{2, {1.5, -0.5}};
    script->next_result = WriteResult{true, 0};
    CHECK(backend.Write(10, output).accepted);
    CHECK(script->sends.size() == 1);
    CHECK(script->sends[0].output.count == 2);
    CHECK(script->sends[0].output.torque[0] == 1.5);
    CHECK(script->sends[0].output.torque[1] == -0.5);

    const int sends = script->send_calls;
    CHECK(!backend.Write(20, TorqueOutput{1, {1.0}}).accepted);
    TorqueOutput not_finite{2, {1.0, std::numeric_limits<double>::quiet_NaN()}};
    CHECK(!backend.Write(20, not_finite).accepted);
    CHECK(script->send_calls == sends);

    script->next_result = WriteResult{false, 0x80000001u};
    const WriteResult refused = backend.Write(30, output);
    CHECK(!refused.accepted);
    CHECK(refused.error_code == 0x80000001u);
    backend.Close();
  }

  // A failed Open closes what it opened and releases the lease.
  {
    auto port = std::make_unique<ScriptedDriverPort>();
    ScriptedDriverPort *script = port.get();
    script->open_error = true;
    RealBackend backend(config, std::move(port));
    CHECK(InitializeThrows(backend, "scripted open failure"));
    CHECK(script->open_calls == 1);
    CHECK(script->close_calls == 1);
    DeviceLease lease =
        DeviceLease::Acquire(*config.hardware);
    CHECK(lease.held());
  }

  // Another owner blocks the start before the port is opened.
  {
    DeviceLease owner = DeviceLease::Acquire(*config.hardware);
    auto port = std::make_unique<ScriptedDriverPort>();
    ScriptedDriverPort *script = port.get();
    RealBackend backend(config, std::move(port));
    CHECK(InitializeThrows(backend, "can-41-0"));
    CHECK(script->open_calls == 0);
    CHECK(script->close_calls == 0);
  }

  // No hardware table is refused rather than turned into an empty device.
  {
    RuntimeConfig without_hardware = config;
    without_hardware.hardware.reset();
    auto port = std::make_unique<ScriptedDriverPort>();
    RealBackend backend(without_hardware, std::move(port));
    CHECK(InitializeThrows(backend, "hardware table"));
  }
  return 0;
}
