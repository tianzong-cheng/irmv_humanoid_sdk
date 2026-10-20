#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "irmv/hardware/device_lease.hpp"
#include "irmv/hardware/driver_port.hpp"
#include "irmv/maintenance/commands.hpp"

namespace {

// The maintenance tool owns the same devices as the runtime. The lease is
// acquired before the port opens the first device and released only after the
// port closed it.
class LeasedDriverPort final : public irmv::hardware::DriverPort {
 public:
  explicit LeasedDriverPort(irmv::hardware::DriverPort &inner) : inner_(inner) {}

  void Open(const irmv::configuration::HardwareConfig &hardware) override {
    lease_ = irmv::hardware::DeviceLease::Acquire(hardware);
    try {
      inner_.Open(hardware);
    } catch (...) {
      lease_.Release();
      throw;
    }
  }

  irmv::protocol::FeedbackSnapshot ReadLatest() override {
    return inner_.ReadLatest();
  }

  irmv::hardware::WriteResult SendTorque(
      const irmv::hardware::TorqueOutput &output) override {
    return inner_.SendTorque(output);
  }

  irmv::hardware::WriteResult SendMotorCommand(const std::string &bus_name,
                                               std::uint32_t motor_id,
                                               std::uint8_t command) override {
    return inner_.SendMotorCommand(bus_name, motor_id, command);
  }

  void Close() noexcept override {
    inner_.Close();
    lease_.Release();
  }

 private:
  irmv::hardware::DriverPort &inner_;
  irmv::hardware::DeviceLease lease_;
};

}  // namespace

int main(int argc, char **argv) {
  std::vector<std::string> arguments;
  arguments.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
  for (int index = 1; index < argc; ++index) {
    arguments.emplace_back(argv[index]);
  }

  std::unique_ptr<irmv::hardware::DriverPort> port =
      irmv::hardware::MakeIrmvDriverPort();
  if (!port) {
    std::cerr << "irmv_maintenance: this build has no real driver port\n";
    return 1;
  }
  LeasedDriverPort leased(*port);
  irmv::maintenance::DriverPortMaintenancePort maintenance(leased);
  try {
    return irmv::maintenance::Run(arguments, leased, maintenance, std::cout);
  } catch (const std::exception &error) {
    std::cerr << "irmv_maintenance: " << error.what() << "\n";
    return 1;
  }
}
