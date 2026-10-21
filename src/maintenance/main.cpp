#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "irmv/hardware/device_lease.hpp"
#include "irmv/hardware/driver_port.hpp"
#include "irmv/maintenance/commands.hpp"
#include "irmv/maintenance/leased_driver_port.hpp"

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
  irmv::maintenance::LeasedDriverPort leased(*port);
  irmv::maintenance::DriverPortMaintenancePort maintenance(leased);
  try {
    return irmv::maintenance::Run(arguments, leased, maintenance, std::cout);
  } catch (const std::exception &error) {
    std::cerr << "irmv_maintenance: " << error.what() << "\n";
    return 1;
  }
}
