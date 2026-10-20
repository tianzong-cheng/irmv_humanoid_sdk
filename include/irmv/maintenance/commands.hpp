#pragma once

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/hardware/driver_port.hpp"

namespace irmv::maintenance {

// The two commands the bench tool may send beyond a torque frame. They are
// deliberately the only extra writes the maintenance path has.
class MaintenancePort {
 public:
  virtual ~MaintenancePort() = default;
  virtual hardware::WriteResult Disable(const std::string &bus_name,
                                        std::uint32_t motor_id) = 0;
  virtual hardware::WriteResult ResetZero(const std::string &bus_name,
                                          std::uint32_t motor_id) = 0;
};

// Adapter over the same DriverPort the load test uses, so the tool and the
// runtime share one device owner and one protocol codec.
class DriverPortMaintenancePort final : public MaintenancePort {
 public:
  explicit DriverPortMaintenancePort(hardware::DriverPort &port)
      : port_(port) {}
  hardware::WriteResult Disable(const std::string &bus_name,
                                std::uint32_t motor_id) override;
  hardware::WriteResult ResetZero(const std::string &bus_name,
                                  std::uint32_t motor_id) override;

 private:
  hardware::DriverPort &port_;
};

// One maintenance program, five subcommands:
//
//   irmv_maintenance --config PATH <subcommand> [options]
//
// Every command parses the whole configuration and its own arguments before a
// device is opened. Write commands default to a dry run and require --execute.
int Run(const std::vector<std::string> &arguments, hardware::DriverPort &port,
        MaintenancePort &maintenance, std::ostream &out);

}  // namespace irmv::maintenance
