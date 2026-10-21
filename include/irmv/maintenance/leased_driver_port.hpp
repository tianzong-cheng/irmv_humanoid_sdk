#pragma once

#include <cstdint>
#include <string>

#include "irmv/hardware/device_lease.hpp"
#include "irmv/hardware/driver_port.hpp"

namespace irmv::maintenance {

// The maintenance tool owns the same devices as the runtime. The lease is
// acquired before the port opens the first device and released only after the
// port closed it. Every DriverPort operation is forwarded, including the
// explicit per-motor enable; a missing override would silently turn the load
// test into a permanent "enable refused".
class LeasedDriverPort final : public hardware::DriverPort {
 public:
  explicit LeasedDriverPort(hardware::DriverPort &inner) : inner_(inner) {}

  void Open(const configuration::HardwareConfig &hardware) override;
  protocol::FeedbackSnapshot ReadLatest() override;
  hardware::WriteResult SendTorque(
      const hardware::TorqueOutput &output) override;
  hardware::WriteResult EnableMotor(const std::string &bus_name,
                                    std::uint32_t motor_id) override;
  hardware::WriteResult SendMotorCommand(const std::string &bus_name,
                                         std::uint32_t motor_id,
                                         std::uint8_t command) override;
  void Close() noexcept override;

 private:
  hardware::DriverPort &inner_;
  hardware::DeviceLease lease_;
};

}  // namespace irmv::maintenance
