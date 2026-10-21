#include "irmv/maintenance/leased_driver_port.hpp"

namespace irmv::maintenance {

void LeasedDriverPort::Open(const configuration::HardwareConfig &hardware) {
  lease_ = hardware::DeviceLease::Acquire(hardware);
  try {
    inner_.Open(hardware);
  } catch (...) {
    lease_.Release();
    throw;
  }
}

protocol::FeedbackSnapshot LeasedDriverPort::ReadLatest() {
  return inner_.ReadLatest();
}

hardware::WriteResult LeasedDriverPort::SendTorque(
    const hardware::TorqueOutput &output) {
  return inner_.SendTorque(output);
}

hardware::WriteResult LeasedDriverPort::EnableMotor(const std::string &bus_name,
                                                    std::uint32_t motor_id) {
  return inner_.EnableMotor(bus_name, motor_id);
}

hardware::WriteResult LeasedDriverPort::SendMotorCommand(
    const std::string &bus_name, std::uint32_t motor_id, std::uint8_t command) {
  return inner_.SendMotorCommand(bus_name, motor_id, command);
}

void LeasedDriverPort::Close() noexcept {
  inner_.Close();
  lease_.Release();
}

}  // namespace irmv::maintenance
