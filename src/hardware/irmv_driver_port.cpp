#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware/runtime_io.h"
#include "irmv/hardware/driver_port.hpp"
#include "irmv/hardware/joint_conversion.hpp"

namespace irmv::hardware {
namespace {

namespace upstream = irmv_control::runtime_io;

// The upstream snapshot already reports invalid samples after its timeout; the
// runtime applies the configured feedback timeout as well. This value only
// bounds how long an unobserved sample can stay "valid" inside the adapter.
constexpr std::uint64_t kAdapterFeedbackTimeoutUs = 100'000;

}  // namespace

class IrmvDriverPort final : public DriverPort {
 public:
  void Open(const configuration::HardwareConfig &hardware) override {
    upstream::Options options;
    if (hardware.joints.empty() ||
        hardware.joints.size() > upstream::kMaxJoints) {
      throw std::runtime_error("driver adapter: invalid joint count");
    }
    if (hardware.buses.empty() || hardware.buses.size() > upstream::kMaxBuses) {
      throw std::runtime_error("driver adapter: invalid bus count");
    }
    options.feedback_timeout_us = kAdapterFeedbackTimeoutUs;

    std::unordered_map<std::string, std::uint32_t> bus_indices;
    for (std::size_t index = 0; index < hardware.buses.size(); ++index) {
      const configuration::BusConfig &bus = hardware.buses[index];
      upstream::BusOptions &target = options.buses[index];
      target.name = bus.name;
      target.device_type = bus.device_type;
      target.device_index = bus.device_index;
      target.channel = bus.channel;
      target.arbitration_bps = bus.arbitration_bps;
      target.data_bps = bus.data_bps;
      target.iso_mode = bus.iso_mode;
      target.resistance_enabled = bus.resistance_enabled;
      bus_indices.emplace(bus.name, static_cast<std::uint32_t>(index));
    }
    options.bus_count = hardware.buses.size();

    joints_ = hardware.joints;
    for (std::size_t index = 0; index < hardware.joints.size(); ++index) {
      const configuration::HardwareJointConfig &joint = hardware.joints[index];
      const auto bus = bus_indices.find(joint.bus);
      if (bus == bus_indices.end()) {
        throw std::runtime_error("driver adapter: joint \"" + joint.name +
                                 "\" names an unknown bus");
      }
      upstream::JointOptions &target = options.joints[index];
      target.bus_index = bus->second;
      target.motor_id = joint.motor_id;
      target.torque_constant = joint.torque_constant;
    }
    options.joint_count = hardware.joints.size();

    bus_indices_ = std::move(bus_indices);
    if (hardware.imu) {
      options.imu_present = true;
      options.imu.hardware_id = hardware.imu->hardware_id;
      options.imu.baudrate = hardware.imu->baudrate;
    }

    io_.Open(options);
    opened_ = true;
  }

  protocol::FeedbackSnapshot ReadLatest() override {
    protocol::FeedbackSnapshot snapshot;
    snapshot.count = static_cast<std::uint32_t>(joints_.size());
    if (!opened_) {
      return snapshot;
    }
    const upstream::Snapshot raw = io_.ReadLatest();
    snapshot.stamp_us = raw.stamp_us;
    snapshot.snapshot_seq = raw.seq;
    const std::size_t count =
        std::min<std::size_t>(joints_.size(), raw.joint_count);
    snapshot.count = static_cast<std::uint32_t>(count);
    for (std::size_t index = 0; index < count; ++index) {
      const configuration::HardwareJointConfig &mapping = joints_[index];
      const upstream::JointSample &sample = raw.joints[index];
      protocol::JointFeedback &joint = snapshot.joints[index];
      const double direction = static_cast<double>(mapping.direction);
      joint.valid = sample.valid;
      joint.connected = sample.connected;
      joint.q = direction * (sample.position_rad - mapping.encoder_offset) /
                mapping.reduction_ratio;
      joint.dq = direction * sample.speed_rad_s / mapping.reduction_ratio;
      joint.tau_est = direction * mapping.reduction_ratio *
                      mapping.torque_constant * sample.current_a;
      joint.rx_stamp_us = sample.rx_stamp_us;
      joint.freq_hz = sample.freq_hz;
      joint.error_code = sample.error_code;
    }
    snapshot.imu.valid = raw.imu.valid;
    snapshot.imu.orientation_rpy = raw.imu.orientation_rpy;
    snapshot.imu.angular_velocity = raw.imu.angular_velocity;
    snapshot.imu.linear_acceleration = raw.imu.linear_acceleration;
    snapshot.imu.rx_stamp_us = raw.imu.rx_stamp_us;
    return snapshot;
  }

  WriteResult SendTorque(const TorqueOutput &output) override {
    WriteResult result;
    if (!opened_) {
      result.error_code = 1;
      return result;
    }
    if (output.count != joints_.size()) {
      result.error_code = 2;
      return result;
    }
    upstream::TorqueFrame frame;
    frame.count = joints_.size();
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      const configuration::HardwareJointConfig &mapping = joints_[index];
      if (!std::isfinite(output.torque[index])) {
        result.error_code = 2;
        return result;
      }
      // Joint output torque -> motor side, once.
      frame.targets[index].torque_nm = static_cast<double>(mapping.direction) *
                                       output.torque[index] /
                                       mapping.reduction_ratio;
    }
    const upstream::SendResult sent = io_.SendTorque(frame);
    result.accepted = sent.accepted;
    result.error_code = sent.error_code;
    return result;
  }

  WriteResult SendMotorCommand(const std::string &bus_name,
                               std::uint32_t motor_id,
                               std::uint8_t command) override {
    WriteResult result;
    const auto bus = bus_indices_.find(bus_name);
    if (!opened_ || bus == bus_indices_.end() || motor_id == 0 ||
        motor_id > 31) {
      result.error_code = 1;
      return result;
    }
    const upstream::SendResult sent =
        io_.SendMotorCommand(bus->second, motor_id, command);
    result.accepted = sent.accepted;
    result.error_code = sent.error_code;
    return result;
  }

  void Close() noexcept override {
    io_.Close();
    opened_ = false;
  }

 private:
  upstream::RuntimeIo io_;
  std::vector<configuration::HardwareJointConfig> joints_;
  std::unordered_map<std::string, std::uint32_t> bus_indices_;
  bool opened_{false};
};

std::unique_ptr<DriverPort> MakeIrmvDriverPort() {
  return std::make_unique<IrmvDriverPort>();
}

}  // namespace irmv::hardware
