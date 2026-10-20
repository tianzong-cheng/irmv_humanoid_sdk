#pragma once

namespace irmv::hardware {

// One joint's single, explicit conversion between rotor-side driver quantities
// and joint-output protocol quantities. The adapter is the only caller; the
// helper exists so the signs and ratios are testable without a device.
struct JointConversion {
  int direction{1};
  double reduction_ratio{1.0};
  double encoder_offset{0.0};
  double torque_constant{0.0};

  double ToJointPosition(double rotor_rad) const {
    return static_cast<double>(direction) * (rotor_rad - encoder_offset) /
           reduction_ratio;
  }

  double ToJointVelocity(double rotor_rad_s) const {
    return static_cast<double>(direction) * rotor_rad_s / reduction_ratio;
  }

  double ToJointTorque(double rotor_current_a) const {
    return static_cast<double>(direction) * reduction_ratio * torque_constant *
           rotor_current_a;
  }

  double ToMotorTorque(double joint_torque_nm) const {
    return static_cast<double>(direction) * joint_torque_nm / reduction_ratio;
  }
};

}  // namespace irmv::hardware
