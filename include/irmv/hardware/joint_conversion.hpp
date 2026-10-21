#pragma once

#include <cstdint>

#include "irmv/protocol/types.hpp"

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

// One untouched upstream sample, before any configured conversion. It is the
// driver's own all-default value for a joint that has never delivered a frame:
// unobserved, unconnected, no receive stamp, no driver status and every
// measured value zero.
struct RotorJointSample {
  bool valid{false};
  bool connected{false};
  double position_rad{0.0};
  double speed_rad_s{0.0};
  double current_a{0.0};
  std::uint64_t rx_stamp_us{0};
  double freq_hz{0.0};
  std::uint32_t error_code{0};
};

// Whether the sample carries no observation at all. This must be decided before
// applying an encoder offset: a conversion of the zero-valued default can
// produce a non-zero position through a non-zero offset, which the safety layer
// would then read as evidence of a measurement and an immediate startup fault.
// Anything that differs from the documented default - including malformed
// values such as NaN - is evidence and is converted and judged normally.
inline bool IsUntouchedRotorSample(const RotorJointSample &sample) noexcept {
  return !sample.valid && !sample.connected && sample.rx_stamp_us == 0 &&
         sample.error_code == 0 && sample.position_rad == 0.0 &&
         sample.speed_rad_s == 0.0 && sample.current_a == 0.0 &&
         sample.freq_hz == 0.0;
}

// The protocol feedback of one rotor sample. An untouched sample maps to the
// protocol's own all-default shape; every other sample, including one that
// merely claims absence while carrying values, is converted so that the safety
// layer can judge it.
inline protocol::JointFeedback ToProtocolFeedback(
    const JointConversion &conversion,
    const RotorJointSample &sample) noexcept {
  protocol::JointFeedback joint;
  if (IsUntouchedRotorSample(sample)) {
    return joint;
  }
  joint.valid = sample.valid;
  joint.connected = sample.connected;
  joint.q = conversion.ToJointPosition(sample.position_rad);
  joint.dq = conversion.ToJointVelocity(sample.speed_rad_s);
  joint.tau_est = conversion.ToJointTorque(sample.current_a);
  joint.rx_stamp_us = sample.rx_stamp_us;
  joint.freq_hz = sample.freq_hz;
  joint.error_code = sample.error_code;
  return joint;
}

}  // namespace irmv::hardware
