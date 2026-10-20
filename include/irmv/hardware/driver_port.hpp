#pragma once

#include <memory>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::hardware {

// The SDK-side boundary to the vendor driver. Everything the upper layers see
// is the protocol's canonical joint order; the adapter owns the conversion from
// motor-side quantities and the sparse (bus, motor_id) addressing.
class DriverPort {
 public:
  virtual ~DriverPort() = default;

  // Opens every configured bus and starts reception. Throws on failure and
  // leaves nothing open.
  virtual void Open(const configuration::HardwareConfig &hardware) = 0;

  // Latest coherent snapshot in canonical joint order. Never blocks on a
  // device read and never advances a receive stamp by itself.
  virtual protocol::FeedbackSnapshot ReadLatest() = 0;

  // Sends one canonical torque request. Acceptance covers every bus the
  // request touches; a partial submission is not accepted.
  virtual WriteResult SendTorque(const TorqueOutput &output) = 0;

  virtual void Close() noexcept = 0;
};

// Only the real build links the upstream runtime I/O. A build without
// IRMV_ENABLE_REAL_BACKEND returns nullptr, and the factory refuses instead of
// silently falling back to a fake device.
std::unique_ptr<DriverPort> MakeIrmvDriverPort();

}  // namespace irmv::hardware
