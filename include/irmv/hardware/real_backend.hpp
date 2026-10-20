#pragma once

#include <memory>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/hardware/device_lease.hpp"
#include "irmv/hardware/driver_port.hpp"

namespace irmv::hardware {

// The device-owning backend. It acquires the exclusive lease before the port is
// opened, keeps both for its lifetime, and never falls back to a fake device or
// to an old command cache when a write is refused.
//
// The backend does not implement PD, damping or the safety state machine: the
// runtime computes the target once and this class only transports it.
class RealBackend final : public Backend {
 public:
  RealBackend(const configuration::RuntimeConfig &config,
              std::unique_ptr<DriverPort> port);
  ~RealBackend() override;

  RealBackend(const RealBackend &) = delete;
  RealBackend &operator=(const RealBackend &) = delete;

  // Throws when the configuration carries no hardware table, when the lease is
  // held elsewhere, or when the port cannot open. A partial start closes the
  // port and releases the lease before the exception leaves.
  void Initialize() override;
  protocol::FeedbackSnapshot Read(protocol::TimeUs now_us) override;
  WriteResult Write(protocol::TimeUs now_us,
                    const TorqueOutput &output) override;
  void Close() override;

 private:
  configuration::RuntimeConfig config_;
  std::unique_ptr<DriverPort> port_;
  DeviceLease lease_;
  bool initialized_{false};
};

}  // namespace irmv::hardware
