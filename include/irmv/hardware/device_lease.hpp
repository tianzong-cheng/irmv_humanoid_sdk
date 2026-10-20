#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "irmv/configuration/config.hpp"

namespace irmv::hardware {

// Exclusive ownership of the physical devices a configuration names.
//
// The lease is keyed by device identity, not by the configuration-local bus
// name: two documents that call the same USB device "can0" and "left" still
// contend for one lock. Every lock file is taken with flock(LOCK_EX | LOCK_NB)
// in a stable order; a partial acquisition releases everything before the
// exception leaves Acquire.
//
// The lock is advisory at the operating-system level: it excludes every process
// that honours it (the SDK runtime and the maintenance tool), not an unrelated
// program that opens the vendor device directly. It must be acquired before any
// device is opened, and it is held for the whole lifetime of the backend.
class DeviceLease {
 public:
  // Default lock directory: IRMV_LOCK_DIR when set, otherwise /run/lock/irmv.
  static std::filesystem::path DefaultDirectory();

  // Throws std::runtime_error naming the resource that could not be locked.
  static DeviceLease Acquire(const configuration::HardwareConfig &config);
  static DeviceLease Acquire(const configuration::HardwareConfig &config,
                             const std::filesystem::path &directory);

  DeviceLease() = default;
  ~DeviceLease();

  DeviceLease(const DeviceLease &) = delete;
  DeviceLease &operator=(const DeviceLease &) = delete;
  DeviceLease(DeviceLease &&other) noexcept;
  DeviceLease &operator=(DeviceLease &&other) noexcept;

  bool held() const noexcept { return !descriptors_.empty(); }
  // Identities held, in acquisition order. Stable and free of configuration
  // bus names, so a log line names the device.
  const std::vector<std::string> &identities() const noexcept {
    return identities_;
  }

  void Release() noexcept;

 private:
  std::vector<int> descriptors_;
  std::vector<std::string> identities_;
};

}  // namespace irmv::hardware
