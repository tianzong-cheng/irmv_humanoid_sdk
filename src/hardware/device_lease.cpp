#include "irmv/hardware/device_lease.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace irmv::hardware {
namespace {

std::string Sanitize(std::string_view value) {
  std::string result;
  result.reserve(value.size());
  for (const char character : value) {
    const bool safe = (character >= 'a' && character <= 'z') ||
                      (character >= 'A' && character <= 'Z') ||
                      (character >= '0' && character <= '9') ||
                      character == '-' || character == '_' || character == '.';
    result.push_back(safe ? character : '_');
  }
  return result;
}

std::vector<std::string> Identities(
    const configuration::HardwareConfig &config) {
  std::vector<std::string> identities;
  for (const configuration::BusConfig &bus : config.buses) {
    identities.push_back("can-" + std::to_string(bus.device_type) + "-" +
                         std::to_string(bus.device_index));
  }
  if (config.imu) {
    identities.push_back("imu-" + Sanitize(config.imu->hardware_id));
  }
  std::sort(identities.begin(), identities.end());
  identities.erase(std::unique(identities.begin(), identities.end()),
                   identities.end());
  return identities;
}

}  // namespace

std::filesystem::path DeviceLease::DefaultDirectory() {
  const char *configured = std::getenv("IRMV_LOCK_DIR");
  if (configured != nullptr && configured[0] != '\0') {
    return std::filesystem::path(configured);
  }
  return std::filesystem::path("/run/lock/irmv");
}

DeviceLease DeviceLease::Acquire(
    const configuration::HardwareConfig &config) {
  return Acquire(config, DefaultDirectory());
}

DeviceLease DeviceLease::Acquire(
    const configuration::HardwareConfig &config,
    const std::filesystem::path &directory) {
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    throw std::runtime_error("cannot create the device lock directory " +
                             directory.string() + ": " + error.message());
  }

  DeviceLease lease;
  for (const std::string &identity : Identities(config)) {
    const std::filesystem::path path = directory / (identity + ".lock");
    // O_NOFOLLOW refuses a symlink planted in the lock directory; O_CLOEXEC
    // keeps the descriptor out of every child the runtime starts.
    const int descriptor =
        ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0660);
    if (descriptor < 0) {
      throw std::runtime_error("cannot open the lock for device " + identity +
                               " at " + path.string() + ": " +
                               std::strerror(errno));
    }
    if (::flock(descriptor, LOCK_EX | LOCK_NB) != 0) {
      const int lock_error = errno;
      ::close(descriptor);
      throw std::runtime_error("device " + identity +
                               " is already owned by another process (" +
                               std::strerror(lock_error) + ")");
    }
    lease.descriptors_.push_back(descriptor);
    lease.identities_.push_back(identity);
  }
  return lease;
}

DeviceLease::~DeviceLease() { Release(); }

DeviceLease::DeviceLease(DeviceLease &&other) noexcept
    : descriptors_(std::move(other.descriptors_)),
      identities_(std::move(other.identities_)) {
  other.descriptors_.clear();
  other.identities_.clear();
}

DeviceLease &DeviceLease::operator=(DeviceLease &&other) noexcept {
  if (this != &other) {
    Release();
    descriptors_ = std::move(other.descriptors_);
    identities_ = std::move(other.identities_);
    other.descriptors_.clear();
    other.identities_.clear();
  }
  return *this;
}

void DeviceLease::Release() noexcept {
  for (const int descriptor : descriptors_) {
    // Closing the descriptor releases the flock; LOCK_UN is explicit so the
    // intent is visible even if a future change keeps a descriptor around.
    static_cast<void>(::flock(descriptor, LOCK_UN));
    ::close(descriptor);
  }
  descriptors_.clear();
  identities_.clear();
}

}  // namespace irmv::hardware
