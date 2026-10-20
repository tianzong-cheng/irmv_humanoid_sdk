#include "irmv/hardware/real_backend.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace irmv::hardware {
namespace {

constexpr std::uint32_t kBackendUnavailableError = 1;
constexpr std::uint32_t kInvalidOutputError = 3;

}  // namespace

RealBackend::RealBackend(const configuration::RuntimeConfig &config,
                         std::unique_ptr<DriverPort> port)
    : config_(config), port_(std::move(port)) {}

RealBackend::~RealBackend() { Close(); }

void RealBackend::Initialize() {
  Close();
  if (!config_.hardware) {
    throw std::runtime_error(
        "the real backend requires a robot hardware table");
  }
  if (!port_) {
    throw std::runtime_error(
        "the real backend was built without a driver port");
  }

  // Lease first: no device is opened and no frame is sent while another process
  // owns the hardware.
  DeviceLease lease = DeviceLease::Acquire(*config_.hardware);
  try {
    port_->Open(*config_.hardware);
  } catch (...) {
    port_->Close();
    throw;
  }
  lease_ = std::move(lease);
  initialized_ = true;
}

protocol::FeedbackSnapshot RealBackend::Read(protocol::TimeUs now_us) {
  static_cast<void>(now_us);
  protocol::FeedbackSnapshot snapshot;
  snapshot.count = static_cast<std::uint32_t>(config_.joint_count());
  if (!initialized_ || !port_) {
    return snapshot;
  }
  return port_->ReadLatest();
}

WriteResult RealBackend::Write(protocol::TimeUs now_us,
                               const TorqueOutput &output) {
  static_cast<void>(now_us);
  WriteResult result;
  if (!initialized_ || !port_) {
    result.error_code = kBackendUnavailableError;
    return result;
  }
  if (output.count != config_.joint_count()) {
    result.error_code = kInvalidOutputError;
    return result;
  }
  for (std::size_t index = 0; index < config_.joint_count(); ++index) {
    if (!std::isfinite(output.torque[index])) {
      result.error_code = kInvalidOutputError;
      return result;
    }
  }
  return port_->SendTorque(output);
}

void RealBackend::Close() {
  if (port_ && (initialized_ || lease_.held())) {
    port_->Close();
  }
  lease_.Release();
  initialized_ = false;
}

}  // namespace irmv::hardware
