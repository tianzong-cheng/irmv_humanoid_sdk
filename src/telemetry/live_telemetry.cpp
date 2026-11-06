#include "irmv/telemetry/live_telemetry.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

#include "irmv/configuration/sha256.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"
#include "irmv/telemetry/fault_record.hpp"
#include "irmv/telemetry/record.hpp"
#include "irmv/telemetry/velocity_record.hpp"

namespace irmv::telemetry {
namespace {

using Json = nlohmann::json;

constexpr std::size_t kMaxEnvelopeBytes = 64 * 1024;

struct SendHook {
  int (*hook)(void *context, std::string_view bytes){nullptr};
  void *context{nullptr};
};

std::mutex &HookMutex() {
  static std::mutex mutex;
  return mutex;
}

SendHook &Hook() {
  static SendHook hook;
  return hook;
}

Json EncodeDouble(double value) {
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value < 0 ? "-Inf" : "+Inf";
  }
  return value;
}

std::string HexId(const protocol::Id &id) {
  static const char kDigits[] = "0123456789abcdef";
  std::string text;
  text.reserve(2 * id.size());
  for (const std::uint8_t byte : id) {
    text.push_back(kDigits[byte >> 4]);
    text.push_back(kDigits[byte & 0x0FU]);
  }
  return text;
}

std::string HexBytes(std::string_view bytes) {
  static const char kDigits[] = "0123456789abcdef";
  std::string text;
  text.reserve(bytes.size() * 2);
  for (const char raw : bytes) {
    const unsigned char byte = static_cast<unsigned char>(raw);
    text.push_back(kDigits[byte >> 4]);
    text.push_back(kDigits[byte & 0x0FU]);
  }
  return text;
}

std::string MetadataString(const Json &metadata, const char *key,
                           const std::string &fallback) {
  if (metadata.contains(key) && metadata[key].is_string()) {
    return metadata[key].get<std::string>();
  }
  return fallback;
}

}  // namespace

struct LiveTelemetrySender::Impl {
  bool enabled{false};
  std::string error;
  std::string emitter_id;
  std::string run_path;
  std::string source;
  int socket_fd{-1};
  sockaddr_un address{};
  socklen_t address_length{0};
  std::uint64_t transport_seq{0};
  std::uint64_t sent{0};
  std::uint64_t failures{0};
  std::uint64_t oversize{0};
};

LiveTelemetrySender::LiveTelemetrySender(LiveTelemetryOptions options,
                                         std::string metadata_json) noexcept
    : impl_(std::make_unique<Impl>()) {
  if (options.socket_path.empty()) {
    impl_->error = "the live telemetry outlet is disabled";
    return;
  }
  const std::string path = options.socket_path.string();
  if (path.size() >= sizeof(impl_->address.sun_path)) {
    impl_->error = "the live telemetry socket path is too long";
    return;
  }
  const Json metadata = Json::parse(metadata_json, nullptr, false);
  if (metadata.is_discarded() || !metadata.is_object()) {
    impl_->error = "the live telemetry metadata is not a JSON object";
    return;
  }
  impl_->source = MetadataString(metadata, "source", "runtime");
  if (impl_->source != "runtime" && impl_->source != "application") {
    impl_->error = "the live telemetry source is not runtime or application";
    return;
  }
  impl_->run_path = MetadataString(metadata, "run_path", "");
  std::string clock;
  std::string boot_id;
  if (metadata.contains("clock") && metadata["clock"].is_object()) {
    clock = MetadataString(metadata["clock"], "clock", "");
    boot_id = MetadataString(metadata["clock"], "boot_id", "");
  }
  const std::string config_id = MetadataString(metadata, "config_id", "");
  const std::string layout_id = MetadataString(metadata, "layout_id", "");
  const std::string sample_hz = metadata.contains("sample_hz")
                                    ? metadata["sample_hz"].dump()
                                    : std::string();
  impl_->emitter_id =
      configuration::Sha256(impl_->source + "|" + impl_->run_path + "|" +
                            clock + "|" + boot_id + "|" + config_id + "|" +
                            layout_id + "|" + sample_hz)
          .substr(0, 32);
  impl_->address.sun_family = AF_UNIX;
  std::memcpy(impl_->address.sun_path, path.c_str(), path.size());
  impl_->address_length =
      static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + path.size() + 1);
  impl_->socket_fd =
      ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (impl_->socket_fd < 0) {
    impl_->error = std::string("the live telemetry socket could not open: ") +
                   std::strerror(errno);
    return;
  }
  impl_->enabled = true;
}

LiveTelemetrySender::~LiveTelemetrySender() {
  if (impl_ && impl_->socket_fd >= 0) {
    ::close(impl_->socket_fd);
    impl_->socket_fd = -1;
  }
}

bool LiveTelemetrySender::TrySend(std::string_view kind,
                                  std::string_view payload_json) noexcept {
  if (!impl_ || !impl_->enabled) {
    return false;
  }
  impl_->transport_seq += 1;
  if (payload_json.size() > kMaxEnvelopeBytes) {
    impl_->oversize += 1;
    return false;
  }
  Json payload = Json::parse(payload_json, nullptr, false);
  if (payload.is_discarded()) {
    impl_->failures += 1;
    return false;
  }
  Json envelope;
  envelope["version"] = 1;
  envelope["emitter_id"] = impl_->emitter_id;
  envelope["run_path"] = impl_->run_path;
  envelope["source"] = impl_->source;
  envelope["transport_seq"] = std::to_string(impl_->transport_seq);
  envelope["kind"] = std::string(kind);
  envelope["payload"] = std::move(payload);
  std::string bytes;
  try {
    bytes = envelope.dump(-1, ' ', false, Json::error_handler_t::replace);
  } catch (const std::exception &) {
    impl_->failures += 1;
    return false;
  }
  if (bytes.size() > kMaxEnvelopeBytes) {
    impl_->oversize += 1;
    return false;
  }
  int (*hook)(void *, std::string_view) = nullptr;
  void *hook_context = nullptr;
  {
    const std::lock_guard<std::mutex> lock(HookMutex());
    hook = Hook().hook;
    hook_context = Hook().context;
  }
  bool accepted = false;
  if (hook != nullptr) {
    accepted = hook(hook_context, bytes) == 0;
  } else {
    const ssize_t written =
        ::sendto(impl_->socket_fd, bytes.data(), bytes.size(), MSG_DONTWAIT,
                 reinterpret_cast<const sockaddr *>(&impl_->address),
                 impl_->address_length);
    accepted =
        written >= 0 && static_cast<std::size_t>(written) == bytes.size();
  }
  if (accepted) {
    impl_->sent += 1;
    return true;
  }
  impl_->failures += 1;
  return false;
}

bool LiveTelemetrySender::enabled() const noexcept {
  return impl_ && impl_->enabled;
}

std::uint64_t LiveTelemetrySender::sent() const noexcept {
  return impl_ ? impl_->sent : 0;
}

std::uint64_t LiveTelemetrySender::failures() const noexcept {
  return impl_ ? impl_->failures : 0;
}

std::uint64_t LiveTelemetrySender::oversize() const noexcept {
  return impl_ ? impl_->oversize : 0;
}

std::uint64_t LiveTelemetrySender::transport_seq() const noexcept {
  return impl_ ? impl_->transport_seq : 0;
}

const std::string &LiveTelemetrySender::error() const noexcept {
  static const std::string kEmpty;
  return impl_ ? impl_->error : kEmpty;
}

std::string EncodeLiveSample(const RecordSample &sample) {
  Json payload;
  payload["kind"] = "sample";
  payload["stamp_us"] = std::to_string(sample.stamp_us);
  payload["state_seq"] = std::to_string(sample.state_seq);
  payload["execution_state"] =
      static_cast<std::uint32_t>(sample.execution_state);
  payload["output_action"] = static_cast<std::uint32_t>(sample.output_action);
  payload["stop_reason"] = static_cast<std::uint32_t>(sample.stop_reason);
  payload["fault_id"] = std::to_string(sample.fault_id);
  payload["healthy"] = sample.healthy;
  payload["damping_eligible"] = sample.damping_eligible;
  payload["command_present"] = sample.command_present;
  payload["producer_id"] = HexId(sample.producer_id);
  payload["epoch"] = std::to_string(sample.epoch);
  payload["command_seq"] = std::to_string(sample.command_seq);
  payload["source_state_seq"] = std::to_string(sample.source_state_seq);
  payload["command_rx_us"] = std::to_string(sample.command_rx_us);
  payload["feedback_seq"] = std::to_string(sample.feedback_seq);
  payload["feedback_stamp_us"] = std::to_string(sample.feedback_stamp_us);
  payload["feedback_valid"] = sample.feedback_valid;
  payload["imu_valid"] = sample.imu_valid;
  payload["imu_rx_stamp_us"] = std::to_string(sample.imu_rx_stamp_us);
  payload["send_accepted"] = sample.send_accepted;
  payload["output_valid"] = sample.output_valid;
  payload["joint_count"] = sample.joint_count;
  const std::uint32_t count =
      std::min<std::uint32_t>(sample.joint_count, protocol::kMaxJoints);
  Json orientations = Json::array();
  Json angular = Json::array();
  Json linear = Json::array();
  for (std::size_t axis = 0; axis < 3; ++axis) {
    orientations.push_back(EncodeDouble(sample.imu_orientation_rpy[axis]));
    angular.push_back(EncodeDouble(sample.imu_angular_velocity[axis]));
    linear.push_back(EncodeDouble(sample.imu_linear_acceleration[axis]));
  }
  payload["imu_orientation_rpy"] = std::move(orientations);
  payload["imu_angular_velocity"] = std::move(angular);
  payload["imu_linear_acceleration"] = std::move(linear);
  for (const auto &column :
       {std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>("q",
                                                                    &sample.q),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>("dq",
                                                                    &sample.dq),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>(
            "tau_ff", &sample.tau_ff),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>("kp",
                                                                    &sample.kp),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>("kd",
                                                                    &sample.kd),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>(
            "measured_q", &sample.measured_q),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>(
            "measured_dq", &sample.measured_dq),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>(
            "measured_tau_est", &sample.measured_tau_est),
        std::pair<const char *,
                  const std::array<double, protocol::kMaxJoints> *>(
            "output_torque", &sample.output_torque)}) {
    Json values = Json::array();
    for (std::uint32_t index = 0; index < count; ++index) {
      values.push_back(EncodeDouble((*column.second)[index]));
    }
    payload[column.first] = std::move(values);
  }
  Json valid = Json::array();
  for (std::uint32_t index = 0; index < count; ++index) {
    valid.push_back(sample.joint_valid[index]);
  }
  payload["joint_valid"] = std::move(valid);
  return payload.dump(-1, ' ', false, Json::error_handler_t::replace);
}

std::string EncodeLiveFault(const RecordedFault &fault) {
  const protocol::FaultDetail &detail = fault.detail;
  Json payload;
  payload["kind"] = "fault";
  payload["record_seq"] = std::to_string(fault.record_seq);
  payload["runtime_id"] = HexId(fault.runtime_id);
  payload["state_seq"] = std::to_string(fault.state_seq);
  payload["fault_id"] = std::to_string(fault.fault_id);
  payload["reason"] = static_cast<std::uint32_t>(detail.reason);
  payload["condition"] = static_cast<std::uint32_t>(detail.condition);
  payload["condition_name"] = protocol::FaultConditionName(detail.condition);
  payload["joint_index"] = detail.joint_index == protocol::kNoJointIndex
                               ? Json(nullptr)
                               : Json(detail.joint_index);
  payload["error_code"] = detail.error_code;
  payload["observed_value"] = EncodeDouble(detail.observed_value);
  payload["limit"] = EncodeDouble(detail.limit);
  payload["sample_stamp_us"] = std::to_string(detail.sample_stamp_us);
  payload["detected_stamp_us"] = std::to_string(detail.detected_stamp_us);
  payload["output_action"] = static_cast<std::uint32_t>(detail.output_action);
  payload["message"] = std::string(detail.message.View());
  payload["message_hex"] = HexBytes(detail.message.View());
  return payload.dump(-1, ' ', false, Json::error_handler_t::replace);
}

std::string EncodeLiveVelocity(const VelocitySample &sample) {
  Json payload;
  payload["kind"] = "sample";
  payload["stamp_us"] = std::to_string(sample.stamp_us);
  payload["sample_seq"] = std::to_string(sample.sample_seq);
  payload["identity_known"] = sample.identity_known;
  payload["runtime_id"] =
      sample.identity_known ? HexId(sample.runtime_id) : std::string();
  payload["session_id"] =
      sample.identity_known ? HexId(sample.session_id) : std::string();
  payload["latest_state_seq"] = std::to_string(sample.latest_state_seq);
  payload["forward"] = EncodeDouble(sample.forward);
  payload["lateral"] = EncodeDouble(sample.lateral);
  payload["yaw"] = EncodeDouble(sample.yaw);
  return payload.dump(-1, ' ', false, Json::error_handler_t::replace);
}

void SetLiveTelemetrySendHookForTest(int (*hook)(void *context,
                                                 std::string_view bytes),
                                     void *context) noexcept {
  const std::lock_guard<std::mutex> lock(HookMutex());
  Hook().hook = hook;
  Hook().context = context;
}

}  // namespace irmv::telemetry
