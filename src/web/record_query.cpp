#include "irmv/web/record_query.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "irmv/configuration/sha256.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"
#include "irmv/telemetry/fault_record.hpp"
#include "irmv/telemetry/record.hpp"
#include "irmv/telemetry/record_reader.hpp"
#include "irmv/telemetry/velocity_record.hpp"
#include "irmv/web/range_reduce.hpp"

namespace irmv::web {
namespace {

using Json = nlohmann::json;
using protocol::Id;
using protocol::TimeUs;

constexpr TimeUs kNeverUs = std::numeric_limits<TimeUs>::max();

enum class RunKind { Runtime, Application, Unknown };

struct RunEntry {
  std::string id;
  std::string name;
  std::filesystem::path directory;
  RunKind kind{RunKind::Unknown};
  bool readable{false};
  std::string error;
  Json metadata;
  // Runtime runs only: the reader opened for the metadata and chunk index.
  std::shared_ptr<telemetry::RecordReader> reader;
};

std::string HexId(const Id &id) {
  static const char kDigits[] = "0123456789abcdef";
  std::string text;
  text.reserve(2 * id.size());
  for (const std::uint8_t byte : id) {
    text.push_back(kDigits[byte >> 4]);
    text.push_back(kDigits[byte & 0x0FU]);
  }
  return text;
}

std::string Decimal(std::uint64_t value) { return std::to_string(value); }

// The canonical part of a signal identity: source, escaped joint name and
// field. A joint name may hold any byte, so everything outside a conservative
// unreserved set is percent-encoded; the field names this bridge emits are
// already safe. Array position is never part of the identity.
std::string EscapeJoint(std::string_view name) {
  static const char kDigits[] = "0123456789ABCDEF";
  std::string escaped;
  for (const char raw : name) {
    const unsigned char byte = static_cast<unsigned char>(raw);
    if (std::isalnum(byte) != 0 || raw == '_' || raw == '-' || raw == '.') {
      escaped.push_back(raw);
      continue;
    }
    escaped.push_back('%');
    escaped.push_back(kDigits[byte >> 4]);
    escaped.push_back(kDigits[byte & 0x0FU]);
  }
  return escaped;
}

std::string SignalId(std::string_view source, std::string_view joint_name,
                     std::string_view field) {
  return std::string(source) + "/" + EscapeJoint(joint_name) + "/" +
         std::string(field);
}

Json Signal(std::string_view source, std::string_view joint_name,
            std::string_view field, std::string_view label,
            std::string_view unit, std::string_view quantity,
            std::string_view value_kind, std::string_view validity) {
  Json signal;
  signal["id"] = SignalId(source, joint_name, field);
  signal["source"] = std::string(source);
  signal["joint_name"] = std::string(joint_name);
  signal["field"] = std::string(field);
  signal["label"] = std::string(label);
  signal["unit"] = std::string(unit);
  signal["quantity"] = std::string(quantity);
  signal["value_kind"] = std::string(value_kind);
  signal["validity"] = std::string(validity);
  return signal;
}

bool ReadJsonFile(const std::filesystem::path &path, Json *out) {
  std::ifstream file(path);
  if (!file) {
    return false;
  }
  *out = Json::parse(file, nullptr, false);
  return !out->is_discarded();
}

bool IsWithin(const std::filesystem::path &root,
              const std::filesystem::path &candidate) {
  auto root_part = root.begin();
  auto candidate_part = candidate.begin();
  for (; root_part != root.end(); ++root_part, ++candidate_part) {
    if (candidate_part == candidate.end() || *root_part != *candidate_part) {
      return false;
    }
  }
  return true;
}

std::string CatalogRevision(const std::vector<RunEntry> &runs) {
  std::string material;
  for (const RunEntry &run : runs) {
    std::error_code code;
    const auto stamp = std::filesystem::last_write_time(run.directory, code)
                           .time_since_epoch()
                           .count();
    material += run.name;
    material += '|';
    material += std::to_string(stamp);
    material += '|';
    material +=
        run.kind == RunKind::Runtime
            ? "runtime"
            : (run.kind == RunKind::Application ? "application" : "unknown");
    material += '|';
    material += run.readable ? "readable" : "unreadable";
    material += ';';
  }
  return configuration::Sha256(material).substr(0, 16);
}

const char *KindName(RunKind kind) {
  switch (kind) {
    case RunKind::Runtime:
      return "runtime";
    case RunKind::Application:
      return "application";
    default:
      return "unknown";
  }
}

struct JointField {
  const char *field;
  const char *label;
  const char *unit;
  const char *quantity;
  const char *validity;
};

constexpr JointField kJointFields[] = {
    {"q", "position target", "rad", "joint_position", "command_present"},
    {"dq", "velocity target", "rad_s", "joint_velocity", "command_present"},
    {"tau_ff", "feedforward torque", "Nm", "joint_torque", "command_present"},
    {"kp", "stiffness", "Nm_per_rad", "joint_stiffness", "command_present"},
    {"kd", "damping", "Nm_s_per_rad", "joint_damping", "command_present"},
    {"measured_q", "measured position", "rad", "joint_position",
     "feedback_valid+joint_valid"},
    {"measured_dq", "measured velocity", "rad_s", "joint_velocity",
     "feedback_valid+joint_valid"},
    {"measured_tau_est", "estimated torque", "Nm", "joint_torque",
     "feedback_valid+joint_valid"},
    {"output_torque", "output torque", "Nm", "joint_torque", "output_valid"},
};

struct ScalarField {
  const char *field;
  const char *label;
  const char *unit;
  const char *quantity;
  const char *value_kind;
};

constexpr ScalarField kRuntimeScalars[] = {
    {"stamp_us", "controller time", "us", "time", "scalar"},
    {"state_seq", "state sequence", "count", "sequence", "scalar"},
    {"command_rx_us", "command receive time", "us", "time", "scalar"},
    {"feedback_stamp_us", "feedback time", "us", "time", "scalar"},
    {"imu_rx_stamp_us", "IMU receive time", "us", "time", "scalar"},
    {"feedback_seq", "feedback sequence", "count", "sequence", "scalar"},
    {"command_seq", "command sequence", "count", "sequence", "scalar"},
    {"source_state_seq", "command source sequence", "count", "sequence",
     "scalar"},
    {"epoch", "authorization epoch", "count", "sequence", "scalar"},
    {"fault_id", "fault identity", "count", "sequence", "scalar"},
    {"joint_count", "joint count", "count", "count", "scalar"},
    {"execution_state", "execution state", "", "execution_state", "enum"},
    {"output_action", "output action", "", "output_action", "enum"},
    {"stop_reason", "stop reason", "", "stop_reason", "enum"},
    {"healthy", "healthy", "", "flag", "bool"},
    {"damping_eligible", "damping eligible", "", "flag", "bool"},
    {"command_present", "command present", "", "flag", "bool"},
    {"feedback_valid", "feedback valid", "", "flag", "bool"},
    {"imu_valid", "IMU valid", "", "flag", "bool"},
    {"send_accepted", "send accepted", "", "flag", "bool"},
    {"output_valid", "output valid", "", "flag", "bool"},
};

struct ImuField {
  const char *field;
  const char *label;
  const char *quantity;
};

constexpr ImuField kImuFields[] = {
    {"imu_orientation_roll", "IMU roll", "imu_orientation"},
    {"imu_orientation_pitch", "IMU pitch", "imu_orientation"},
    {"imu_orientation_yaw", "IMU yaw", "imu_orientation"},
    {"imu_angular_velocity_x", "IMU angular velocity x",
     "imu_angular_velocity"},
    {"imu_angular_velocity_y", "IMU angular velocity y",
     "imu_angular_velocity"},
    {"imu_angular_velocity_z", "IMU angular velocity z",
     "imu_angular_velocity"},
    {"imu_linear_acceleration_x", "IMU linear acceleration x",
     "imu_linear_acceleration"},
    {"imu_linear_acceleration_y", "IMU linear acceleration y",
     "imu_linear_acceleration"},
    {"imu_linear_acceleration_z", "IMU linear acceleration z",
     "imu_linear_acceleration"},
};

constexpr ScalarField kApplicationScalars[] = {
    {"stamp_us", "application time", "us", "time", "scalar"},
    {"sample_seq", "sample sequence", "count", "sequence", "scalar"},
    {"latest_state_seq", "latest state sequence", "count", "sequence",
     "scalar"},
    {"identity_known", "runtime identity known", "", "flag", "bool"},
};

struct AppValueField {
  const char *field;
  const char *label;
  const char *unit;
  const char *quantity;
};

constexpr AppValueField kApplicationValues[] = {
    {"forward", "forward velocity", "m_s", "body_forward_velocity"},
    {"lateral", "lateral velocity", "m_s", "body_lateral_velocity"},
    {"yaw", "yaw rate", "rad_s", "body_yaw_rate"},
};

Json RuntimeSignals(const telemetry::RunMetadata &metadata) {
  Json signals = Json::array();
  for (const std::string &joint : metadata.joints) {
    for (const JointField &field : kJointFields) {
      signals.push_back(Signal("runtime", joint, field.field, field.label,
                               field.unit, field.quantity, "scalar",
                               field.validity));
    }
  }
  for (const ScalarField &field : kRuntimeScalars) {
    signals.push_back(Signal("runtime", "", field.field, field.label,
                             field.unit, field.quantity, field.value_kind,
                             "always"));
  }
  Json units = Json::object();
  if (!metadata.units_json.empty()) {
    units = Json::parse(metadata.units_json, nullptr, false);
    if (units.is_discarded() || !units.is_object()) {
      units = Json::object();
    }
  }
  const auto unit_of = [&units](const char *key) {
    if (units.contains(key) && units[key].is_string()) {
      return units[key].get<std::string>();
    }
    return std::string();
  };
  const std::string orientation_unit = unit_of("imu_orientation_rpy");
  const std::string angular_unit = unit_of("imu_angular_velocity");
  const std::string linear_unit = unit_of("imu_linear_acceleration");
  for (const ImuField &field : kImuFields) {
    const std::string &unit =
        std::string(field.field).rfind("imu_orientation", 0) == 0
            ? orientation_unit
            : (std::string(field.field).rfind("imu_angular_velocity", 0) == 0
                   ? angular_unit
                   : linear_unit);
    signals.push_back(Signal("runtime", "", field.field, field.label, unit,
                             field.quantity, "scalar", "imu_valid"));
  }
  return signals;
}

Json ApplicationSignals() {
  Json signals = Json::array();
  for (const ScalarField &field : kApplicationScalars) {
    signals.push_back(Signal("application", "", field.field, field.label,
                             field.unit, field.quantity, field.value_kind,
                             "always"));
  }
  for (const AppValueField &field : kApplicationValues) {
    signals.push_back(Signal("application", "", field.field, field.label,
                             field.unit, field.quantity, "scalar", "always"));
  }
  return signals;
}

Json ClockOf(const Json &metadata) {
  if (metadata.contains("clock") && metadata["clock"].is_object()) {
    return metadata["clock"];
  }
  return Json::object();
}

std::string BootIdOf(const Json &metadata) {
  const Json clock = ClockOf(metadata);
  if (clock.contains("boot_id") && clock["boot_id"].is_string()) {
    return clock["boot_id"].get<std::string>();
  }
  return std::string();
}

std::string StringMember(const Json &object, const char *key) {
  if (object.contains(key) && object[key].is_string()) {
    return object[key].get<std::string>();
  }
  return std::string();
}

void AddSimulationContext(Json *context, const RunEntry &run) {
  if (!run.metadata.contains("simulation")) return;
  (*context)["simulation"] = run.metadata["simulation"];
  const auto relative =
      run.metadata["simulation"].value("sidecar", std::string());
  std::error_code code;
  const auto path =
      std::filesystem::weakly_canonical(run.directory / relative, code);
  telemetry::RecordClockIdentity clock;
  std::string summary, error;
  if (!relative.empty() && !std::filesystem::path(relative).is_absolute() &&
      !code && IsWithin(run.directory.parent_path(), path) &&
      telemetry::DecodeRecordClock(ClockOf(run.metadata).dump(), &clock)) {
    telemetry::ReadSimulationEvidence(path, clock, {}, &summary, &error);
    (*context)["simulation_evidence"] = Json::parse(summary);
  } else
    (*context)["simulation_evidence"] = {
        {"evidence_complete", false},
        {"error", "invalid simulation reference"}};
}

Json RuntimeContext(const RunEntry &run) {
  const telemetry::RecordReader &reader = *run.reader;
  Json context;
  context["run_id"] = run.id;
  context["kind"] = "runtime";
  context["name"] = run.name;
  context["readable"] = true;
  context["metadata"] = run.metadata;
  context["config_id"] = StringMember(run.metadata, "config_id");
  context["layout_id"] = StringMember(run.metadata, "layout_id");
  context["backend"] = StringMember(run.metadata, "backend");
  context["sdk_git_sha"] = StringMember(run.metadata, "sdk_git_sha");
  context["clock"] = ClockOf(run.metadata);
  AddSimulationContext(&context, run);
  context["control_hz"] = run.metadata.value("control_hz", 0.0);
  context["sample_hz"] = run.metadata.value("sample_hz", 0U);
  context["joint_count"] = reader.metadata().joint_count;
  context["joints"] = reader.metadata().joints;
  if (run.metadata.contains("units")) {
    context["units"] = run.metadata["units"];
  }
  if (run.metadata.contains("imu")) {
    context["imu"] = run.metadata["imu"];
  }
  context["signals"] = RuntimeSignals(reader.metadata());

  Json coverage;
  coverage["begin_us"] = nullptr;
  coverage["end_us"] = nullptr;
  coverage["begin_state_seq"] = nullptr;
  coverage["end_state_seq"] = nullptr;
  if (!reader.chunks().empty()) {
    const telemetry::ChunkInfo &first = reader.chunks().front();
    const telemetry::ChunkInfo &last = reader.chunks().back();
    coverage["begin_us"] = Decimal(first.first_stamp_us);
    coverage["end_us"] = Decimal(last.last_stamp_us);
    coverage["begin_state_seq"] = Decimal(first.first_state_seq);
    coverage["end_state_seq"] = Decimal(last.last_state_seq);
    context["stamp_us"] = Decimal(last.last_stamp_us);
    context["state_seq"] = Decimal(last.last_state_seq);
  } else {
    context["stamp_us"] = nullptr;
    context["state_seq"] = nullptr;
  }

  Json losses = Json::array();
  for (const telemetry::RecordGap &gap : reader.gaps()) {
    Json loss;
    loss["first_seq"] = Decimal(gap.first_seq);
    loss["last_seq"] = Decimal(gap.last_seq);
    loss["first_stamp_us"] = Decimal(gap.first_stamp_us);
    loss["last_stamp_us"] = Decimal(gap.last_stamp_us);
    loss["dropped"] = Decimal(gap.dropped);
    switch (gap.reason) {
      case telemetry::RecordGapReason::QueueFull:
        loss["reason"] = "queue_full";
        break;
      case telemetry::RecordGapReason::WriteFailed:
        loss["reason"] = "write_failed";
        break;
      case telemetry::RecordGapReason::Stopped:
        loss["reason"] = "stopped";
        break;
      default:
        loss["reason"] = "none";
        break;
    }
    losses.push_back(std::move(loss));
  }
  Json integrity;
  integrity["complete"] = reader.complete();
  integrity["truncated"] = false;
  integrity["corrupt"] = false;
  integrity["losses"] = std::move(losses);
  integrity["errors"] = Json::array();
  context["integrity"] = std::move(integrity);
  context["coverage"] = std::move(coverage);

  Json sessions = Json::array();
  Json identities = Json::array();
  for (const telemetry::RecordSessionEvent &event : reader.sessions()) {
    Json entry;
    entry["runtime_id"] = HexId(event.runtime_id);
    entry["session_id"] = HexId(event.session_id);
    entry["producer_id"] = HexId(event.producer_id);
    entry["epoch"] = Decimal(event.epoch);
    entry["stamp_us"] = Decimal(event.stamp_us);
    entry["state_seq"] = Decimal(event.state_seq);
    entry["model_sha256"] = std::string(event.model_sha256.data());
    sessions.push_back(entry);
    identities.push_back({{"runtime_id", HexId(event.runtime_id)},
                          {"session_id", HexId(event.session_id)}});
  }
  context["sessions"] = std::move(sessions);
  context["runtime_identities"] = std::move(identities);
  return context;
}

struct AppRead {
  bool ok{false};
  std::string error;
  telemetry::VelocityRecordRange range;
};

AppRead ReadApplication(const std::filesystem::path &directory) {
  AppRead read;
  read.ok = telemetry::ReadVelocityRange(directory, 0, kNeverUs, &read.range);
  if (!read.ok) {
    read.error = read.range.error;
  }
  return read;
}

Json ApplicationContext(const RunEntry &run, const AppRead &read) {
  const telemetry::VelocityRecordRange &range = read.range;
  Json context;
  context["run_id"] = run.id;
  context["kind"] = "application";
  context["name"] = run.name;
  context["readable"] = true;
  context["metadata"] = run.metadata;
  context["config_id"] = StringMember(run.metadata, "config_id");
  context["application_config_id"] =
      StringMember(run.metadata, "application_config_id");
  context["layout_id"] = StringMember(run.metadata, "layout_id");
  context["sdk_git_sha"] = StringMember(run.metadata, "sdk_git_sha");
  context["source"] = StringMember(run.metadata, "source");
  context["clock"] = ClockOf(run.metadata);
  AddSimulationContext(&context, run);
  context["sample_hz"] = run.metadata.value("sample_hz", 0U);
  context["joint_count"] = run.metadata.value("joint_count", 0U);
  if (run.metadata.contains("model")) {
    context["model"] = run.metadata["model"];
  }
  if (run.metadata.contains("units")) {
    context["units"] = run.metadata["units"];
  }
  context["signals"] = ApplicationSignals();

  Json coverage;
  coverage["begin_us"] = nullptr;
  coverage["end_us"] = nullptr;
  coverage["begin_state_seq"] = nullptr;
  coverage["end_state_seq"] = nullptr;
  if (!range.rows.empty()) {
    const telemetry::VelocitySample &first = range.rows.front();
    const telemetry::VelocitySample &last = range.rows.back();
    coverage["begin_us"] = Decimal(first.stamp_us);
    coverage["end_us"] = Decimal(last.stamp_us);
    coverage["begin_state_seq"] = Decimal(first.latest_state_seq);
    coverage["end_state_seq"] = Decimal(last.latest_state_seq);
    context["stamp_us"] = Decimal(last.stamp_us);
    context["state_seq"] = Decimal(last.latest_state_seq);
    Json identity;
    identity["known"] = last.identity_known;
    identity["runtime_id"] =
        last.identity_known ? HexId(last.runtime_id) : std::string();
    identity["session_id"] =
        last.identity_known ? HexId(last.session_id) : std::string();
    context["identity"] = std::move(identity);
  } else {
    context["stamp_us"] = nullptr;
    context["state_seq"] = nullptr;
    context["identity"] = {
        {"known", false}, {"runtime_id", ""}, {"session_id", ""}};
  }

  Json losses = Json::array();
  for (const telemetry::VelocityLossSpan &loss : range.losses) {
    Json entry;
    entry["first_sample_seq"] = Decimal(loss.first_sample_seq);
    entry["last_sample_seq"] = Decimal(loss.last_sample_seq);
    entry["first_stamp_us"] = Decimal(loss.first_stamp_us);
    entry["last_stamp_us"] = Decimal(loss.last_stamp_us);
    entry["dropped"] = Decimal(loss.dropped);
    switch (loss.reason) {
      case telemetry::VelocityLossReason::QueueFull:
        entry["reason"] = "queue_full";
        break;
      case telemetry::VelocityLossReason::WriteFailed:
        entry["reason"] = "write_failed";
        break;
      default:
        entry["reason"] = "none";
        break;
    }
    losses.push_back(std::move(entry));
  }
  Json integrity;
  integrity["complete"] = read.ok && range.complete;
  integrity["truncated"] = range.truncated;
  integrity["corrupt"] = false;
  integrity["losses"] = std::move(losses);
  integrity["errors"] = Json::array();
  if (!read.ok && !read.error.empty()) {
    integrity["errors"].push_back(read.error);
  }
  context["integrity"] = std::move(integrity);
  context["coverage"] = std::move(coverage);
  return context;
}

Json Association(const Json &runtime_context, const RunEntry &application,
                 const AppRead &app_read) {
  Json association;
  const std::string runtime_boot = BootIdOf(runtime_context["metadata"]);
  const std::string app_boot = BootIdOf(application.metadata);
  const bool boot_match =
      !runtime_boot.empty() && !app_boot.empty() && runtime_boot == app_boot;
  telemetry::RecordClockIdentity runtime_clock, app_clock;
  const bool clock_match =
      telemetry::DecodeRecordClock(ClockOf(runtime_context["metadata"]).dump(),
                                   &runtime_clock) &&
      telemetry::DecodeRecordClock(ClockOf(application.metadata).dump(),
                                   &app_clock) &&
      telemetry::SameRecordClock(runtime_clock, app_clock);
  std::set<std::pair<std::string, std::string>> runtime_identities;
  for (const Json &entry : runtime_context["runtime_identities"]) {
    runtime_identities.insert({entry.value("runtime_id", std::string()),
                               entry.value("session_id", std::string())});
  }
  bool identity_match = false;
  for (const telemetry::VelocitySample &row : app_read.range.rows) {
    if (!row.identity_known) {
      continue;
    }
    if (runtime_identities.count(
            {HexId(row.runtime_id), HexId(row.session_id)}) > 0) {
      identity_match = true;
      break;
    }
  }
  association["boot_id_match"] = boot_match;
  association["identity_match"] = identity_match;
  association["clock_identity_match"] = clock_match;
  association["ok"] = clock_match && identity_match;
  if (!clock_match) {
    association["reason"] = runtime_clock.clock_domain == "simulation" ||
                                    app_clock.clock_domain == "simulation"
                                ? "simulation clock identity mismatch"
                                : "boot_id mismatch";
  } else if (!identity_match) {
    association["reason"] = "no matching runtime/session identity";
  } else {
    association["reason"] = std::string();
  }
  association["runtime_boot_id"] = runtime_boot;
  association["application_boot_id"] = app_boot;
  return association;
}

bool ParseDecimal(const Json &object, const char *key, std::uint64_t *out) {
  if (!object.contains(key)) {
    return false;
  }
  const Json &value = object[key];
  if (value.is_string()) {
    const std::string text = value.get<std::string>();
    if (text.empty()) {
      return false;
    }
    std::uint64_t parsed = 0;
    const auto result =
        std::from_chars(text.data(), text.data() + text.size(), parsed);
    if (result.ec != std::errc() || result.ptr != text.data() + text.size()) {
      return false;
    }
    *out = parsed;
    return true;
  }
  if (value.is_number_unsigned()) {
    *out = value.get<std::uint64_t>();
    return true;
  }
  return false;
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

const char *OutputActionText(protocol::OutputAction action) {
  switch (action) {
    case protocol::OutputAction::ZeroTorque:
      return "ZeroTorque";
    case protocol::OutputAction::Damping:
      return "Damping";
    case protocol::OutputAction::Tracking:
      return "Tracking";
    default:
      return "Unknown";
  }
}

std::uint32_t RuntimeFieldBit(const std::string &field) {
  if (field == "q") return 1U << 0;
  if (field == "dq") return 1U << 1;
  if (field == "tau_ff") return 1U << 2;
  if (field == "kp") return 1U << 3;
  if (field == "kd") return 1U << 4;
  if (field == "measured_q") return 1U << 5;
  if (field == "measured_dq") return 1U << 6;
  if (field == "output_torque") return 1U << 7;
  if (field == "measured_tau_est") return 1U << 8;
  return 0;
}

struct ResolvedSignal {
  Json signal;
  std::string source;
  std::string joint_name;
  std::string field;
  std::string value_kind;
  std::string validity;
  int joint_index{-1};
};

const Json *FindCatalogSignal(const Json &signals, const std::string &id) {
  for (const Json &signal : signals) {
    if (signal.value("id", std::string()) == id) {
      return &signal;
    }
  }
  return nullptr;
}

bool ResolveRuntimeSignals(const telemetry::RunMetadata &metadata,
                           const Json &requested,
                           std::vector<ResolvedSignal> *out,
                           std::string *error) {
  const Json signals = RuntimeSignals(metadata);
  for (const Json &id_value : requested) {
    if (!id_value.is_string()) {
      *error = "signal_ids must be strings";
      return false;
    }
    const std::string id = id_value.get<std::string>();
    const Json *signal = FindCatalogSignal(signals, id);
    if (signal == nullptr) {
      *error = "unknown signal id " + id;
      return false;
    }
    ResolvedSignal resolved;
    resolved.signal = *signal;
    resolved.source = signal->value("source", std::string());
    resolved.joint_name = signal->value("joint_name", std::string());
    resolved.field = signal->value("field", std::string());
    resolved.value_kind = signal->value("value_kind", std::string());
    resolved.validity = signal->value("validity", std::string());
    if (!resolved.joint_name.empty()) {
      const auto found = std::find(metadata.joints.begin(),
                                   metadata.joints.end(), resolved.joint_name);
      if (found == metadata.joints.end()) {
        *error = "signal " + id + " names a joint this run does not have";
        return false;
      }
      resolved.joint_index = static_cast<int>(found - metadata.joints.begin());
    }
    out->push_back(std::move(resolved));
  }
  return true;
}

bool ResolveApplicationSignals(const Json &requested,
                               std::vector<ResolvedSignal> *out,
                               std::string *error) {
  const Json signals = ApplicationSignals();
  for (const Json &id_value : requested) {
    if (!id_value.is_string()) {
      *error = "signal_ids must be strings";
      return false;
    }
    const std::string id = id_value.get<std::string>();
    const Json *signal = FindCatalogSignal(signals, id);
    if (signal == nullptr) {
      *error = "unknown application signal id " + id;
      return false;
    }
    ResolvedSignal resolved;
    resolved.signal = *signal;
    resolved.source = "application";
    resolved.field = signal->value("field", std::string());
    resolved.value_kind = signal->value("value_kind", std::string());
    resolved.validity = "always";
    out->push_back(std::move(resolved));
  }
  return true;
}

bool RuntimeSignalValid(const telemetry::RecordSample &sample,
                        const ResolvedSignal &signal) {
  if (signal.validity == "command_present") {
    return sample.command_present;
  }
  if (signal.validity == "feedback_valid+joint_valid") {
    if (!sample.feedback_valid || signal.joint_index < 0) {
      return false;
    }
    return sample.joint_valid[static_cast<std::size_t>(signal.joint_index)];
  }
  if (signal.validity == "output_valid") {
    return sample.output_valid;
  }
  if (signal.validity == "imu_valid") {
    return sample.imu_valid;
  }
  return true;
}

bool RuntimeSignalValue(const telemetry::RecordSample &sample,
                        const ResolvedSignal &signal, double *value) {
  const std::string &field = signal.field;
  if (signal.joint_index >= 0) {
    const std::size_t index = static_cast<std::size_t>(signal.joint_index);
    if (field == "q") {
      *value = sample.q[index];
    } else if (field == "dq") {
      *value = sample.dq[index];
    } else if (field == "tau_ff") {
      *value = sample.tau_ff[index];
    } else if (field == "kp") {
      *value = sample.kp[index];
    } else if (field == "kd") {
      *value = sample.kd[index];
    } else if (field == "measured_q") {
      *value = sample.measured_q[index];
    } else if (field == "measured_dq") {
      *value = sample.measured_dq[index];
    } else if (field == "measured_tau_est") {
      *value = sample.measured_tau_est[index];
    } else if (field == "output_torque") {
      *value = sample.output_torque[index];
    } else {
      return false;
    }
    return true;
  }
  if (field == "stamp_us") {
    *value = static_cast<double>(sample.stamp_us);
  } else if (field == "state_seq") {
    *value = static_cast<double>(sample.state_seq);
  } else if (field == "command_rx_us") {
    *value = static_cast<double>(sample.command_rx_us);
  } else if (field == "feedback_stamp_us") {
    *value = static_cast<double>(sample.feedback_stamp_us);
  } else if (field == "imu_rx_stamp_us") {
    *value = static_cast<double>(sample.imu_rx_stamp_us);
  } else if (field == "feedback_seq") {
    *value = static_cast<double>(sample.feedback_seq);
  } else if (field == "command_seq") {
    *value = static_cast<double>(sample.command_seq);
  } else if (field == "source_state_seq") {
    *value = static_cast<double>(sample.source_state_seq);
  } else if (field == "epoch") {
    *value = static_cast<double>(sample.epoch);
  } else if (field == "fault_id") {
    *value = static_cast<double>(sample.fault_id);
  } else if (field == "joint_count") {
    *value = static_cast<double>(sample.joint_count);
  } else if (field == "execution_state") {
    *value =
        static_cast<double>(static_cast<std::uint32_t>(sample.execution_state));
  } else if (field == "output_action") {
    *value =
        static_cast<double>(static_cast<std::uint32_t>(sample.output_action));
  } else if (field == "stop_reason") {
    *value =
        static_cast<double>(static_cast<std::uint32_t>(sample.stop_reason));
  } else if (field == "healthy") {
    *value = sample.healthy ? 1.0 : 0.0;
  } else if (field == "damping_eligible") {
    *value = sample.damping_eligible ? 1.0 : 0.0;
  } else if (field == "command_present") {
    *value = sample.command_present ? 1.0 : 0.0;
  } else if (field == "feedback_valid") {
    *value = sample.feedback_valid ? 1.0 : 0.0;
  } else if (field == "imu_valid") {
    *value = sample.imu_valid ? 1.0 : 0.0;
  } else if (field == "send_accepted") {
    *value = sample.send_accepted ? 1.0 : 0.0;
  } else if (field == "output_valid") {
    *value = sample.output_valid ? 1.0 : 0.0;
  } else if (field == "imu_orientation_roll") {
    *value = sample.imu_orientation_rpy[0];
  } else if (field == "imu_orientation_pitch") {
    *value = sample.imu_orientation_rpy[1];
  } else if (field == "imu_orientation_yaw") {
    *value = sample.imu_orientation_rpy[2];
  } else if (field == "imu_angular_velocity_x") {
    *value = sample.imu_angular_velocity[0];
  } else if (field == "imu_angular_velocity_y") {
    *value = sample.imu_angular_velocity[1];
  } else if (field == "imu_angular_velocity_z") {
    *value = sample.imu_angular_velocity[2];
  } else if (field == "imu_linear_acceleration_x") {
    *value = sample.imu_linear_acceleration[0];
  } else if (field == "imu_linear_acceleration_y") {
    *value = sample.imu_linear_acceleration[1];
  } else if (field == "imu_linear_acceleration_z") {
    *value = sample.imu_linear_acceleration[2];
  } else {
    return false;
  }
  return true;
}

bool ApplicationSignalValue(const telemetry::VelocitySample &sample,
                            const std::string &field, double *value) {
  if (field == "stamp_us") {
    *value = static_cast<double>(sample.stamp_us);
  } else if (field == "sample_seq") {
    *value = static_cast<double>(sample.sample_seq);
  } else if (field == "latest_state_seq") {
    *value = static_cast<double>(sample.latest_state_seq);
  } else if (field == "identity_known") {
    *value = sample.identity_known ? 1.0 : 0.0;
  } else if (field == "forward") {
    *value = sample.forward;
  } else if (field == "lateral") {
    *value = sample.lateral;
  } else if (field == "yaw") {
    *value = sample.yaw;
  } else {
    return false;
  }
  return true;
}

struct SegmentTracker {
  protocol::TimeUs period_us{1};
  const std::vector<telemetry::RecordGap> *gaps{nullptr};
  std::uint32_t segment{0};
  bool have_last{false};
  protocol::TimeUs last_stamp_us{0};
  bool last_valid{false};

  void Step(protocol::TimeUs stamp_us, bool valid) {
    if (have_last) {
      bool broken = stamp_us <= last_stamp_us ||
                    stamp_us - last_stamp_us > (period_us * 3) / 2;
      if (!broken && gaps != nullptr) {
        for (const telemetry::RecordGap &gap : *gaps) {
          if (gap.first_stamp_us > last_stamp_us &&
              gap.first_stamp_us <= stamp_us) {
            broken = true;
            break;
          }
        }
      }
      if (!broken && valid != last_valid) {
        broken = true;
      }
      if (broken) {
        ++segment;
      }
    } else if (!valid) {
      ++segment;
    }
    have_last = true;
    last_stamp_us = stamp_us;
    last_valid = valid;
  }
};

struct ApplicationSegmentTracker {
  protocol::TimeUs period_us{1};
  const std::vector<telemetry::VelocityLossSpan> *losses{nullptr};
  std::uint32_t segment{0};
  bool have_last{false};
  protocol::TimeUs last_stamp_us{0};
  bool last_identity_known{false};
  protocol::Id last_runtime_id{};
  protocol::Id last_session_id{};

  void Step(const telemetry::VelocitySample &sample) {
    if (have_last) {
      bool broken = sample.stamp_us <= last_stamp_us ||
                    sample.stamp_us - last_stamp_us > (period_us * 3) / 2 ||
                    sample.identity_known != last_identity_known ||
                    sample.runtime_id != last_runtime_id ||
                    sample.session_id != last_session_id;
      if (!broken && losses != nullptr) {
        for (const telemetry::VelocityLossSpan &loss : *losses) {
          if (loss.first_stamp_us < sample.stamp_us &&
              loss.last_stamp_us >= last_stamp_us) {
            broken = true;
            break;
          }
        }
      }
      if (broken) {
        ++segment;
      }
    }
    have_last = true;
    last_stamp_us = sample.stamp_us;
    last_identity_known = sample.identity_known;
    last_runtime_id = sample.runtime_id;
    last_session_id = sample.session_id;
  }
};

Json ReducedSeriesJson(const Json &signal, const ReducedSeries &series) {
  Json output;
  output["signal"] = signal;
  output["input_points"] = Decimal(series.input_points);
  output["bucket_width_us"] = Decimal(series.bucket_width_us);
  output["points"] = Json::array();
  std::uint32_t segments = 0;
  bool have_segment = false;
  std::uint32_t last_segment = 0;
  for (const ReducedPoint &point : series.points) {
    Json entry;
    entry["t"] = Decimal(point.stamp_us);
    entry["seq"] = Decimal(point.sequence);
    entry["v"] = EncodeDouble(point.value);
    entry["valid"] = point.valid;
    entry["segment"] = point.segment;
    entry["agg"] = point.aggregation;
    output["points"].push_back(std::move(entry));
    if (!have_segment || point.segment != last_segment) {
      ++segments;
      have_segment = true;
      last_segment = point.segment;
    }
  }
  output["segments"] = segments;
  return output;
}

Json LossesJson(const std::vector<telemetry::RecordGap> &gaps) {
  Json losses = Json::array();
  for (const telemetry::RecordGap &gap : gaps) {
    Json entry;
    entry["first_seq"] = Decimal(gap.first_seq);
    entry["last_seq"] = Decimal(gap.last_seq);
    entry["first_stamp_us"] = Decimal(gap.first_stamp_us);
    entry["last_stamp_us"] = Decimal(gap.last_stamp_us);
    entry["dropped"] = Decimal(gap.dropped);
    entry["reason"] =
        gap.reason == telemetry::RecordGapReason::QueueFull
            ? "queue_full"
            : (gap.reason == telemetry::RecordGapReason::WriteFailed
                   ? "write_failed"
                   : (gap.reason == telemetry::RecordGapReason::Stopped
                          ? "stopped"
                          : "none"));
    losses.push_back(std::move(entry));
  }
  return losses;
}

Json ApplicationLossesJson(
    const std::vector<telemetry::VelocityLossSpan> &losses) {
  Json output = Json::array();
  for (const telemetry::VelocityLossSpan &loss : losses) {
    Json entry;
    entry["first_sample_seq"] = Decimal(loss.first_sample_seq);
    entry["last_sample_seq"] = Decimal(loss.last_sample_seq);
    entry["first_stamp_us"] = Decimal(loss.first_stamp_us);
    entry["last_stamp_us"] = Decimal(loss.last_stamp_us);
    entry["dropped"] = Decimal(loss.dropped);
    entry["reason"] =
        loss.reason == telemetry::VelocityLossReason::QueueFull
            ? "queue_full"
            : (loss.reason == telemetry::VelocityLossReason::WriteFailed
                   ? "write_failed"
                   : "none");
    output.push_back(std::move(entry));
  }
  return output;
}

bool BuildRuntimeRange(const RunEntry &run,
                       const telemetry::RunMetadata &metadata,
                       protocol::TimeUs begin_us, protocol::TimeUs end_us,
                       std::uint32_t pixel_width,
                       const std::vector<ResolvedSignal> &signals,
                       std::vector<ReducedSeries> *series,
                       std::vector<bool> *has_series, Json *integrity,
                       std::string *error) {
  const std::size_t count = signals.size();
  std::vector<RangeReducer> reducers;
  reducers.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    reducers.emplace_back(begin_us, end_us, pixel_width);
  }
  const protocol::TimeUs period_us =
      metadata.sample_hz == 0
          ? 1
          : (1000000ULL + metadata.sample_hz / 2U) / metadata.sample_hz;
  std::vector<SegmentTracker> trackers(count);
  std::vector<bool> invalid_pushed(count, false);
  for (std::size_t index = 0; index < count; ++index) {
    trackers[index].period_us = period_us;
    trackers[index].gaps = &run.reader->gaps();
  }
  telemetry::ChunkProjection projection;
  for (const ResolvedSignal &signal : signals) {
    if (signal.source != "runtime" || signal.joint_index < 0) {
      continue;
    }
    projection.joints.push_back(static_cast<std::uint32_t>(signal.joint_index));
    projection.fields_mask |= RuntimeFieldBit(signal.field);
  }
  std::sort(projection.joints.begin(), projection.joints.end());
  projection.joints.erase(
      std::unique(projection.joints.begin(), projection.joints.end()),
      projection.joints.end());
  telemetry::RecordRange summary;
  const bool visited = run.reader->VisitRangeProjected(
      begin_us, end_us, projection,
      [&](const telemetry::RecordSample &sample) {
        for (std::size_t index = 0; index < count; ++index) {
          if (signals[index].source != "runtime") {
            continue;
          }
          const bool valid = RuntimeSignalValid(sample, signals[index]);
          double value = 0.0;
          if (!RuntimeSignalValue(sample, signals[index], &value)) {
            continue;
          }
          trackers[index].Step(sample.stamp_us, valid);
          if (!valid && invalid_pushed[index]) {
            continue;
          }
          invalid_pushed[index] = !valid;
          reducers[index].Push(SeriesPoint{sample.stamp_us, sample.state_seq,
                                           valid ? value : 0.0, valid,
                                           trackers[index].segment});
        }
        return true;
      },
      &summary);
  if (!visited) {
    *error = "the runtime range could not be read";
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (signals[index].source == "runtime") {
      (*series)[index] = reducers[index].Finish();
      (*has_series)[index] = true;
    }
  }
  Json stream;
  stream["complete"] = run.reader->complete();
  stream["truncated"] = summary.truncated;
  stream["corrupt"] = summary.corrupt;
  stream["losses"] = LossesJson(summary.gaps);
  stream["errors"] = Json::array();
  if (!summary.error.empty()) {
    stream["errors"].push_back(summary.error);
  }
  (*integrity)["runtime"] = std::move(stream);
  return true;
}

bool BuildApplicationRange(const RunEntry &application,
                           protocol::TimeUs begin_us, protocol::TimeUs end_us,
                           std::uint32_t pixel_width,
                           const std::vector<ResolvedSignal> &signals,
                           std::vector<ReducedSeries> *series,
                           std::vector<bool> *has_series, Json *integrity,
                           std::string *error) {
  const std::size_t count = signals.size();
  std::vector<RangeReducer> reducers;
  reducers.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    reducers.emplace_back(begin_us, end_us, pixel_width);
  }
  const std::uint32_t sample_hz = application.metadata.value("sample_hz", 100U);
  const protocol::TimeUs period_us =
      sample_hz == 0 ? 1 : (1000000ULL + sample_hz / 2U) / sample_hz;
  // Read the loss table before the rows: the segment rules need every indexed
  // loss, not only the ones that happen to be seen while streaming.
  telemetry::VelocityRecordRange meta;
  if (!telemetry::ReadVelocityRange(application.directory, 1, 0, &meta)) {
    *error = meta.error;
    return false;
  }
  std::vector<ApplicationSegmentTracker> trackers(count);
  for (std::size_t index = 0; index < count; ++index) {
    trackers[index].period_us = period_us;
    trackers[index].losses = &meta.losses;
  }
  telemetry::VelocityRecordRange summary;
  const bool visited = telemetry::VisitVelocityRange(
      application.directory, begin_us, end_us,
      [&](const telemetry::VelocitySample &sample) {
        for (std::size_t index = 0; index < count; ++index) {
          if (signals[index].source != "application") {
            continue;
          }
          double value = 0.0;
          if (!ApplicationSignalValue(sample, signals[index].field, &value)) {
            continue;
          }
          trackers[index].Step(sample);
          reducers[index].Push(SeriesPoint{sample.stamp_us, sample.sample_seq,
                                           value, true,
                                           trackers[index].segment});
        }
        return true;
      },
      &summary);
  if (!visited) {
    *error = summary.error.empty() ? "the application range could not be read"
                                   : summary.error;
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (signals[index].source == "application") {
      (*series)[index] = reducers[index].Finish();
      (*has_series)[index] = true;
    }
  }
  summary.losses = meta.losses;
  Json stream;
  stream["complete"] = summary.complete;
  stream["truncated"] = summary.truncated;
  stream["corrupt"] = false;
  stream["losses"] = ApplicationLossesJson(meta.losses);
  stream["errors"] = Json::array();
  if (!summary.error.empty()) {
    stream["errors"].push_back(summary.error);
  }
  (*integrity)["application"] = std::move(stream);
  return true;
}

struct CursorResult {
  bool found{false};
  double value{0.0};
  protocol::TimeUs stamp_us{0};
  protocol::TimeUs age_us{0};
  std::string reason;
};

CursorResult CursorRuntime(const RunEntry &run, const ResolvedSignal &signal,
                           protocol::TimeUs cursor_us) {
  CursorResult result;
  result.reason = "no recorded sample at or before the cursor";
  const std::vector<telemetry::ChunkInfo> &chunks = run.reader->chunks();
  for (auto chunk = chunks.rbegin(); chunk != chunks.rend(); ++chunk) {
    if (chunk->first_stamp_us > cursor_us) {
      continue;
    }
    const protocol::TimeUs stop =
        std::min<protocol::TimeUs>(chunk->last_stamp_us, cursor_us) + 1;
    bool saw = false;
    bool valid = false;
    double value = 0.0;
    protocol::TimeUs stamp = 0;
    run.reader->VisitRange(
        chunk->first_stamp_us, stop,
        [&](const telemetry::RecordSample &sample) {
          saw = true;
          stamp = sample.stamp_us;
          valid = RuntimeSignalValid(sample, signal);
          if (!RuntimeSignalValue(sample, signal, &value)) {
            valid = false;
          }
          return true;
        },
        nullptr);
    if (!saw) {
      continue;
    }
    if (!valid) {
      result.reason = "the latest sample at or before the cursor is invalid";
      return result;
    }
    for (const telemetry::RecordGap &gap : run.reader->gaps()) {
      if (gap.first_stamp_us > stamp && gap.first_stamp_us <= cursor_us) {
        result.reason = "the cursor crosses a recorded gap";
        return result;
      }
    }
    result.found = true;
    result.value = value;
    result.stamp_us = stamp;
    result.age_us = cursor_us - stamp;
    result.reason.clear();
    return result;
  }
  return result;
}

VelocityField ToVelocityField(const std::string &field, bool *ok) {
  *ok = true;
  if (field == "forward") {
    return VelocityField::Forward;
  }
  if (field == "lateral") {
    return VelocityField::Lateral;
  }
  if (field == "yaw") {
    return VelocityField::Yaw;
  }
  *ok = false;
  return VelocityField::Forward;
}

CursorResult CursorApplication(const RunEntry &application,
                               const ResolvedSignal &signal,
                               protocol::TimeUs cursor_us) {
  CursorResult result;
  const std::uint32_t sample_hz = application.metadata.value("sample_hz", 100U);
  const protocol::TimeUs period_us =
      sample_hz == 0 ? 1 : (1000000ULL + sample_hz / 2U) / sample_hz;
  const protocol::TimeUs window_begin =
      cursor_us > period_us * 4 ? cursor_us - period_us * 4 : 0;
  telemetry::VelocityRecordRange range;
  if (!telemetry::ReadVelocityRange(application.directory, window_begin,
                                    cursor_us + 1, &range)) {
    result.reason = range.error;
    return result;
  }
  if (signal.field == "stamp_us" || signal.field == "sample_seq" ||
      signal.field == "latest_state_seq" || signal.field == "identity_known") {
    if (range.rows.empty()) {
      result.reason = "no application row at or before the cursor";
      return result;
    }
    const telemetry::VelocitySample &row = range.rows.back();
    double value = 0.0;
    ApplicationSignalValue(row, signal.field, &value);
    result.found = true;
    result.value = value;
    result.stamp_us = row.stamp_us;
    result.age_us = cursor_us - row.stamp_us;
    return result;
  }
  bool known = false;
  const VelocityField field = ToVelocityField(signal.field, &known);
  if (!known) {
    result.reason = "the field is not a held application value";
    return result;
  }
  const HoldValue hold =
      HoldVelocityAt(range.rows, range.losses, cursor_us, sample_hz, field);
  result.found = hold.found;
  result.value = hold.value;
  result.stamp_us = hold.stamp_us;
  result.age_us = hold.age_us;
  result.reason = hold.reason;
  return result;
}

Json CursorSeriesJson(const Json &signal, protocol::TimeUs cursor_us,
                      const CursorResult &result) {
  Json entry;
  entry["signal"] = signal;
  entry["cursor_us"] = Decimal(cursor_us);
  entry["valid"] = result.found;
  entry["value"] = result.found ? EncodeDouble(result.value) : Json(nullptr);
  entry["sample_stamp_us"] =
      result.found ? Json(Decimal(result.stamp_us)) : Json(nullptr);
  entry["age_us"] = result.found ? Json(Decimal(result.age_us)) : Json(nullptr);
  entry["reason"] = result.reason;
  return entry;
}

Json EncodeFault(const telemetry::RecordedFault &fault,
                 const telemetry::RunMetadata &metadata) {
  const protocol::FaultDetail &detail = fault.detail;
  Json event;
  event["kind"] = "fault";
  event["event_id"] = Decimal(fault.record_seq);
  event["record_seq"] = Decimal(fault.record_seq);
  event["runtime_id"] = HexId(fault.runtime_id);
  event["state_seq"] = Decimal(fault.state_seq);
  event["fault_id"] = Decimal(fault.fault_id);
  event["reason"] = static_cast<std::uint32_t>(detail.reason);
  event["reason_name"] = protocol::FaultReasonName(detail.reason);
  event["condition"] = static_cast<std::uint32_t>(detail.condition);
  event["condition_name"] = protocol::FaultConditionName(detail.condition);
  if (detail.joint_index == protocol::kNoJointIndex) {
    event["joint_index"] = nullptr;
    event["joint_name"] = nullptr;
  } else {
    event["joint_index"] = detail.joint_index;
    if (detail.joint_index < metadata.joints.size()) {
      event["joint_name"] = metadata.joints[detail.joint_index];
    } else {
      event["joint_name"] = nullptr;
    }
  }
  event["error_code"] = detail.error_code;
  event["observed_value"] = EncodeDouble(detail.observed_value);
  event["limit"] = EncodeDouble(detail.limit);
  event["sample_stamp_us"] = Decimal(detail.sample_stamp_us);
  event["detected_stamp_us"] = Decimal(detail.detected_stamp_us);
  event["output_action"] = static_cast<std::uint32_t>(detail.output_action);
  event["output_action_name"] = OutputActionText(detail.output_action);
  event["message"] = std::string(detail.message.View());
  event["message_hex"] = HexBytes(detail.message.View());
  return event;
}

bool ResolveRequestedSignals(const RunEntry *run, const RunEntry *app,
                             const Json &requested,
                             std::vector<ResolvedSignal> *out,
                             std::string *error) {
  if (!requested.is_array()) {
    *error = "signal_ids must be an array";
    return false;
  }
  if (requested.empty()) {
    *error = "signal_ids is empty";
    return false;
  }
  if (requested.size() > 128) {
    *error = "signal_ids holds more than 128 signals";
    return false;
  }
  for (const Json &value : requested) {
    if (!value.is_string()) {
      *error = "signal_ids must be strings";
      return false;
    }
    const std::string id = value.get<std::string>();
    std::vector<ResolvedSignal> one;
    if (id.rfind("runtime/", 0) == 0) {
      if (run == nullptr || run->kind != RunKind::Runtime) {
        *error = "signal " + id +
                 " is a runtime signal but no runtime run was "
                 "given";
        return false;
      }
      Json single = Json::array({id});
      if (!ResolveRuntimeSignals(run->reader->metadata(), single, &one,
                                 error)) {
        return false;
      }
    } else if (id.rfind("application/", 0) == 0) {
      if (app == nullptr) {
        *error = "signal " + id + " needs an application run";
        return false;
      }
      Json single = Json::array({id});
      if (!ResolveApplicationSignals(single, &one, error)) {
        return false;
      }
    } else {
      *error = "unknown signal id " + id;
      return false;
    }
    out->push_back(std::move(one.front()));
  }
  return true;
}

bool AnySource(const std::vector<ResolvedSignal> &signals, const char *source) {
  for (const ResolvedSignal &signal : signals) {
    if (signal.source == source) {
      return true;
    }
  }
  return false;
}

std::string ErrorReply(const std::string &request_id,
                       const std::string &error) {
  Json reply;
  reply["request_id"] = request_id;
  reply["ok"] = false;
  reply["error"] = error;
  return reply.dump(-1, ' ', false, Json::error_handler_t::replace);
}

}  // namespace

struct RecordQuery::Impl {
  struct CachedReader {
    std::shared_ptr<telemetry::RecordReader> reader;
    std::filesystem::file_time_type index_time{};
    std::uintmax_t index_size{0};
  };

  std::filesystem::path root;
  bool root_ok{false};
  std::string root_error;
  mutable std::map<std::string, CachedReader> readers;

  explicit Impl(std::filesystem::path records_root)
      : root(std::move(records_root)) {
    std::error_code code;
    const std::filesystem::path canonical =
        std::filesystem::canonical(root, code);
    if (code || !std::filesystem::is_directory(canonical, code) || code) {
      root_error = "the records root is not a readable directory";
      return;
    }
    root = canonical;
    root_ok = true;
  }

  std::vector<RunEntry> Scan() const {
    std::vector<RunEntry> runs;
    if (!root_ok) {
      return runs;
    }
    std::error_code code;
    for (std::filesystem::directory_iterator iterator(root, code);
         !code && iterator != std::filesystem::directory_iterator();
         iterator.increment(code)) {
      if (!iterator->is_directory(code) || code) {
        code.clear();
        continue;
      }
      std::error_code canonical_code;
      const std::filesystem::path directory =
          std::filesystem::canonical(iterator->path(), canonical_code);
      if (canonical_code || !IsWithin(root, directory)) {
        // A symlink that leaves the configured root is not a run of this
        // workbench, whatever its name says.
        continue;
      }
      Json metadata;
      if (!ReadJsonFile(directory / std::string(telemetry::kRecordMetadataFile),
                        &metadata) ||
          !metadata.is_object()) {
        continue;
      }
      RunEntry run;
      run.directory = directory;
      run.name = directory.filename().string();
      run.metadata = std::move(metadata);
      run.id = configuration::Sha256(
                   directory.lexically_relative(root).generic_string())
                   .substr(0, 32);

      const std::string schema = StringMember(run.metadata, "schema");
      if (!schema.empty()) {
        if (schema != std::string(telemetry::kVelocitySchema)) {
          run.error = "the run declares unsupported schema " + schema;
        } else if (run.metadata.value("version", 0U) !=
                   telemetry::kVelocitySchemaVersion) {
          run.error = "the run declares application velocity version " +
                      run.metadata["version"].dump() +
                      "; this build implements " +
                      std::to_string(telemetry::kVelocitySchemaVersion);
        } else {
          run.kind = RunKind::Application;
          run.readable = true;
        }
      } else if (run.metadata.contains("schema_version") &&
                 run.metadata["schema_version"].is_number_integer()) {
        if (run.metadata["schema_version"].get<std::uint64_t>() !=
            telemetry::kRecordSchemaVersion) {
          run.error = "the run declares schema version " +
                      run.metadata["schema_version"].dump() +
                      "; this build implements " +
                      std::to_string(telemetry::kRecordSchemaVersion);
        } else {
          run.kind = RunKind::Runtime;
          const std::filesystem::path index_path =
              directory / std::string(telemetry::kRecordIndexFile);
          std::error_code index_code;
          const auto index_time =
              std::filesystem::last_write_time(index_path, index_code);
          const std::uintmax_t index_size =
              std::filesystem::file_size(index_path, index_code);
          const std::string key = directory.string();
          const auto cached = readers.find(key);
          if (!index_code && cached != readers.end() &&
              cached->second.index_time == index_time &&
              cached->second.index_size == index_size) {
            run.reader = cached->second.reader;
            run.readable = run.reader != nullptr;
            if (!run.readable) {
              run.error = "the run index could not be opened";
            }
          } else {
            auto reader = std::make_shared<telemetry::RecordReader>();
            std::string open_error;
            if (telemetry::RecordReader::Open(directory, reader.get(),
                                              &open_error)) {
              run.readable = true;
              run.reader = reader;
              readers[key] = CachedReader{reader, index_time, index_size};
            } else {
              run.error = open_error;
              readers.erase(key);
            }
          }
        }
      } else if (run.metadata.contains("schema_version")) {
        run.error = "the run declares a non-integer schema version";
      } else {
        run.error = "the run has no schema identity";
      }
      runs.push_back(std::move(run));
    }
    std::sort(runs.begin(), runs.end(),
              [](const RunEntry &left, const RunEntry &right) {
                return left.name < right.name;
              });
    return runs;
  }

  static const RunEntry *Find(const std::vector<RunEntry> &runs,
                              const std::string &id) {
    for (const RunEntry &run : runs) {
      if (run.id == id) {
        return &run;
      }
    }
    return nullptr;
  }
};

RecordQuery::RecordQuery(std::filesystem::path records_root)
    : impl_(std::make_unique<Impl>(std::move(records_root))) {}

RecordQuery::~RecordQuery() = default;

std::string RecordQuery::Execute(std::string_view request_json) {
  Json request = Json::parse(request_json, nullptr, false);
  if (request.is_discarded() || !request.is_object()) {
    return ErrorReply("", "the request is not a JSON object");
  }
  std::string request_id;
  if (request.contains("request_id") && request["request_id"].is_string()) {
    request_id = request["request_id"].get<std::string>();
  }
  if (!impl_->root_ok) {
    return ErrorReply(request_id, impl_->root_error);
  }
  const std::string op = StringMember(request, "op");
  if (op.empty()) {
    return ErrorReply(request_id, "the request has no op");
  }
  const std::string run_id = StringMember(request, "run_id");
  const std::string app_run_id = StringMember(request, "app_run_id");

  const std::vector<RunEntry> runs = impl_->Scan();
  const std::string revision = CatalogRevision(runs);

  if (op == "describe" && run_id.empty()) {
    Json reply;
    reply["request_id"] = request_id;
    reply["ok"] = true;
    reply["catalog_revision"] = revision;
    Json catalog = Json::array();
    for (const RunEntry &run : runs) {
      Json entry;
      entry["run_id"] = run.id;
      entry["name"] = run.name;
      entry["kind"] = KindName(run.kind);
      entry["readable"] = run.readable;
      if (!run.error.empty()) {
        entry["error"] = run.error;
      }
      if (run.reader != nullptr && !run.reader->chunks().empty()) {
        entry["begin_us"] =
            Decimal(run.reader->chunks().front().first_stamp_us);
        entry["end_us"] = Decimal(run.reader->chunks().back().last_stamp_us);
      } else {
        entry["begin_us"] = nullptr;
        entry["end_us"] = nullptr;
      }
      if (run.reader != nullptr) {
        entry["complete"] = run.reader->complete();
      } else {
        entry["complete"] = nullptr;
      }
      catalog.push_back(std::move(entry));
    }
    reply["runs"] = std::move(catalog);
    reply["context"] = {{"run_count", runs.size()}};
    return reply.dump(-1, ' ', false, Json::error_handler_t::replace);
  }

  if (op != "describe" && op != "context" && op != "range" && op != "cursor" &&
      op != "events") {
    return ErrorReply(request_id,
                      "op '" + op + "' is not implemented by this build");
  }
  if (run_id.empty()) {
    return ErrorReply(request_id, "the request has no run_id");
  }
  const RunEntry *run = Impl::Find(runs, run_id);
  if (run == nullptr) {
    return ErrorReply(request_id, "unknown run_id");
  }
  if (!run->readable) {
    return ErrorReply(request_id, run->error.empty() ? "the run is not readable"
                                                     : run->error);
  }

  const RunEntry *app = nullptr;
  if (run->kind == RunKind::Application) {
    if (!app_run_id.empty() && app_run_id != run_id) {
      return ErrorReply(request_id,
                        "an application run cannot select another application "
                        "run");
    }
    app = run;
  } else if (!app_run_id.empty()) {
    app = Impl::Find(runs, app_run_id);
    if (app == nullptr || app->kind != RunKind::Application) {
      return ErrorReply(request_id, "unknown application run_id");
    }
    if (!app->readable) {
      return ErrorReply(request_id, app->error.empty()
                                        ? "the application run is not readable"
                                        : app->error);
    }
  }

  if (op == "describe" || op == "context") {
    Json context;
    if (run->kind == RunKind::Runtime) {
      context = RuntimeContext(*run);
    } else {
      const AppRead read = ReadApplication(run->directory);
      context = ApplicationContext(*run, read);
      if (!read.ok) {
        return ErrorReply(request_id, read.error.empty()
                                          ? "the application run is not "
                                            "readable"
                                          : read.error);
      }
    }
    if (app != nullptr && run->kind == RunKind::Runtime) {
      const AppRead app_read = ReadApplication(app->directory);
      if (!app_read.ok) {
        return ErrorReply(request_id, app_read.error.empty()
                                          ? "the application run is not "
                                            "readable"
                                          : app_read.error);
      }
      Json app_context = ApplicationContext(*app, app_read);
      context["association"] = Association(context, *app, app_read);
      context["application"] = std::move(app_context);
    }
    Json reply;
    reply["request_id"] = request_id;
    reply["ok"] = true;
    reply["catalog_revision"] = revision;
    reply["series"] = Json::array();
    reply["events"] = Json::array();
    reply["integrity"] = context["integrity"];
    reply["coverage"] = context["coverage"];
    reply["context"] = std::move(context);
    return reply.dump(-1, ' ', false, Json::error_handler_t::replace);
  }

  if (op == "events") {
    if (run->kind != RunKind::Runtime) {
      return ErrorReply(request_id, "fault events belong to a runtime run");
    }
    protocol::TimeUs begin_us = 0;
    protocol::TimeUs end_us = kNeverUs;
    (void)ParseDecimal(request, "begin_us", &begin_us);
    (void)ParseDecimal(request, "end_us", &end_us);
    std::uint64_t max_rows = 50;
    if (request.contains("max_rows")) {
      if (!ParseDecimal(request, "max_rows", &max_rows)) {
        return ErrorReply(request_id, "max_rows is not a decimal integer");
      }
    }
    max_rows = std::min<std::uint64_t>(max_rows, 100);
    if (max_rows == 0) {
      return ErrorReply(request_id, "max_rows must be positive");
    }
    std::uint64_t offset = 0;
    if (request.contains("continuation")) {
      if (!ParseDecimal(request, "continuation", &offset)) {
        return ErrorReply(request_id, "continuation is not a decimal integer");
      }
    }
    telemetry::FaultRecordRange faults;
    if (!telemetry::ReadFaultRange(run->directory, begin_us, end_us, &faults)) {
      return ErrorReply(request_id, faults.error.empty()
                                        ? "the fault stream could not be read"
                                        : faults.error);
    }
    Json events = Json::array();
    const std::uint64_t total = faults.rows.size();
    const std::uint64_t stop =
        std::min<std::uint64_t>(total, offset + max_rows);
    for (std::uint64_t index = offset; index < stop; ++index) {
      events.push_back(
          EncodeFault(faults.rows[index], run->reader->metadata()));
    }
    Json losses = Json::array();
    for (const telemetry::FaultLossSpan &loss : faults.losses) {
      Json entry;
      entry["first_record_seq"] = Decimal(loss.first_record_seq);
      entry["last_record_seq"] = Decimal(loss.last_record_seq);
      entry["first_state_seq"] = Decimal(loss.first_state_seq);
      entry["last_state_seq"] = Decimal(loss.last_state_seq);
      entry["first_stamp_us"] = Decimal(loss.first_stamp_us);
      entry["last_stamp_us"] = Decimal(loss.last_stamp_us);
      entry["dropped"] = Decimal(loss.dropped);
      entry["reason"] =
          loss.reason == telemetry::FaultLossReason::QueueFull
              ? "queue_full"
              : (loss.reason == telemetry::FaultLossReason::WriteFailed
                     ? "write_failed"
                     : (loss.reason == telemetry::FaultLossReason::Generation
                            ? "generation"
                            : "none"));
      losses.push_back(std::move(entry));
    }
    Json reply;
    reply["request_id"] = request_id;
    reply["ok"] = true;
    reply["catalog_revision"] = revision;
    reply["series"] = Json::array();
    reply["events"] = std::move(events);
    reply["integrity"] = {
        {"faults",
         {{"complete", faults.complete},
          {"truncated", faults.truncated},
          {"corrupt", false},
          {"losses", std::move(losses)},
          {"errors", faults.error.empty() ? Json::array()
                                          : Json::array({faults.error})}}}};
    reply["coverage"] = {
        {"begin_us",
         begin_us >= end_us ? Json(nullptr) : Json(Decimal(begin_us))},
        {"end_us", begin_us >= end_us ? Json(nullptr) : Json(Decimal(end_us))}};
    reply["context"] = {{"run_id", run->id},
                        {"kind", "events"},
                        {"offset", Decimal(offset)},
                        {"returned", Decimal(stop - std::min(offset, stop))},
                        {"total", Decimal(total)}};
    if (stop < total) {
      reply["continuation"] = Decimal(stop);
    }
    return reply.dump(-1, ' ', false, Json::error_handler_t::replace);
  }

  // Range and cursor share the signal resolution and the run selection.
  if (!request.contains("signal_ids")) {
    return ErrorReply(request_id, "signal_ids is required");
  }
  std::vector<ResolvedSignal> signals;
  std::string resolve_error;
  if (!ResolveRequestedSignals(run, app, request["signal_ids"], &signals,
                               &resolve_error)) {
    return ErrorReply(request_id, resolve_error);
  }
  if (AnySource(signals, "runtime") && run->kind != RunKind::Runtime) {
    return ErrorReply(request_id, "runtime signals need a runtime run");
  }
  if (AnySource(signals, "application") && app == nullptr) {
    return ErrorReply(request_id,
                      "application signals need an application run");
  }

  if (op == "cursor") {
    protocol::TimeUs cursor_us = 0;
    if (!ParseDecimal(request, "cursor_us", &cursor_us)) {
      return ErrorReply(request_id, "cursor_us is required and decimal");
    }
    Json series = Json::array();
    for (const ResolvedSignal &signal : signals) {
      CursorResult result;
      if (signal.source == "runtime") {
        result = CursorRuntime(*run, signal, cursor_us);
      } else {
        result = CursorApplication(*app, signal, cursor_us);
      }
      series.push_back(CursorSeriesJson(signal.signal, cursor_us, result));
    }
    Json reply;
    reply["request_id"] = request_id;
    reply["ok"] = true;
    reply["catalog_revision"] = revision;
    reply["series"] = std::move(series);
    reply["events"] = Json::array();
    reply["integrity"] =
        run->kind == RunKind::Runtime
            ? Json{{"runtime", {{"complete", run->reader->complete()}}}}
            : Json{{"application", {{"complete", Json(nullptr)}}}};
    reply["coverage"] = {{"begin_us", nullptr}, {"end_us", nullptr}};
    reply["context"] = {{"run_id", run->id},
                        {"kind", "cursor"},
                        {"cursor_us", Decimal(cursor_us)}};
    return reply.dump(-1, ' ', false, Json::error_handler_t::replace);
  }

  protocol::TimeUs begin_us = 0;
  protocol::TimeUs end_us = kNeverUs;
  const bool has_begin = ParseDecimal(request, "begin_us", &begin_us);
  const bool has_end = ParseDecimal(request, "end_us", &end_us);
  if (run->kind == RunKind::Runtime && !run->reader->chunks().empty()) {
    if (!has_begin) {
      begin_us = run->reader->chunks().front().first_stamp_us;
    }
    if (!has_end) {
      end_us = run->reader->chunks().back().last_stamp_us + 1;
    }
  }
  std::uint64_t pixel_width = 800;
  if (request.contains("pixel_width")) {
    if (!ParseDecimal(request, "pixel_width", &pixel_width)) {
      return ErrorReply(request_id, "pixel_width is not a decimal integer");
    }
  }
  if (pixel_width == 0 || pixel_width > 4096) {
    return ErrorReply(request_id, "pixel_width must be 1..4096");
  }
  std::vector<ReducedSeries> reduced(signals.size());
  std::vector<bool> has_series(signals.size(), false);
  Json integrity = Json::object();
  std::string range_error;
  if (AnySource(signals, "runtime") &&
      !BuildRuntimeRange(*run, run->reader->metadata(), begin_us, end_us,
                         static_cast<std::uint32_t>(pixel_width), signals,
                         &reduced, &has_series, &integrity, &range_error)) {
    return ErrorReply(request_id, range_error);
  }
  if (AnySource(signals, "application") &&
      !BuildApplicationRange(*app, begin_us, end_us,
                             static_cast<std::uint32_t>(pixel_width), signals,
                             &reduced, &has_series, &integrity, &range_error)) {
    return ErrorReply(request_id, range_error);
  }
  Json series = Json::array();
  for (std::size_t index = 0; index < signals.size(); ++index) {
    if (!has_series[index]) {
      return ErrorReply(request_id,
                        "signal " +
                            signals[index].signal.value("id", std::string()) +
                            " produced no series");
    }
    series.push_back(ReducedSeriesJson(signals[index].signal, reduced[index]));
  }
  Json reply;
  reply["request_id"] = request_id;
  reply["ok"] = true;
  reply["catalog_revision"] = revision;
  reply["series"] = std::move(series);
  reply["events"] = Json::array();
  reply["integrity"] = std::move(integrity);
  reply["coverage"] = {
      {"begin_us",
       begin_us >= end_us ? Json(nullptr) : Json(Decimal(begin_us))},
      {"end_us", begin_us >= end_us ? Json(nullptr) : Json(Decimal(end_us))}};
  if (run->kind == RunKind::Runtime && !run->reader->chunks().empty()) {
    reply["committed"] = {
        {"begin_us", Decimal(run->reader->chunks().front().first_stamp_us)},
        {"end_us", Decimal(run->reader->chunks().back().last_stamp_us + 1)},
        {"complete", run->reader->complete()}};
  } else {
    reply["committed"] = nullptr;
  }
  reply["context"] = {
      {"run_id", run->id},
      {"app_run_id",
       app != nullptr && run != app ? Json(app->id) : Json(nullptr)},
      {"kind", "range"},
      {"begin_us", Decimal(begin_us)},
      {"end_us", Decimal(end_us)},
      {"pixel_width", Decimal(pixel_width)},
      {"signal_count", signals.size()}};
  return reply.dump(-1, ' ', false, Json::error_handler_t::replace);
}

}  // namespace irmv::web
