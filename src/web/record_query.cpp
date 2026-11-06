#include "irmv/web/record_query.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "irmv/configuration/sha256.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/fault_record.hpp"
#include "irmv/telemetry/record.hpp"
#include "irmv/telemetry/record_reader.hpp"
#include "irmv/telemetry/velocity_record.hpp"

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
    const auto stamp =
        std::filesystem::last_write_time(run.directory, code).time_since_epoch()
            .count();
    material += run.name;
    material += '|';
    material += std::to_string(stamp);
    material += '|';
    material += run.kind == RunKind::Runtime
                     ? "runtime"
                     : (run.kind == RunKind::Application ? "application"
                                                         : "unknown");
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
    {"tau_ff", "feedforward torque", "Nm", "feedforward_torque",
     "command_present"},
    {"kp", "stiffness", "Nm_per_rad", "joint_stiffness", "command_present"},
    {"kd", "damping", "Nm_s_per_rad", "joint_damping", "command_present"},
    {"measured_q", "measured position", "rad", "measured_joint_position",
     "feedback_valid+joint_valid"},
    {"measured_dq", "measured velocity", "rad_s", "measured_joint_velocity",
     "feedback_valid+joint_valid"},
    {"measured_tau_est", "estimated torque", "Nm", "estimated_joint_torque",
     "feedback_valid+joint_valid"},
    {"output_torque", "output torque", "Nm", "output_joint_torque",
     "output_valid"},
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
    const std::string &unit = std::string(field.field).rfind(
                                  "imu_orientation", 0) == 0
                                  ? orientation_unit
                                  : (std::string(field.field).rfind(
                                         "imu_angular_velocity", 0) == 0
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
    identities.push_back(
        {{"runtime_id", HexId(event.runtime_id)},
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
    context["identity"] = {{"known", false}, {"runtime_id", ""},
                           {"session_id", ""}};
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
  std::set<std::pair<std::string, std::string>> runtime_identities;
  for (const Json &entry : runtime_context["runtime_identities"]) {
    runtime_identities.insert(
        {entry.value("runtime_id", std::string()),
         entry.value("session_id", std::string())});
  }
  bool identity_match = false;
  for (const telemetry::VelocitySample &row : app_read.range.rows) {
    if (!row.identity_known) {
      continue;
    }
    if (runtime_identities.count({HexId(row.runtime_id), HexId(row.session_id)}) >
        0) {
      identity_match = true;
      break;
    }
  }
  association["boot_id_match"] = boot_match;
  association["identity_match"] = identity_match;
  association["ok"] = boot_match && identity_match;
  if (!boot_match) {
    association["reason"] = "boot_id mismatch";
  } else if (!identity_match) {
    association["reason"] = "no matching runtime/session identity";
  } else {
    association["reason"] = std::string();
  }
  association["runtime_boot_id"] = runtime_boot;
  association["application_boot_id"] = app_boot;
  return association;
}

std::string ErrorReply(const std::string &request_id, const std::string &error) {
  Json reply;
  reply["request_id"] = request_id;
  reply["ok"] = false;
  reply["error"] = error;
  return reply.dump();
}

}  // namespace

struct RecordQuery::Impl {
  std::filesystem::path root;
  bool root_ok{false};
  std::string root_error;

  explicit Impl(std::filesystem::path records_root)
      : root(std::move(records_root)) {
    std::error_code code;
    const std::filesystem::path canonical = std::filesystem::canonical(root,
                                                                       code);
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
      run.id =
          configuration::Sha256(directory.lexically_relative(root).generic_string())
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
          auto reader = std::make_shared<telemetry::RecordReader>();
          std::string open_error;
          if (telemetry::RecordReader::Open(directory, reader.get(),
                                            &open_error)) {
            run.readable = true;
            run.reader = std::move(reader);
          } else {
            run.error = open_error;
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
    return reply.dump();
  }

  if (op != "describe" && op != "context") {
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
    return ErrorReply(request_id, run->error.empty()
                                      ? "the run is not readable"
                                      : run->error);
  }

  Json context;
  if (run->kind == RunKind::Runtime) {
    context = RuntimeContext(*run);
  } else {
    const AppRead read = ReadApplication(run->directory);
    context = ApplicationContext(*run, read);
    if (!read.ok) {
      return ErrorReply(request_id, read.error.empty()
                                        ? "the application run is not readable"
                                        : read.error);
    }
  }

  if (!app_run_id.empty() && run->kind == RunKind::Runtime) {
    const RunEntry *app = Impl::Find(runs, app_run_id);
    if (app == nullptr || app->kind != RunKind::Application) {
      return ErrorReply(request_id, "unknown application run_id");
    }
    if (!app->readable) {
      return ErrorReply(request_id, app->error.empty()
                                        ? "the application run is not readable"
                                        : app->error);
    }
    const AppRead app_read = ReadApplication(app->directory);
    if (!app_read.ok) {
      return ErrorReply(request_id, app_read.error.empty()
                                        ? "the application run is not readable"
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
  return reply.dump();
}

}  // namespace irmv::web
