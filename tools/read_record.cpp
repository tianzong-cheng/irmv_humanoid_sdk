// read_record: walk a recorded run and print what it holds, or a range of it.
//
//   read_record --run <run-directory> [--from-us N] [--to-us N]
//   [--summary-only]
//
// The directory is dispatched by its own metadata: a runtime run declares
// `schema_version` and holds control-tick samples plus the fault sidecar; an
// application run declares `schema` = `irmv.application_velocity` and holds the
// independent velocity stream plus its policy-session sidecar. One JSON object
// per line is printed, grouped by stream - samples then fault occurrences, or
// velocity then policy sessions - with each group in the order the run recorded
// it. A final summary line reports the metadata, the number of rows, the range
// they span and every gap or occurrence loss the run knows about.
//
// Every row carries `record_type` so a consumer never has to guess which stream
// a line belongs to: `sample`, `fault`, `velocity` or `policy_session`. A fault
// occurrence is an event, not a sample, and is printed as its own line.
//
// JSON has no non-finite double. The tool therefore writes a non-finite value
// as the explicit string "NaN", "+Inf" or "-Inf" rather than letting the
// serializer turn it into null; the exporter restores it, so a NaN and an
// infinity never collapse into the same number. A finite value is a number.
//
// A gap is never hidden: a caller that reads a stretch of time is told what is
// missing from it as well as what is there. A malformed complete line, an
// unknown schema or version, or an unreadable range is an error on stderr with
// a nonzero exit; no partial line is ever interpreted as data.
//
// The tool only reads: it opens no DDS endpoint, publishes nothing and cannot
// command a machine.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/application/policy_session_record.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/protocol/validation.hpp"
#include "irmv/telemetry/fault_record.hpp"
#include "irmv/telemetry/record_reader.hpp"
#include "irmv/telemetry/velocity_record.hpp"

namespace {

using Json = nlohmann::json;
using irmv::protocol::Id;
using irmv::protocol::TimeUs;
using irmv::telemetry::FaultRecordRange;
using irmv::telemetry::RecordedFault;
using irmv::telemetry::RecordRange;
using irmv::telemetry::RecordReader;
using irmv::telemetry::RecordSample;
using irmv::telemetry::VelocityRecordRange;
using irmv::telemetry::VelocitySample;

constexpr std::string_view kUsage =
    "usage: read_record --run <run-directory> [--from-us N] [--to-us N] "
    "[--summary-only]";

// A 16-byte identity as its lower-case hex form, the same rendering the log and
// the trace use. An identity is opaque: a JSON number would neither hold all
// sixteen bytes nor stay comparable as text, and a consumer that records which
// producer a run's commands came from needs all of them.
std::string IdHex(const Id &id) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string text(2 * id.size(), '0');
  for (std::size_t index = 0; index < id.size(); ++index) {
    text[2 * index] = kHex[(id[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHex[id[index] & 0x0f];
  }
  return text;
}

std::string HexBytes(const std::uint8_t *bytes, std::size_t size) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string text(2 * size, '0');
  for (std::size_t index = 0; index < size; ++index) {
    text[2 * index] = kHex[(bytes[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHex[bytes[index] & 0x0f];
  }
  return text;
}

// The explicit non-finite encoding every double column uses. A finite value is
// the number itself; JSON never sees a null for a measurement.
Json EncodeDouble(double value) {
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value > 0.0 ? "+Inf" : "-Inf";
  }
  return value;
}

// The stable fault-reason names of the sidecar. The protocol exposes the
// condition names; the reason names are part of the fault file's contract and
// are reproduced here so a reader sees the same text the file uses.
const char *ReasonName(irmv::protocol::FaultReason reason) {
  using irmv::protocol::FaultReason;
  switch (reason) {
    case FaultReason::None:
      return "None";
    case FaultReason::BootFailed:
      return "BootFailed";
    case FaultReason::FeedbackInvalid:
      return "FeedbackInvalid";
    case FaultReason::DriverError:
      return "DriverError";
    case FaultReason::ImuInvalid:
      return "ImuInvalid";
    case FaultReason::JointPositionExceeded:
      return "JointPositionExceeded";
    case FaultReason::JointVelocityExceeded:
      return "JointVelocityExceeded";
    case FaultReason::CommandInvalid:
      return "CommandInvalid";
    case FaultReason::WriteFailed:
      return "WriteFailed";
    case FaultReason::ProtocolExhausted:
      return "ProtocolExhausted";
  }
  return "Unknown";
}

// Every field of one row, so a consumer never has to know the column order of
// the file to use it.
Json RowToJson(const RecordSample &sample) {
  Json row;
  row["record_type"] = "sample";
  row["stamp_us"] = sample.stamp_us;
  row["state_seq"] = sample.state_seq;
  row["execution_state"] = static_cast<std::uint32_t>(sample.execution_state);
  row["output_action"] = static_cast<std::uint32_t>(sample.output_action);
  row["stop_reason"] = static_cast<std::uint32_t>(sample.stop_reason);
  row["fault_id"] = sample.fault_id;
  row["healthy"] = sample.healthy;
  row["damping_eligible"] = sample.damping_eligible;
  row["command_present"] = sample.command_present;
  row["producer_id"] = IdHex(sample.producer_id);
  row["epoch"] = sample.epoch;
  row["command_seq"] = sample.command_seq;
  row["source_state_seq"] = sample.source_state_seq;
  row["command_rx_us"] = sample.command_rx_us;
  row["feedback_seq"] = sample.feedback_seq;
  row["feedback_stamp_us"] = sample.feedback_stamp_us;
  row["feedback_valid"] = sample.feedback_valid;
  row["imu_valid"] = sample.imu_valid;
  // The IMU snapshot of the tick: three axes per vector, in the driver
  // contract's order, and the host receive time as an unsigned microsecond
  // count. nlohmann writes the u64 as an unsigned integer, so a stamp above
  // 2**53 keeps its low bit.
  Json orientation = Json::array();
  Json angular = Json::array();
  Json linear = Json::array();
  for (std::size_t index = 0; index < sample.imu_orientation_rpy.size();
       ++index) {
    orientation.push_back(EncodeDouble(sample.imu_orientation_rpy[index]));
    angular.push_back(EncodeDouble(sample.imu_angular_velocity[index]));
    linear.push_back(EncodeDouble(sample.imu_linear_acceleration[index]));
  }
  row["imu_orientation_rpy"] = std::move(orientation);
  row["imu_angular_velocity"] = std::move(angular);
  row["imu_linear_acceleration"] = std::move(linear);
  row["imu_rx_stamp_us"] = sample.imu_rx_stamp_us;
  row["send_accepted"] = sample.send_accepted;
  row["output_valid"] = sample.output_valid;
  row["joint_count"] = sample.joint_count;
  Json q = Json::array();
  Json dq = Json::array();
  Json tau_ff = Json::array();
  Json kp = Json::array();
  Json kd = Json::array();
  Json measured_q = Json::array();
  Json measured_dq = Json::array();
  Json measured_tau_est = Json::array();
  Json output_torque = Json::array();
  Json joint_valid = Json::array();
  for (std::uint32_t index = 0; index < sample.joint_count; ++index) {
    q.push_back(EncodeDouble(sample.q[index]));
    dq.push_back(EncodeDouble(sample.dq[index]));
    tau_ff.push_back(EncodeDouble(sample.tau_ff[index]));
    kp.push_back(EncodeDouble(sample.kp[index]));
    kd.push_back(EncodeDouble(sample.kd[index]));
    measured_q.push_back(EncodeDouble(sample.measured_q[index]));
    measured_dq.push_back(EncodeDouble(sample.measured_dq[index]));
    measured_tau_est.push_back(EncodeDouble(sample.measured_tau_est[index]));
    output_torque.push_back(EncodeDouble(sample.output_torque[index]));
    joint_valid.push_back(sample.joint_valid[index]);
  }
  row["q"] = std::move(q);
  row["dq"] = std::move(dq);
  row["tau_ff"] = std::move(tau_ff);
  row["kp"] = std::move(kp);
  row["kd"] = std::move(kd);
  row["measured_q"] = std::move(measured_q);
  row["measured_dq"] = std::move(measured_dq);
  row["measured_tau_est"] = std::move(measured_tau_est);
  row["output_torque"] = std::move(output_torque);
  row["joint_valid"] = std::move(joint_valid);
  return row;
}

Json FaultToJson(const RecordedFault &fault) {
  Json row;
  row["record_type"] = "fault";
  row["record_seq"] = fault.record_seq;
  row["runtime_id"] = IdHex(fault.runtime_id);
  // The file writes the two 64-bit sequences as hex so every JSON consumer
  // keeps them whole; the tool prints them as unsigned numbers because it is
  // itself a JSON producer and a consumer of this line gets the same width.
  row["state_seq"] = fault.state_seq;
  row["fault_id"] = fault.fault_id;
  row["reason"] = static_cast<std::uint32_t>(fault.detail.reason);
  row["reason_name"] = ReasonName(fault.detail.reason);
  row["condition"] = static_cast<std::uint32_t>(fault.detail.condition);
  row["condition_name"] =
      irmv::protocol::FaultConditionName(fault.detail.condition);
  if (fault.detail.joint_index == irmv::protocol::kNoJointIndex) {
    row["joint_index"] = nullptr;
  } else {
    row["joint_index"] = fault.detail.joint_index;
  }
  row["error_code"] = fault.detail.error_code;
  row["observed_value"] = EncodeDouble(fault.detail.observed_value);
  row["limit"] = EncodeDouble(fault.detail.limit);
  row["sample_stamp_us"] = fault.detail.sample_stamp_us;
  row["detected_stamp_us"] = fault.detail.detected_stamp_us;
  row["output_action"] = static_cast<std::uint32_t>(fault.detail.output_action);
  const std::string_view message = fault.detail.message.View();
  row["message"] = std::string(message);
  row["message_hex"] = HexBytes(
      reinterpret_cast<const std::uint8_t *>(message.data()), message.size());
  return row;
}

Json VelocityToJson(const VelocitySample &sample) {
  Json row;
  row["record_type"] = "velocity";
  row["stamp_us"] = sample.stamp_us;
  row["sample_seq"] = sample.sample_seq;
  row["identity_known"] = sample.identity_known;
  row["runtime_id"] =
      sample.identity_known ? IdHex(sample.runtime_id) : std::string();
  row["session_id"] =
      sample.identity_known ? IdHex(sample.session_id) : std::string();
  row["latest_state_seq"] = sample.latest_state_seq;
  row["forward"] = EncodeDouble(sample.forward);
  row["lateral"] = EncodeDouble(sample.lateral);
  row["yaw"] = EncodeDouble(sample.yaw);
  return row;
}

bool ParseTime(const char *text, TimeUs *out) {
  char *end = nullptr;
  const unsigned long long value = std::strtoull(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  *out = static_cast<TimeUs>(value);
  return true;
}

bool ReadJsonFile(const std::filesystem::path &path, Json *document,
                  std::string *error) {
  std::ifstream file(path);
  if (!file) {
    *error = "cannot open " + path.string();
    return false;
  }
  *document = Json::parse(file, nullptr, false);
  if (document->is_discarded()) {
    *error = path.string() + " is not valid JSON";
    return false;
  }
  return true;
}

// The application's policy-session sidecar. It has no index of its own: the
// file is one JSON object per line, flushed after every line, so a final
// segment without a newline is a write that did not finish and is never parsed.
bool ReadPolicySessions(const std::filesystem::path &path, TimeUs begin_us,
                        TimeUs end_us, Json *rows, bool *truncated,
                        std::string *error) {
  *rows = Json::array();
  *truncated = false;
  // An empty or inverted range reads nothing, exactly like the sample and
  // velocity readers. The rest of the file is still parsed and validated, so a
  // damaged sidecar is never hidden by a window that happens to select nothing.
  const bool range_empty = begin_us >= end_us;
  std::error_code code;
  if (!std::filesystem::exists(path, code)) {
    if (code) {
      *error = "cannot inspect " + path.string();
      return false;
    }
    return true;
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    *error = "cannot open " + path.string();
    return false;
  }
  const std::string content{std::istreambuf_iterator<char>(file),
                            std::istreambuf_iterator<char>()};
  if (file.bad()) {
    *error = "cannot read " + path.string();
    return false;
  }
  std::size_t start = 0;
  std::size_t line_number = 0;
  while (start < content.size()) {
    const std::size_t newline = content.find('\n', start);
    if (newline == std::string::npos) {
      *truncated = true;
      break;
    }
    ++line_number;
    const std::string_view text(content.data() + start, newline - start);
    Json row = Json::parse(text.begin(), text.end(), nullptr, false);
    if (row.is_discarded() || !row.is_object()) {
      *error = path.string() + ":" + std::to_string(line_number) +
               ": not a JSON object";
      return false;
    }
    const auto schema = row.find("schema");
    const auto version = row.find("version");
    if (schema == row.end() || !schema->is_string() ||
        schema->get<std::string>() != irmv::application::kPolicySessionSchema ||
        version == row.end() || !version->is_number_unsigned() ||
        version->get<std::uint64_t>() !=
            irmv::application::kPolicySessionSchemaVersion) {
      *error = path.string() + ":" + std::to_string(line_number) +
               ": unknown schema/version";
      return false;
    }
    const auto monotonic = row.find("monotonic_us");
    if (monotonic == row.end() || !monotonic->is_number_unsigned()) {
      *error = path.string() + ":" + std::to_string(line_number) +
               ": monotonic_us is missing or not a number";
      return false;
    }
    const TimeUs stamp = monotonic->get<TimeUs>();
    if (range_empty || stamp < begin_us || stamp >= end_us) {
      start = newline + 1;
      continue;
    }
    row["record_type"] = "policy_session";
    rows->push_back(std::move(row));
    start = newline + 1;
  }
  return true;
}

// The final index counts the recorder published for the fault handoff. They
// are part of the run's own accounting, so the summary copies them rather than
// recomputing a weaker number from the rows it happened to read.
void AddFaultIndexCounts(const std::filesystem::path &run_directory,
                         Json *faults) {
  Json index;
  std::string error;
  if (!ReadJsonFile(run_directory / "index.json", &index, &error) ||
      !index.is_object()) {
    return;
  }
  const auto object = index.find("faults");
  if (object == index.end() || !object->is_object()) {
    return;
  }
  for (const char *key : {"recorded", "written", "queue_dropped",
                          "generation_dropped", "write_failures"}) {
    const auto value = object->find(key);
    if (value != object->end() && value->is_number_unsigned()) {
      (*faults)[key] = *value;
    }
  }
}

void AddMetaExtras(Json *summary, const Json &meta) {
  const auto sha = meta.find("sdk_git_sha");
  if (sha != meta.end() && sha->is_string()) {
    (*summary)["sdk_git_sha"] = *sha;
  }
  // The clock the stamps are from, and the boot the run belongs to, are what
  // makes two independent runs of one deployment comparable. The metadata
  // wrote them; the reader copies the declaration verbatim.
  const auto clock = meta.find("clock");
  if (clock != meta.end() && clock->is_object()) {
    (*summary)["clock"] = *clock;
  }
}

void AddLossArray(Json *summary, const Json &losses) {
  (*summary)["losses"] = losses;
  std::uint64_t missing = 0;
  for (const Json &loss : losses) {
    const auto dropped = loss.find("dropped");
    if (dropped != loss.end() && dropped->is_number_unsigned()) {
      missing += dropped->get<std::uint64_t>();
    }
  }
  (*summary)["missing_samples"] = missing;
}

void PrintJsonLine(const Json &line) {
  const std::string text =
      line.dump(-1, ' ', false, Json::error_handler_t::replace);
  std::fputs(text.c_str(), stdout);
  std::fputc('\n', stdout);
}

enum class RunKind { Runtime, Application };

bool Dispatch(const Json &meta, RunKind *kind, std::string *error) {
  // A declared schema is authoritative. Falling through to the runtime reader
  // because a document also happens to carry a numeric `schema_version` would
  // reinterpret a run of an unknown format as this build's chunks.
  const auto schema = meta.find("schema");
  if (schema != meta.end()) {
    if (!schema->is_string()) {
      *error = "meta.json carries a schema that is not a string";
      return false;
    }
    const std::string name = schema->get<std::string>();
    if (name != irmv::telemetry::kVelocitySchema) {
      *error = "meta.json declares the unknown schema \"" + name +
               "\"; the run is not interpreted";
      return false;
    }
    const auto version = meta.find("version");
    if (version == meta.end() || !version->is_number_unsigned() ||
        version->get<std::uint64_t>() !=
            irmv::telemetry::kVelocitySchemaVersion) {
      *error = "meta.json declares " + name + " with an unsupported version";
      return false;
    }
    *kind = RunKind::Application;
    return true;
  }
  const auto version = meta.find("schema_version");
  if (version != meta.end() && version->is_number_unsigned()) {
    *kind = RunKind::Runtime;
    return true;
  }
  *error =
      "meta.json declares neither the runtime schema_version nor "
      "irmv.application_velocity; the run is not interpreted";
  return false;
}

int RunRuntime(const std::string &run_directory, TimeUs begin_us, TimeUs end_us,
               bool summary_only, const Json &meta) {
  RecordReader reader;
  std::string error;
  if (!RecordReader::Open(run_directory, &reader, &error)) {
    std::fprintf(stderr, "read_record: %s\n", error.c_str());
    return 1;
  }

  RecordRange range;
  if (!reader.ReadRange(begin_us, end_us, &range)) {
    std::fprintf(stderr, "read_record: the range could not be read\n");
    return 1;
  }
  // Fault occurrences are a separate stream over the same run: they are not
  // sampled at the recorder's rate and they are not rows of the chunk files.
  // A malformed complete line makes the stream unreadable as a whole; the rows
  // before it are still returned, but the caller is not told the run is intact.
  FaultRecordRange faults;
  if (!irmv::telemetry::ReadFaultRange(run_directory, begin_us, end_us,
                                       &faults)) {
    std::fprintf(stderr, "read_record: %s\n", faults.error.c_str());
    return 1;
  }

  if (!summary_only) {
    for (const RecordSample &sample : range.samples) {
      PrintJsonLine(RowToJson(sample));
    }
    for (const RecordedFault &fault : faults.rows) {
      PrintJsonLine(FaultToJson(fault));
    }
  }

  Json summary;
  summary["record_kind"] = "runtime";
  summary["run"] = run_directory;
  summary["schema_version"] = reader.metadata().schema_version;
  summary["config_id"] = reader.metadata().config_id;
  summary["layout_id"] = reader.metadata().layout_id;
  summary["backend"] = reader.metadata().backend;
  summary["control_hz"] = reader.metadata().control_hz;
  summary["sample_hz"] = reader.metadata().sample_hz;
  summary["joint_count"] = reader.metadata().joint_count;
  summary["joints"] = reader.metadata().joints;
  AddMetaExtras(&summary, meta);
  // The units are the run's own names for its numbers, carried as the object
  // the metadata declared: a column of radians that arrives without them is a
  // number a consumer has to guess at. A run that declares none reports none
  // rather than inventing a table.
  if (!reader.metadata().units_json.empty()) {
    summary["units"] = Json::parse(reader.metadata().units_json);
  }
  // The IMU convention object travels with the archive: an unknown axis frame
  // or gravity convention is part of what the run declares, and a consumer
  // that receives only the exported rows must not lose it.
  if (!reader.metadata().imu_json.empty()) {
    summary["imu_conventions"] = Json::parse(reader.metadata().imu_json);
  }
  summary["chunks"] = reader.chunks().size();
  summary["rows_in_range"] = range.samples.size();
  // A run that was killed before its worker finalized has no final loss table,
  // and its last chunk may be only what the crash left. The reader already
  // knows this; the export has to say it rather than let a consumer infer a
  // complete run from the absence of truncation and corruption.
  summary["complete"] = reader.complete();
  summary["truncated"] = range.truncated;
  // A corrupt chunk is reported as itself rather than folded into the
  // truncation flag: a caller has to be able to tell a run that was cut short
  // from a run whose bytes no longer match what was written, and the error
  // names the file and the check that failed.
  summary["corrupt"] = range.corrupt;
  if (!range.error.empty()) {
    summary["error"] = range.error;
  }
  if (!range.samples.empty()) {
    summary["first_stamp_us"] = range.samples.front().stamp_us;
    summary["last_stamp_us"] = range.samples.back().stamp_us;
    summary["first_state_seq"] = range.samples.front().state_seq;
    summary["last_state_seq"] = range.samples.back().state_seq;
  }
  Json gaps = Json::array();
  std::uint64_t missing = 0;
  for (const irmv::telemetry::RecordGap &gap : range.gaps) {
    Json entry;
    entry["first_seq"] = gap.first_seq;
    entry["last_seq"] = gap.last_seq;
    entry["dropped"] = gap.dropped;
    entry["reason"] = static_cast<std::uint32_t>(gap.reason);
    gaps.push_back(std::move(entry));
    missing += gap.dropped;
  }
  summary["gaps"] = std::move(gaps);
  summary["missing_samples"] = missing;
  // The session sidecar: one entry per authorization context the runtime
  // adopted, in arrival order. The digest is carried verbatim - the runtime
  // writes "unknown" because it holds no model - and the reader never
  // substitutes one.
  Json sessions = Json::array();
  for (const irmv::telemetry::RecordSessionEvent &event : reader.sessions()) {
    Json entry;
    entry["schema_version"] = event.schema_version;
    entry["runtime_id"] = IdHex(event.runtime_id);
    entry["session_id"] = IdHex(event.session_id);
    entry["producer_id"] = IdHex(event.producer_id);
    entry["epoch"] = event.epoch;
    entry["stamp_us"] = event.stamp_us;
    entry["state_seq"] = event.state_seq;
    entry["joint_count"] = event.joint_count;
    Json kp = Json::array();
    Json kd = Json::array();
    for (std::uint32_t index = 0;
         index < event.joint_count && index < irmv::protocol::kMaxJoints;
         ++index) {
      kp.push_back(EncodeDouble(event.kp[index]));
      kd.push_back(EncodeDouble(event.kd[index]));
    }
    entry["kp"] = std::move(kp);
    entry["kd"] = std::move(kd);
    entry["model_sha256"] = std::string(event.model_sha256.data());
    sessions.push_back(std::move(entry));
  }
  summary["sessions"] = std::move(sessions);
  summary["session_count"] = reader.sessions().size();

  // The fault stream's own accounting: completion, truncation, the loss spans
  // and the final counts the recorder published. The sidecar is created when
  // the recorder starts, so ReadFaultRange refuses a run whose file is missing
  // rather than presenting the loss as an empty stream.
  Json fault_summary;
  fault_summary["complete"] = faults.complete;
  fault_summary["truncated"] = faults.truncated;
  fault_summary["rows_in_range"] = faults.rows.size();
  std::uint64_t lost = 0;
  Json fault_losses = Json::array();
  for (const irmv::telemetry::FaultLossSpan &loss : faults.losses) {
    Json entry;
    entry["first_record_seq"] = loss.first_record_seq;
    entry["last_record_seq"] = loss.last_record_seq;
    entry["first_state_seq"] = loss.first_state_seq;
    entry["last_state_seq"] = loss.last_state_seq;
    entry["first_stamp_us"] = loss.first_stamp_us;
    entry["last_stamp_us"] = loss.last_stamp_us;
    entry["dropped"] = loss.dropped;
    entry["reason"] = irmv::telemetry::FaultLossReasonName(loss.reason);
    fault_losses.push_back(std::move(entry));
    lost += loss.dropped;
  }
  fault_summary["losses"] = std::move(fault_losses);
  fault_summary["missing_occurrences"] = lost;
  AddFaultIndexCounts(run_directory, &fault_summary);
  summary["faults"] = std::move(fault_summary);
  PrintJsonLine(summary);
  return 0;
}

int RunApplication(const std::string &run_directory, TimeUs begin_us,
                   TimeUs end_us, bool summary_only, const Json &meta) {
  VelocityRecordRange range;
  if (!irmv::telemetry::ReadVelocityRange(run_directory, begin_us, end_us,
                                          &range)) {
    std::fprintf(stderr, "read_record: %s\n", range.error.c_str());
    return 1;
  }
  Json policy_rows;
  bool policy_truncated = false;
  std::string error;
  if (!ReadPolicySessions(
          std::filesystem::path(run_directory) /
              std::string(irmv::application::kPolicySessionFile),
          begin_us, end_us, &policy_rows, &policy_truncated, &error)) {
    std::fprintf(stderr, "read_record: %s\n", error.c_str());
    return 1;
  }

  if (!summary_only) {
    for (const VelocitySample &sample : range.rows) {
      PrintJsonLine(VelocityToJson(sample));
    }
    for (const Json &row : policy_rows) {
      PrintJsonLine(row);
    }
  }

  Json summary;
  summary["record_kind"] = "application";
  summary["run"] = run_directory;
  // Both names are carried: `schema` is the application document's own identity
  // and `schema_version` is the version number under a key a runtime reader
  // already understands.
  if (const auto schema = meta.find("schema"); schema != meta.end()) {
    summary["schema"] = *schema;
  }
  if (const auto version = meta.find("version"); version != meta.end()) {
    summary["version"] = *version;
    summary["schema_version"] = *version;
  }
  for (const char *key :
       {"sdk_git_sha", "application_config_id", "config_id", "layout_id",
        "joint_count", "sample_hz", "loop_period_us", "sampling", "source",
        "units", "model"}) {
    const auto value = meta.find(key);
    if (value != meta.end()) {
      summary[key] = *value;
    }
  }
  AddMetaExtras(&summary, meta);
  summary["rows_in_range"] = range.rows.size();
  summary["complete"] = range.complete;
  summary["truncated"] = range.truncated;
  if (!range.rows.empty()) {
    summary["first_stamp_us"] = range.rows.front().stamp_us;
    summary["last_stamp_us"] = range.rows.back().stamp_us;
    summary["first_sample_seq"] = range.rows.front().sample_seq;
    summary["last_sample_seq"] = range.rows.back().sample_seq;
  }
  Json losses = Json::array();
  for (const irmv::telemetry::VelocityLossSpan &loss : range.losses) {
    Json entry;
    entry["first_sample_seq"] = loss.first_sample_seq;
    entry["last_sample_seq"] = loss.last_sample_seq;
    entry["first_stamp_us"] = loss.first_stamp_us;
    entry["last_stamp_us"] = loss.last_stamp_us;
    entry["dropped"] = loss.dropped;
    entry["reason"] = irmv::telemetry::VelocityLossReasonName(loss.reason);
    losses.push_back(std::move(entry));
  }
  AddLossArray(&summary, losses);
  summary["policy_sessions"] = policy_rows.size();
  summary["policy_session_truncated"] = policy_truncated;
  PrintJsonLine(summary);
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  std::string run_directory;
  TimeUs begin_us = 0;
  TimeUs end_us = ~static_cast<TimeUs>(0);
  bool summary_only = false;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    const bool has_value = index + 1 < argc;
    if (argument == "--run" && has_value) {
      run_directory = argv[++index];
    } else if (argument == "--from-us" && has_value) {
      if (!ParseTime(argv[++index], &begin_us)) {
        std::fprintf(stderr, "read_record: --from-us must be microseconds\n");
        return 2;
      }
    } else if (argument == "--to-us" && has_value) {
      if (!ParseTime(argv[++index], &end_us)) {
        std::fprintf(stderr, "read_record: --to-us must be microseconds\n");
        return 2;
      }
    } else if (argument == "--summary-only") {
      summary_only = true;
    } else {
      std::fprintf(stderr, "read_record: unknown argument %s\n%.*s\n",
                   argv[index], static_cast<int>(kUsage.size()), kUsage.data());
      return 2;
    }
  }
  if (run_directory.empty()) {
    std::fprintf(stderr, "%.*s\n", static_cast<int>(kUsage.size()),
                 kUsage.data());
    return 2;
  }

  Json meta;
  std::string error;
  if (!ReadJsonFile(std::filesystem::path(run_directory) / "meta.json", &meta,
                    &error)) {
    std::fprintf(stderr, "read_record: %s\n", error.c_str());
    return 1;
  }
  if (!meta.is_object()) {
    std::fprintf(stderr, "read_record: meta.json is not a JSON object\n");
    return 1;
  }
  RunKind kind = RunKind::Runtime;
  if (!Dispatch(meta, &kind, &error)) {
    std::fprintf(stderr, "read_record: %s\n", error.c_str());
    return 1;
  }
  if (kind == RunKind::Application) {
    return RunApplication(run_directory, begin_us, end_us, summary_only, meta);
  }
  return RunRuntime(run_directory, begin_us, end_us, summary_only, meta);
}
