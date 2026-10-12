// read_record: walk a recorded run and print what it holds, or a range of it.
//
//   read_record --run <run-directory> [--from-us N] [--to-us N] [--summary-only]
//
// One JSON object per line, in the order the run recorded them, followed by a
// summary line that reports the metadata - the units the run declared among
// them - the number of rows, the range they span and every gap the run knows
// about. A gap is never hidden: a caller that reads a stretch of time is told
// what is missing from it as well as what is there.
//
// The tool only reads: it opens no DDS endpoint, publishes nothing and cannot
// command a machine.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "irmv/telemetry/record_reader.hpp"

namespace {

using Json = nlohmann::json;
using irmv::protocol::Id;
using irmv::protocol::TimeUs;
using irmv::telemetry::RecordRange;
using irmv::telemetry::RecordReader;
using irmv::telemetry::RecordSample;

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

// Every field of one row, so a consumer never has to know the column order of
// the file to use it.
Json RowToJson(const RecordSample &sample) {
  Json row;
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
  Json output_torque = Json::array();
  for (std::uint32_t index = 0; index < sample.joint_count; ++index) {
    q.push_back(sample.q[index]);
    dq.push_back(sample.dq[index]);
    tau_ff.push_back(sample.tau_ff[index]);
    kp.push_back(sample.kp[index]);
    kd.push_back(sample.kd[index]);
    measured_q.push_back(sample.measured_q[index]);
    measured_dq.push_back(sample.measured_dq[index]);
    output_torque.push_back(sample.output_torque[index]);
  }
  row["q"] = std::move(q);
  row["dq"] = std::move(dq);
  row["tau_ff"] = std::move(tau_ff);
  row["kp"] = std::move(kp);
  row["kd"] = std::move(kd);
  row["measured_q"] = std::move(measured_q);
  row["measured_dq"] = std::move(measured_dq);
  row["output_torque"] = std::move(output_torque);
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
                   argv[index], static_cast<int>(kUsage.size()),
                   kUsage.data());
      return 2;
    }
  }
  if (run_directory.empty()) {
    std::fprintf(stderr, "%.*s\n", static_cast<int>(kUsage.size()),
                 kUsage.data());
    return 2;
  }

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

  if (!summary_only) {
    for (const RecordSample &sample : range.samples) {
      std::fputs(RowToJson(sample).dump().c_str(), stdout);
      std::fputc('\n', stdout);
    }
  }

  Json summary;
  summary["run"] = run_directory;
  summary["schema_version"] = reader.metadata().schema_version;
  summary["config_id"] = reader.metadata().config_id;
  summary["layout_id"] = reader.metadata().layout_id;
  summary["backend"] = reader.metadata().backend;
  summary["control_hz"] = reader.metadata().control_hz;
  summary["sample_hz"] = reader.metadata().sample_hz;
  summary["joint_count"] = reader.metadata().joint_count;
  summary["joints"] = reader.metadata().joints;
  // The units are the run's own names for its numbers, carried as the object
  // the metadata declared: a column of radians that arrives without them is a
  // number a consumer has to guess at. A run that declares none reports none
  // rather than inventing a table.
  if (!reader.metadata().units_json.empty()) {
    summary["units"] = Json::parse(reader.metadata().units_json);
  }
  summary["chunks"] = reader.chunks().size();
  summary["rows_in_range"] = range.samples.size();
  summary["truncated"] = range.truncated;
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
      kp.push_back(event.kp[index]);
      kd.push_back(event.kd[index]);
    }
    entry["kp"] = std::move(kp);
    entry["kd"] = std::move(kd);
    entry["model_sha256"] = std::string(event.model_sha256.data());
    sessions.push_back(std::move(entry));
  }
  summary["sessions"] = std::move(sessions);
  summary["session_count"] = reader.sessions().size();
  std::fputs(summary.dump().c_str(), stdout);
  std::fputc('\n', stdout);
  return 0;
}
