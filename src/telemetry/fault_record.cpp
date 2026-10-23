#include "irmv/telemetry/fault_record.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "fault_writer.hpp"
#include "irmv/protocol/validation.hpp"

namespace irmv::telemetry {
namespace {

using Json = nlohmann::json;

constexpr char kHexDigits[] = "0123456789abcdef";

std::string HexBytes(const std::uint8_t *bytes, std::size_t count) {
  std::string text(2 * count, '0');
  for (std::size_t index = 0; index < count; ++index) {
    text[2 * index] = kHexDigits[(bytes[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHexDigits[bytes[index] & 0x0f];
  }
  return text;
}

std::string HexId(const protocol::Id &id) {
  return HexBytes(id.data(), id.size());
}

// A 64-bit sequence as exactly 16 hex digits: the width is part of the schema,
// so a reader never has to guess whether a shorter value was truncated.
std::string HexSequence(protocol::Sequence value) {
  std::uint8_t bytes[8] = {};
  for (int index = 0; index < 8; ++index) {
    bytes[index] =
        static_cast<std::uint8_t>((value >> (8 * (7 - index))) & 0xFFU);
  }
  return HexBytes(bytes, sizeof(bytes));
}

bool HexValue(std::string_view text, std::size_t digits, std::uint64_t *out) {
  if (text.size() != digits) {
    return false;
  }
  std::uint64_t value = 0;
  for (const char character : text) {
    std::uint32_t digit = 0;
    if (character >= '0' && character <= '9') {
      digit = static_cast<std::uint32_t>(character - '0');
    } else if (character >= 'a' && character <= 'f') {
      digit = static_cast<std::uint32_t>(character - 'a') + 10;
    } else if (character >= 'A' && character <= 'F') {
      digit = static_cast<std::uint32_t>(character - 'A') + 10;
    } else {
      return false;
    }
    value = (value << 4) | digit;
  }
  *out = value;
  return true;
}

bool HexIdValue(std::string_view text, protocol::Id *out) {
  if (text.size() != 2 * out->size()) {
    return false;
  }
  for (std::size_t index = 0; index < out->size(); ++index) {
    std::uint64_t byte = 0;
    if (!HexValue(text.substr(2 * index, 2), 2, &byte)) {
      return false;
    }
    (*out)[index] = static_cast<std::uint8_t>(byte);
  }
  return true;
}

// The stable fault-reason names. The protocol exposes the condition names; the
// reason names are kept here so the telemetry library does not reinterpret a
// runtime fact and a reader can still see the enum's meaning.
const char *FaultReasonStableName(protocol::FaultReason reason) {
  switch (reason) {
    case protocol::FaultReason::None:
      return "None";
    case protocol::FaultReason::BootFailed:
      return "BootFailed";
    case protocol::FaultReason::FeedbackInvalid:
      return "FeedbackInvalid";
    case protocol::FaultReason::DriverError:
      return "DriverError";
    case protocol::FaultReason::ImuInvalid:
      return "ImuInvalid";
    case protocol::FaultReason::JointPositionExceeded:
      return "JointPositionExceeded";
    case protocol::FaultReason::JointVelocityExceeded:
      return "JointVelocityExceeded";
    case protocol::FaultReason::CommandInvalid:
      return "CommandInvalid";
    case protocol::FaultReason::WriteFailed:
      return "WriteFailed";
    case protocol::FaultReason::ProtocolExhausted:
      return "ProtocolExhausted";
  }
  return "Unknown";
}

// A non-finite double must survive a JSON round trip without becoming null.
// The three spellings are part of the line schema; the reader restores exactly
// these strings and refuses any other string value.
Json EncodeDouble(double value) {
  if (std::isnan(value)) {
    return "NaN";
  }
  if (std::isinf(value)) {
    return value > 0.0 ? "+Inf" : "-Inf";
  }
  return value;
}

bool DecodeDouble(const Json &value, double *out) {
  if (value.is_number()) {
    const double number = value.get<double>();
    // A finite double is written as a number. A JSON number that parses to an
    // infinity or NaN did not come from this writer.
    if (!std::isfinite(number)) {
      return false;
    }
    *out = number;
    return true;
  }
  if (!value.is_string()) {
    return false;
  }
  const std::string text = value.get<std::string>();
  if (text == "NaN") {
    *out = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  if (text == "+Inf") {
    *out = std::numeric_limits<double>::infinity();
    return true;
  }
  if (text == "-Inf") {
    *out = -std::numeric_limits<double>::infinity();
    return true;
  }
  return false;
}

bool FieldU64(const Json &object, const char *key, std::uint64_t *out) {
  const auto found = object.find(key);
  if (found == object.end() || !found->is_number_unsigned()) {
    return false;
  }
  *out = found->get<std::uint64_t>();
  return true;
}

bool FieldU32(const Json &object, const char *key, std::uint32_t *out) {
  std::uint64_t value = 0;
  if (!FieldU64(object, key, &value) || value > 0xFFFFFFFFULL) {
    return false;
  }
  *out = static_cast<std::uint32_t>(value);
  return true;
}

bool FieldString(const Json &object, const char *key, std::string *out) {
  const auto found = object.find(key);
  if (found == object.end() || !found->is_string()) {
    return false;
  }
  *out = found->get<std::string>();
  return true;
}

bool DecodeFault(const Json &line, RecordedFault *fault, std::string *why) {
  if (!line.is_object()) {
    *why = "the line is not a JSON object";
    return false;
  }
  std::uint32_t schema_version = 0;
  if (!FieldU32(line, "schema_version", &schema_version)) {
    *why = "schema_version is missing or not a number";
    return false;
  }
  if (schema_version != kRecordSchemaVersion) {
    *why = "the line declares schema " + std::to_string(schema_version);
    return false;
  }
  if (!FieldU64(line, "record_seq", &fault->record_seq)) {
    *why = "record_seq is missing or not an unsigned number";
    return false;
  }
  std::string runtime_id;
  if (!FieldString(line, "runtime_id", &runtime_id) ||
      !HexIdValue(runtime_id, &fault->runtime_id)) {
    *why = "runtime_id is not a 32-digit hex identity";
    return false;
  }
  std::string state_seq;
  if (!FieldString(line, "state_seq", &state_seq) ||
      !HexValue(state_seq, 16, &fault->state_seq)) {
    *why = "state_seq is not a 16-digit hex sequence";
    return false;
  }
  std::string fault_id;
  if (!FieldString(line, "fault_id", &fault_id) ||
      !HexValue(fault_id, 16, &fault->fault_id)) {
    *why = "fault_id is not a 16-digit hex sequence";
    return false;
  }
  std::uint32_t reason = 0;
  if (!FieldU32(line, "reason", &reason)) {
    *why = "reason is missing or not a number";
    return false;
  }
  fault->detail.reason = static_cast<protocol::FaultReason>(reason);
  if (!protocol::IsValid(fault->detail.reason)) {
    *why = "reason is not a known fault reason";
    return false;
  }
  std::string reason_name;
  if (!FieldString(line, "reason_name", &reason_name)) {
    *why = "reason_name is missing or not a string";
    return false;
  }
  std::uint32_t condition = 0;
  if (!FieldU32(line, "condition", &condition)) {
    *why = "condition is missing or not a number";
    return false;
  }
  fault->detail.condition = static_cast<protocol::FaultCondition>(condition);
  if (!protocol::IsValid(fault->detail.condition)) {
    *why = "condition is not a known fault condition";
    return false;
  }
  std::string condition_name;
  if (!FieldString(line, "condition_name", &condition_name)) {
    *why = "condition_name is missing or not a string";
    return false;
  }
  const auto joint = line.find("joint_index");
  if (joint == line.end()) {
    *why = "joint_index is missing";
    return false;
  }
  if (joint->is_null()) {
    fault->detail.joint_index = protocol::kNoJointIndex;
  } else {
    std::uint64_t joint_index = 0;
    if (!joint->is_number_unsigned() ||
        (joint_index = joint->get<std::uint64_t>()) >= protocol::kMaxJoints) {
      *why = "joint_index is not null or a joint index";
      return false;
    }
    fault->detail.joint_index = static_cast<std::uint32_t>(joint_index);
  }
  if (!FieldU32(line, "error_code", &fault->detail.error_code)) {
    *why = "error_code is missing or not a number";
    return false;
  }
  const auto observed = line.find("observed_value");
  if (observed == line.end() ||
      !DecodeDouble(*observed, &fault->detail.observed_value)) {
    *why = "observed_value is not a finite number or an explicit non-finite";
    return false;
  }
  const auto limit = line.find("limit");
  if (limit == line.end() || !DecodeDouble(*limit, &fault->detail.limit)) {
    *why = "limit is not a finite number or an explicit non-finite";
    return false;
  }
  if (!FieldU64(line, "sample_stamp_us", &fault->detail.sample_stamp_us)) {
    *why = "sample_stamp_us is missing or not a number";
    return false;
  }
  if (!FieldU64(line, "detected_stamp_us", &fault->detail.detected_stamp_us)) {
    *why = "detected_stamp_us is missing or not a number";
    return false;
  }
  std::uint32_t output_action = 0;
  if (!FieldU32(line, "output_action", &output_action)) {
    *why = "output_action is missing or not a number";
    return false;
  }
  fault->detail.output_action =
      static_cast<protocol::OutputAction>(output_action);
  if (!protocol::IsValid(fault->detail.output_action)) {
    *why = "output_action is not a known action";
    return false;
  }
  std::string message;
  if (!FieldString(line, "message", &message)) {
    *why = "message is missing or not a string";
    return false;
  }
  // The exact bytes live in `message_hex`; `message` is the human form and may
  // have had invalid UTF-8 replaced. Both are part of the schema, but only the
  // hex preserves the protocol's fixed byte array.
  std::string message_hex;
  if (!FieldString(line, "message_hex", &message_hex)) {
    *why = "message_hex is missing or not a string";
    return false;
  }
  if (message_hex.size() % 2 != 0 ||
      message_hex.size() > 2 * protocol::kMaxReasonBytes) {
    *why = "message_hex is not a bounded even-length hex string";
    return false;
  }
  std::string raw(message_hex.size() / 2, '\0');
  for (std::size_t index = 0; index < raw.size(); ++index) {
    std::uint64_t byte = 0;
    if (!HexValue(std::string_view(message_hex).substr(2 * index, 2), 2,
                  &byte)) {
      *why = "message_hex is not lower-case hex";
      return false;
    }
    raw[index] = static_cast<char>(byte);
  }
  if (!fault->detail.message.Assign(raw)) {
    *why = "message_hex is longer than the protocol's fixed capacity";
    return false;
  }
  return true;
}

bool DecodeLoss(const Json &entry, FaultLossSpan *loss, std::string *why) {
  if (!entry.is_object()) {
    *why = "a loss entry is not a JSON object";
    return false;
  }
  if (!FieldU64(entry, "first_record_seq", &loss->first_record_seq) ||
      !FieldU64(entry, "last_record_seq", &loss->last_record_seq) ||
      !FieldU64(entry, "first_state_seq", &loss->first_state_seq) ||
      !FieldU64(entry, "last_state_seq", &loss->last_state_seq) ||
      !FieldU64(entry, "first_stamp_us", &loss->first_stamp_us) ||
      !FieldU64(entry, "last_stamp_us", &loss->last_stamp_us) ||
      !FieldU64(entry, "dropped", &loss->dropped)) {
    *why = "a loss entry is missing an unsigned field";
    return false;
  }
  std::string reason;
  if (!FieldString(entry, "reason", &reason)) {
    *why = "a loss entry is missing its reason";
    return false;
  }
  if (reason == "generation") {
    loss->reason = FaultLossReason::Generation;
  } else if (reason == "queue_full") {
    loss->reason = FaultLossReason::QueueFull;
  } else if (reason == "write_failed") {
    loss->reason = FaultLossReason::WriteFailed;
  } else {
    *why = "a loss entry names an unknown reason";
    return false;
  }
  return true;
}

}  // namespace

const char *FaultLossReasonName(FaultLossReason reason) noexcept {
  switch (reason) {
    case FaultLossReason::None:
      return "none";
    case FaultLossReason::Generation:
      return "generation";
    case FaultLossReason::QueueFull:
      return "queue_full";
    case FaultLossReason::WriteFailed:
      return "write_failed";
  }
  return "unknown";
}

// The hook and its context are replaced as one pair under a mutex, so the
// worker can never observe a function from one installation and the context of
// another. A call already in flight keeps the pair it copied; a test must keep
// its context alive until the worker is quiescent.
std::mutex g_fault_line_hook_mutex;
int (*g_fault_line_hook)(void *, std::string_view) = nullptr;
void *g_fault_line_hook_context = nullptr;

void SetFaultLineWriteHookForTest(int (*hook)(void *context,
                                              std::string_view line),
                                  void *context) noexcept {
  const std::lock_guard<std::mutex> lock(g_fault_line_hook_mutex);
  g_fault_line_hook = hook;
  g_fault_line_hook_context = context;
}

struct FaultWriter::State {
  explicit State(std::filesystem::path file_path)
      : path(std::move(file_path)) {}

  std::filesystem::path path;
  std::ofstream file;
  // Bytes of complete, flushed lines. A failed write rolls the file back to
  // this boundary, so a torn object can never be joined by a later line.
  std::uint64_t committed_bytes{0};
};

FaultWriter::FaultWriter(std::filesystem::path path)
    : state_(std::make_unique<State>(std::move(path))) {}

FaultWriter::~FaultWriter() = default;

void FaultWriter::RollBack() noexcept {
  if (state_->file.is_open()) {
    state_->file.close();
  }
  state_->file.clear();
  std::error_code ignored;
  std::filesystem::resize_file(state_->path, state_->committed_bytes, ignored);
}

bool FaultWriter::Prepare(std::string *error) {
  std::ofstream file(state_->path, std::ios::binary | std::ios::trunc);
  if (!file) {
    if (error != nullptr) {
      *error = "the recorder could not create " + state_->path.string();
    }
    return false;
  }
  file.flush();
  if (!file.good()) {
    if (error != nullptr) {
      *error = "the recorder could not write " + state_->path.string();
    }
    return false;
  }
  state_->committed_bytes = 0;
  return true;
}

bool FaultWriter::Write(const RecordedFault &fault) {
  Json line;
  line["schema_version"] = kRecordSchemaVersion;
  line["record_seq"] = fault.record_seq;
  line["runtime_id"] = HexId(fault.runtime_id);
  line["state_seq"] = HexSequence(fault.state_seq);
  line["fault_id"] = HexSequence(fault.fault_id);
  line["reason"] = static_cast<std::uint32_t>(fault.detail.reason);
  line["reason_name"] = FaultReasonStableName(fault.detail.reason);
  line["condition"] = static_cast<std::uint32_t>(fault.detail.condition);
  line["condition_name"] = protocol::FaultConditionName(fault.detail.condition);
  if (fault.detail.joint_index == protocol::kNoJointIndex) {
    line["joint_index"] = nullptr;
  } else {
    line["joint_index"] = fault.detail.joint_index;
  }
  line["error_code"] = fault.detail.error_code;
  line["observed_value"] = EncodeDouble(fault.detail.observed_value);
  line["limit"] = EncodeDouble(fault.detail.limit);
  line["sample_stamp_us"] = fault.detail.sample_stamp_us;
  line["detected_stamp_us"] = fault.detail.detected_stamp_us;
  line["output_action"] =
      static_cast<std::uint32_t>(fault.detail.output_action);
  // The protocol's message is raw bytes. JSON text cannot carry every byte
  // sequence losslessly, so the exact bytes travel as hex and the human form is
  // informational only.
  const std::string_view message = fault.detail.message.View();
  line["message"] = std::string(message);
  line["message_hex"] = HexBytes(
      reinterpret_cast<const std::uint8_t *>(message.data()), message.size());
  const std::string text =
      line.dump(-1, ' ', false, Json::error_handler_t::replace);

  int (*hook)(void *, std::string_view) = nullptr;
  void *hook_context = nullptr;
  {
    const std::lock_guard<std::mutex> lock(g_fault_line_hook_mutex);
    hook = g_fault_line_hook;
    hook_context = g_fault_line_hook_context;
  }
  const int action = hook == nullptr ? 0 : hook(hook_context, text);
  if (action == 1) {
    // A refused write leaves no bytes behind.
    RollBack();
    return false;
  }

  if (!state_->file.is_open()) {
    state_->file.open(state_->path, std::ios::binary | std::ios::app);
    if (!state_->file) {
      RollBack();
      return false;
    }
  }
  if (action == 2) {
    // A torn write: put a prefix (without a newline) on the device, then fail.
    // The rollback removes it, so the next complete line starts at a boundary.
    const std::size_t prefix = text.empty() ? 0 : text.size() / 2;
    if (prefix != 0) {
      state_->file.write(text.data(), static_cast<std::streamsize>(prefix));
    }
    RollBack();
    return false;
  }
  state_->file.write(text.data(), static_cast<std::streamsize>(text.size()));
  state_->file.put('\n');
  state_->file.flush();
  if (!state_->file.good()) {
    RollBack();
    return false;
  }
  state_->committed_bytes += static_cast<std::uint64_t>(text.size() + 1);
  return true;
}

bool ReadFaultRange(const std::filesystem::path &run_directory,
                    protocol::TimeUs begin_us, protocol::TimeUs end_us,
                    FaultRecordRange *range) {
  if (range == nullptr) {
    return false;
  }
  *range = FaultRecordRange{};
  std::error_code code;

  // The index owns the loss table and the final completion marker. Without it
  // the run is readable but incomplete: the worker never published its final
  // state.
  const std::filesystem::path index_path =
      run_directory / std::string(kRecordIndexFile);
  const bool index_exists = std::filesystem::exists(index_path, code);
  if (code) {
    range->error = "cannot inspect " + index_path.string();
    return false;
  }
  if (index_exists) {
    std::ifstream file(index_path);
    if (!file) {
      range->error = "cannot open " + index_path.string();
      return false;
    }
    const Json document = Json::parse(file, nullptr, false);
    if (document.is_discarded() || !document.is_object()) {
      range->error = index_path.string() + " is not valid JSON";
      return false;
    }
    const auto faults = document.find("faults");
    if (faults != document.end()) {
      if (!faults->is_object()) {
        range->error = index_path.string() + " has no fault object";
        return false;
      }
      const auto complete = faults->find("complete");
      if (complete != faults->end()) {
        if (!complete->is_boolean()) {
          range->error = index_path.string() + " has no fault marker";
          return false;
        }
        range->complete = complete->get<bool>();
      }
      const auto losses = faults->find("losses");
      if (losses != faults->end()) {
        if (!losses->is_array()) {
          range->error = index_path.string() + " has no fault loss table";
          return false;
        }
        for (const Json &entry : *losses) {
          FaultLossSpan loss;
          std::string why;
          if (!DecodeLoss(entry, &loss, &why)) {
            range->error = index_path.string() + ": " + why;
            return false;
          }
          if (begin_us < end_us && loss.first_stamp_us < end_us &&
              loss.last_stamp_us >= begin_us) {
            range->losses.push_back(loss);
          }
        }
      }
    }
  }

  const std::filesystem::path faults_path =
      run_directory / std::string(kRecordFaultsFile);
  const bool faults_exists = std::filesystem::exists(faults_path, code);
  if (code) {
    range->error = "cannot inspect " + faults_path.string();
    return false;
  }
  if (!faults_exists) {
    range->ok = true;
    return true;
  }
  std::ifstream file(faults_path, std::ios::binary);
  if (!file) {
    range->error = "cannot open " + faults_path.string();
    return false;
  }
  const std::string content{std::istreambuf_iterator<char>(file),
                            std::istreambuf_iterator<char>()};
  if (file.bad()) {
    range->error = "cannot read " + faults_path.string();
    return false;
  }
  // Only newline-terminated lines are complete. The final segment without one
  // is a write that did not finish: it is named truncated and never parsed,
  // because a half-written object is not evidence of a malformed occurrence.
  std::size_t start = 0;
  std::size_t line_number = 0;
  while (start < content.size()) {
    const std::size_t newline = content.find('\n', start);
    if (newline == std::string::npos) {
      range->truncated = true;
      break;
    }
    ++line_number;
    const std::string_view text(content.data() + start, newline - start);
    const Json line = Json::parse(text.begin(), text.end(), nullptr, false);
    if (line.is_discarded()) {
      range->error = faults_path.string() + ":" + std::to_string(line_number) +
                     ": not valid JSON";
      return false;
    }
    RecordedFault fault;
    std::string why;
    if (!DecodeFault(line, &fault, &why)) {
      range->error =
          faults_path.string() + ":" + std::to_string(line_number) + ": " + why;
      return false;
    }
    if (begin_us < end_us && fault.detail.detected_stamp_us >= begin_us &&
        fault.detail.detected_stamp_us < end_us) {
      range->rows.push_back(std::move(fault));
    }
    start = newline + 1;
  }
  range->ok = true;
  return true;
}

}  // namespace irmv::telemetry
