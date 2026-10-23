#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/record.hpp"

// One fault occurrence as the runtime reported it, and the read side of the
// run's `faults.jsonl`.
//
// An occurrence is a single event, not a level: the state machine decides when
// a condition is new (a sustained fault is reported once) and the recorder
// never deduplicates. Every occurrence therefore names the run's ordinal for
// it (`record_seq`), the state it was observed against (`runtime_id`,
// `state_seq`, `fault_id`) and the complete detail the check established.
//
// `RecordedFault` is fixed capacity and trivially copyable: it crosses the
// bounded fault handoff to the recorder's worker by copy, so the control thread
// never allocates, formats or writes a file.
//
// The worker appends one JSON object per line to `faults.jsonl`, flushing each
// line before it moves on (there is no fsync promise). The line is:
//
//   {
//     "schema_version": 1,
//     "record_seq": 12,                       // decimal u64
//     "runtime_id": "<32 lower-case hex>",
//     "state_seq": "<16 lower-case hex>",     // hex so a 64-bit value
//     "fault_id": "<16 lower-case hex>",      // survives every JSON consumer
//     "reason": 5, "reason_name": "JointPositionExceeded",
//     "condition": 8, "condition_name": "JointPositionExceeded",
//     "joint_index": 3,                       // null when no joint applies
//     "error_code": 3735928559,
//     "observed_value": 1.25,
//     "limit": 1.0,
//     "sample_stamp_us": 993,
//     "detected_stamp_us": 1000,
//     "output_action": 0,
//     "message": "...",                       // human form, UTF-8 replaced
//     "message_hex": "<2*N lower-case hex>"   // the exact N <= 256 bytes
//   }
//
// JSON has no representation for a non-finite double, and null would lose the
// distinction between "no value" and "not a number". The writer therefore
// encodes a non-finite `observed_value` or `limit` as the explicit strings
// "NaN", "+Inf" and "-Inf"; the reader restores them. Null is never used for a
// double. The numeric reason and condition travel with their stable protocol
// names, so a consumer can read the line without this build's enum table.
//
// The protocol's message is fixed raw bytes, not UTF-8: a driver may put any
// byte in it. JSON strings cannot carry every byte sequence losslessly, so
// `message_hex` always carries the exact bytes and the reader restores the
// message from it. `message` is the human-readable form, with invalid UTF-8
// replaced by U+FFFD; it is informational and never the source of truth.
namespace irmv::telemetry {

// The fault sidecar of a run.
inline constexpr std::string_view kRecordFaultsFile = "faults.jsonl";
// The schema version of one `faults.jsonl` line. It is independent of the
// binary chunk schema (`kRecordSchemaVersion`): a fault line may gain a field
// without changing a chunk layout, and a chunk layout change must not silently
// relabel the fault stream.
inline constexpr std::uint32_t kFaultRecordSchemaVersion = 1;

struct RecordedFault {
  // The runtime's ordinal for this occurrence. It advances by one for every
  // attempt, including an attempt the handoff refused, so a loss span's
  // record sequences name exactly the events that never became rows.
  std::uint64_t record_seq{0};
  // The published state the occurrence was observed against.
  protocol::Id runtime_id{};
  protocol::Sequence state_seq{0};
  protocol::Sequence fault_id{0};
  // Everything the check established, including its own sample and detection
  // stamps and the output action in force.
  protocol::FaultDetail detail;
};

static_assert(std::is_trivially_copyable<RecordedFault>::value,
              "a fault crosses the handoff to the worker by copy");
static_assert(std::is_trivially_copyable<protocol::FaultDetail>::value,
              "the fault detail is fixed storage");

// Why an occurrence the runtime established is not a row of `faults.jsonl`.
enum class FaultLossReason : std::uint32_t {
  None = 0,
  // The state machine's observation batch was full: the runtime could not even
  // enumerate these occurrences. There is no record sequence for them.
  Generation = 1,
  // The recorder's bounded fault handoff was full when the runtime offered the
  // occurrence. An offer refused after Stop() began is still counted in
  // faults_dropped(), but it has no span: the final loss table may already have
  // been published.
  QueueFull = 2,
  // The worker accepted the occurrence but could not write its line. The
  // writer rolls a torn line back, so a later line never joins its bytes.
  WriteFailed = 3,
};

// One stretch of lost fault occurrences, in the runtime's own ordinals and the
// published state sequence. `first_record_seq`/`last_record_seq` are 0 for a
// generation-side loss, which never received an ordinal; its `dropped` count is
// the batch's own count and `first_state_seq`/`last_state_seq` name the tick it
// was reported on.
const char *FaultLossReasonName(FaultLossReason reason) noexcept;

struct FaultLossSpan {
  std::uint64_t first_record_seq{0};
  std::uint64_t last_record_seq{0};
  protocol::Sequence first_state_seq{0};
  protocol::Sequence last_state_seq{0};
  protocol::TimeUs first_stamp_us{0};
  protocol::TimeUs last_stamp_us{0};
  std::uint64_t dropped{0};
  FaultLossReason reason{FaultLossReason::None};
};

// One read of a run's fault stream.
struct FaultRecordRange {
  // False when `error` names why the stream could not be read as a whole: a
  // complete line that is not a valid occurrence, or an unreadable index. The
  // rows before the bad line are still in `rows`, but the caller must not treat
  // the run as intact.
  bool ok{false};
  std::string error;
  // The complete occurrences whose `detected_stamp_us` is in the range, in the
  // order the run recorded them.
  std::vector<RecordedFault> rows;
  // The loss spans overlapping the range, from the run's final index.
  std::vector<FaultLossSpan> losses;
  // True only when the worker finalized the fault stream: the index carries the
  // final fault marker and no accepted line failed to write. A run killed
  // before finalization, or one whose fault writes failed, reads false.
  bool complete{false};
  // True when the last line of `faults.jsonl` has no terminating newline: the
  // write was cut short. Its bytes are ignored; every complete line before it
  // is still read.
  bool truncated{false};
};

// Reads the complete occurrences of `run_directory` whose detection stamp lies
// in `[begin_us, end_us)`, plus the overlapping loss spans and the run's fault
// completion marker. An empty or inverted range reads no rows and is not an
// error. A missing `faults.jsonl` (a run with no faults, or a truncated run) is
// not an error either; a newline-terminated malformed line is. This is a
// startup and tool path, so allocation and file reads are expected.
bool ReadFaultRange(const std::filesystem::path &run_directory,
                    protocol::TimeUs begin_us, protocol::TimeUs end_us,
                    FaultRecordRange *range);

}  // namespace irmv::telemetry
