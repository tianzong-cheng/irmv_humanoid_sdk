#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"

// The application's own velocity stream: one row per sampling deadline of the
// body-velocity intent the control loop was holding, in a run directory of its
// own.
//
// It is independent of the runtime recorder. The runtime records what the
// machine did at its control rate; this stream records what the application
// asked for at the application's own sampling rate, with the runtime/session
// identity and latest state sequence in force at that instant. A sample does
// not go into the runtime's `RecordSample` joint columns, and a slow velocity
// disk cannot take room from the runtime's sample handoff: the two processes
// and the two recorders share only their configured base directory.
//
// The control path offers a sample to one fixed 256-slot handoff and returns.
// `TryRecord` never allocates, formats, blocks or touches a file; a full queue
// costs the sample and is counted, and the run's index names the sequence
// stretch the loss covers. A sample offered after Stop() began is counted only:
// the final loss table may already be published. The worker owns the file, the
// metadata and the index, exactly like the runtime recorder: Stop() closes the
// handoff, asks the worker to finalize and waits against a deadline; a device
// that blocks past the deadline leaves the run to the worker, which keeps its
// own state alive and finishes on its own. No filesystem write happens on the
// caller's thread after that deadline.
//
// Directory and schema
// --------------------
// Each Start() creates `app_run_<UTC>_<unique>` under the configured recorder
// directory. `<UTC>` is the host wall clock in ISO-8601 basic form and
// `<unique>` is the process id plus a process-local counter, so two
// applications - or two recorders in one process - can never share a run
// directory. The directory holds:
//
//   meta.json       the caller's metadata, verbatim (schema, version, SDK
//                   revision, application/runtime configuration digests, clock
//                   identity, sampling rate, units and the explicit model
//                   status);
//   velocity.jsonl  one JSON object per accepted sample;
//   index.json      counts, loss spans and the final `complete` marker.
//
// A reader refuses metadata, index or line of another schema or version. The
// completion marker is published only by the worker's finalization; a run
// killed before it reads complete=false. A failed write never counts as a row:
// the writer rolls a torn line back and the index carries the loss.
namespace irmv::telemetry {

inline constexpr std::string_view kVelocitySchema = "irmv.application_velocity";
inline constexpr std::uint32_t kVelocitySchemaVersion = 1;
inline constexpr std::string_view kVelocityFile = "velocity.jsonl";
inline constexpr std::string_view kVelocityMetadataFile = "meta.json";
inline constexpr std::string_view kVelocityIndexFile = "index.json";

// The fixed capacity of the handoff. A configuration may ask for less, never
// for more; at the shipped 100 Hz this is roughly two and a half seconds of a
// slow disk falling behind before samples are lost.
inline constexpr std::size_t kVelocityQueueCapacity = 256;

// The bound Stop() gives the worker. A blocked device is left to the worker,
// which owns its state, instead of holding the control loop past this deadline.
inline constexpr std::uint32_t kVelocityStopTimeoutMs = 2000;

// One due sample, as the control loop held it at the sampling instant. Plain,
// fixed-size data: it crosses the bounded handoff by copy.
//
// `sample_seq` is the run's monotonic sample ordinal, assigned once per
// sampling deadline (0 for the first). A deadline the loop missed is not
// replayed; the next row simply carries the next ordinal with the real
// timestamp, so a gap in the sequence is not invented history.
//
// `identity_known` is true only when both `runtime_id` and `session_id` are
// known at the sampling instant. Before the runtime is discovered, or before a
// session is acquired, the ids are the all-zero "none" value and the row says
// so; after a restart or a session change the row carries the new identity and
// never the old one. `latest_state_seq` is the sequence of the newest trusted
// state the loop held, or 0 while it held none.
struct VelocitySample {
  protocol::TimeUs stamp_us{0};
  std::uint64_t sample_seq{0};
  bool identity_known{false};
  protocol::Id runtime_id{};
  protocol::Id session_id{};
  protocol::Sequence latest_state_seq{0};
  double forward{0.0};
  double lateral{0.0};
  double yaw{0.0};
};

// Why an offered sample did not become a row of `velocity.jsonl`.
enum class VelocityLossReason : std::uint32_t {
  None = 0,
  // The fixed handoff was full when the control loop offered the sample.
  QueueFull = 1,
  // The worker accepted the sample but could not persist its line. The writer
  // rolls a torn line back, so a later line never joins its bytes.
  WriteFailed = 2,
};

const char *VelocityLossReasonName(VelocityLossReason reason) noexcept;

// One stretch of samples that never became rows, in the run's own sample
// ordinals and timestamps.
struct VelocityLossSpan {
  std::uint64_t first_sample_seq{0};
  std::uint64_t last_sample_seq{0};
  protocol::TimeUs first_stamp_us{0};
  protocol::TimeUs last_stamp_us{0};
  std::uint64_t dropped{0};
  VelocityLossReason reason{VelocityLossReason::None};
};

// One read of a run's velocity stream.
struct VelocityRecordRange {
  // False when `error` names why the stream could not be read as a whole: an
  // unknown metadata/index/line version, an unreadable file or a complete line
  // that is not a valid sample. The rows before the bad line are still in
  // `rows`, but the caller must not treat the run as intact.
  bool ok{false};
  std::string error;
  // The complete samples whose `stamp_us` lies in `[begin_us, end_us)`, in the
  // order the run recorded them.
  std::vector<VelocitySample> rows;
  // The loss spans of the run's index. They are returned whole: a loss is a
  // fact of the stream, not a row filtered by the requested range.
  std::vector<VelocityLossSpan> losses;
  // True only when the worker published its final completion marker and no
  // accepted line failed to write.
  bool complete{false};
  // True when the last line of `velocity.jsonl` has no terminating newline:
  // the write was cut short. Its bytes are ignored; every complete line before
  // it is still read.
  bool truncated{false};
};

// The bounded, non-blocking writer of one application run's velocity stream.
class VelocityRecorder {
 public:
  VelocityRecorder();
  ~VelocityRecorder();

  VelocityRecorder(const VelocityRecorder &) = delete;
  VelocityRecorder &operator=(const VelocityRecorder &) = delete;

  // Creates the run directory, writes `meta_json` there and starts the worker.
  // Returns false - with `error` filled in - when the configuration or the
  // metadata is refused or the run cannot be created; no recorder state is
  // left behind in that case. `meta_json` must be a JSON object declaring
  // `schema` = kVelocitySchema and `version` = kVelocitySchemaVersion; its
  // remaining fields are written verbatim and interpreted by readers.
  bool Start(const configuration::RecorderConfig &config, std::string meta_json,
             std::string *error);

  // Offers one sample. Never blocks, allocates, throws or touches a file.
  // Returns false when the handoff was full or the recorder is not running;
  // the sample is counted as dropped and, while the run is still open, the
  // refusal is published as a loss span.
  bool TryRecord(const VelocitySample &sample) noexcept;

  // Closes the handoff, asks the worker to drain and finalize, and waits for it
  // against kVelocityStopTimeoutMs. Returns true when the worker finished and
  // was joined; false when the deadline expired, in which case the worker still
  // owns every queue, buffer and file and finishes on its own. Idempotent, and
  // called by the destructor.
  bool Stop();

  // True while the handoff accepts samples.
  bool started() const noexcept;
  // The directory of the run this recorder created; it keeps naming it after
  // Stop(), so a caller can still read the run it just closed.
  const std::filesystem::path &run_directory() const noexcept;
  // Samples the handoff accepted.
  std::uint64_t recorded() const noexcept;
  // Lines the worker wrote and flushed.
  std::uint64_t written() const noexcept;
  // Samples refused because the handoff was full or the recorder was not
  // running.
  std::uint64_t dropped() const noexcept;
  // Accepted samples whose line the worker could not persist.
  std::uint64_t write_failures() const noexcept;
  // Empty on success; otherwise why the recorder is not running. Never thrown
  // from TryRecord.
  const std::string &error() const noexcept;

 private:
  struct Impl;

  std::shared_ptr<Impl> impl_;
  std::string start_error_;
  std::uint64_t unattached_dropped_{0};
};

// Reads the complete samples of `run_directory` whose `stamp_us` lies in
// `[begin_us, end_us)`, plus the run's loss table, truncation and completion
// facts. An empty or inverted range reads no rows and is not an error. The
// metadata must exist and declare this schema/version; a velocity file that is
// missing, a newline-terminated malformed line, or an index of another version
// is an error. This is a startup and tool path, so allocation and file reads
// are expected.
bool ReadVelocityRange(const std::filesystem::path &run_directory,
                       protocol::TimeUs begin_us, protocol::TimeUs end_us,
                       VelocityRecordRange *range);

// Test-only seam: called on the worker before one line is written, with the
// serialized line. It returns the action:
//   0 - write the whole line;
//   1 - refuse it (no bytes are written);
//   2 - simulate a torn write (a prefix is written, then the sink fails).
// A test can also block inside the hook (a blocked device). Production never
// sets it. The function and its context are read as one pair under a mutex; a
// test must keep the context alive until the worker is quiescent, because an
// in-flight call may already hold the pair after removal.
void SetVelocityLineWriteHookForTest(int (*hook)(void *context,
                                                 std::string_view line),
                                     void *context) noexcept;

}  // namespace irmv::telemetry
