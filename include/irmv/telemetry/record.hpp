#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "irmv/protocol/types.hpp"

// One coherent sample of the control loop, and the shape of a gap in a run.
//
// "Coherent" is the whole point: every field of a sample describes the *same*
// tick - the state the runtime published, the command that was in force for it
// and the target that was actually applied - so a reader can never line up a
// position from one tick with a gain from another. The control thread builds
// one sample per recorded tick out of the values it already holds; nothing here
// reads a clock, a device or another thread.
//
// The sample is fixed capacity and trivially copyable: it crosses the bounded
// handoff to the recorder's worker by copy, and the control thread never
// allocates, formats or writes a file.
namespace irmv::telemetry {

// The version of the on-disk sample layout. A reader refuses a run of another
// version instead of reinterpreting its bytes.
//
// 3 gave every chunk an explicit payload length and a CRC-32 over its own
// header and over its payload, so a chunk that was corrupted in place - a torn
// write, a bad sector, a stray edit - is named as corrupt and contributes no
// rows. Version 2 wrote the same columns but no length and no checksum, so a
// chunk whose length happened to match was decoded as valid data whatever its
// bytes held; version 2 runs are refused, never reinterpreted.
//
// 2 added the joint feedback the tick used (`measured_q`/`measured_dq`), the
// output stage's own result (`output_valid`/`output_torque`) and the session
// sidecar (`sessions.json`). Version 1 carried only the target and the
// feedback's identity, so a reader could not see what the machine measured or
// what it was asked to apply; version 1 runs are refused, never reinterpreted.
inline constexpr std::uint32_t kRecordSchemaVersion = 3;

// A SHA-256 digest rendered as 64 lower-case hex digits, plus a terminator. A
// session event carries one as fixed storage so the control thread can push it
// without allocating; the runtime itself always writes "unknown".
inline constexpr std::size_t kModelDigestBytes = 65;

// One tick of the run.
struct RecordSample {
  std::uint32_t schema_version{kRecordSchemaVersion};
  // The controller's monotonic time for this tick and the sequence the runtime
  // published it under. A reader aligns a run with the state topic by these two
  // and never by a host clock.
  protocol::TimeUs stamp_us{0};
  protocol::Sequence state_seq{0};

  // The lower layer's present facts for this tick.
  protocol::ExecutionState execution_state{protocol::ExecutionState::Boot};
  protocol::OutputAction output_action{protocol::OutputAction::ZeroTorque};
  protocol::StopReason stop_reason{protocol::StopReason::None};
  protocol::Sequence fault_id{0};
  bool healthy{false};
  bool damping_eligible{false};

  // The command that was in force: the identity it arrived under, the local
  // instant the runtime received it, and the state sequence its generator used.
  // `command_present` is false while no command is in force, which is not the
  // same as a command of all zeros.
  bool command_present{false};
  protocol::Id producer_id{};
  protocol::Sequence epoch{0};
  protocol::Sequence command_seq{0};
  protocol::Sequence source_state_seq{0};
  protocol::TimeUs command_rx_us{0};

  // The coherent feedback snapshot of this tick and whether it was usable. The
  // identity above is not the measurement: `measured_q`/`measured_dq` below are
  // what the machine reported, and they are meaningful only when
  // `feedback_valid` holds and only for the first `joint_count` joints.
  protocol::Sequence feedback_seq{0};
  protocol::TimeUs feedback_stamp_us{0};
  bool feedback_valid{false};
  bool imu_valid{false};

  // The final output of this very tick and the host send path's answer to it.
  // `send_accepted` is the host path's answer and is never a device
  // acknowledgement; `output_torque` is the torque the output stage asked the
  // backend to apply, not what the hardware measured.
  bool send_accepted{false};
  bool output_valid{false};

  // The target that was applied, in the configuration's joint order: the first
  // `joint_count` entries of each array are meaningful.
  std::uint32_t joint_count{0};
  std::array<double, protocol::kMaxJoints> q{};
  std::array<double, protocol::kMaxJoints> dq{};
  std::array<double, protocol::kMaxJoints> tau_ff{};
  std::array<double, protocol::kMaxJoints> kp{};
  std::array<double, protocol::kMaxJoints> kd{};
  // The feedback the tick actually used, in the same order. `measured_q` and
  // `measured_dq` are the measured joint position and velocity behind
  // `feedback_seq`, and are meaningful only while `feedback_valid` holds.
  std::array<double, protocol::kMaxJoints> measured_q{};
  std::array<double, protocol::kMaxJoints> measured_dq{};
  // The torques the output stage produced for the backend this tick, meaningful
  // only while `output_valid` holds.
  std::array<double, protocol::kMaxJoints> output_torque{};

  bool operator==(const RecordSample &other) const {
    return schema_version == other.schema_version &&
           stamp_us == other.stamp_us && state_seq == other.state_seq &&
           execution_state == other.execution_state &&
           output_action == other.output_action &&
           stop_reason == other.stop_reason && fault_id == other.fault_id &&
           healthy == other.healthy &&
           damping_eligible == other.damping_eligible &&
           command_present == other.command_present &&
           producer_id == other.producer_id && epoch == other.epoch &&
           command_seq == other.command_seq &&
           source_state_seq == other.source_state_seq &&
           command_rx_us == other.command_rx_us &&
           feedback_seq == other.feedback_seq &&
           feedback_stamp_us == other.feedback_stamp_us &&
           feedback_valid == other.feedback_valid &&
           imu_valid == other.imu_valid &&
           send_accepted == other.send_accepted &&
           output_valid == other.output_valid &&
           joint_count == other.joint_count && q == other.q && dq == other.dq &&
           tau_ff == other.tau_ff && kp == other.kp && kd == other.kd &&
           measured_q == other.measured_q && measured_dq == other.measured_dq &&
           output_torque == other.output_torque;
  }
};

// One authorization context the runtime adopted, for the recorder's session
// sidecar. A session is not a tick: the runtime emits one of these when the
// context it observes changes - a new epoch, a new producer, or the context
// ending - never once per recorded sample.
//
// The runtime holds no model and must not link the inference stack, so it has
// no digest to write and never invents one: `model_sha256` carries the literal
// "unknown". A real digest belongs to the application, the only side that
// loaded a model, and travels in the application's own session record. The type
// is fixed capacity and trivially copyable, so the control thread can push it
// onto the bounded session handoff without allocating, blocking or throwing.
struct RecordSessionEvent {
  std::uint32_t schema_version{kRecordSchemaVersion};
  // The runtime instance and session whose context this is, and the producer
  // and epoch that authorized it.
  protocol::Id runtime_id{};
  protocol::Id session_id{};
  protocol::Id producer_id{};
  protocol::Sequence epoch{0};
  // The tick the context was adopted on: its controller stamp and the sequence
  // the runtime published it under.
  protocol::TimeUs stamp_us{0};
  protocol::Sequence state_seq{0};
  // The gains of the first command adopted with this context, in the
  // configuration's canonical joint order: the first `joint_count` entries are
  // meaningful.
  std::uint32_t joint_count{0};
  std::array<double, protocol::kMaxJoints> kp{};
  std::array<double, protocol::kMaxJoints> kd{};
  // The model digest the application ran, or "unknown". Never derived here.
  std::array<char, kModelDigestBytes> model_sha256{};

  bool operator==(const RecordSessionEvent &other) const {
    return schema_version == other.schema_version &&
           runtime_id == other.runtime_id && session_id == other.session_id &&
           producer_id == other.producer_id && epoch == other.epoch &&
           stamp_us == other.stamp_us && state_seq == other.state_seq &&
           joint_count == other.joint_count && kp == other.kp &&
           kd == other.kd && model_sha256 == other.model_sha256;
  }
};

// Why samples are missing from a run.
enum class RecordGapReason : std::uint32_t {
  None = 0,
  // The handoff to the recorder's worker was full when the control thread
  // offered the sample: the tick ran on, and the loss is recorded rather than
  // paid for with a delay.
  QueueFull = 1,
  // The recorder could not write a chunk: a full disk, a permission change, a
  // removed directory. The control loop is unaffected and every later sample of
  // that stretch is missing too.
  WriteFailed = 2,
  // Recording stopped while samples were still arriving.
  Stopped = 3,
};

// One stretch of missing samples, in the sequence space the runtime published.
// A reader that walks a run reports its gaps instead of presenting the samples
// around them as consecutive.
struct RecordGap {
  protocol::Sequence first_seq{0};
  protocol::Sequence last_seq{0};
  protocol::TimeUs first_stamp_us{0};
  protocol::TimeUs last_stamp_us{0};
  std::uint64_t dropped{0};
  RecordGapReason reason{RecordGapReason::None};
};

// One chunk file as it lies on disk. The layout is fixed by the schema version
// and shared by the writer and every reader, so both sides agree on where a
// field is rather than each holding its own copy of the offsets.
//
// A chunk is a fixed header, then the payload: the fixed-width per-row fields,
// then the column-major joint columns. The header carries the payload's exact
// length and two CRC-32 checksums, so a reader can tell a corrupt chunk from a
// truncated one before it trusts a single row:
//
//   offset  size  field
//   0       8     magic (`kChunkMagic`)
//   8       4     schema version (u32)
//   12      4     joint count (u32)
//   16      4     row count (u32)
//   20      4     payload length in bytes (u32)
//   24      8     first stamp (u64)
//   32      8     last stamp (u64)
//   40      8     first state sequence (u64)
//   48      8     last state sequence (u64)
//   56      4     CRC-32 of the payload (u32)
//   60      4     CRC-32 of the header's first 60 bytes (u32)
//
// The header checksum covers the length field and every other structural field,
// which is what lets a corrupt length be named as corruption instead of being
// mistaken for a short file: a file cut short by a crash cannot have a valid
// header checksum over a wrong length, and a file whose length was edited
// cannot either. The payload checksum covers the rows; a chunk whose bytes no
// longer match the checksum they were written with contributes no rows.
inline constexpr char kChunkMagic[8] = {'I', 'R', 'M', 'V', 'C', 'H', '0', '1'};
// The fixed header's size, the payload's checksum offset and the header's own
// checksum offset. The payload follows the header.
inline constexpr std::size_t kChunkHeaderBytes = 64;
inline constexpr std::size_t kChunkPayloadCrcOffset = 56;
inline constexpr std::size_t kChunkHeaderCrcOffset = 60;
// The fixed-width part of one row, before the column-major joint columns: two
// u64, three u32, a u64, seven flag bytes, the producer identity and six u64.
inline constexpr std::size_t kChunkFixedRowBytes =
    2 * sizeof(std::uint64_t) + 3 * sizeof(std::uint32_t) +
    sizeof(std::uint64_t) + 7 * sizeof(std::uint8_t) + protocol::kIdSize +
    6 * sizeof(std::uint64_t);

// The payload length a chunk of `rows` rows and `joint_count` joints must
// declare: the fixed-width rows plus eight column-major double columns of
// `rows * joint_count` values. A header that declares another length cannot
// describe those rows, and the reader refuses it rather than reading past it.
inline constexpr std::uint64_t ChunkPayloadBytes(std::uint32_t rows,
                                                 std::uint32_t joint_count) {
  return static_cast<std::uint64_t>(rows) * kChunkFixedRowBytes +
         8 * static_cast<std::uint64_t>(rows) * joint_count * sizeof(double);
}

// The CRC-32 of the buffer (the IEEE 802.3 polynomial, reflected). A chunk
// carries one over its payload and one over its header, so a reader can tell
// corruption in place from a file that was cut short. It is table-free on
// purpose: a chunk is checksummed once as it is written and once as it is read,
// so a static table would buy microseconds at the cost of a shared mutable
// state that a control-side caller must not have.
inline std::uint32_t ChunkCrc32(const std::uint8_t *bytes,
                                std::size_t size) noexcept {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= bytes[index];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

// The name of a level of a run that a reader can find without a clock: the
// directory of one run and the files inside it.
inline constexpr std::string_view kRecordMetadataFile = "meta.json";
inline constexpr std::string_view kRecordIndexFile = "index.json";
inline constexpr std::string_view kRecordSessionsFile = "sessions.json";
inline constexpr std::string_view kRecordChunkPrefix = "chunk_";
inline constexpr std::string_view kRecordChunkSuffix = ".bin";

}  // namespace irmv::telemetry
