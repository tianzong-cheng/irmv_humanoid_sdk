#pragma once

#include <array>
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
inline constexpr std::uint32_t kRecordSchemaVersion = 1;

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

  // The coherent feedback snapshot of this tick and whether it was usable.
  protocol::Sequence feedback_seq{0};
  protocol::TimeUs feedback_stamp_us{0};
  bool feedback_valid{false};
  bool imu_valid{false};

  // Whether the host send path accepted the output of this tick. It is the host
  // path's answer, never a device acknowledgement.
  bool send_accepted{false};

  // The target that was applied, in the configuration's joint order: the first
  // `joint_count` entries of each array are meaningful.
  std::uint32_t joint_count{0};
  std::array<double, protocol::kMaxJoints> q{};
  std::array<double, protocol::kMaxJoints> dq{};
  std::array<double, protocol::kMaxJoints> tau_ff{};
  std::array<double, protocol::kMaxJoints> kp{};
  std::array<double, protocol::kMaxJoints> kd{};

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
           joint_count == other.joint_count && q == other.q && dq == other.dq &&
           tau_ff == other.tau_ff && kp == other.kp && kd == other.kd;
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

// The name of a level of a run that a reader can find without a clock: the
// directory of one run and the files inside it.
inline constexpr std::string_view kRecordMetadataFile = "meta.json";
inline constexpr std::string_view kRecordIndexFile = "index.json";
inline constexpr std::string_view kRecordChunkPrefix = "chunk_";
inline constexpr std::string_view kRecordChunkSuffix = ".bin";

}  // namespace irmv::telemetry
