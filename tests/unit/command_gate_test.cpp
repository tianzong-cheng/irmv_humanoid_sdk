#include "irmv/runtime/command_gate.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::LoadConfig;
using irmv::configuration::RuntimeConfig;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::RejectReason;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::runtime::CommandGate;
using irmv::runtime::GateDecision;
using irmv::runtime::GateResult;

// The fixture timeouts the literals below rely on.
constexpr TimeUs kCommandTimeoutUs = 100'000;
constexpr TimeUs kSourceTimeoutUs = 150'000;
constexpr irmv::protocol::Sequence kOwnerEpoch = 1;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Id MakeId(std::uint8_t seed) {
  Id id{};
  id.fill(seed);
  return id;
}

Id RuntimeId() { return MakeId(0x11); }
Id SessionId() { return MakeId(0x22); }
Id ProducerId() { return MakeId(0x33); }
Id OtherProducerId() { return MakeId(0x77); }
Id ForeignRuntimeId() { return MakeId(0x99); }
Id ForeignSessionId() { return MakeId(0x98); }

// A frame of the authorized owner with the identity and epoch the plan example
// authorizes (`Authorize(runtime, session, producer, 1)`).
JointCommand CommandFor(const RuntimeConfig &config) {
  JointCommand command;
  command.protocol_version = kProtocolVersion;
  command.runtime_id = RuntimeId();
  command.session_id = SessionId();
  command.producer_id = ProducerId();
  command.epoch = kOwnerEpoch;
  command.joints.count = static_cast<std::uint32_t>(config.joint_count());
  static_cast<void>(command.layout_id.Assign(config.layout_id));
  return command;
}

// Feeds one owner frame that must be rejected as an invalid owner command and
// verifies that neither the accepted target nor any deadline was refreshed.
int CheckRejectedOwnerFrame(const RuntimeConfig &config, const Id &runtime,
                            const Id &session, const Id &producer,
                            JointCommand frame, RejectReason expected) {
  CommandGate gate(config);
  gate.RecordState(10, 55'000);
  gate.Authorize(runtime, session, producer, 1);

  JointCommand valid = CommandFor(config);
  valid.command_seq = 3;
  valid.source_state_seq = 10;
  CHECK(gate.Accept({valid, 60'000}, 60'000).accepted());
  CHECK(gate.current_target().command_seq == 3);
  CHECK(gate.current_source_generation_us() == 55'000);

  frame.source_state_seq = 10;
  frame.command_seq = 9;
  const GateResult result = gate.Accept({frame, 200'000}, 200'000);
  CHECK(!result.accepted());
  CHECK(result.decision == GateDecision::InvalidOwnerCommand);
  CHECK(result.reason == expected);

  // The rejected frame refreshed nothing: deadline, target and stored source
  // generation time are all unchanged.
  CHECK(gate.last_valid_command_rx_us() == 60'000);
  CHECK(gate.current_source_generation_us() == 55'000);
  CHECK(gate.current_target().command_seq == 3);
  CHECK(gate.ExpireReason(160'000) == StopReason::None);
  CHECK(gate.ExpireReason(160'001) == StopReason::CommandTimeout);

  // Because the rejected frame did not advance the accepted sequence, the same
  // sequence is still usable once the payload is valid again.
  JointCommand retry = valid;
  retry.command_seq = 9;
  CHECK(gate.Accept({retry, 70'000}, 70'000).accepted());
  CHECK(gate.last_valid_command_rx_us() == 70'000);
  return 0;
}

}  // namespace

int main() {
  const RuntimeConfig config = LoadConfig("config/sdk.json");
  CHECK(config.joint_count() == 2);
  CHECK(config.timeouts.command_us == kCommandTimeoutUs);
  CHECK(config.timeouts.source_state_us == kSourceTimeoutUs);
  CHECK(config.state_window_capacity == 1024);
  CHECK(config.position_max[0] == 2.0);
  CHECK(config.target_velocity_limits[0] == 10.0);

  const Id runtime = RuntimeId();
  const Id session = SessionId();
  const Id producer = ProducerId();

  // ---- plan example ----
  {
    CommandGate gate(config);
    JointCommand command = CommandFor(config);
    gate.RecordState(10, 1000);
    gate.Authorize(runtime, session, producer, 1);
    command.source_state_seq = 10;
    CHECK(gate.Accept({command, 2000}, 2000).accepted());
    CHECK(!gate.Accept({command, 90000}, 90000).accepted());
    CHECK(gate.ExpireReason(102001) == StopReason::CommandTimeout);
  }

  // ---- sequence monotonicity and the queue wait ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 2'000);
    gate.RecordState(11, 53'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand first = CommandFor(config);
    first.source_state_seq = 10;
    first.command_seq = 5;

    // The packet waited in the queue: it is accepted long after it was
    // received, and the validity origin is the transport receive stamp.
    CHECK(gate.Accept({first, 2'000}, 50'000).accepted());
    CHECK(gate.last_valid_command_rx_us() == 2'000);
    CHECK(gate.current_source_generation_us() == 2'000);
    CHECK(gate.ExpireReason(2'000 + kCommandTimeoutUs) == StopReason::None);
    CHECK(gate.ExpireReason(2'001 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);

    // A repeated sequence is ignored and must not refresh the deadline.
    const GateResult duplicate = gate.Accept({first, 51'000}, 51'000);
    CHECK(!duplicate.accepted());
    CHECK(duplicate.decision == GateDecision::Ignored);
    CHECK(duplicate.reason == RejectReason::StaleRequest);
    CHECK(gate.last_valid_command_rx_us() == 2'000);

    JointCommand older = first;
    older.command_seq = 4;
    CHECK(gate.Accept({older, 52'000}, 52'000).reason ==
          RejectReason::StaleRequest);
    CHECK(gate.last_valid_command_rx_us() == 2'000);

    JointCommand newer = first;
    newer.command_seq = 6;
    newer.source_state_seq = 11;
    CHECK(gate.Accept({newer, 53'000}, 53'000).accepted());
    CHECK(gate.last_valid_command_rx_us() == 53'000);
    CHECK(gate.current_source_generation_us() == 53'000);
    CHECK(gate.ExpireReason(53'000 + kCommandTimeoutUs) == StopReason::None);
    CHECK(gate.ExpireReason(53'001 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);
  }

  // ---- source state index: unknown, ignored records, stale, fresh ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 1'000);
    // Equal and older sequences are ignored, so the generation time of
    // sequence 10 stays the first recorded one.
    gate.RecordState(10, 2'000);
    gate.RecordState(9, 3'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand command = CommandFor(config);

    command.source_state_seq = 9;
    const GateResult unknown = gate.Accept({command, 151'500}, 151'500);
    CHECK(unknown.decision == GateDecision::Ignored);
    CHECK(unknown.reason == RejectReason::StaleSource);
    CHECK(!gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 0);

    // Sequence 10 is only fresh through its first recorded generation time.
    command.source_state_seq = 10;
    CHECK(gate.Accept({command, 151'500}, 151'500).reason ==
          RejectReason::StaleSource);
    CHECK(!gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 0);

    gate.RecordState(11, 151'000);
    command.source_state_seq = 11;
    CHECK(gate.Accept({command, 151'500}, 151'500).accepted());
    CHECK(gate.current_source_generation_us() == 151'000);
    CHECK(gate.last_valid_command_rx_us() == 151'500);

    // The command deadline is stricter than the source deadline here, so it
    // wins and is itself strict at the boundary.
    CHECK(gate.ExpireReason(151'500 + kCommandTimeoutUs) == StopReason::None);
    CHECK(gate.ExpireReason(151'501 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);

    // Freshness accepts exactly source_state_us of age and rejects one
    // microsecond more.
    gate.RecordState(12, 150'000);
    JointCommand boundary_frame = command;
    boundary_frame.source_state_seq = 12;
    boundary_frame.command_seq = 1;
    CHECK(gate.Accept({boundary_frame, 300'000}, 300'000).accepted());
    CHECK(gate.current_source_generation_us() == 150'000);

    JointCommand one_microsecond_late = boundary_frame;
    one_microsecond_late.command_seq = 2;
    const GateResult late =
        gate.Accept({one_microsecond_late, 300'001}, 300'001);
    CHECK(late.decision == GateDecision::Ignored);
    CHECK(late.reason == RejectReason::StaleSource);
    CHECK(gate.last_valid_command_rx_us() == 300'000);
  }

  // ---- eviction of the source index entry keeps the accepted deadlines ----
  {
    CommandGate gate(config);
    gate.RecordState(100, 1'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand command = CommandFor(config);
    command.source_state_seq = 100;
    CHECK(gate.Accept({command, 60'000}, 60'000).accepted());
    CHECK(gate.current_source_generation_us() == 1'000);

    // Push the recorded sequence out of the bounded window.
    for (std::uint64_t offset = 1; offset <= config.state_window_capacity + 1;
         ++offset) {
      gate.RecordState(100 + offset, 60'000 + offset);
    }

    // The accepted target still expires at its original generation time.
    CHECK(gate.current_source_generation_us() == 1'000);
    CHECK(gate.last_valid_command_rx_us() == 60'000);
    CHECK(gate.ExpireReason(1'000 + kSourceTimeoutUs) == StopReason::None);
    CHECK(gate.ExpireReason(1'001 + kSourceTimeoutUs) ==
          StopReason::SourceStateTimeout);
    CHECK(gate.ExpireReason(60'001 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);

    // A new frame for the evicted sequence is rejected as stale source.
    JointCommand evicted = command;
    evicted.command_seq = 1;
    const GateResult after_eviction = gate.Accept({evicted, 200'000}, 200'000);
    CHECK(after_eviction.decision == GateDecision::Ignored);
    CHECK(after_eviction.reason == RejectReason::StaleSource);
    CHECK(gate.current_source_generation_us() == 1'000);
    CHECK(gate.last_valid_command_rx_us() == 60'000);
  }

  // ---- expiry boundaries ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 55'000);
    gate.Authorize(runtime, session, producer, 1);
    JointCommand command = CommandFor(config);
    command.source_state_seq = 10;
    CHECK(gate.Accept({command, 60'000}, 60'000).accepted());
    CHECK(gate.ExpireReason(60'000 + kCommandTimeoutUs) == StopReason::None);
    CHECK(gate.ExpireReason(60'001 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);

    // A target adopted from an older source expires on its source instead.
    CommandGate source_gate(config);
    source_gate.RecordState(4, 1'000);
    source_gate.Authorize(runtime, session, producer, 1);
    JointCommand sourced = CommandFor(config);
    sourced.source_state_seq = 4;
    CHECK(source_gate.Accept({sourced, 60'000}, 60'000).accepted());
    CHECK(source_gate.ExpireReason(1'000 + kSourceTimeoutUs) ==
          StopReason::None);
    CHECK(source_gate.ExpireReason(1'001 + kSourceTimeoutUs) ==
          StopReason::SourceStateTimeout);
    CHECK(source_gate.ExpireReason(60'001 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);

    // Nothing authorized: no targets, no deadlines, no ownership.
    CommandGate unowned(config);
    const GateResult result = unowned.Accept({command, 60'000}, 60'000);
    CHECK(result.decision == GateDecision::Ignored);
    CHECK(result.reason == RejectReason::WrongInstance);
    CHECK(!unowned.authorized());
    CHECK(!unowned.has_target());
    CHECK(unowned.last_valid_command_rx_us() == 0);
    CHECK(unowned.ExpireReason(100'000'000) == StopReason::None);
  }

  // ---- revoke, new epoch and ownership reasons ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 1'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand first = CommandFor(config);
    first.source_state_seq = 10;
    first.command_seq = 7;
    CHECK(gate.Accept({first, 60'000}, 60'000).accepted());
    CHECK(gate.authorized());
    CHECK(gate.has_target());
    CHECK(gate.ExpireReason(1'001 + kSourceTimeoutUs) ==
          StopReason::SourceStateTimeout);

    gate.Revoke();
    CHECK(!gate.authorized());
    CHECK(!gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 0);
    CHECK(gate.current_source_generation_us() == 0);
    CHECK(gate.ExpireReason(1'001 + kSourceTimeoutUs) == StopReason::None);
    CHECK(gate.ExpireReason(100'000'000) == StopReason::None);

    JointCommand after_revoke = first;
    after_revoke.command_seq = 8;
    const GateResult revoked = gate.Accept({after_revoke, 61'000}, 61'000);
    CHECK(revoked.decision == GateDecision::Ignored);
    CHECK(revoked.reason == RejectReason::WrongInstance);
    CHECK(!gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 0);

    // A new epoch starts from a clean sequence state: its first frame is
    // accepted although its command_seq is lower than the sequence accepted
    // before the revoke.
    gate.Authorize(runtime, session, producer, 2);
    CHECK(gate.authorized());
    CHECK(!gate.has_target());

    JointCommand old_epoch = first;
    const GateResult epoch_result = gate.Accept({old_epoch, 62'000}, 62'000);
    CHECK(epoch_result.decision == GateDecision::Ignored);
    CHECK(epoch_result.reason == RejectReason::EpochMismatch);
    CHECK(!gate.has_target());

    JointCommand new_epoch = first;
    new_epoch.epoch = 2;
    new_epoch.command_seq = 3;
    CHECK(gate.Accept({new_epoch, 63'000}, 63'000).accepted());
    CHECK(gate.has_target());
    CHECK(gate.current_target().epoch == 2);
    CHECK(gate.last_valid_command_rx_us() == 63'000);
    CHECK(gate.Accept({new_epoch, 64'000}, 64'000).reason ==
          RejectReason::StaleRequest);

    // Another producer takes over: the old producer is ignored, the new one is
    // accepted even for an already used sequence number.
    const Id other_producer = OtherProducerId();
    gate.Authorize(runtime, session, other_producer, 3);
    JointCommand old_producer = new_epoch;
    old_producer.command_seq = 4;
    old_producer.epoch = 3;
    const GateResult producer_result =
        gate.Accept({old_producer, 65'000}, 65'000);
    CHECK(producer_result.decision == GateDecision::Ignored);
    CHECK(producer_result.reason == RejectReason::WrongInstance);
    CHECK(gate.last_valid_command_rx_us() == 0);

    JointCommand new_producer = old_producer;
    new_producer.producer_id = other_producer;
    new_producer.epoch = 3;
    CHECK(gate.Accept({new_producer, 66'000}, 66'000).accepted());
    CHECK(gate.last_valid_command_rx_us() == 66'000);

    JointCommand foreign_runtime = new_producer;
    foreign_runtime.command_seq = 5;
    foreign_runtime.runtime_id = ForeignRuntimeId();
    CHECK(gate.Accept({foreign_runtime, 67'000}, 67'000).reason ==
          RejectReason::WrongInstance);

    JointCommand foreign_session = new_producer;
    foreign_session.command_seq = 5;
    foreign_session.session_id = ForeignSessionId();
    CHECK(gate.Accept({foreign_session, 67'000}, 67'000).reason ==
          RejectReason::WrongSession);

    JointCommand old_version = new_producer;
    old_version.command_seq = 5;
    old_version.protocol_version = kProtocolVersion + 1;
    const GateResult version_result =
        gate.Accept({old_version, 67'000}, 67'000);
    CHECK(version_result.decision == GateDecision::Ignored);
    CHECK(version_result.reason == RejectReason::Malformed);
    CHECK(gate.last_valid_command_rx_us() == 66'000);

    JointCommand next = new_producer;
    next.command_seq = 5;
    CHECK(gate.Accept({next, 68'000}, 68'000).accepted());
  }

  // ---- foreign and old-context garbage never faults the owner ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 55'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand garbage = CommandFor(config);
    garbage.source_state_seq = 10;
    garbage.runtime_id = ForeignRuntimeId();
    garbage.joints.q[0] = kNaN;
    garbage.joints.kp[1] = -1.0;
    garbage.joints.kd[1] = -1.0;
    const GateResult foreign = gate.Accept({garbage, 200'000}, 200'000);
    CHECK(foreign.decision == GateDecision::Ignored);
    CHECK(foreign.reason == RejectReason::WrongInstance);
    CHECK(!gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 0);
    CHECK(gate.current_source_generation_us() == 0);
    CHECK(gate.ExpireReason(200'000) == StopReason::None);

    JointCommand wrong_epoch = garbage;
    wrong_epoch.runtime_id = runtime;
    wrong_epoch.epoch = 0;
    const GateResult epoch_result =
        gate.Accept({wrong_epoch, 201'000}, 201'000);
    CHECK(epoch_result.decision == GateDecision::Ignored);
    CHECK(epoch_result.reason == RejectReason::EpochMismatch);
    CHECK(!gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 0);

    JointCommand wrong_producer = garbage;
    wrong_producer.runtime_id = runtime;
    wrong_producer.epoch = 1;
    wrong_producer.producer_id = OtherProducerId();
    CHECK(gate.Accept({wrong_producer, 202'000}, 202'000).reason ==
          RejectReason::WrongInstance);
    CHECK(!gate.has_target());

    // The owner is still fully usable: the foreign garbage neither faulted it
    // nor consumed the sequence number.
    JointCommand good = CommandFor(config);
    good.source_state_seq = 10;
    CHECK(gate.Accept({good, 60'000}, 60'000).accepted());
    CHECK(gate.has_target());
    CHECK(gate.last_valid_command_rx_us() == 60'000);
    CHECK(gate.current_source_generation_us() == 55'000);
    CHECK(gate.ExpireReason(60'001 + kCommandTimeoutUs) ==
          StopReason::CommandTimeout);
  }

  // ---- owner frames that must be rejected as invalid ----
  {
    JointCommand nan_position = CommandFor(config);
    nan_position.joints.q[0] = kNaN;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  nan_position,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand inf_velocity = CommandFor(config);
    inf_velocity.joints.dq[1] = kInf;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  inf_velocity,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand nan_gain = CommandFor(config);
    nan_gain.joints.kd[0] = kNaN;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer, nan_gain,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand negative_kp = CommandFor(config);
    negative_kp.joints.kp[0] = -1.0;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  negative_kp,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand negative_kd = CommandFor(config);
    negative_kd.joints.kd[1] = -0.5;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  negative_kd,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand above_max = CommandFor(config);
    above_max.joints.q[0] = config.position_max[0] + 0.001;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer, above_max,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand below_min = CommandFor(config);
    below_min.joints.q[1] = config.position_min[1] - 0.001;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer, below_min,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand fast = CommandFor(config);
    fast.joints.dq[0] = config.target_velocity_limits[0] + 1e-9;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer, fast,
                                  RejectReason::InvalidCommand) == 0);

    JointCommand other_layout = CommandFor(config);
    CHECK(other_layout.layout_id.Assign("other_layout"));
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  other_layout, RejectReason::Malformed) == 0);

    JointCommand short_count = CommandFor(config);
    short_count.joints.count = 1;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  short_count, RejectReason::Malformed) == 0);

    JointCommand zero_count = CommandFor(config);
    zero_count.joints.count = 0;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  zero_count, RejectReason::Malformed) == 0);

    JointCommand huge_count = CommandFor(config);
    huge_count.joints.count = 65;
    CHECK(CheckRejectedOwnerFrame(config, runtime, session, producer,
                                  huge_count, RejectReason::Malformed) == 0);
  }

  // ---- an empty command is classified by ownership before shape ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 55'000);
    gate.Authorize(runtime, session, producer, 1);
    JointCommand valid = CommandFor(config);
    valid.source_state_seq = 10;
    valid.command_seq = 3;
    CHECK(gate.Accept({valid, 60'000}, 60'000).accepted());

    // A foreign empty frame cannot fault the owner: ownership is checked
    // before the payload shape. The accepted target and deadline stay intact.
    JointCommand foreign_empty = valid;
    foreign_empty.producer_id = OtherProducerId();
    foreign_empty.joints.count = 0;
    foreign_empty.command_seq = 9;
    const GateResult foreign = gate.Accept({foreign_empty, 200'000}, 200'000);
    CHECK(foreign.decision == GateDecision::Ignored);
    CHECK(foreign.reason == RejectReason::WrongInstance);
    CHECK(gate.current_target().command_seq == 3);
    CHECK(gate.last_valid_command_rx_us() == 60'000);

    // The current owner's empty frame is an invalid owner command: the runtime
    // latches CommandInvalid immediately rather than waiting for a timeout.
    JointCommand owner_empty = valid;
    owner_empty.joints.count = 0;
    owner_empty.command_seq = 9;
    const GateResult owner = gate.Accept({owner_empty, 200'000}, 200'000);
    CHECK(owner.decision == GateDecision::InvalidOwnerCommand);
    CHECK(owner.reason == RejectReason::Malformed);
    CHECK(gate.current_target().command_seq == 3);
    CHECK(gate.last_valid_command_rx_us() == 60'000);
  }

  // ---- limits are inclusive and the accepted frame is stored by value ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 55'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand boundary = CommandFor(config);
    boundary.source_state_seq = 10;
    boundary.joints.q[0] = config.position_max[0];
    boundary.joints.q[1] = config.position_min[1];
    boundary.joints.dq[0] = config.target_velocity_limits[0];
    boundary.joints.dq[1] = -config.target_velocity_limits[1];
    boundary.joints.kp[0] = 4.0;
    boundary.joints.tau_ff[1] = 1.5;
    CHECK(gate.Accept({boundary, 60'000}, 60'000).accepted());

    boundary.joints.kp[0] = 42.0;
    boundary.joints.tau_ff[1] = 42.0;
    CHECK(gate.current_target().joints.kp[0] == 4.0);
    CHECK(gate.current_target().joints.tau_ff[1] == 1.5);
    CHECK(gate.current_target().joints.dq[0] ==
          config.target_velocity_limits[0]);
    CHECK(gate.current_target().source_state_seq == 10);
  }

  // ---- identity, sequence and freshness stages run before payload validation
  // ----
  {
    CommandGate gate(config);
    gate.RecordState(10, 55'000);
    gate.Authorize(runtime, session, producer, 1);

    JointCommand accepted = CommandFor(config);
    accepted.command_seq = 4;
    accepted.source_state_seq = 10;
    CHECK(gate.Accept({accepted, 60'000}, 60'000).accepted());

    // A bad payload with an unknown source state is rejected by the freshness
    // stage, never classified as an owner payload fault.
    JointCommand bad_unknown_source = CommandFor(config);
    bad_unknown_source.command_seq = 5;
    bad_unknown_source.source_state_seq = 999;
    bad_unknown_source.joints.q[0] = kNaN;
    const GateResult unknown_source =
        gate.Accept({bad_unknown_source, 61'000}, 61'000);
    CHECK(unknown_source.decision == GateDecision::Ignored);
    CHECK(unknown_source.reason == RejectReason::StaleSource);

    // A bad payload carrying an already accepted sequence is rejected by the
    // sequence stage.
    JointCommand bad_duplicate = CommandFor(config);
    bad_duplicate.command_seq = 4;
    bad_duplicate.source_state_seq = 10;
    bad_duplicate.joints.kp[0] = -1.0;
    const GateResult duplicate = gate.Accept({bad_duplicate, 62'000}, 62'000);
    CHECK(duplicate.decision == GateDecision::Ignored);
    CHECK(duplicate.reason == RejectReason::StaleRequest);

    // Neither rejection touched the accepted target or any deadline.
    CHECK(gate.current_target().command_seq == 4);
    CHECK(gate.last_valid_command_rx_us() == 60'000);
    CHECK(gate.current_source_generation_us() == 55'000);
  }

  // ---- the source index is clamped to the hard 8192-entry cap ----
  {
    RuntimeConfig wide = config;
    wide.state_window_capacity = 9'000;
    CommandGate gate(wide);
    constexpr TimeUs kNow = 1'000'000;
    for (std::uint64_t seq = 1; seq <= 8'242; ++seq) {
      gate.RecordState(seq, kNow - 1'000);
    }
    gate.Authorize(runtime, session, producer, 1);

    JointCommand newest = CommandFor(wide);
    newest.command_seq = 1;
    newest.source_state_seq = 8'242;
    CHECK(gate.Accept({newest, kNow}, kNow).accepted());

    // With the hard cap the retained window is 51..8242, so 50 is gone even
    // though the configured capacity of 9000 would have kept it.
    JointCommand evicted = CommandFor(wide);
    evicted.command_seq = 2;
    evicted.source_state_seq = 50;
    const GateResult dropped = gate.Accept({evicted, kNow}, kNow);
    CHECK(dropped.decision == GateDecision::Ignored);
    CHECK(dropped.reason == RejectReason::StaleSource);
    CHECK(gate.current_target().command_seq == 1);
  }

  return 0;
}
