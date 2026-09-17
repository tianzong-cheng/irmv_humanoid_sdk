#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::runtime {

// A payload together with the host receive timestamp of the packet that carried
// it. The timestamp is the transport receive stamp, not the time the payload is
// consumed.
template <typename T>
struct Received {
  T value{};
  protocol::TimeUs rx_us{0};
};

enum class GateDecision : std::uint32_t {
  Accepted = 0,
  Ignored = 1,
  InvalidOwnerCommand = 2,
};

struct GateResult {
  GateDecision decision{GateDecision::Ignored};
  protocol::RejectReason reason{protocol::RejectReason::None};

  bool accepted() const { return decision == GateDecision::Accepted; }

  bool invalid_owner_command() const {
    return decision == GateDecision::InvalidOwnerCommand;
  }
};

// Gates incoming joint commands on ownership, sequence and source freshness,
// and tracks the deadlines of the last accepted target. All times are passed in
// by the caller: the gate reads no clock, allocates nothing and never throws.
//
// The gate stores a const reference to the immutable configuration snapshot,
// which is loaded once at startup and never mutated; the referenced config must
// outlive the gate. The state index, the accepted target and the owner context
// are owned by value and bounded to fixed storage.
class CommandGate {
 public:
  // Hard cap on the source-state index, independent of the configuration.
  static constexpr std::size_t kMaxStateWindowCapacity = 8192;

  explicit CommandGate(const configuration::RuntimeConfig &config);

  // Records a locally generated state snapshot. The index keeps at most
  // min(config.state_window_capacity, kMaxStateWindowCapacity) distinct
  // sequences; sequences that are not strictly newer than the newest recorded
  // one are ignored.
  void RecordState(protocol::Sequence snapshot_seq,
                   protocol::TimeUs generation_us);

  // Takes ownership for the given runtime instance, session, producer and
  // epoch, and clears the accepted target so that the first identity-matching
  // packet of the new epoch is accepted regardless of its command_seq.
  void Authorize(protocol::Id runtime_id, protocol::Id session_id,
                 protocol::Id producer_id, protocol::Sequence epoch);

  // Classifies one received packet. Only an Accepted packet refreshes the
  // command deadline and the stored source generation time.
  GateResult Accept(const Received<protocol::JointCommand> &packet,
                    protocol::TimeUs now_us);

  // Deadline of the accepted target, checked in this order: no authorization or
  // no accepted target reports None, the command timeout takes precedence over
  // the source-state timeout. Both comparisons are strict.
  protocol::StopReason ExpireReason(protocol::TimeUs now_us) const;

  // Drops the owner context, the accepted target and all deadlines.
  void Revoke();

  bool authorized() const { return authorized_; }

  bool has_target() const { return has_target_; }

  // Last accepted frame; a default-constructed command while has_target() is
  // false.
  const protocol::JointCommand &current_target() const { return target_; }

  // Transport receive stamp of the last accepted frame, zero while no target
  // has been accepted.
  protocol::TimeUs last_valid_command_rx_us() const {
    return last_valid_rx_us_;
  }

  // Generation time of the source state of the last accepted frame, zero while
  // no target has been accepted. It survives eviction of its index entry.
  protocol::TimeUs current_source_generation_us() const {
    return source_generation_us_;
  }

 private:
  struct StateEntry {
    protocol::Sequence seq{0};
    protocol::TimeUs generation_us{0};
  };

  // Looks up the generation time of a recorded snapshot sequence.
  bool FindGeneration(protocol::Sequence snapshot_seq,
                      protocol::TimeUs *generation_us) const;

  const configuration::RuntimeConfig &config_;
  std::array<StateEntry, kMaxStateWindowCapacity> entries_{};
  std::size_t head_{0};
  std::size_t size_{0};
  std::size_t capacity_{0};
  protocol::Sequence newest_seq_{0};
  bool has_state_{false};

  bool authorized_{false};
  protocol::Id owner_runtime_{};
  protocol::Id owner_session_{};
  protocol::Id owner_producer_{};
  protocol::Sequence owner_epoch_{0};

  protocol::JointCommand target_{};
  bool has_target_{false};
  protocol::Sequence last_command_seq_{0};
  protocol::TimeUs last_valid_rx_us_{0};
  protocol::TimeUs source_generation_us_{0};
};

}  // namespace irmv::runtime
