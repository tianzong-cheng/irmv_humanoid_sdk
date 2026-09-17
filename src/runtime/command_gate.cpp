#include "irmv/runtime/command_gate.hpp"

#include <algorithm>
#include <cstddef>

#include "irmv/runtime/safety.hpp"

namespace irmv::runtime {

CommandGate::CommandGate(const configuration::RuntimeConfig &config)
    : config_(config),
      capacity_(
          std::min(config.state_window_capacity, kMaxStateWindowCapacity)) {}

void CommandGate::RecordState(protocol::Sequence snapshot_seq,
                              protocol::TimeUs generation_us) {
  if (capacity_ == 0) {
    return;
  }
  if (has_state_ && snapshot_seq <= newest_seq_) {
    return;
  }

  const std::size_t tail = (head_ + size_) % capacity_;
  entries_[tail] = StateEntry{snapshot_seq, generation_us};
  if (size_ < capacity_) {
    ++size_;
  } else {
    head_ = (head_ + 1) % capacity_;
  }
  newest_seq_ = snapshot_seq;
  has_state_ = true;
}

bool CommandGate::FindGeneration(protocol::Sequence snapshot_seq,
                                 protocol::TimeUs *generation_us) const {
  if (size_ == 0) {
    return false;
  }
  // Entries are stored in insertion order, so sequence numbers increase from
  // head_ to the end of the array and continue at index 0 once the ring wraps.
  // Each of those two spans is individually sorted, so both are searched as
  // independent segments.
  const std::size_t first_length = std::min(size_, capacity_ - head_);
  const std::size_t lengths[2] = {first_length, size_ - first_length};
  const std::size_t offsets[2] = {head_, 0};
  for (std::size_t segment = 0; segment < 2; ++segment) {
    if (lengths[segment] == 0) {
      continue;
    }
    const StateEntry *begin = entries_.data() + offsets[segment];
    const StateEntry *end = begin + lengths[segment];
    const StateEntry probe{snapshot_seq, 0};
    const StateEntry *found = std::lower_bound(
        begin, end, probe,
        [](const StateEntry &left, const StateEntry &right) {
          return left.seq < right.seq;
        });
    if (found != end && found->seq == snapshot_seq) {
      *generation_us = found->generation_us;
      return true;
    }
  }
  return false;
}

void CommandGate::Authorize(protocol::Id runtime_id, protocol::Id session_id,
                            protocol::Id producer_id,
                            protocol::Sequence epoch) {
  authorized_ = true;
  owner_runtime_ = runtime_id;
  owner_session_ = session_id;
  owner_producer_ = producer_id;
  owner_epoch_ = epoch;
  target_ = protocol::JointCommand{};
  has_target_ = false;
  last_command_seq_ = 0;
  last_valid_rx_us_ = 0;
  source_generation_us_ = 0;
}

bool CommandGate::GenerationOf(protocol::Sequence snapshot_seq,
                              protocol::TimeUs *generation_us) const {
  if (generation_us == nullptr) {
    return false;
  }
  return FindGeneration(snapshot_seq, generation_us);
}

GateResult CommandGate::Accept(const Received<protocol::JointCommand> &packet,
                               protocol::TimeUs now_us) {
  const protocol::JointCommand &command = packet.value;
  if (command.protocol_version != protocol::kProtocolVersion) {
    return {GateDecision::Ignored, protocol::RejectReason::Malformed};
  }
  // Ownership first: a foreign or old-context packet is dropped without
  // inspecting its payload, so it can neither fault the owner nor refresh a
  // deadline.
  if (!authorized_) {
    return {GateDecision::Ignored, protocol::RejectReason::WrongInstance};
  }
  if (command.runtime_id != owner_runtime_) {
    return {GateDecision::Ignored, protocol::RejectReason::WrongInstance};
  }
  if (command.session_id != owner_session_) {
    return {GateDecision::Ignored, protocol::RejectReason::WrongSession};
  }
  if (command.epoch != owner_epoch_) {
    return {GateDecision::Ignored, protocol::RejectReason::EpochMismatch};
  }
  if (command.producer_id != owner_producer_) {
    return {GateDecision::Ignored, protocol::RejectReason::WrongInstance};
  }

  if (has_target_ && command.command_seq <= last_command_seq_) {
    return {GateDecision::Ignored, protocol::RejectReason::StaleRequest};
  }

  protocol::TimeUs generation_us = 0;
  if (!FindGeneration(command.source_state_seq, &generation_us) ||
      now_us - generation_us > config_.timeouts.source_state_us) {
    return {GateDecision::Ignored, protocol::RejectReason::StaleSource};
  }

  const protocol::ValidationResult validation =
      ValidateTrackingCommand(config_, command);
  if (!validation.ok) {
    return {GateDecision::InvalidOwnerCommand, validation.reason};
  }

  target_ = command;
  has_target_ = true;
  last_command_seq_ = command.command_seq;
  last_valid_rx_us_ = packet.rx_us;
  source_generation_us_ = generation_us;
  return {GateDecision::Accepted, protocol::RejectReason::None};
}

protocol::StopReason CommandGate::ExpireReason(protocol::TimeUs now_us) const {
  if (!authorized_ || !has_target_) {
    return protocol::StopReason::None;
  }
  if (now_us - last_valid_rx_us_ > config_.timeouts.command_us) {
    return protocol::StopReason::CommandTimeout;
  }
  if (now_us - source_generation_us_ > config_.timeouts.source_state_us) {
    return protocol::StopReason::SourceStateTimeout;
  }
  return protocol::StopReason::None;
}

void CommandGate::Revoke() {
  authorized_ = false;
  owner_runtime_ = protocol::kNoId;
  owner_session_ = protocol::kNoId;
  owner_producer_ = protocol::kNoId;
  owner_epoch_ = 0;
  target_ = protocol::JointCommand{};
  has_target_ = false;
  last_command_seq_ = 0;
  last_valid_rx_us_ = 0;
  source_generation_us_ = 0;
}

}  // namespace irmv::runtime
