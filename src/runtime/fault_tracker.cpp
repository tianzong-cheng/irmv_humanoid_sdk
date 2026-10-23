#include "irmv/runtime/fault_tracker.hpp"

#include <cstddef>
#include <cstdint>

namespace irmv::runtime {

void AppendFaultObservation(FaultObservationBatch &batch,
                            const protocol::FaultDetail &detail) noexcept {
  if (batch.count >= kFaultObservationCapacity) {
    ++batch.dropped;
    return;
  }
  batch.details[batch.count] = detail;
  ++batch.count;
}

bool FaultTracker::ErrorKeyed(protocol::FaultCondition condition) noexcept {
  return condition == protocol::FaultCondition::JointDriverError ||
         condition == protocol::FaultCondition::PrimaryWriteFailed ||
         condition == protocol::FaultCondition::FallbackWriteFailed;
}

std::uint32_t FaultTracker::HashKey(protocol::FaultCondition condition,
                                    std::uint32_t joint_index,
                                    std::uint32_t error_code) noexcept {
  std::uint32_t hash = static_cast<std::uint32_t>(condition) * 0x9e3779b1U;
  hash += joint_index * 0x85ebca6bU;
  hash += error_code * 0xc2b2ae35U;
  hash ^= hash >> 16;
  hash *= 0x7feb352dU;
  hash ^= hash >> 15;
  hash *= 0x846ca68bU;
  hash ^= hash >> 16;
  return hash;
}

FaultTracker::Slot *FaultTracker::AcquireSlot(
    const protocol::FaultDetail &detail, bool *created) noexcept {
  *created = false;
  const std::uint32_t error_code =
      ErrorKeyed(detail.condition) ? detail.error_code : 0;
  const std::size_t mask = kSlotCapacity - 1;
  const std::size_t start =
      HashKey(detail.condition, detail.joint_index, error_code) & mask;
  for (std::size_t probe = 0; probe < kSlotCapacity; ++probe) {
    Slot &slot = slots_[(start + probe) & mask];
    if (!slot.used) {
      slot.used = true;
      slot.condition = detail.condition;
      slot.joint_index = detail.joint_index;
      slot.error_code = error_code;
      slot.last_seen_tick = 0;
      *created = true;
      return &slot;
    }
    if (slot.condition == detail.condition &&
        slot.joint_index == detail.joint_index &&
        slot.error_code == error_code) {
      return &slot;
    }
  }

  // The table is full. Only a key that has been absent for a full tick can be
  // evicted: a slot last seen in the previous tick may still be observed later
  // in this tick, and evicting it would report the sustained condition as new.
  // (BeginTick clears the whole table when the generation counter wraps, so
  // generations never cross.) The first comparison guards the UINT32_MAX wrap:
  // without it, last_seen_tick + 1 of a slot seen in this tick would overflow
  // to 0, pass the comparison and evict a slot that is still active.
  for (Slot &slot : slots_) {
    if (slot.last_seen_tick != tick_ && slot.last_seen_tick + 1 < tick_) {
      slot.condition = detail.condition;
      slot.joint_index = detail.joint_index;
      slot.error_code = error_code;
      slot.last_seen_tick = 0;
      *created = true;
      return &slot;
    }
  }

  // Every slot is still needed: the new key cannot be tracked without losing
  // the activity of a condition that may be observed later. It is reported as
  // not new and the loss is counted so the caller can see it.
  return nullptr;
}

void FaultTracker::BeginTick() noexcept {
  observations_.count = 0;
  observations_.dropped = 0;
  ++tick_;
  if (tick_ == 0) {
    // After four billion ticks the generation numbers would repeat and a stale
    // slot could look active again. Clearing the table once is cheaper than
    // widening every slot to a 64-bit counter.
    for (Slot &slot : slots_) {
      slot = Slot{};
    }
    tick_ = 1;
  }
}

bool FaultTracker::Observe(const protocol::FaultDetail &detail) noexcept {
  bool created = false;
  Slot *slot = AcquireSlot(detail, &created);
  if (slot == nullptr) {
    // The key cannot be tracked without evicting an active condition. It is
    // not reported as new and the loss is visible in the tick's counter.
    ++observations_.dropped;
    return false;
  }
  // A duplicate inside one tick is never new and is never stored twice.
  if (!created && slot->last_seen_tick == tick_) {
    return false;
  }
  const bool newly_appeared = created || slot->last_seen_tick != tick_ - 1;
  slot->last_seen_tick = tick_;
  if (newly_appeared) {
    AppendFaultObservation(observations_, detail);
  }
  return newly_appeared;
}

void FaultTracker::EndTick() noexcept {}

void FaultTracker::NoteDropped(std::uint64_t count) noexcept {
  observations_.dropped += count;
}

}  // namespace irmv::runtime
