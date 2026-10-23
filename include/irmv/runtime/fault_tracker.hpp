#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "irmv/protocol/types.hpp"

namespace irmv::runtime {

// The complete set of fault conditions one feedback check can establish. The
// published FaultReport is bounded by the protocol payload and drops entries at
// capacity; this batch is the internal diagnostic, so it is never derived from
// a truncated report and keeps every established condition of one check.
inline constexpr std::size_t kFaultObservationCapacity = 1024;

struct FaultObservationBatch {
  std::uint32_t count{0};
  // Only the first `count` entries are meaningful. The array is preallocated
  // because the batch is a member of the tracker and of the safety report, not
  // a per-tick heap object.
  std::array<protocol::FaultDetail, kFaultObservationCapacity> details;
  // Observations lost in this tick: entries not stored because the batch was
  // full, and new keys that could not be tracked because every slot still held
  // a condition present in the previous tick. Tracking keeps updating for the
  // keys that do have slots, so a dropped occurrence still suppresses a
  // duplicate on the next tick and reappears as new after it disappears.
  std::uint64_t dropped{0};
};

// One snapshot establishes at most eight joint conditions per joint (the
// separated invalid, disconnected, non-finite, stale, low-frequency, driver,
// position and velocity causes), the four shared feedback conditions (three
// IMU conditions and a malformed shape) and the five non-feedback conditions
// the runtime adds itself. Write failures are additionally keyed by error code
// in the tracker, but one tick performs at most one primary and one fallback
// write.
inline constexpr std::size_t kJointFaultConditionCount = 8;
inline constexpr std::size_t kSharedFaultConditionCount = 4;
inline constexpr std::size_t kNonFeedbackFaultConditionCount = 5;
static_assert(protocol::kMaxJoints * kJointFaultConditionCount +
                      kSharedFaultConditionCount +
                      kNonFeedbackFaultConditionCount <=
                  kFaultObservationCapacity,
              "the observation batch must hold every condition of one check");

// Appends one observation to `batch`. When the batch is full, the observation
// is not stored and `batch.dropped` is incremented; this is the single place
// that owns the batch's overflow accounting.
void AppendFaultObservation(FaultObservationBatch &batch,
                            const protocol::FaultDetail &detail) noexcept;

// Tracks which fault conditions are currently present, so a recorder can log
// the moment a condition appears instead of repeating it on every tick.
//
// A slot is keyed by condition and joint index; the driver-error and write
// failure slots additionally compare the error code, so a driver reporting a
// different error is a new occurrence while the same error persists. Nothing
// volatile is part of a key: timestamps, messages and changing measured values
// do not make a sustained condition new. Repeated observations of one key
// inside the same tick are deduplicated.
//
// The slot table and the new-observation batch are members, preallocated when
// the tracker is embedded in the execution state machine. BeginTick, Observe,
// NoteDropped and EndTick allocate nothing, never block and never format text;
// they only copy the details of newly appeared conditions into the batch.
class FaultTracker {
 public:
  FaultTracker() noexcept = default;

  FaultTracker(const FaultTracker &) = delete;
  FaultTracker &operator=(const FaultTracker &) = delete;

  // Starts a tick: clears the new-observation batch of the previous tick.
  void BeginTick() noexcept;

  // Records one observation and returns true when the condition was not
  // present in the previous tick. The detail is copied into the batch only for
  // a new occurrence. A key that cannot be tracked because every slot still
  // holds a condition from the previous tick returns false (it is not reported
  // as new, which would be a false occurrence) and increments the tick's
  // dropped counter.
  bool Observe(const protocol::FaultDetail &detail) noexcept;

  // Closes the tick. Activity is recorded by Observe, so this only marks the
  // boundary at which the next BeginTick advances the tick counter.
  void EndTick() noexcept;

  // Adds occurrences that a producer other than this tracker could not
  // enumerate (for example a full safety report) to the tick's overflow
  // counter. The loss is visible through NewObservations().dropped exactly
  // like an observation this tracker could not store. Never allocates, blocks
  // or throws.
  void NoteDropped(std::uint64_t count) noexcept;

  // Observations that newly appeared in the tick just observed. Later
  // recorder stages copy only `count` entries, never the full capacity.
  const FaultObservationBatch &NewObservations() const noexcept {
    return observations_;
  }

 private:
  struct Slot {
    protocol::FaultCondition condition{protocol::FaultCondition::None};
    std::uint32_t joint_index{protocol::kNoJointIndex};
    std::uint32_t error_code{0};
    bool used{false};
    // Tick in which this key was last observed. A value of tick_ - 1 means the
    // condition persisted from the previous tick; any older value means it
    // disappeared and the current observation is new.
    std::uint32_t last_seen_tick{0};
  };

  // Twice the batch capacity keeps linear probing below half load: a tick can
  // observe at most one key per stored observation, and the table must keep
  // those keys while it looks for a free slot.
  static constexpr std::size_t kSlotCapacity = 2 * kFaultObservationCapacity;

  static bool ErrorKeyed(protocol::FaultCondition condition) noexcept;
  static std::uint32_t HashKey(protocol::FaultCondition condition,
                               std::uint32_t joint_index,
                               std::uint32_t error_code) noexcept;
  Slot *AcquireSlot(const protocol::FaultDetail &detail,
                    bool *created) noexcept;

  std::array<Slot, kSlotCapacity> slots_{};
  FaultObservationBatch observations_;
  std::uint32_t tick_{0};
};

}  // namespace irmv::runtime
