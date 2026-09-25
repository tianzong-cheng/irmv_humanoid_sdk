#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace irmv::transport {

// The two handoff shapes the transport needs, both fixed capacity, allocation
// free, lock free and non-blocking on both sides. Which one a lane uses is a
// statement about its receiver: a FIFO hands over every accepted entry exactly
// once and in order, a keep-last-1 channel hands over the newest entry only.
//
// Both shapes are race free by ownership rather than by re-checking a sequence
// after the fact: a payload slot has exactly one writer at any instant, and the
// side that does not own the slot never touches it. Every instance has exactly
// one producer thread and one consumer thread - the control thread produces the
// outbound channels and consumes the inbound ones, the transport thread is the
// other side of each - and that is all the index arithmetic below assumes. No
// query is a synchronisation point: size/empty/full/dropped are exact on the
// owning side and approximate once the other side is running.
//
// The assignment of every instance in this project: requests, results and stop
// requests are FIFO queues, because their order carries meaning and a refusal
// loses only the newest sample; commands, heartbeats and the robot state keep
// only their newest value, because an entry that was superseded before anyone
// read it was never going to be used.

// FIFO ring between exactly two threads, race free by construction rather than
// by re-checking an entry after it was copied. The producer writes only the
// slot `write_ % Capacity`, and only once it has observed `write_ - read_ <
// Capacity`; the `read_` it observes may be stale, but never larger than the
// index the consumer holds, so the test is at worst more conservative, and the
// consumer never advances `read_` past `write_`, so the two indices stay within
// one capacity of each other. Two slot numbers of indices that differ by less
// than the capacity are different, and the consumer only ever copies the slot
// `read_ % Capacity` of an entry the producer has already published through the
// release store of `write_`, so the producer's write slot is never the
// consumer's copy slot and the two payload accesses cannot overlap. The release
// store of `write_` after the payload and the acquire load of it before an
// entry is copied order that payload write before the read of that entry.
template <typename T, std::size_t Capacity>
class BoundedChannel {
 public:
  static_assert(Capacity >= 1, "BoundedChannel needs room for one element");

  // Producer side. Returns false and leaves the channel untouched when the ring
  // is full. A refusal is the whole drop report, so every caller owns its own
  // drop counter.
  bool TryPush(const T &value) {
    const std::size_t write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) >= Capacity) {
      return false;
    }
    storage_[write % Capacity] = value;
    write_.store(write + 1, std::memory_order_release);
    return true;
  }

  // Consumer side. Returns false when the channel is empty and leaves `out`
  // untouched.
  bool TryPop(T &out) {
    const std::size_t read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) {
      return false;
    }
    out = storage_[read % Capacity];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }

  // Entries that were pushed and not yet popped.
  std::size_t size() const {
    return write_.load(std::memory_order_relaxed) -
           read_.load(std::memory_order_relaxed);
  }

  bool empty() const {
    return write_.load(std::memory_order_relaxed) ==
           read_.load(std::memory_order_relaxed);
  }

  bool full() const {
    return write_.load(std::memory_order_relaxed) -
               read_.load(std::memory_order_relaxed) >=
           Capacity;
  }

  // Consumer side: empties the ring. Callers must ensure the producer is not
  // pushing concurrently; the payload slots are not touched, so they need no
  // reset either.
  void Clear() {
    read_.store(0, std::memory_order_relaxed);
    write_.store(0, std::memory_order_release);
  }

 private:
  std::array<T, Capacity> storage_{};
  std::atomic<std::size_t> write_{0};
  std::atomic<std::size_t> read_{0};
};

// Keep-last-1 handoff between exactly two threads, for the lanes where only the
// newest sample matters. At most one entry is ever unread; publishing a new
// value replaces an unread one, and the producer reports that loss through
// *evicted, so the caller counts the drop.
//
// Slot ownership is the synchronisation, not a sequence re-checked after the
// payload was touched. Every slot carries one atomic state, and each state
// transition is a claim (a compare-exchange) performed by exactly one side:
//   * kFree      - nobody owns the slot;
//   * kWriting   - the producer owns it and is the only writer of the payload;
//   * kPublished - the payload is complete and may be the newest entry;
//   * kHeld      - the consumer owns it and is the only reader of the payload.
// The producer claims the published slot first (a compare-exchange on
// kPublished, so a slot the consumer already emptied is not mistaken for an
// unread entry) and otherwise claims a free slot; the consumer claims the
// published slot. Neither side can reach a payload slot the other owns, so the
// copy in TryTake cannot overlap a write in Publish.
//
// Three slots are enough, which is why the "no slot could be claimed" branch is
// a defensive bound rather than a routine path. There is one producer, so at
// most one slot is kWriting at any instant, and one consumer, so at most one
// slot is kHeld; the published index names the single slot that is published,
// is being replaced by this very call, or was already taken and is now free. A
// Publish whose claim of the published slot failed has therefore found that
// slot free - it is itself the free slot - or held by the consumer, and in the
// second case the one writer owns nothing, so the two remaining slots are free.
// A free slot always exists and the claim of it succeeds; should the one
// producer and one consumer contract ever be broken by a caller, the fallback
// counts the value in dropped(), reports failure and leaves the channel
// consistent.
template <typename T>
class LatestValueChannel {
 public:
  // Producer side. Stores `value` as the newest entry and returns true.
  // *evicted reports whether an unread published entry was replaced, which is
  // the loss the caller counts; it is false when the value was merely stored or
  // when nothing could be stored. Returns false only when no slot could be
  // claimed, in which case the value is counted in dropped() and *evicted is
  // false. The pointer may be null.
  bool Publish(const T &value, bool *evicted) {
    bool replaced = false;
    std::size_t slot = kNothingPublished;
    const std::size_t published = published_.load(std::memory_order_acquire);
    if (published < kSlots &&
        Claim(state_[published], SlotState::kPublished, SlotState::kWriting)) {
      slot = published;
      replaced = true;
    }
    if (slot == kNothingPublished) {
      for (std::size_t index = 0; index < kSlots; ++index) {
        if (Claim(state_[index], SlotState::kFree, SlotState::kWriting)) {
          slot = index;
          break;
        }
      }
    }
    if (slot == kNothingPublished) {
      ++dropped_;
      if (evicted != nullptr) {
        *evicted = false;
      }
      return false;
    }
    // The payload is written while this side owns the slot, so no reader can be
    // copying it, and the release store of the state publishes it.
    storage_[slot] = value;
    state_[slot].store(SlotState::kPublished, std::memory_order_release);
    published_.store(slot, std::memory_order_release);
    if (evicted != nullptr) {
      *evicted = replaced;
    }
    return true;
  }

  // Consumer side. Copies the newest entry into `out` and returns true; returns
  // false when nothing is published, which is also the signal a TryTake after a
  // successful take reports. A producer that replaces the entry while it is
  // claimed cannot touch the payload, so the copy is exclusive; a take that
  // loses the claim retries with a fresh published index a bounded number of
  // times and then reports nothing taken.
  bool TryTake(T &out) {
    for (int attempt = 0; attempt < kMaxTakeAttempts; ++attempt) {
      const std::size_t published = published_.load(std::memory_order_acquire);
      if (published >= kSlots) {
        return false;
      }
      if (!Claim(state_[published], SlotState::kPublished, SlotState::kHeld)) {
        continue;
      }
      out = storage_[published];
      state_[published].store(SlotState::kFree, std::memory_order_release);
      return true;
    }
    return false;
  }

  // Values the producer could not store because no slot could be claimed.
  // Producer side.
  std::uint64_t dropped() const { return dropped_; }

  // Empties the channel. Callers must ensure the producer is not publishing and
  // the consumer is not taking concurrently.
  void Clear() {
    published_.store(kNothingPublished, std::memory_order_relaxed);
    for (std::size_t index = 0; index < kSlots; ++index) {
      state_[index].store(SlotState::kFree, std::memory_order_relaxed);
    }
  }

 private:
  enum class SlotState : std::uint8_t {
    kFree,
    kWriting,
    kPublished,
    kHeld,
  };

  // Three slots, the count the class comment proves sufficient: at most one is
  // being written, at most one is held, and the published index names at most
  // one more, so a free slot is always available.
  static constexpr std::size_t kSlots = 3;
  // The published index is a slot number; any value outside the slots means
  // that nothing is published. TryTake never clears it, because the claim is on
  // the slot state and a takeable slot is always kPublished.
  static constexpr std::size_t kNothingPublished = kSlots;
  // Bounded retries when a producer replaces the entry between the published
  // index load and the claim, which keeps TryTake non-blocking.
  static constexpr int kMaxTakeAttempts = 4;

  // Acquire on success: the claim synchronises with the release store that
  // published or freed the slot, so a payload written before it is complete and
  // one read before it cannot be overwritten.
  static bool Claim(std::atomic<SlotState> &state, SlotState expected,
                    SlotState desired) {
    SlotState value = expected;
    return state.compare_exchange_strong(
        value, desired, std::memory_order_acquire, std::memory_order_relaxed);
  }

  std::array<T, kSlots> storage_{};
  std::array<std::atomic<SlotState>, kSlots> state_{};
  std::atomic<std::size_t> published_{kNothingPublished};
  std::uint64_t dropped_{0};
};

}  // namespace irmv::transport
