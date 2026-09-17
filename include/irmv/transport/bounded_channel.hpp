#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace irmv::transport {

// Fixed-capacity ring between exactly two threads: one producer side that only
// calls TryPush/TryPushEvictOldest and one consumer side that only calls
// TryPop. There is no allocation, no lock and no waiting, so neither side can
// block the other.
//
// Ownership rule for every instance in this project: the control thread is the
// producer of the outbound channels and the consumer of the inbound channels,
// and the transport thread is the other side of each of them. Only the indices
// cross the thread boundary: `write_` is written by the producer alone, `read_`
// by the consumer alone, and both are published with release/acquire ordering.
// All queries (size/empty/full/dropped) are exact on the owning side and
// approximate while the other side is running.
//
// Eviction: TryPushEvictOldest always stores its value. When the ring is full
// it overwrites the oldest entry, which the consumer may be reading at that
// instant, so every slot carries a generation sequence:
//   * the producer marks the slot odd, writes the payload, marks it even and
//     only then publishes the new write index;
//   * the consumer accepts an entry only when the slot holds exactly the
//     generation it wants before and after copying the payload.
// An entry that was recycled while it was copied is therefore never returned;
// the consumer retries with the refreshed window and gives up after
// kMaxPopAttempts, which keeps TryPop bounded and non-blocking. The evicted
// entry itself is counted by the producer, not by the consumer.
template <typename T, std::size_t Capacity>
class BoundedChannel {
 public:
  static_assert(Capacity >= 1, "BoundedChannel needs room for one element");

  // Producer side. Returns false and leaves the channel untouched when full.
  bool TryPush(const T &value) {
    const std::size_t write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) >= Capacity) {
      return false;
    }
    PublishSlot(write, value);
    return true;
  }

  // Producer side. Always stores the value: when the channel is full the oldest
  // entry is evicted and counted as dropped. Set *evicted when the ring was
  // full, that is when an entry was evicted; the pointer may be null.
  bool TryPushEvictOldest(const T &value, bool *evicted) {
    const std::size_t write = write_.load(std::memory_order_relaxed);
    const bool full = write - read_.load(std::memory_order_acquire) >= Capacity;
    PublishSlot(write, value);
    if (full) {
      ++dropped_;
    }
    if (evicted != nullptr) {
      *evicted = full;
    }
    return true;
  }

  // Consumer side. Returns false when the channel is empty and leaves `out`
  // untouched; an entry that the producer recycled while it was copied is
  // skipped as well, which is reported as an empty read.
  bool TryPop(T &out) {
    for (int attempt = 0; attempt < kMaxPopAttempts; ++attempt) {
      const std::size_t write = write_.load(std::memory_order_acquire);
      std::size_t read = read_.load(std::memory_order_relaxed);
      if (read >= write) {
        return false;
      }
      if (write - read > Capacity) {
        // Entries older than the window were overwritten by the producer.
        read = write - Capacity;
        read_.store(read, std::memory_order_release);
      }
      const std::size_t index = read % Capacity;
      const std::size_t generation = Generation(read);
      if (slot_generation_[index].load(std::memory_order_acquire) !=
          generation) {
        continue;
      }
      // The producer may overwrite this slot at any time, whatever the ring
      // looked like when the indices above were read: it publishes the new
      // write index only after the payload and both markers, so a producer that
      // laps the consumer can be writing this slot right now and a check before
      // the copy cannot rule it out. Copy through a local and accept the entry
      // only when the slot still holds the same generation.
      const T candidate = storage_[index];
      if (slot_generation_[index].load(std::memory_order_acquire) !=
          generation) {
        continue;
      }
      out = candidate;
      read_.store(read + 1, std::memory_order_release);
      return true;
    }
    return false;
  }

  // Entries the producer has evicted are not part of the channel any more, so
  // the pending count never exceeds the capacity.
  std::size_t size() const {
    const std::size_t pending = write_.load(std::memory_order_relaxed) -
                                read_.load(std::memory_order_relaxed);
    return pending > Capacity ? Capacity : pending;
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

  // Consumer side: empties the channel and resets the slot generations.
  // Callers must ensure the producer is not pushing concurrently. The dropped
  // counter is cumulative and survives.
  void Clear() {
    read_.store(0, std::memory_order_relaxed);
    write_.store(0, std::memory_order_release);
    for (std::size_t index = 0; index < Capacity; ++index) {
      slot_generation_[index].store(0, std::memory_order_relaxed);
    }
  }

  // Number of entries evicted by TryPushEvictOldest. Producer side.
  std::uint64_t dropped() const { return dropped_; }

 private:
  // Bounded number of attempts to read an entry that the producer keeps
  // recycling. Giving up is not a loss: the entry was evicted and counted by
  // the producer, and the next entry is the one that matters.
  static constexpr int kMaxPopAttempts = 4;

  // Even generation while the slot is readable, odd while it is being written.
  static constexpr std::size_t Generation(std::size_t write) {
    return 2 * write + 2;
  }

  void PublishSlot(std::size_t write, const T &value) {
    const std::size_t index = write % Capacity;
    slot_generation_[index].store(Generation(write) - 1,
                                  std::memory_order_release);
    storage_[index] = value;
    slot_generation_[index].store(Generation(write), std::memory_order_release);
    write_.store(write + 1, std::memory_order_release);
  }

  std::array<T, Capacity> storage_{};
  std::array<std::atomic<std::size_t>, Capacity> slot_generation_{};
  std::atomic<std::size_t> write_{0};
  std::atomic<std::size_t> read_{0};
  std::uint64_t dropped_{0};
};

}  // namespace irmv::transport
