#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/execution_context.hpp"
#include "irmv/transport/bounded_channel.hpp"
#include "irmv/transport/dds_transport.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::RuntimeConfig;
using irmv::protocol::ExecutionRequest;
using irmv::protocol::JointCommand;
using irmv::protocol::Operation;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::runtime::kRequestBatchCapacity;
using irmv::runtime::kResultCapacity;
using irmv::runtime::kStopRequestBatchCapacity;
using irmv::runtime::Received;
using irmv::runtime::TickInput;
using irmv::runtime::TickResult;
using irmv::transport::BoundedChannel;
using irmv::transport::DdsTransport;
using irmv::transport::IsStopLaneOperation;
using irmv::transport::LatestValueChannel;

Received<ExecutionRequest> MakeRequest(Operation operation, Sequence request_id,
                                       TimeUs rx_us) {
  Received<ExecutionRequest> received;
  received.value.operation = operation;
  received.value.request_id = request_id;
  received.rx_us = rx_us;
  return received;
}

// Payload of the two-thread handoff case. Every word is derived from the one
// counter that produced the sample, so a sample assembled from two different
// pushes is inconsistent in almost every word; the payload is much wider than a
// cache line, so a producer that laps the consumer is visible as tearing.
struct StressPayload {
  static constexpr std::size_t kWords = 64;
  // Odd multiplier that spreads the counter across every word.
  static constexpr std::uint64_t kStride = 0x9E3779B97F4A7C15ULL;

  std::array<std::uint64_t, kWords> body{};

  void Set(std::uint64_t counter) {
    for (std::size_t index = 0; index < body.size(); ++index) {
      body[index] = index * kStride + counter;
    }
  }

  std::uint64_t Counter() const { return body[0]; }

  bool Consistent() const {
    const std::uint64_t counter = body[0];
    for (std::size_t index = 0; index < body.size(); ++index) {
      if (body[index] != index * kStride + counter) {
        return false;
      }
    }
    return true;
  }
};

// Pops the handoff case performs by default: enough to interleave the two
// threads heavily and still finish well inside a second.
constexpr std::uint64_t kStressPops = 200'000;

// The committed budget is fixed; a larger budget is available for manual
// reproduction of the pre-fix race through IRMV_CHANNEL_STRESS_POPS.
std::uint64_t StressBudget() {
  if (const char *value = std::getenv("IRMV_CHANNEL_STRESS_POPS")) {
    const unsigned long long parsed = std::strtoull(value, nullptr, 10);
    if (parsed > 0) {
      return parsed;
    }
  }
  return kStressPops;
}

struct StressTally {
  std::uint64_t budget{0};
  std::uint64_t observed{0};
  std::uint64_t torn{0};
  std::uint64_t regressions{0};
};

// One producer against one consumer, on the keep-last-1 handoff: the producer
// replaces the unread entry as fast as it can, the consumer takes what is
// published. Every accepted payload must be consistent inside itself - a
// payload that mixes two publishes is the data race this case exists for - and
// the counters the consumer sees may never go backwards, because a taken entry
// is always the newest one at the instant it was claimed.
StressTally StressLatestValue() {
  StressTally tally;
  tally.budget = StressBudget();

  LatestValueChannel<StressPayload> channel;
  std::atomic<bool> producing{true};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> regressions{0};

  std::thread producer([&channel, &producing]() {
    std::uint64_t counter = 0;
    while (producing.load(std::memory_order_relaxed)) {
      StressPayload payload;
      payload.Set(++counter);
      channel.Publish(payload, nullptr);
    }
  });

  std::thread consumer([&channel, &torn, &regressions, &tally]() {
    StressPayload payload;
    std::uint64_t last = 0;
    while (tally.observed < tally.budget) {
      if (!channel.TryTake(payload)) {
        std::this_thread::yield();
        continue;
      }
      ++tally.observed;
      if (!payload.Consistent()) {
        torn.fetch_add(1, std::memory_order_relaxed);
      } else if (payload.Counter() <= last) {
        regressions.fetch_add(1, std::memory_order_relaxed);
      }
      last = payload.Counter();
    }
  });

  consumer.join();
  producing.store(false, std::memory_order_relaxed);
  producer.join();
  tally.torn = torn.load(std::memory_order_relaxed);
  tally.regressions = regressions.load(std::memory_order_relaxed);
  return tally;
}

// The same two threads on the FIFO handoff, at the capacity where the producer
// most often finds the ring full: a refused push is retried with the same
// payload, so the consumer must see every counter exactly once and in order.
template <typename Channel>
StressTally StressFifo(Channel &channel) {
  StressTally tally;
  tally.budget = StressBudget();

  std::atomic<bool> producing{true};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> regressions{0};

  std::thread producer([&channel, &producing]() {
    std::uint64_t counter = 0;
    while (producing.load(std::memory_order_relaxed)) {
      StressPayload payload;
      payload.Set(++counter);
      while (producing.load(std::memory_order_relaxed) &&
             !channel.TryPush(payload)) {
        std::this_thread::yield();
      }
    }
  });

  std::thread consumer([&channel, &torn, &regressions, &tally]() {
    StressPayload payload;
    std::uint64_t last = 0;
    while (tally.observed < tally.budget) {
      if (!channel.TryPop(payload)) {
        std::this_thread::yield();
        continue;
      }
      ++tally.observed;
      if (!payload.Consistent()) {
        torn.fetch_add(1, std::memory_order_relaxed);
      } else if (payload.Counter() != last + 1) {
        regressions.fetch_add(1, std::memory_order_relaxed);
      }
      last = payload.Counter();
    }
  });

  consumer.join();
  producing.store(false, std::memory_order_relaxed);
  producer.join();
  tally.torn = torn.load(std::memory_order_relaxed);
  tally.regressions = regressions.load(std::memory_order_relaxed);
  return tally;
}

}  // namespace

int main() {
  // FIFO order, capacity and a refused push that leaves the ring untouched.
  {
    BoundedChannel<int, 3> channel;
    CHECK(channel.empty());
    CHECK(!channel.full());
    CHECK(channel.size() == 0);
    CHECK(channel.TryPush(1));
    CHECK(channel.TryPush(2));
    CHECK(channel.TryPush(3));
    CHECK(channel.full());
    CHECK(channel.size() == 3);
    CHECK(!channel.TryPush(4));
    CHECK(channel.size() == 3);

    int value = 0;
    CHECK(channel.TryPop(value) && value == 1);
    CHECK(channel.TryPop(value) && value == 2);
    CHECK(channel.TryPop(value) && value == 3);
    CHECK(!channel.TryPop(value));
    CHECK(channel.empty());

    // A refused push on a full ring leaves no slot written: the entries that
    // were queued keep their order and their values.
    CHECK(channel.TryPush(5));
    CHECK(channel.TryPush(6));
    CHECK(channel.TryPush(7));
    CHECK(channel.full());
    CHECK(!channel.TryPush(8));
    CHECK(channel.size() == 3);
    CHECK(channel.TryPop(value) && value == 5);
    CHECK(channel.TryPop(value) && value == 6);
    CHECK(channel.TryPop(value) && value == 7);
    CHECK(channel.empty());

    // A successful pop writes its destination, a pop from an empty ring leaves
    // it alone.
    value = 42;
    CHECK(channel.TryPush(9));
    CHECK(channel.TryPop(value) && value == 9);
    value = 42;
    CHECK(!channel.TryPop(value));
    CHECK(value == 42);

    // Clear empties the ring and the capacity is available again.
    CHECK(channel.TryPush(10));
    channel.Clear();
    CHECK(channel.empty());
    CHECK(channel.size() == 0);
    CHECK(!channel.full());
    CHECK(!channel.TryPop(value));
    CHECK(value == 42);
    CHECK(channel.TryPush(11));
    CHECK(channel.size() == 1);
    CHECK(channel.TryPop(value) && value == 11);
  }

  // Keep-last-1: a publish replaces an unread entry, which the producer reports
  // through *evicted, and a take hands over the newest entry exactly once.
  {
    LatestValueChannel<int> channel;
    int value = 0;
    CHECK(!channel.TryTake(value));
    CHECK(channel.dropped() == 0);

    bool evicted = false;
    CHECK(channel.Publish(1, &evicted));
    CHECK(!evicted);
    CHECK(channel.dropped() == 0);

    // The second and third publishes replace the unread entry in place; the
    // replacement is the loss the producer counts through *evicted, so the
    // channel's own counter stays zero.
    CHECK(channel.Publish(2, nullptr));
    CHECK(channel.Publish(3, &evicted));
    CHECK(evicted);
    CHECK(channel.dropped() == 0);

    // Three publishes, one entry: the take returns the newest of them.
    CHECK(channel.TryTake(value) && value == 3);
    // The entry was consumed, so a second take reports the normal empty signal.
    value = 42;
    CHECK(!channel.TryTake(value));
    CHECK(value == 42);

    // A value the consumer took is not a loss: the next publish stores without
    // reporting an eviction.
    CHECK(channel.Publish(4, &evicted));
    CHECK(!evicted);
    CHECK(channel.dropped() == 0);
    CHECK(channel.TryTake(value) && value == 4);

    // Clear empties the channel, and it accepts values again afterwards.
    CHECK(channel.Publish(5, &evicted));
    channel.Clear();
    CHECK(!channel.TryTake(value));
    CHECK(channel.Publish(6, &evicted));
    CHECK(!evicted);
    CHECK(channel.TryTake(value) && value == 6);
    CHECK(channel.dropped() == 0);
  }

  // The stop lane is a separate channel with its own capacity: a full ordinary
  // request queue cannot consume, block or drop it.
  {
    CHECK(IsStopLaneOperation(Operation::Damping));
    CHECK(IsStopLaneOperation(Operation::ZeroTorque));
    CHECK(IsStopLaneOperation(Operation::Reset));
    CHECK(!IsStopLaneOperation(Operation::AcquireSession));
    CHECK(!IsStopLaneOperation(Operation::Tracking));

    BoundedChannel<Received<ExecutionRequest>, kRequestBatchCapacity> requests;
    BoundedChannel<Received<ExecutionRequest>, kStopRequestBatchCapacity>
        stop_requests;
    for (std::size_t index = 0; index < kRequestBatchCapacity; ++index) {
      CHECK(requests.TryPush(
          MakeRequest(Operation::Tracking, index + 1, index + 1000)));
    }
    CHECK(requests.full());
    CHECK(!requests.TryPush(MakeRequest(Operation::Tracking, 99, 0)));

    CHECK(stop_requests.TryPush(MakeRequest(Operation::Damping, 100, 5000)));
    CHECK(stop_requests.size() == 1);
    Received<ExecutionRequest> popped;
    CHECK(stop_requests.TryPop(popped));
    CHECK(popped.value.operation == Operation::Damping);
    CHECK(popped.value.request_id == 100);
    CHECK(popped.rx_us == 5000);

    // The ordinary queue is untouched by the stop lane traffic.
    CHECK(requests.size() == kRequestBatchCapacity);
    CHECK(requests.TryPop(popped));
    CHECK(popped.value.request_id == 1);
  }

  // Receive stamps are written once by the transport thread and survive the
  // handoff: a waiting entry and an entry that a newer one replaces or a
  // refusal never rewrite them.
  {
    BoundedChannel<Received<JointCommand>, 3> commands;
    Received<JointCommand> first;
    first.value.command_seq = 11;
    first.rx_us = 1'700'000;
    CHECK(commands.TryPush(first));
    for (int index = 0; index < 2; ++index) {
      Received<JointCommand> other;
      other.value.command_seq = static_cast<Sequence>(20 + index);
      other.rx_us = static_cast<TimeUs>(1'800'000 + index);
      CHECK(commands.TryPush(other));
    }
    CHECK(commands.full());

    // The refused fourth push must not disturb the stamps of the three entries
    // that stay, and the ring hands them over in order, each with the stamp the
    // transport thread wrote once.
    Received<JointCommand> refused;
    refused.value.command_seq = 30;
    refused.rx_us = 1'900'000;
    CHECK(!commands.TryPush(refused));

    Received<JointCommand> popped;
    const TimeUs expected_stamps[3] = {1'700'000, 1'800'000, 1'800'001};
    const Sequence expected_seqs[3] = {11, 20, 21};
    for (std::size_t index = 0; index < 3; ++index) {
      CHECK(commands.TryPop(popped));
      CHECK(popped.value.command_seq == expected_seqs[index]);
      CHECK(popped.rx_us == expected_stamps[index]);
    }
    CHECK(commands.empty());

    // The keep-last-1 lane replaces an unread entry in place, so the newest
    // command carries its own receive stamp and the superseded one is gone.
    LatestValueChannel<Received<JointCommand>> latest;
    Received<JointCommand> older;
    older.value.command_seq = 40;
    older.rx_us = 1'900'000;
    bool evicted = false;
    CHECK(latest.Publish(older, &evicted));
    CHECK(!evicted);

    Received<JointCommand> newest;
    newest.value.command_seq = 41;
    newest.rx_us = 1'950'000;
    CHECK(latest.Publish(newest, &evicted));
    CHECK(evicted);
    CHECK(latest.TryTake(popped));
    CHECK(popped.value.command_seq == 41);
    CHECK(popped.rx_us == 1'950'000);
  }

  // Two threads racing on the same channel must never hand out a payload that
  // mixes two stores: at capacity 1 and at a multi-slot capacity on the FIFO,
  // and on the keep-last-1 handoff, which is where a replacement can hit a
  // payload the consumer is copying.
  {
    const StressTally latest = StressLatestValue();
    std::cout << "[channel_test] latest observed=" << latest.observed
              << " budget=" << latest.budget << " torn=" << latest.torn
              << " regressions=" << latest.regressions << '\n';
    CHECK(latest.observed == latest.budget);
    CHECK(latest.torn == 0);
    CHECK(latest.regressions == 0);

    BoundedChannel<StressPayload, 1> single;
    const StressTally tight = StressFifo(single);
    std::cout << "[channel_test] fifo capacity=1 observed=" << tight.observed
              << " budget=" << tight.budget << " torn=" << tight.torn
              << " regressions=" << tight.regressions << '\n';
    CHECK(tight.observed == tight.budget);
    CHECK(tight.torn == 0);
    CHECK(tight.regressions == 0);

    BoundedChannel<StressPayload, 8> multi;
    const StressTally wide = StressFifo(multi);
    std::cout << "[channel_test] fifo capacity=8 observed=" << wide.observed
              << " budget=" << wide.budget << " torn=" << wide.torn
              << " regressions=" << wide.regressions << '\n';
    CHECK(wide.observed == wide.budget);
    CHECK(wide.torn == 0);
    CHECK(wide.regressions == 0);
  }

  // Output pressure is bounded and counted without DDS, and a transport that
  // was never started has no input.
  {
    RuntimeConfig config;
    config.dds.domain = 1;
    config.dds.interface = "lo";
    DdsTransport transport(config);
    CHECK(!transport.started());

    const TickInput &input = transport.TakeInput();
    CHECK(input.request_count == 0);
    CHECK(input.stop_request_count == 0);
    CHECK(input.command_count == 0);
    CHECK(input.heartbeat_count == 0);

    TickResult tick;
    tick.result_count = 1;
    transport.Publish(tick);
    transport.Publish(tick);
    // The state channel keeps only the newest snapshot; the result queue has
    // room for both results.
    CHECK(transport.dropped_state_count() == 1);
    CHECK(transport.dropped_result_count() == 0);

    // A result batch larger than the free room in the queue is refused for
    // exactly the results that do not fit, and the state snapshot replaces the
    // unread one.
    TickResult batch;
    batch.result_count = static_cast<std::uint32_t>(kResultCapacity);
    transport.Publish(batch);
    CHECK(transport.dropped_result_count() == 2);
    CHECK(transport.dropped_state_count() == 2);

    CHECK(transport.dropped_request_count() == 0);
    CHECK(transport.dropped_command_count() == 0);
    CHECK(transport.dropped_heartbeat_count() == 0);
    CHECK(transport.inbound_rejected_count() == 0);

    // Stop is idempotent on a transport that never started.
    transport.Stop();
    transport.Stop();
    CHECK(!transport.started());
  }

  return 0;
}
