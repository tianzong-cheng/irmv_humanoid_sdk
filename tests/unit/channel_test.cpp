#include <cstddef>
#include <cstdint>

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

Received<ExecutionRequest> MakeRequest(Operation operation, Sequence request_id,
                                       TimeUs rx_us) {
  Received<ExecutionRequest> received;
  received.value.operation = operation;
  received.value.request_id = request_id;
  received.rx_us = rx_us;
  return received;
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
  }

  // Eviction keeps the newest entries, is reported to the producer and counted.
  {
    BoundedChannel<int, 3> channel;
    CHECK(channel.TryPush(1));
    CHECK(channel.TryPush(2));
    CHECK(channel.TryPush(3));

    bool evicted = false;
    CHECK(channel.TryPushEvictOldest(4, &evicted));
    CHECK(evicted);
    CHECK(channel.dropped() == 1);
    CHECK(channel.TryPushEvictOldest(5, &evicted));
    CHECK(evicted);
    CHECK(channel.dropped() == 2);
    CHECK(channel.size() == 3);

    int value = 0;
    CHECK(channel.TryPop(value) && value == 3);
    CHECK(channel.TryPop(value) && value == 4);
    CHECK(channel.TryPop(value) && value == 5);

    // A push into a channel that is not full evicts nothing.
    CHECK(channel.TryPushEvictOldest(6, &evicted));
    CHECK(!evicted);
    CHECK(channel.dropped() == 2);

    // A successful pop writes its destination, a pop from an empty channel
    // leaves it alone.
    value = 42;
    CHECK(channel.TryPop(value) && value == 6);
    value = 42;
    CHECK(!channel.TryPop(value));
    CHECK(value == 42);

    // Clear empties the ring and keeps the cumulative drop counter.
    CHECK(channel.TryPush(7));
    channel.Clear();
    CHECK(channel.empty());
    CHECK(channel.size() == 0);
    CHECK(!channel.TryPop(value));
    CHECK(channel.dropped() == 2);
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
  // queue: waiting in the ring and evicting older entries never rewrites them.
  {
    BoundedChannel<Received<JointCommand>, 4> commands;
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

    Received<JointCommand> popped;
    CHECK(commands.TryPop(popped));
    CHECK(popped.value.command_seq == 11);
    CHECK(popped.rx_us == 1'700'000);

    // Two more pushes fill the ring again; the following one evicts the oldest
    // entry and must not disturb the stamps of the entries that stay.
    for (int index = 0; index < 2; ++index) {
      Received<JointCommand> filler;
      filler.value.command_seq = static_cast<Sequence>(22 + index);
      filler.rx_us = static_cast<TimeUs>(1'850'000 + 10'000 * index);
      CHECK(commands.TryPush(filler));
    }
    CHECK(commands.full());

    bool evicted = false;
    Received<JointCommand> newest;
    newest.value.command_seq = 30;
    newest.rx_us = 1'900'000;
    CHECK(commands.TryPushEvictOldest(newest, &evicted));
    CHECK(evicted);
    CHECK(commands.dropped() == 1);

    const TimeUs expected_stamps[4] = {1'800'001, 1'850'000, 1'860'000,
                                       1'900'000};
    for (const TimeUs expected : expected_stamps) {
      CHECK(commands.TryPop(popped));
      CHECK(popped.rx_us == expected);
    }
    CHECK(commands.empty());
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

    // A full result batch evicts exactly the results that were already queued.
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
