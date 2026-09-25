#include "irmv/transport/client_transport.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <dds/dds.hpp>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "dds_loop.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"
#include "irmv/transport/dds_transport.hpp"

namespace irmv::transport {
namespace {

namespace wire = irmv::protocol::wire;

}  // namespace

// Every DDS entity lives here and is created by the constructor, which only
// runs from Start(). The transport thread owns all of them after that.
struct ClientTransport::Impl {
  Impl(const configuration::ApplicationConfig &config, ClientTransport &owner)
      : owner(owner),
        participant(config.dds().domain,
                    dds::domain::qos::DomainParticipantQos(), nullptr,
                    dds::core::status::StatusMask::none(),
                    config.dds().interface.empty()
                        ? std::string()
                        : InterfaceConfig(config.dds().interface)),
        publisher(participant),
        subscriber(participant),
        request_topic(participant,
                      std::string(protocol::kExecutionRequestTopic.name)),
        command_topic(participant,
                      std::string(protocol::kJointCommandTopic.name)),
        heartbeat_topic(participant,
                        std::string(protocol::kSupervisorHeartbeatTopic.name)),
        result_topic(participant,
                     std::string(protocol::kExecutionResultTopic.name)),
        state_topic(participant, std::string(protocol::kRobotStateTopic.name)),
        request_writer(
            publisher, request_topic,
            codec::ToWriterQos(protocol::kExecutionRequestTopic.qos)),
        command_writer(publisher, command_topic,
                       codec::ToWriterQos(protocol::kJointCommandTopic.qos)),
        heartbeat_writer(
            publisher, heartbeat_topic,
            codec::ToWriterQos(protocol::kSupervisorHeartbeatTopic.qos)),
        result_reader(subscriber, result_topic,
                      codec::ToReaderQos(protocol::kExecutionResultTopic.qos)),
        state_reader(subscriber, state_topic,
                     codec::ToReaderQos(protocol::kRobotStateTopic.qos)),
        result_condition(result_reader, dds::sub::status::DataState::any()),
        state_condition(state_reader, dds::sub::status::DataState::any()) {
    waitset += result_condition;
    waitset += state_condition;
  }

  void Start() { thread = std::thread(&Impl::Run, this); }

  void Run();
  bool ReadInbound();
  void ReadResults();
  void ReadStates();
  void WriteOutbound();
  void Stop();

  ClientTransport &owner;
  dds::domain::DomainParticipant participant;
  dds::pub::Publisher publisher;
  dds::sub::Subscriber subscriber;
  dds::topic::Topic<wire::ExecutionRequest> request_topic;
  dds::topic::Topic<wire::JointCommand> command_topic;
  dds::topic::Topic<wire::SupervisorHeartbeat> heartbeat_topic;
  dds::topic::Topic<wire::ExecutionResult> result_topic;
  dds::topic::Topic<wire::RobotState> state_topic;
  dds::pub::DataWriter<wire::ExecutionRequest> request_writer;
  dds::pub::DataWriter<wire::JointCommand> command_writer;
  dds::pub::DataWriter<wire::SupervisorHeartbeat> heartbeat_writer;
  dds::sub::DataReader<wire::ExecutionResult> result_reader;
  dds::sub::DataReader<wire::RobotState> state_reader;
  dds::sub::cond::ReadCondition result_condition;
  dds::sub::cond::ReadCondition state_condition;
  dds::core::cond::WaitSet waitset;
  std::thread thread;
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool finished{false};
};

void ClientTransport::Impl::Run() {
  while (!stop_requested.load(std::memory_order_relaxed)) {
    bool data_available = false;
    try {
      data_available =
          !waitset.wait(dds::core::Duration::from_millisecs(kWaitTimeoutMs))
               .empty();
    } catch (const dds::core::TimeoutError &) {
      // The pinned WaitSet reports a timeout by throwing; that is the normal
      // "nothing to read within the wait window" path.
      data_available = false;
    } catch (const std::exception &) {
      // A failing wait means the entities this thread owns are unusable. Leave
      // the loop instead of spinning on the error: the channels keep working
      // and started() turns false.
      break;
    }
    if (data_available && !ReadInbound()) {
      break;
    }
    WriteOutbound();
  }

  {
    std::lock_guard<std::mutex> lock(done_mutex);
    finished = true;
  }
  done_cv.notify_all();
  owner.started_.store(false, std::memory_order_release);
}

bool ClientTransport::Impl::ReadInbound() {
  // Each lane is drained behind the same guard: a failing read ends the
  // transport loop cleanly and is counted, and the lane that follows is left
  // untouched.
  return ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadResults(); }) &&
         ReadGuarded(owner.inbound_read_failures_, [this]() { ReadStates(); });
}

void ClientTransport::Impl::ReadResults() {
  // All samples of one read share the host receive time of that read.
  const protocol::TimeUs rx_us = NowUs();
  auto samples = result_reader.select()
                     .max_samples(kReadBatch)
                     .state(dds::sub::status::DataState::any())
                     .take();
  for (const auto &sample : samples) {
    if (!sample.info().valid()) {
      continue;
    }
    protocol::ExecutionResult value;
    if (!codec::FromWire(sample.data(), value)) {
      owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    // The result lane is FIFO: an answer that does not fit is refused and
    // counted, and the answers already queued keep their order, which is the
    // order the runtime produced them in.
    const bool stored = owner.results_.TryPush(
        runtime::Received<protocol::ExecutionResult>{value, rx_us});
    if (!stored) {
      owner.dropped_results_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void ClientTransport::Impl::ReadStates() {
  const protocol::TimeUs rx_us = NowUs();
  auto samples = state_reader.select()
                     .max_samples(kReadBatch)
                     .state(dds::sub::status::DataState::any())
                     .take();
  for (const auto &sample : samples) {
    if (!sample.info().valid()) {
      continue;
    }
    protocol::RobotState value;
    if (!codec::FromWire(sample.data(), value)) {
      owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    // The state lane keeps only the newest entry: a sample that supersedes an
    // unread one, and one that cannot be stored at all, are both losses the
    // supervision thread would never see otherwise.
    bool evicted = false;
    const bool stored = owner.states_.Publish(
        runtime::Received<protocol::RobotState>{value, rx_us}, &evicted);
    if (!stored || evicted) {
      owner.dropped_states_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void ClientTransport::Impl::WriteOutbound() {
  // The stop lane is drained before the ordinary lane: a stop must never be
  // stuck behind ordinary traffic, and both lanes share the one reliable
  // request endpoint, so this order is what gives a stop its priority.
  protocol::ExecutionRequest request;
  while (owner.stop_requests_.TryPop(request)) {
    wire::ExecutionRequest wire_request;
    codec::ToWire(request, wire_request);
    TryWrite(request_writer, wire_request, owner.dropped_requests_);
  }
  while (owner.requests_.TryPop(request)) {
    wire::ExecutionRequest wire_request;
    codec::ToWire(request, wire_request);
    TryWrite(request_writer, wire_request, owner.dropped_requests_);
  }

  // The command lane keeps one frame, so a take either hands it over or reports
  // nothing published; the wire representation is built only in the first case,
  // which keeps the loop cheap while no frame is pending.
  protocol::JointCommand command;
  if (owner.commands_.TryTake(command)) {
    wire::JointCommand wire_command;
    codec::ToWire(command, wire_command);
    TryWrite(command_writer, wire_command, owner.dropped_commands_);
  }

  // The heartbeat lane is FIFO and drained until empty, so every heartbeat the
  // state machine still has to judge reaches it in receive order.
  protocol::SupervisorHeartbeat heartbeat;
  while (owner.heartbeats_.TryPop(heartbeat)) {
    wire::SupervisorHeartbeat wire_heartbeat;
    codec::ToWire(heartbeat, wire_heartbeat);
    TryWrite(heartbeat_writer, wire_heartbeat, owner.dropped_heartbeats_);
  }
}

void ClientTransport::Impl::Stop() {
  stop_requested.store(true, std::memory_order_relaxed);
  {
    // Bounded wait for the thread to leave its loop; every blocking operation
    // in that loop is bounded as well, so the join below is immediate.
    std::unique_lock<std::mutex> lock(done_mutex);
    done_cv.wait_for(lock, std::chrono::milliseconds(kStopTimeoutMs),
                     [this] { return finished; });
  }
  if (thread.joinable()) {
    thread.join();
  }
}

ClientTransport::ClientTransport(const configuration::ApplicationConfig &config)
    : config_(config) {}

ClientTransport::~ClientTransport() { Stop(); }

void ClientTransport::Start() {
  if (impl_) {
    return;
  }
  // One invariant, the same one DdsTransport keeps: started() is true only
  // while an owned implementation has a launched thread that has not left its
  // loop. The implementation is owned and the flag is published before the
  // launch, so a loop that ends immediately cannot be reported as running just
  // because the store had not happened yet, and a launch that throws restores
  // both - otherwise started() would stay true over a null implementation, Stop
  // would return early and the publish gate below would accept samples nobody
  // could write. The transport thread clears the flag on its way out
  // (Impl::Run).
  std::unique_ptr<Impl> impl = std::make_unique<Impl>(config_, *this);
  impl_ = std::move(impl);
  started_.store(true, std::memory_order_release);
  try {
    impl_->Start();
  } catch (...) {
    started_.store(false, std::memory_order_release);
    impl_.reset();
    throw;
  }
}

void ClientTransport::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
  impl_.reset();
  started_.store(false, std::memory_order_release);
}

bool ClientTransport::started() const {
  return started_.load(std::memory_order_acquire);
}

bool ClientTransport::PublishRequest(
    const protocol::ExecutionRequest &request) {
  // Nobody is draining the lanes before Start() and after Stop(), so a sample
  // handed over then would be lost without a trace: it is refused and counted
  // like any other refusal instead.
  if (!started()) {
    dropped_requests_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // A stop travels on its own lane; everything else shares the ordinary one.
  if (IsStopLaneOperation(request.operation)) {
    if (stop_requests_.TryPush(request)) {
      stop_lane_published_.fetch_add(1, std::memory_order_relaxed);
      return true;
    }
  } else if (requests_.TryPush(request)) {
    return true;
  }
  dropped_requests_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

bool ClientTransport::PublishCommand(const protocol::JointCommand &command) {
  // Nothing drains the command lane while the transport is not running.
  if (!started()) {
    dropped_commands_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // The command lane keeps the newest frame: replacing an unread one is the
  // loss the counter reports, and so is a frame that cannot be stored. Either
  // way the newest frame is the one that reaches the peer, so the caller learns
  // about the loss from the counter, not from the return value.
  bool evicted = false;
  const bool stored = commands_.Publish(command, &evicted);
  if (!stored || evicted) {
    dropped_commands_.fetch_add(1, std::memory_order_relaxed);
  }
  return stored;
}

bool ClientTransport::PublishHeartbeat(
    const protocol::SupervisorHeartbeat &heartbeat) {
  // Nothing drains the heartbeat lane while the transport is not running.
  if (!started()) {
    dropped_heartbeats_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!heartbeats_.TryPush(heartbeat)) {
    dropped_heartbeats_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

const ClientEvents &ClientTransport::TakeEvents() {
  events_.result_count = 0;

  // The result lane is FIFO, so it is drained until empty up to the size of its
  // destination; a TryPop that reports false after the last entry is the normal
  // empty signal. The state lane keeps one entry, so a single take drains
  // everything it holds. `has_state` is the reader's only guard, so the state
  // payload is written just when there is one: clearing it on every call would
  // copy twenty-three kilobytes per empty take for nothing.
  while (events_.result_count < kClientResultCapacity &&
         results_.TryPop(events_.results[events_.result_count])) {
    ++events_.result_count;
  }
  if (states_.TryTake(state_scratch_)) {
    events_.has_state = true;
    events_.state = state_scratch_.value;
    events_.state_rx_us = state_scratch_.rx_us;
  } else {
    events_.has_state = false;
  }
  return events_;
}

std::uint64_t ClientTransport::dropped_result_count() const {
  return dropped_results_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::dropped_state_count() const {
  return dropped_states_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::dropped_request_count() const {
  return dropped_requests_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::dropped_command_count() const {
  return dropped_commands_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::dropped_heartbeat_count() const {
  return dropped_heartbeats_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::inbound_rejected_count() const {
  return inbound_rejected_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::inbound_read_failure_count() const {
  return inbound_read_failures_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::stop_lane_publish_count() const {
  return stop_lane_published_.load(std::memory_order_relaxed);
}

}  // namespace irmv::transport
