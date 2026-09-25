#include "irmv/transport/dds_transport.hpp"

#include <algorithm>
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
#include <utility>

#include "dds_loop.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"

namespace irmv::transport {
namespace {

namespace wire = irmv::protocol::wire;

// Reads the effective policies back from a live endpoint. Diagnostic only: the
// control path never calls it, and a failing query is reported instead of being
// thrown at the caller.
template <typename Endpoint>
EndpointQos ReportEndpointQos(const Endpoint &endpoint, bool reader) {
  const auto durability =
      endpoint.qos().template policy<dds::core::policy::Durability>();
  const auto reliability =
      endpoint.qos().template policy<dds::core::policy::Reliability>();
  const auto history =
      endpoint.qos().template policy<dds::core::policy::History>();
  const auto limits =
      endpoint.qos().template policy<dds::core::policy::ResourceLimits>();

  EndpointQos report;
  report.available = true;
  report.reader = reader;
  report.topic_name = endpoint.topic_description().name();
  report.durable =
      durability.kind() != dds::core::policy::DurabilityKind::VOLATILE;
  report.reliable =
      reliability.kind() == dds::core::policy::ReliabilityKind::RELIABLE;
  report.max_blocking_time_us = static_cast<std::uint64_t>(
      reliability.max_blocking_time().to_microsecs());
  report.keep_last =
      history.kind() == dds::core::policy::HistoryKind::KEEP_LAST;
  report.history_depth = history.depth();
  report.max_samples = limits.max_samples();
  report.max_instances = limits.max_instances();
  report.max_samples_per_instance = limits.max_samples_per_instance();
  return report;
}

}  // namespace

bool IsStopLaneOperation(protocol::Operation operation) {
  return operation == protocol::Operation::Damping ||
         operation == protocol::Operation::ZeroTorque ||
         operation == protocol::Operation::Reset;
}

// Every DDS entity lives here and is created by the constructor, which only
// runs from Start(). The transport thread owns all of them after that.
struct DdsTransport::Impl {
  Impl(const configuration::RuntimeConfig &config,
       DdsTransport &owner_transport)
      : owner(owner_transport),
        participant(config.dds.domain, dds::domain::qos::DomainParticipantQos(),
                    nullptr, dds::core::status::StatusMask::none(),
                    config.dds.interface.empty()
                        ? std::string()
                        : InterfaceConfig(config.dds.interface)),
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
        request_reader(
            subscriber, request_topic,
            codec::ToReaderQos(protocol::kExecutionRequestTopic.qos)),
        command_reader(subscriber, command_topic,
                       codec::ToReaderQos(protocol::kJointCommandTopic.qos)),
        heartbeat_reader(
            subscriber, heartbeat_topic,
            codec::ToReaderQos(protocol::kSupervisorHeartbeatTopic.qos)),
        result_writer(publisher, result_topic,
                      codec::ToWriterQos(protocol::kExecutionResultTopic.qos)),
        state_writer(publisher, state_topic,
                     codec::ToWriterQos(protocol::kRobotStateTopic.qos)),
        request_condition(request_reader, dds::sub::status::DataState::any()),
        command_condition(command_reader, dds::sub::status::DataState::any()),
        heartbeat_condition(heartbeat_reader,
                            dds::sub::status::DataState::any()) {
    waitset += request_condition;
    waitset += command_condition;
    waitset += heartbeat_condition;
  }

  void Start() { thread = std::thread(&Impl::Run, this); }

  void Run();
  bool ReadInbound();
  void ReadRequests();
  void ReadCommands();
  void ReadHeartbeats();
  void WriteOutbound();
  void Stop();

  DdsTransport &owner;
  dds::domain::DomainParticipant participant;
  dds::pub::Publisher publisher;
  dds::sub::Subscriber subscriber;
  dds::topic::Topic<wire::ExecutionRequest> request_topic;
  dds::topic::Topic<wire::JointCommand> command_topic;
  dds::topic::Topic<wire::SupervisorHeartbeat> heartbeat_topic;
  dds::topic::Topic<wire::ExecutionResult> result_topic;
  dds::topic::Topic<wire::RobotState> state_topic;
  dds::sub::DataReader<wire::ExecutionRequest> request_reader;
  dds::sub::DataReader<wire::JointCommand> command_reader;
  dds::sub::DataReader<wire::SupervisorHeartbeat> heartbeat_reader;
  dds::pub::DataWriter<wire::ExecutionResult> result_writer;
  dds::pub::DataWriter<wire::RobotState> state_writer;
  dds::sub::cond::ReadCondition request_condition;
  dds::sub::cond::ReadCondition command_condition;
  dds::sub::cond::ReadCondition heartbeat_condition;
  dds::core::cond::WaitSet waitset;
  std::thread thread;
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool finished{false};
};

void DdsTransport::Impl::Run() {
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

bool DdsTransport::Impl::ReadInbound() {
  // Each lane is drained behind the same guard: a failing read ends the
  // transport loop cleanly and is counted, and the lanes that follow are left
  // untouched.
  return ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadRequests(); }) &&
         ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadCommands(); }) &&
         ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadHeartbeats(); });
}

void DdsTransport::Impl::ReadRequests() {
  // All samples of one read share the host receive time of that read.
  {
    const protocol::TimeUs rx_us = NowUs();
    auto samples = request_reader.select()
                       .max_samples(kReadBatch)
                       .state(dds::sub::status::DataState::any())
                       .take();
    for (const auto &sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }
      protocol::ExecutionRequest value;
      if (!codec::FromWire(sample.data(), value)) {
        owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      runtime::Received<protocol::ExecutionRequest> received{value, rx_us};
      // Both request lanes are FIFO, so a refusal costs this newest sample and
      // the entries already queued keep their order; a full ordinary queue
      // cannot reach the stop lane, which has its own channel.
      const bool stored = IsStopLaneOperation(value.operation)
                              ? owner.stop_requests_.TryPush(received)
                              : owner.requests_.TryPush(received);
      if (!stored) {
        owner.dropped_requests_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::ReadCommands() {
  {
    const protocol::TimeUs rx_us = NowUs();
    auto samples = command_reader.select()
                       .max_samples(kReadBatch)
                       .state(dds::sub::status::DataState::any())
                       .take();
    for (const auto &sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }
      protocol::JointCommand value;
      if (!codec::FromWire(sample.data(), value)) {
        owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      // The command lane keeps only the newest entry: a sample that supersedes
      // an unread one, and one that cannot be stored at all, are both losses
      // the control thread would never see otherwise.
      bool evicted = false;
      const bool stored = owner.commands_.Publish(
          runtime::Received<protocol::JointCommand>{value, rx_us}, &evicted);
      if (!stored || evicted) {
        owner.dropped_commands_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::ReadHeartbeats() {
  {
    const protocol::TimeUs rx_us = NowUs();
    auto samples = heartbeat_reader.select()
                       .max_samples(kReadBatch)
                       .state(dds::sub::status::DataState::any())
                       .take();
    for (const auto &sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }
      protocol::SupervisorHeartbeat value;
      if (!codec::FromWire(sample.data(), value)) {
        owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      // The heartbeat lane is a FIFO of kHeartbeatBatchCapacity, like the
      // request lanes: only the state machine can tell an owner heartbeat from
      // a foreign or reordered one, so every received sample has to reach it in
      // receive order. A full lane refuses this newest sample and keeps the
      // older ones the machine still has to judge.
      const bool stored = owner.heartbeats_.TryPush(
          runtime::Received<protocol::SupervisorHeartbeat>{value, rx_us});
      if (!stored) {
        owner.dropped_heartbeats_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::WriteOutbound() {
  // The state channel keeps one snapshot, so a take either hands it over or
  // reports nothing published; the wire representation is built only in the
  // first case, which keeps the loop cheap while no snapshot is pending.
  protocol::RobotState state;
  if (owner.state_.TryTake(state)) {
    wire::RobotState wire_state;
    codec::ToWire(state, wire_state);
    TryWrite(state_writer, wire_state, owner.dropped_states_);
  }

  // The result queue is FIFO and bounded, so it is drained until it is empty;
  // TryPop turning false after the last entry is the normal empty signal.
  protocol::ExecutionResult result;
  while (owner.results_.TryPop(result)) {
    wire::ExecutionResult wire_result;
    codec::ToWire(result, wire_result);
    TryWrite(result_writer, wire_result, owner.dropped_results_);
  }
}

void DdsTransport::Impl::Stop() {
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

DdsTransport::DdsTransport(const configuration::RuntimeConfig &config)
    : config_(config) {}

DdsTransport::~DdsTransport() { Stop(); }

void DdsTransport::Start() {
  if (impl_) {
    return;
  }
  std::unique_ptr<Impl> impl = std::make_unique<Impl>(config_, *this);
  impl->Start();
  impl_ = std::move(impl);
  started_.store(true, std::memory_order_release);
}

void DdsTransport::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
  impl_.reset();
  started_.store(false, std::memory_order_release);
}

bool DdsTransport::started() const {
  return started_.load(std::memory_order_acquire);
}

const runtime::TickInput &DdsTransport::TakeInput() {
  input_.request_count = 0;
  input_.stop_request_count = 0;
  input_.command_count = 0;
  input_.heartbeat_count = 0;
  // This instance is filled from received samples only, so the trusted local
  // stop is cleared here rather than relied upon to stay false: nothing that
  // reaches this class may ever ask the state machine to stop the process.
  input_.local_stop_requested = false;

  // The request lanes, the heartbeat lane and the result lane are FIFO, so each
  // is drained until empty up to the size of its destination. The command lane
  // keeps one entry, so a single take drains everything it holds; a TryTake
  // that reports false right after a take is the normal empty signal.
  while (input_.request_count < runtime::kRequestBatchCapacity &&
         requests_.TryPop(input_.requests[input_.request_count])) {
    ++input_.request_count;
  }
  while (
      input_.stop_request_count < runtime::kStopRequestBatchCapacity &&
      stop_requests_.TryPop(input_.stop_requests[input_.stop_request_count])) {
    ++input_.stop_request_count;
  }
  if (input_.command_count < runtime::kCommandBatchCapacity &&
      commands_.TryTake(input_.commands[input_.command_count])) {
    ++input_.command_count;
  }
  while (input_.heartbeat_count < runtime::kHeartbeatBatchCapacity &&
         heartbeats_.TryPop(input_.heartbeats[input_.heartbeat_count])) {
    ++input_.heartbeat_count;
  }
  return input_;
}

void DdsTransport::Publish(const runtime::TickResult &result) {
  // The state lane keeps the newest snapshot: replacing an unread one is the
  // loss the counter reports, and so is a snapshot that cannot be stored.
  bool evicted = false;
  if (!state_.Publish(result.state, &evicted) || evicted) {
    dropped_states_.fetch_add(1, std::memory_order_relaxed);
  }

  // Results are FIFO: a push that does not fit is refused, and only the results
  // beyond the queue capacity are dropped in addition.
  const auto capacity = static_cast<std::uint32_t>(runtime::kResultCapacity);
  const std::uint32_t count = std::min(result.result_count, capacity);
  for (std::uint32_t index = 0; index < count; ++index) {
    if (!results_.TryPush(result.results[index])) {
      dropped_results_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  if (result.result_count > count) {
    dropped_results_.fetch_add(result.result_count - count,
                               std::memory_order_relaxed);
  }
}

std::uint64_t DdsTransport::dropped_result_count() const {
  return dropped_results_.load(std::memory_order_relaxed);
}

std::uint64_t DdsTransport::dropped_state_count() const {
  return dropped_states_.load(std::memory_order_relaxed);
}

std::uint64_t DdsTransport::dropped_request_count() const {
  return dropped_requests_.load(std::memory_order_relaxed);
}

std::uint64_t DdsTransport::dropped_command_count() const {
  return dropped_commands_.load(std::memory_order_relaxed);
}

std::uint64_t DdsTransport::dropped_heartbeat_count() const {
  return dropped_heartbeats_.load(std::memory_order_relaxed);
}

std::uint64_t DdsTransport::inbound_rejected_count() const {
  return inbound_rejected_.load(std::memory_order_relaxed);
}

std::uint64_t DdsTransport::inbound_read_failure_count() const {
  return inbound_read_failures_.load(std::memory_order_relaxed);
}

EndpointQos DdsTransport::endpoint_qos(ChannelKind kind) const {
  if (!impl_) {
    return EndpointQos{};
  }
  try {
    switch (kind) {
      case ChannelKind::Request:
        return ReportEndpointQos(impl_->request_reader, true);
      case ChannelKind::Result:
        return ReportEndpointQos(impl_->result_writer, false);
      case ChannelKind::Command:
        return ReportEndpointQos(impl_->command_reader, true);
      case ChannelKind::Heartbeat:
        return ReportEndpointQos(impl_->heartbeat_reader, true);
      case ChannelKind::State:
        return ReportEndpointQos(impl_->state_writer, false);
    }
  } catch (const std::exception &) {
    return EndpointQos{};
  }
  return EndpointQos{};
}

}  // namespace irmv::transport
