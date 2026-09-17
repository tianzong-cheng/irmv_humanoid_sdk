#include "irmv/transport/dds_transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <dds/dds.hpp>

#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"

namespace irmv::transport {
namespace {

namespace wire = irmv::protocol::wire;

// How long the transport thread may wait for inbound traffic before it looks at
// the outbound channels again. The value bounds both the delay of an outbound
// sample and the time Stop() needs to be noticed.
constexpr int kWaitTimeoutMs = 10;

// Samples read from one reader per wake-up. Bounded so that a burst cannot keep
// the thread from writing pending output or from noticing a stop request.
constexpr int kReadBatch = 16;

// Bound Stop() gives the transport thread to leave its loop before it joins.
// The loop only blocks in bounded operations (a ten millisecond WaitSet wait and
// DDS writes bounded by their max_blocking_time), so the thread leaves well
// inside this bound and the join that follows returns immediately.
constexpr int kStopTimeoutMs = 2000;

protocol::TimeUs NowUs() {
  return static_cast<protocol::TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Pins the participant to one network interface. The pinned C++ API takes a
// configuration string on the participant constructor and parses it as XML when
// it starts with '<'; this is the only per-participant configuration route the
// pinned version has.
std::string InterfaceConfig(const std::string &interface) {
  return "<CycloneDDS><Domain id=\"any\"><General><Interfaces><NetworkInterface "
         "name=\"" +
         interface +
         "\"/></Interfaces></General></Domain></CycloneDDS>";
}

// A reliable writer can refuse a sample when its history is full and the
// blocking time expires. The sample is counted instead of being lost silently,
// and the transport thread keeps running.
template <typename Writer, typename Sample>
void TryWrite(Writer &writer, const Sample &sample,
              std::atomic<std::uint64_t> &dropped) {
  try {
    writer.write(sample);
  } catch (const std::exception &) {
    dropped.fetch_add(1, std::memory_order_relaxed);
  }
}

// Every read in the transport thread runs behind this guard. The thread entry
// function must not let an exception escape it, so a failing DDS call ends the
// transport loop cleanly - the same policy as the guarded WaitSet wait - and is
// counted on its own counter, apart from samples that fail validation.
template <typename Body>
bool ReadGuarded(std::atomic<std::uint64_t> &read_failures, Body body) {
  try {
    body();
    return true;
  } catch (const std::exception &) {
    read_failures.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
}

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
  report.durable = durability.kind() != dds::core::policy::DurabilityKind::VOLATILE;
  report.reliable =
      reliability.kind() == dds::core::policy::ReliabilityKind::RELIABLE;
  report.max_blocking_time_us = static_cast<std::uint64_t>(
      reliability.max_blocking_time().to_microsecs());
  report.keep_last = history.kind() == dds::core::policy::HistoryKind::KEEP_LAST;
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
  Impl(const configuration::RuntimeConfig &config, DdsTransport &owner_transport)
      : owner(owner_transport),
        participant(config.dds.domain,
                    dds::domain::qos::DomainParticipantQos(), nullptr,
                    dds::core::status::StatusMask::none(),
                    config.dds.interface.empty()
                        ? std::string()
                        : InterfaceConfig(config.dds.interface)),
        publisher(participant),
        subscriber(participant),
        request_topic(participant,
                      std::string(protocol::kExecutionRequestTopic.name)),
        command_topic(participant,
                      std::string(protocol::kJointCommandTopic.name)),
        heartbeat_topic(
            participant,
            std::string(protocol::kSupervisorHeartbeatTopic.name)),
        result_topic(participant,
                     std::string(protocol::kExecutionResultTopic.name)),
        state_topic(participant,
                    std::string(protocol::kRobotStateTopic.name)),
        request_reader(subscriber, request_topic,
                       codec::ToReaderQos(protocol::kExecutionRequestTopic.qos)),
        command_reader(subscriber, command_topic,
                       codec::ToReaderQos(protocol::kJointCommandTopic.qos)),
        heartbeat_reader(
            subscriber, heartbeat_topic,
            codec::ToReaderQos(protocol::kSupervisorHeartbeatTopic.qos)),
        result_writer(
            publisher, result_topic,
            codec::ToWriterQos(protocol::kExecutionResultTopic.qos)),
        state_writer(publisher, state_topic,
                     codec::ToWriterQos(protocol::kRobotStateTopic.qos)),
        request_condition(request_reader,
                          dds::sub::status::DataState::any()),
        command_condition(command_reader,
                          dds::sub::status::DataState::any()),
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
      bool evicted = false;
      if (IsStopLaneOperation(value.operation)) {
        owner.stop_requests_.TryPushEvictOldest(received, &evicted);
      } else {
        owner.requests_.TryPushEvictOldest(received, &evicted);
      }
      if (evicted) {
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
      bool evicted = false;
      owner.commands_.TryPushEvictOldest(
          runtime::Received<protocol::JointCommand>{value, rx_us}, &evicted);
      if (evicted) {
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
      bool evicted = false;
      owner.heartbeats_.TryPushEvictOldest(
          runtime::Received<protocol::SupervisorHeartbeat>{value, rx_us},
          &evicted);
      if (evicted) {
        owner.dropped_heartbeats_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::WriteOutbound() {
  // The state and its wire representation are both large, so nothing is
  // constructed while the state channel is empty. A stale empty() read only
  // delays the snapshot by one loop period; the channel keeps the newest value.
  if (!owner.state_.empty()) {
    protocol::RobotState state;
    if (owner.state_.TryPop(state)) {
      wire::RobotState wire_state;
      codec::ToWire(state, wire_state);
      TryWrite(state_writer, wire_state, owner.dropped_states_);
    }
  }

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

  while (input_.request_count < runtime::kRequestBatchCapacity &&
         requests_.TryPop(input_.requests[input_.request_count])) {
    ++input_.request_count;
  }
  while (input_.stop_request_count < runtime::kStopRequestBatchCapacity &&
         stop_requests_.TryPop(input_.stop_requests[input_.stop_request_count])) {
    ++input_.stop_request_count;
  }
  while (input_.command_count < runtime::kCommandBatchCapacity &&
         commands_.TryPop(input_.commands[input_.command_count])) {
    ++input_.command_count;
  }
  while (input_.heartbeat_count < runtime::kHeartbeatBatchCapacity &&
         heartbeats_.TryPop(input_.heartbeats[input_.heartbeat_count])) {
    ++input_.heartbeat_count;
  }
  return input_;
}

void DdsTransport::Publish(const runtime::TickResult &result) {
  bool evicted = false;
  state_.TryPushEvictOldest(result.state, &evicted);
  if (evicted) {
    dropped_states_.fetch_add(1, std::memory_order_relaxed);
  }

  const auto capacity = static_cast<std::uint32_t>(runtime::kResultCapacity);
  const std::uint32_t count = std::min(result.result_count, capacity);
  for (std::uint32_t index = 0; index < count; ++index) {
    bool result_evicted = false;
    results_.TryPushEvictOldest(result.results[index], &result_evicted);
    if (result_evicted) {
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
