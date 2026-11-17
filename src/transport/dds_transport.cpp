#include "irmv/transport/dds_transport.hpp"

#include <poll.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <dds/dds.hpp>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "dds_loop.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"
#include "irmv/transport/event_signal.hpp"

namespace irmv::transport {
namespace {

namespace wire = irmv::protocol::wire;

// Batches read per simulation wake before yielding to outbound writes; a full
// batch means the lane may have more, so the worker self-wakes to continue.
constexpr int kSimulationDrainRounds = 8;

// Shutdown-safety bound for the simulation worker's blocking wait. Wakes are
// published after the data they announce, so this never delays ready work; it
// only lets a worker leave if a wake notification itself failed. Stop() also
// writes the same eventfd.
constexpr int kSimulationWakePollMs = 100;

// Wake-only DDS listener. Middleware callbacks may not take samples or call
// any other DDS operation here: they only make the worker's eventfd readable.
// The sole transport thread performs every read, write and entity operation.
template <typename T>
class DataAvailableWake final : public dds::sub::NoOpDataReaderListener<T> {
 public:
  DataAvailableWake(EventSignal &wake, std::atomic<bool> &failed) noexcept
      : wake_(wake), failed_(failed) {}

  void on_data_available(dds::sub::DataReader<T> &) noexcept override {
    if (!wake_.Notify()) failed_.store(true, std::memory_order_relaxed);
  }

 private:
  EventSignal &wake_;
  std::atomic<bool> &failed_;
};

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
    if (owner.simulation_) {
      // The wake path is an eventfd only: control and middleware callbacks make
      // no DDS call, and this transport thread performs every DDS operation.
      if (!simulation_wake.Open())
        throw std::runtime_error("simulation wake eventfd");
      request_wake =
          std::make_unique<DataAvailableWake<wire::ExecutionRequest>>(
              simulation_wake, owner.worker_wake_failed_);
      command_wake = std::make_unique<DataAvailableWake<wire::JointCommand>>(
          simulation_wake, owner.worker_wake_failed_);
      heartbeat_wake =
          std::make_unique<DataAvailableWake<wire::SupervisorHeartbeat>>(
              simulation_wake, owner.worker_wake_failed_);
      try {
        request_reader.listener(
            request_wake.get(),
            dds::core::status::StatusMask::data_available());
        command_reader.listener(
            command_wake.get(),
            dds::core::status::StatusMask::data_available());
        heartbeat_reader.listener(
            heartbeat_wake.get(),
            dds::core::status::StatusMask::data_available());
      } catch (...) {
        // dds_set_listener can throw after an earlier reader was attached.
        // Quiesce every simulation reader while the listeners and eventfd are
        // still alive before member unwinding destroys them.
        QuiesceReaders();
        throw;
      }
    }
  }
  ~Impl() { QuiesceReaders(); }

  void Start() {
    thread = std::thread(&Impl::Run, this);
    // Registration can precede the thread start; one initial wake makes the
    // worker drain anything that was already available.
    if (simulation_wake.valid()) simulation_wake.Notify();
  }

  void Run();
  bool ReadInbound();
  bool ReadInboundDrain();
  void ReadRequests();
  void ReadCommands();
  void ReadHeartbeats();
  void WriteOutbound();
  void Stop();
  void WakeControl() noexcept;
  void QuiesceReaders() noexcept;

  DdsTransport &owner;
  dds::domain::DomainParticipant participant;
  dds::pub::Publisher publisher;
  dds::sub::Subscriber subscriber;
  dds::topic::Topic<wire::ExecutionRequest> request_topic;
  dds::topic::Topic<wire::JointCommand> command_topic;
  dds::topic::Topic<wire::SupervisorHeartbeat> heartbeat_topic;
  dds::topic::Topic<wire::ExecutionResult> result_topic;
  dds::topic::Topic<wire::RobotState> state_topic;
  // Declared before the readers and the listeners it feeds: the eventfd must
  // outlive every middleware callback that can write it. Opened only when a
  // simulation is configured.
  EventSignal simulation_wake;
  std::unique_ptr<DataAvailableWake<wire::ExecutionRequest>> request_wake;
  std::unique_ptr<DataAvailableWake<wire::JointCommand>> command_wake;
  std::unique_ptr<DataAvailableWake<wire::SupervisorHeartbeat>> heartbeat_wake;
  dds::sub::DataReader<wire::ExecutionRequest> request_reader;
  dds::sub::DataReader<wire::JointCommand> command_reader;
  dds::sub::DataReader<wire::SupervisorHeartbeat> heartbeat_reader;
  dds::pub::DataWriter<wire::ExecutionResult> result_writer;
  dds::pub::DataWriter<wire::RobotState> state_writer;
  dds::sub::cond::ReadCondition request_condition;
  dds::sub::cond::ReadCondition command_condition;
  dds::sub::cond::ReadCondition heartbeat_condition;
  dds::core::cond::WaitSet waitset;
  // Samples taken by the most recent read of each lane, so the simulation
  // drain can tell whether a listener wake still has a backlog behind it.
  std::size_t request_taken_{0}, command_taken_{0}, heartbeat_taken_{0};
  bool simulation_readers_closed_{false};
  std::thread thread;
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool finished{false};
};

void DdsTransport::Impl::Run() {
  while (!stop_requested.load(std::memory_order_relaxed)) {
    bool data_available = false;
    if (simulation_wake.valid()) {
      // Simulation: block on the wake eventfd until control, a DDS listener,
      // or Stop writes it. This thread does every read and write after waking,
      // so there is no polling floor on ready work.
      pollfd descriptor{simulation_wake.fd(), POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, kSimulationWakePollMs);
      if (ready < 0 && errno == EINTR)
        continue;  // outer loop rechecks stop and recomputes the wait
      if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
        break;
      simulation_wake.Drain();
      if (stop_requested.load(std::memory_order_relaxed)) break;
    } else {
      try {
        data_available =
            !waitset.wait(dds::core::Duration::from_millisecs(kWaitTimeoutMs))
                 .empty();
      } catch (const dds::core::TimeoutError &) {
        // The pinned WaitSet reports a timeout by throwing; that is the normal
        // "nothing to read within the wait window" path.
        data_available = false;
      } catch (const std::exception &) {
        // A failing wait means the entities this thread owns are unusable.
        // Leave the loop instead of spinning on the error: the channels keep
        // working and started() turns false.
        break;
      }
    }
    if (owner.simulation_) owner.simulation_->WorkerUpdate();
    if (simulation_wake.valid()) {
      if (!ReadInboundDrain()) break;
    } else if (data_available && !ReadInbound()) {
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

bool DdsTransport::Impl::ReadInboundDrain() {
  // Middleware notifications coalesce. A listener wake therefore does not
  // promise that one batch empties a lane: read while any lane returned a full
  // batch, then self-wake so a remaining backlog continues on the next
  // iteration instead of waiting for the shutdown-safety timeout.
  for (int round = 0; round < kSimulationDrainRounds; ++round) {
    if (!ReadInbound()) return false;
    const auto batch = static_cast<std::size_t>(kReadBatch);
    if (request_taken_ < batch && command_taken_ < batch &&
        heartbeat_taken_ < batch)
      return true;
    if (stop_requested.load(std::memory_order_relaxed)) return true;
  }
  simulation_wake.Notify();
  return true;
}

void DdsTransport::Impl::ReadRequests() {
  // All samples of one read share the host receive time of that read.
  {
    protocol::TimeUs rx_us = NowUs();
    auto samples = request_reader.select()
                       .max_samples(kReadBatch)
                       .state(dds::sub::status::DataState::any())
                       .take();
    request_taken_ = samples.length();
    for (const auto &sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }
      protocol::ExecutionRequest value;
      if (!codec::FromWire(sample.data(), value)) {
        owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      simulation::MessageIdentity identity;
      if (owner.simulation_) {
        const int disposition =
            simulation::Identify(value, &identity)
                ? owner.simulation_->WorkerCheck(identity, &rx_us)
                : -1;
        if (disposition != 1) {
          if (disposition < 0)
            owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
      }

      runtime::Received<protocol::ExecutionRequest> received{value, rx_us};
      // Both request lanes are FIFO, so a refusal costs this newest sample and
      // the entries already queued keep their order; a full ordinary queue
      // cannot reach the stop lane, which has its own channel.
      const bool stored = IsStopLaneOperation(value.operation)
                              ? owner.stop_requests_.TryPush(received)
                              : owner.requests_.TryPush(received);
      if (owner.simulation_) owner.simulation_->WorkerStored(identity, stored);
      if (!stored) {
        owner.dropped_requests_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::ReadCommands() {
  {
    protocol::TimeUs rx_us = NowUs();
    auto samples = command_reader.select()
                       .max_samples(kReadBatch)
                       .state(dds::sub::status::DataState::any())
                       .take();
    command_taken_ = samples.length();
    for (const auto &sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }
      protocol::JointCommand value;
      if (!codec::FromWire(sample.data(), value)) {
        owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      simulation::MessageIdentity identity;
      if (owner.simulation_) {
        const int disposition =
            simulation::Identify(value, &identity)
                ? owner.simulation_->WorkerCheck(identity, &rx_us)
                : -1;
        if (disposition != 1) {
          if (disposition < 0)
            owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
      }

      // The command lane keeps only the newest entry: a sample that supersedes
      // an unread one, and one that cannot be stored at all, are both losses
      // the control thread would never see otherwise.
      bool evicted = false;
      const bool stored =
          owner.simulation_
              ? owner.sim_commands_->TryPush({value, rx_us})
              : owner.commands_.Publish(
                    runtime::Received<protocol::JointCommand>{value, rx_us},
                    &evicted);
      if (owner.simulation_) owner.simulation_->WorkerStored(identity, stored);
      if (!stored || evicted) {
        owner.dropped_commands_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::ReadHeartbeats() {
  {
    protocol::TimeUs rx_us = NowUs();
    auto samples = heartbeat_reader.select()
                       .max_samples(kReadBatch)
                       .state(dds::sub::status::DataState::any())
                       .take();
    heartbeat_taken_ = samples.length();
    for (const auto &sample : samples) {
      if (!sample.info().valid()) {
        continue;
      }
      protocol::SupervisorHeartbeat value;
      if (!codec::FromWire(sample.data(), value)) {
        owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      simulation::MessageIdentity identity;
      if (owner.simulation_) {
        const int disposition =
            simulation::Identify(value, &identity)
                ? owner.simulation_->WorkerCheck(identity, &rx_us)
                : -1;
        if (disposition != 1) {
          if (disposition < 0)
            owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
      }

      // The heartbeat lane is a FIFO of kHeartbeatBatchCapacity, like the
      // request lanes: only the state machine can tell an owner heartbeat from
      // a foreign or reordered one, so every received sample has to reach it in
      // receive order. A full lane refuses this newest sample and keeps the
      // older ones the machine still has to judge.
      const bool stored = owner.heartbeats_.TryPush(
          runtime::Received<protocol::SupervisorHeartbeat>{value, rx_us});
      if (owner.simulation_) owner.simulation_->WorkerStored(identity, stored);
      if (!stored) {
        owner.dropped_heartbeats_.fetch_add(1, std::memory_order_relaxed);
      } else {
        owner.heartbeats_read_.fetch_add(1, std::memory_order_relaxed);
        owner.last_read_heartbeat_seq_.store(value.heartbeat_seq,
                                             std::memory_order_relaxed);
        owner.last_read_heartbeat_us_.store(rx_us, std::memory_order_relaxed);
      }
    }
  }
}

void DdsTransport::Impl::WriteOutbound() {
  // The state channel keeps one snapshot, so a take either hands it over or
  // reports nothing published; the wire representation is built only in the
  // first case, which keeps the loop cheap while no snapshot is pending.
  protocol::RobotState state;
  while (owner.simulation_ ? owner.sim_states_out_->TryPop(state)
                           : owner.state_.TryTake(state)) {
    wire::RobotState wire_state;
    codec::ToWire(state, wire_state);
    TryWrite(state_writer, wire_state, owner.dropped_states_);
    if (!owner.simulation_) break;
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

void DdsTransport::Impl::WakeControl() noexcept {
  if (!simulation_wake.Notify())
    owner.worker_wake_failed_.store(true, std::memory_order_relaxed);
}

void DdsTransport::Impl::QuiesceReaders() noexcept {
  // Reader::close() calls prevent_callbacks() before it clears the listener,
  // so it waits out every C++ callback while listener objects and the eventfd
  // are still alive. Plain listener(nullptr) mutates the raw listener pointer
  // before that wait, which can race an already-entered callback. Idempotent
  // and simulation-only; non-simulation readers keep their normal teardown.
  if (simulation_readers_closed_ || !owner.simulation_) return;
  simulation_readers_closed_ = true;
  try {
    request_reader.close();
  } catch (...) {
  }
  try {
    command_reader.close();
  } catch (...) {
  }
  try {
    heartbeat_reader.close();
  } catch (...) {
  }
}

void DdsTransport::Impl::Stop() {
  stop_requested.store(true, std::memory_order_relaxed);
  simulation_wake.Notify();
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
  // Join first, then quiesce the middleware callbacks: no worker remains and
  // Reader::close waits out any callback already running.
  QuiesceReaders();
}

DdsTransport::DdsTransport(const configuration::RuntimeConfig &config)
    : config_(config) {}

DdsTransport::~DdsTransport() { Stop(); }

bool DdsTransport::ConfigureSimulation(const simulation::LogicalClock &clock) {
  if (impl_ || simulation_) return false;
  sim_commands_ = std::make_unique<
      BoundedChannel<runtime::Received<protocol::JointCommand>, 64>>();
  sim_states_out_ =
      std::make_unique<BoundedChannel<protocol::RobotState, 64>>();
  simulation_ =
      std::make_unique<simulation::Ingress>(simulation::Role::Runtime, clock);
  return true;
}
bool DdsTransport::BeginSimulationPhase(
    const simulation::PhaseManifest &m) noexcept {
  const bool begun = simulation_ && simulation_->ControlBegin(m);
  if (begun) WakeWorker();
  return begun;
}
bool DdsTransport::SimulationPhaseInstalled(
    const simulation::PhaseManifest &m) noexcept {
  return simulation_ && simulation_->ControlInstalled(m);
}
bool DdsTransport::TrySimulationReceipt(simulation::PhaseReceipt *r) noexcept {
  return simulation_ && simulation_->ControlTakeReceipt(r);
}
bool DdsTransport::ReportSimulationProcessed(
    const simulation::MessageIdentity &m) noexcept {
  return simulation_ && simulation_->ControlProcessed(m);
}
bool DdsTransport::TrySimulationAck(simulation::Frame *f) noexcept {
  return simulation_ && simulation_->ControlAck(f);
}
bool DdsTransport::SimulationFailed() const noexcept {
  return simulation_ && (simulation_->Failed() ||
                         worker_wake_failed_.load(std::memory_order_relaxed));
}
int DdsTransport::SimulationWaitFd() const noexcept {
  return simulation_ ? simulation_->ControlWaitFd() : -1;
}
void DdsTransport::DrainSimulationWait() noexcept {
  if (simulation_) simulation_->ControlDrainSignal();
}
void DdsTransport::WakeWorker() noexcept {
  if (!simulation_ || !worker_wake_) return;
  try {
    worker_wake_();
  } catch (...) {
    // The callback is an eventfd write; an exception means the wake path is
    // unusable. Fail the simulation instead of silently falling back to the
    // bounded safety wait.
    worker_wake_failed_.store(true, std::memory_order_relaxed);
  }
}

void DdsTransport::Start() {
  if (impl_) {
    return;
  }
  // One invariant, the same one the application transport keeps: started() is
  // true only while an owned implementation has a launched thread that has not
  // left its loop. The implementation is owned and the flag is published before
  // the launch, so a loop that ends immediately cannot be reported as running
  // just because the store had not happened yet, and a launch that throws
  // restores both. The transport thread clears the flag on its way out
  // (Impl::Run), which is what turns started() false when the loop ends.
  std::unique_ptr<Impl> impl;
  try {
    impl = std::make_unique<Impl>(config_, *this);
  } catch (...) {
    // No callback was installed before construction completed; keep the owner
    // free of any pointer into a failed implementation regardless.
    worker_wake_ = nullptr;
    throw;
  }
  impl_ = std::move(impl);
  started_.store(true, std::memory_order_release);
  if (simulation_) {
    // Install the control wake callback only after the implementation is fully
    // constructed and owned, so a constructor unwind cannot leave the owner
    // holding a callback into a destroyed object.
    try {
      Impl *worker = impl_.get();
      worker_wake_ = [worker] { worker->WakeControl(); };
    } catch (...) {
      started_.store(false, std::memory_order_release);
      worker_wake_ = nullptr;
      impl_.reset();
      throw;
    }
  }
  try {
    impl_->Start();
  } catch (...) {
    started_.store(false, std::memory_order_release);
    worker_wake_ = nullptr;
    impl_.reset();
    throw;
  }
}

void DdsTransport::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
  // The callback captures the implementation; the thread is joined by now.
  worker_wake_ = nullptr;
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
  while (input_.command_count < runtime::kCommandBatchCapacity &&
         (simulation_
              ? sim_commands_->TryPop(input_.commands[input_.command_count])
              : commands_.TryTake(input_.commands[input_.command_count]))) {
    ++input_.command_count;
    if (!simulation_) break;
  }
  while (input_.heartbeat_count < runtime::kHeartbeatBatchCapacity &&
         heartbeats_.TryPop(input_.heartbeats[input_.heartbeat_count])) {
    ++input_.heartbeat_count;
  }
  if (simulation_) {
    auto mark = [this](const auto &value) {
      simulation::MessageIdentity m;
      if (simulation::Identify(value, &m)) simulation_->ControlDequeued(m);
    };
    for (std::size_t i = 0; i < input_.request_count; ++i)
      mark(input_.requests[i].value);
    for (std::size_t i = 0; i < input_.stop_request_count; ++i)
      mark(input_.stop_requests[i].value);
    for (std::size_t i = 0; i < input_.command_count; ++i)
      mark(input_.commands[i].value);
    for (std::size_t i = 0; i < input_.heartbeat_count; ++i)
      mark(input_.heartbeats[i].value);
  }
  return input_;
}

void DdsTransport::Publish(const runtime::TickResult &result) {
  // The state lane keeps the newest snapshot: replacing an unread one is the
  // loss the counter reports, and so is a snapshot that cannot be stored.
  bool evicted = false;
  if (!(simulation_ ? sim_states_out_->TryPush(result.state)
                    : state_.Publish(result.state, &evicted)) ||
      evicted) {
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
  // The state and result writes belong to the transport thread. Without this
  // the control thread's output would wait for the WaitSet timeout.
  WakeWorker();
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

std::uint64_t DdsTransport::heartbeats_read_count() const {
  return heartbeats_read_.load(std::memory_order_relaxed);
}

protocol::Sequence DdsTransport::last_read_heartbeat_seq() const {
  return last_read_heartbeat_seq_.load(std::memory_order_relaxed);
}

protocol::TimeUs DdsTransport::last_read_heartbeat_us() const {
  return last_read_heartbeat_us_.load(std::memory_order_relaxed);
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
