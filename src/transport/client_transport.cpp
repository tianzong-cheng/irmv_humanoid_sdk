#include "irmv/transport/client_transport.hpp"

#include <poll.h>

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

#include "dds_loop.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"
#include "irmv/transport/dds_transport.hpp"
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

}  // namespace

// Every DDS entity lives here and is created by the constructor, which only
// runs from Start(). The transport thread owns all of them after that.
struct ClientTransport::Impl {
  Impl(const configuration::ApplicationConfig &config,
       ClientTransport &owner_transport)
      : owner(owner_transport),
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
    if (owner.simulation_) {
      // The wake path is an eventfd only: control and middleware callbacks make
      // no DDS call, and this transport thread performs every DDS operation.
      if (!simulation_wake.Open())
        throw std::runtime_error("simulation wake eventfd");
      result_wake = std::make_unique<DataAvailableWake<wire::ExecutionResult>>(
          simulation_wake, owner.worker_wake_failed_);
      state_wake = std::make_unique<DataAvailableWake<wire::RobotState>>(
          simulation_wake, owner.worker_wake_failed_);
      try {
        result_reader.listener(result_wake.get(),
                               dds::core::status::StatusMask::data_available());
        state_reader.listener(state_wake.get(),
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
  void ReadResults();
  void ReadStates();
  void WriteOutbound();
  void Stop();
  void WakeControl() noexcept;
  void QuiesceReaders() noexcept;

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
  // Declared before the readers and the listeners it feeds: the eventfd must
  // outlive every middleware callback that can write it. Opened only when a
  // simulation is configured.
  EventSignal simulation_wake;
  std::unique_ptr<DataAvailableWake<wire::ExecutionResult>> result_wake;
  std::unique_ptr<DataAvailableWake<wire::RobotState>> state_wake;
  dds::sub::DataReader<wire::ExecutionResult> result_reader;
  dds::sub::DataReader<wire::RobotState> state_reader;
  dds::sub::cond::ReadCondition result_condition;
  dds::sub::cond::ReadCondition state_condition;
  dds::core::cond::WaitSet waitset;
  // Samples taken by the most recent read of each lane, so the simulation
  // drain can tell whether a listener wake still has a backlog behind it.
  std::size_t result_taken_{0}, state_taken_{0};
  bool simulation_readers_closed_{false};
  std::thread thread;
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool finished{false};
};

void ClientTransport::Impl::Run() {
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

bool ClientTransport::Impl::ReadInbound() {
  // Each lane is drained behind the same guard: a failing read ends the
  // transport loop cleanly and is counted, and the lane that follows is left
  // untouched.
  return ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadResults(); }) &&
         ReadGuarded(owner.inbound_read_failures_, [this]() { ReadStates(); });
}

bool ClientTransport::Impl::ReadInboundDrain() {
  // Middleware notifications coalesce. A listener wake therefore does not
  // promise that one batch empties a lane: read while either lane returned a
  // full batch, then self-wake so a remaining backlog continues on the next
  // iteration instead of waiting for the shutdown-safety timeout.
  for (int round = 0; round < kSimulationDrainRounds; ++round) {
    if (!ReadInbound()) return false;
    const auto batch = static_cast<std::size_t>(kReadBatch);
    if (result_taken_ < batch && state_taken_ < batch) return true;
    if (stop_requested.load(std::memory_order_relaxed)) return true;
  }
  simulation_wake.Notify();
  return true;
}

void ClientTransport::Impl::ReadResults() {
  // All samples of one read share the host receive time of that read.
  protocol::TimeUs rx_us = NowUs();
  auto samples = result_reader.select()
                     .max_samples(kReadBatch)
                     .state(dds::sub::status::DataState::any())
                     .take();
  result_taken_ = samples.length();
  for (const auto &sample : samples) {
    if (!sample.info().valid()) {
      continue;
    }
    protocol::ExecutionResult value;
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

    // The result lane is FIFO: an answer that does not fit is refused and
    // counted, and the answers already queued keep their order, which is the
    // order the runtime produced them in.
    const bool stored = owner.results_.TryPush(
        runtime::Received<protocol::ExecutionResult>{value, rx_us});
    if (owner.simulation_) owner.simulation_->WorkerStored(identity, stored);
    if (!stored) {
      owner.dropped_results_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void ClientTransport::Impl::ReadStates() {
  protocol::TimeUs rx_us = NowUs();
  auto samples = state_reader.select()
                     .max_samples(kReadBatch)
                     .state(dds::sub::status::DataState::any())
                     .take();
  state_taken_ = samples.length();
  for (const auto &sample : samples) {
    if (!sample.info().valid()) {
      continue;
    }
    protocol::RobotState value;
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

    // The state lane keeps only the newest entry: a sample that supersedes an
    // unread one, and one that cannot be stored at all, are both losses the
    // supervision thread would never see otherwise.
    bool evicted = false;
    const bool stored =
        owner.simulation_
            ? owner.sim_states_->TryPush({value, rx_us})
            : owner.states_.Publish(
                  runtime::Received<protocol::RobotState>{value, rx_us},
                  &evicted);
    if (owner.simulation_) owner.simulation_->WorkerStored(identity, stored);
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
  while (owner.simulation_ ? owner.sim_commands_out_->TryPop(command)
                           : owner.commands_.TryTake(command)) {
    wire::JointCommand wire_command;
    codec::ToWire(command, wire_command);
    TryWrite(command_writer, wire_command, owner.dropped_commands_);
    if (!owner.simulation_) break;
  }

  // The heartbeat lane is FIFO and drained until empty, so every heartbeat the
  // state machine still has to judge reaches it in receive order. A heartbeat
  // the writer accepted is a published one, and the newest of them carries the
  // sequence and the monotonic stamp that tie this side to the runtime's.
  protocol::SupervisorHeartbeat heartbeat;
  while (owner.heartbeats_.TryPop(heartbeat)) {
    wire::SupervisorHeartbeat wire_heartbeat;
    codec::ToWire(heartbeat, wire_heartbeat);
    if (TryWrite(heartbeat_writer, wire_heartbeat, owner.dropped_heartbeats_)) {
      owner.heartbeats_published_.fetch_add(1, std::memory_order_relaxed);
      owner.last_published_heartbeat_seq_.store(heartbeat.heartbeat_seq,
                                                std::memory_order_relaxed);
      owner.last_published_heartbeat_us_.store(NowUs(),
                                               std::memory_order_relaxed);
    }
  }
}

void ClientTransport::Impl::WakeControl() noexcept {
  if (!simulation_wake.Notify())
    owner.worker_wake_failed_.store(true, std::memory_order_relaxed);
}

void ClientTransport::Impl::QuiesceReaders() noexcept {
  // Reader::close() calls prevent_callbacks() before it clears the listener,
  // so it waits out every C++ callback while listener objects and the eventfd
  // are still alive. Plain listener(nullptr) mutates the raw listener pointer
  // before that wait, which can race an already-entered callback. Idempotent
  // and simulation-only; non-simulation readers keep their normal teardown.
  if (simulation_readers_closed_ || !owner.simulation_) return;
  simulation_readers_closed_ = true;
  try {
    result_reader.close();
  } catch (...) {
  }
  try {
    state_reader.close();
  } catch (...) {
  }
}

void ClientTransport::Impl::Stop() {
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

ClientTransport::ClientTransport(const configuration::ApplicationConfig &config)
    : config_(config) {}

ClientTransport::~ClientTransport() { Stop(); }

bool ClientTransport::ConfigureSimulation(
    const simulation::LogicalClock &clock) {
  if (impl_ || simulation_) return false;
  sim_states_ = std::make_unique<
      BoundedChannel<runtime::Received<protocol::RobotState>, 64>>();
  sim_commands_out_ =
      std::make_unique<BoundedChannel<protocol::JointCommand, 64>>();
  simulation_ = std::make_unique<simulation::Ingress>(
      simulation::Role::Application, clock);
  return true;
}
bool ClientTransport::BeginSimulationPhase(
    const simulation::PhaseManifest &m) noexcept {
  const bool begun = simulation_ && simulation_->ControlBegin(m);
  if (begun) WakeWorker();
  return begun;
}
bool ClientTransport::SimulationPhaseInstalled(
    const simulation::PhaseManifest &m) noexcept {
  return simulation_ && simulation_->ControlInstalled(m);
}
bool ClientTransport::TrySimulationReceipt(
    simulation::PhaseReceipt *r) noexcept {
  return simulation_ && simulation_->ControlTakeReceipt(r);
}
bool ClientTransport::ReportSimulationProcessed(
    const simulation::MessageIdentity &m) noexcept {
  return simulation_ && simulation_->ControlProcessed(m);
}
bool ClientTransport::TrySimulationAck(simulation::Frame *f) noexcept {
  return simulation_ && simulation_->ControlAck(f);
}
bool ClientTransport::SimulationFailed() const noexcept {
  return simulation_ && (simulation_->Failed() ||
                         worker_wake_failed_.load(std::memory_order_relaxed));
}
int ClientTransport::SimulationWaitFd() const noexcept {
  return simulation_ ? simulation_->ControlWaitFd() : -1;
}
void ClientTransport::DrainSimulationWait() noexcept {
  if (simulation_) simulation_->ControlDrainSignal();
}
void ClientTransport::WakeWorker() noexcept {
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

void ClientTransport::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
  // The callback captures the implementation; the thread is joined by now.
  worker_wake_ = nullptr;
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
      WakeWorker();
      return true;
    }
  } else if (requests_.TryPush(request)) {
    WakeWorker();
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
  const bool stored = simulation_ ? sim_commands_out_->TryPush(command)
                                  : commands_.Publish(command, &evicted);
  if (!stored || evicted) {
    dropped_commands_.fetch_add(1, std::memory_order_relaxed);
  }
  if (stored) WakeWorker();
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
  // The queue accepted the send, so this is the supervision loop's own cadence:
  // measure the interval from the previous accepted send on the same clock. The
  // first send of the run has no predecessor and no interval.
  const protocol::TimeUs now_us = NowUs();
  const protocol::TimeUs previous_us =
      last_queued_heartbeat_us_.exchange(now_us, std::memory_order_relaxed);
  if (previous_us != 0) {
    const protocol::TimeUs gap_us = now_us - previous_us;
    if (gap_us > worst_heartbeat_gap_us_.load(std::memory_order_relaxed)) {
      worst_heartbeat_gap_us_.store(gap_us, std::memory_order_relaxed);
    }
    const protocol::TimeUs period_us = config_.supervision.heartbeat_period_us;
    if (period_us != 0 && gap_us > 2 * period_us) {
      heartbeats_late_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  heartbeats_queued_.fetch_add(1, std::memory_order_relaxed);
  last_queued_heartbeat_seq_.store(heartbeat.heartbeat_seq,
                                   std::memory_order_relaxed);
  WakeWorker();
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
  if (simulation_ ? sim_states_->TryPop(state_scratch_)
                  : states_.TryTake(state_scratch_)) {
    events_.has_state = true;
    events_.state = state_scratch_.value;
    events_.state_rx_us = state_scratch_.rx_us;
  } else {
    events_.has_state = false;
  }
  if (simulation_) {
    simulation::MessageIdentity m;
    for (std::size_t i = 0; i < events_.result_count; ++i)
      if (simulation::Identify(events_.results[i].value, &m))
        simulation_->ControlDequeued(m);
    if (events_.has_state && simulation::Identify(events_.state, &m))
      simulation_->ControlDequeued(m);
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

std::uint64_t ClientTransport::heartbeats_queued_count() const {
  return heartbeats_queued_.load(std::memory_order_relaxed);
}

protocol::Sequence ClientTransport::last_queued_heartbeat_seq() const {
  return last_queued_heartbeat_seq_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::heartbeats_published_count() const {
  return heartbeats_published_.load(std::memory_order_relaxed);
}

protocol::Sequence ClientTransport::last_published_heartbeat_seq() const {
  return last_published_heartbeat_seq_.load(std::memory_order_relaxed);
}

protocol::TimeUs ClientTransport::last_published_heartbeat_us() const {
  return last_published_heartbeat_us_.load(std::memory_order_relaxed);
}

protocol::TimeUs ClientTransport::worst_heartbeat_gap_us() const {
  return worst_heartbeat_gap_us_.load(std::memory_order_relaxed);
}

std::uint64_t ClientTransport::heartbeats_late_count() const {
  return heartbeats_late_.load(std::memory_order_relaxed);
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
