#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/execution_context.hpp"
#include "irmv/transport/bounded_channel.hpp"

namespace irmv::transport {

// Inbound routing of the frozen request lane: Damping, ZeroTorque and Reset
// travel on their own channel so that a full ordinary request queue can never
// block or drop a stop request; AcquireSession and Tracking are ordinary
// requests.
bool IsStopLaneOperation(protocol::Operation operation);

// Bounded DDS transport for the five frozen topics in protocol/types.hpp.
//
// Thread model: one transport thread owns every DDS reader and writer and
// carries every conversion, so the control thread never performs a DDS call.
// Both directions cross a set of fixed-capacity channels between exactly two
// threads; the control thread pushes output (Publish) and pops input
// (TakeInput), the transport thread does the opposite. The transport thread
// waits on a WaitSet with a bounded ten millisecond timeout, which keeps Stop()
// responsive and bounds the delay of an outbound sample to one wait period.
//
// Interface binding: the pinned CycloneDDS 0.10.5 C++ API takes a configuration
// string on the participant constructor, which is the only per-participant
// configuration route it has (there is no property QoS policy and no property
// accessor in the pinned C runtime). Start() passes an inline XML snippet that
// pins Domain/General/Interfaces to config.dds.interface, or no configuration at
// all when the configured interface is empty. CycloneDDS caches domain
// configurations per domain id inside the process and the first creator wins, so
// the binding is process wide for that domain: a participant created earlier
// with other settings keeps them. An interface that does not exist makes
// participant creation fail, because the pinned version marks interfaces listed
// explicitly in the configuration as required to be present.
//
// Dropping: nothing here waits for the peer. Output that does not fit is
// dropped and counted and the state channel keeps only the newest snapshot. The
// inbound lanes keep the newest entries as well, so a burst can never hide the
// latest command or stop request behind older samples. Inbound samples that
// fail validation are rejected whole and counted. The transport never drops an
// already received sample silently.
class DdsTransport {
 public:
  // Stores the immutable configuration snapshot; the referenced configuration
  // must outlive the transport. No DDS entity is created before Start().
  explicit DdsTransport(const configuration::RuntimeConfig &config);
  ~DdsTransport();

  DdsTransport(const DdsTransport &) = delete;
  DdsTransport &operator=(const DdsTransport &) = delete;

  // Creates the participant, the endpoints and the transport thread.
  // Idempotent; throws dds::core::Error when the participant or an endpoint
  // cannot be created.
  void Start();

  // Asks the transport thread to stop and joins it with a bounded wait.
  // Idempotent, safe on a transport that was never started, and also called by
  // the destructor.
  void Stop();

  // True from a successful Start() until the transport thread has ended.
  bool started() const;

  // Drains the inbound channels into the owned tick input and returns it. Only
  // the entries below the returned counts are written, and the counts are reset
  // on every call, so an empty call touches constant work. The reference stays
  // valid until the next call. Receive stamps come from the host monotonic
  // clock at the moment the transport thread read the sample.
  const runtime::TickInput &TakeInput();

  // Hands one tick of output to the transport thread: the robot state and up to
  // kResultCapacity execution results. Never blocks, never touches DDS and
  // never allocates; what does not fit is dropped and counted.
  void Publish(const runtime::TickResult &result);

  // Execution results dropped because the outbound result queue was full, or
  // because the DDS write failed.
  std::uint64_t dropped_result_count() const;
  // Robot states dropped because a newer snapshot superseded them or because
  // the DDS write failed.
  std::uint64_t dropped_state_count() const;
  // Execution requests dropped, from the ordinary and the stop lane.
  std::uint64_t dropped_request_count() const;
  // Joint commands dropped because the inbound command queue was full.
  std::uint64_t dropped_command_count() const;
  // Supervisor heartbeats dropped because the inbound heartbeat queue was full.
  std::uint64_t dropped_heartbeat_count() const;
  // Inbound samples that failed FromWire validation and were discarded whole.
  std::uint64_t inbound_rejected_count() const;

 private:
  struct Impl;

  using RequestChannel =
      BoundedChannel<runtime::Received<protocol::ExecutionRequest>,
                     runtime::kRequestBatchCapacity>;
  using StopRequestChannel =
      BoundedChannel<runtime::Received<protocol::ExecutionRequest>,
                     runtime::kStopRequestBatchCapacity>;
  using CommandChannel =
      BoundedChannel<runtime::Received<protocol::JointCommand>,
                     runtime::kCommandBatchCapacity>;
  using HeartbeatChannel =
      BoundedChannel<runtime::Received<protocol::SupervisorHeartbeat>,
                     runtime::kHeartbeatBatchCapacity>;
  using StateChannel = BoundedChannel<protocol::RobotState, 1>;
  using ResultChannel =
      BoundedChannel<protocol::ExecutionResult, runtime::kResultCapacity>;

  const configuration::RuntimeConfig &config_;
  RequestChannel requests_;
  StopRequestChannel stop_requests_;
  CommandChannel commands_;
  HeartbeatChannel heartbeats_;
  StateChannel state_;
  ResultChannel results_;
  runtime::TickInput input_;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> started_{false};
  std::atomic<std::uint64_t> dropped_results_{0};
  std::atomic<std::uint64_t> dropped_states_{0};
  std::atomic<std::uint64_t> dropped_requests_{0};
  std::atomic<std::uint64_t> dropped_commands_{0};
  std::atomic<std::uint64_t> dropped_heartbeats_{0};
  std::atomic<std::uint64_t> inbound_rejected_{0};
};

}  // namespace irmv::transport
