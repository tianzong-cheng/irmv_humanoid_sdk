#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

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

// Which frozen channel one diagnostic query addresses.
enum class ChannelKind {
  Request,
  Result,
  Command,
  Heartbeat,
  State,
};

// Effective policies of the live DDS endpoint that carries one channel. The
// transport reads Request, Command and Heartbeat and writes Result and State,
// so `reader` names the direction of the queried endpoint. This is a
// read-only diagnostic view for tests and operators; the control path never
// consults it, and `available` is false while the transport is not started or
// the endpoint cannot be queried.
struct EndpointQos {
  bool available{false};
  bool reader{false};
  std::string topic_name;
  bool durable{false};    // true when durability is not VOLATILE
  bool reliable{false};   // reliability kind
  bool keep_last{false};  // history kind
  std::uint32_t history_depth{0};
  std::uint64_t max_blocking_time_us{0};
  std::int32_t max_samples{0};               // -1 means unlimited
  std::int32_t max_instances{0};             // -1 means unlimited
  std::int32_t max_samples_per_instance{0};  // -1 means unlimited
};

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
// pins Domain/General/Interfaces to config.dds.interface, or no configuration
// at all when the configured interface is empty. CycloneDDS caches domain
// configurations per domain id inside the process and the first creator wins,
// so the binding is process wide for that domain: a participant created earlier
// with other settings keeps them. An interface that does not exist makes
// participant creation fail, because the pinned version marks interfaces listed
// explicitly in the configuration as required to be present.
//
// Dropping: nothing here waits for the peer, and nothing here blocks on a full
// channel. The result queue and the two request lanes are FIFO: a push that
// does not fit is refused and counted, so order is preserved and the refusal
// costs the newest sample of that lane. The state, command and heartbeat lanes
// keep only their newest entry: an unread entry that a newer sample replaces is
// counted as dropped, so a burst can never hide the latest command or stop
// request behind older samples. Inbound samples that fail validation are
// rejected whole and counted. The transport never drops an already received
// sample silently.
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

  // Asks the transport thread to stop and joins it. The wait for the thread to
  // leave its loop is bounded (two seconds, more than a hundred loop periods),
  // and every blocking operation inside that loop is itself bounded - the
  // WaitSet wait and each DDS write's max_blocking_time - so the thread always
  // leaves well inside the bound and the join that follows returns promptly.
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
  // Joint commands dropped because a newer command superseded an unread one or
  // because none could be stored.
  std::uint64_t dropped_command_count() const;
  // Supervisor heartbeats dropped because a newer heartbeat superseded an
  // unread one or because none could be stored.
  std::uint64_t dropped_heartbeat_count() const;
  // Inbound samples that failed FromWire validation and were discarded whole.
  std::uint64_t inbound_rejected_count() const;
  // Reads that failed and ended the transport loop, counted separately so that
  // a malformed sample can be told apart from a broken endpoint.
  std::uint64_t inbound_read_failure_count() const;

  // Reads the durability, reliability, history and resource policies back from
  // the live endpoint that carries one frozen channel. Diagnostic only: it does
  // not change transport behaviour, is safe to call while the transport runs,
  // and reports `available == false` instead of throwing when the endpoint is
  // not there.
  EndpointQos endpoint_qos(ChannelKind kind) const;

 private:
  struct Impl;

  // The request and result lanes are FIFO, so their order survives and a full
  // queue refuses the newest entry instead of displacing an older one; the
  // state, command and heartbeat lanes keep only the newest entry, so a burst
  // can never hide the latest sample behind an older one.
  using RequestChannel =
      BoundedChannel<runtime::Received<protocol::ExecutionRequest>,
                     runtime::kRequestBatchCapacity>;
  using StopRequestChannel =
      BoundedChannel<runtime::Received<protocol::ExecutionRequest>,
                     runtime::kStopRequestBatchCapacity>;
  using CommandChannel =
      LatestValueChannel<runtime::Received<protocol::JointCommand>>;
  using HeartbeatChannel =
      LatestValueChannel<runtime::Received<protocol::SupervisorHeartbeat>>;
  using StateChannel = LatestValueChannel<protocol::RobotState>;
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
  std::atomic<std::uint64_t> inbound_read_failures_{0};
};

}  // namespace irmv::transport
