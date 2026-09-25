#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/command_gate.hpp"
#include "irmv/transport/bounded_channel.hpp"

namespace irmv::transport {

// Capacities of the application-side handoff channels. They mirror the
// runtime's own lanes: the two request lanes and the heartbeat lane are FIFO,
// the inbound result lane is FIFO, and the state and command lanes keep only
// their newest entry.
inline constexpr std::size_t kClientResultCapacity = 32;
inline constexpr std::size_t kClientRequestCapacity = 32;
inline constexpr std::size_t kClientStopRequestCapacity = 8;
inline constexpr std::size_t kClientHeartbeatCapacity = 8;

// One drain of the inbound lanes. The transport thread fills the lanes, the
// supervision thread takes them here; this batch describes only that one call,
// so its counts are reset on every take while the drop counters of the
// transport itself are cumulative accessors.
struct ClientEvents {
  std::array<runtime::Received<protocol::ExecutionResult>,
             kClientResultCapacity>
      results{};
  std::size_t result_count{0};
  // The newest state of the state lane, if one was received since the last
  // call. The state lane keeps one entry, so an unread state that a newer
  // sample replaced was counted as dropped instead of being delivered late.
  bool has_state{false};
  // `state` and `state_rx_us` are meaningful only while `has_state` is true:
  // they keep the previous take's values otherwise, which is what lets an empty
  // take cost nothing.
  protocol::RobotState state{};
  // Host monotonic receive stamp of `state`.
  protocol::TimeUs state_rx_us{0};
};

// Application-side mirror of DdsTransport for the five frozen topics: it writes
// the requests (both lanes), the target frames and the heartbeats, and reads
// the execution results and the robot states.
//
// Thread model: one transport thread owns every DDS reader and writer and
// carries every conversion, so the control thread never performs a DDS call.
// Both directions cross fixed-capacity channels between exactly those two
// threads; the control thread pushes output (Publish) and pops input
// (TakeEvents), the transport thread does the opposite. The transport thread
// waits on a WaitSet over the result and the state reader with a bounded ten
// millisecond timeout, which keeps Stop() responsive and bounds the delay of an
// outbound sample to one wait period.
//
// Interface binding and dropping follow DdsTransport exactly: the participant
// is pinned to config.dds.interface through an inline XML snippet, nothing
// waits for a peer, nothing blocks on a full channel, and every refusal, drop
// or rejected sample is counted. The request lanes, the result lane and the
// heartbeat lane are FIFO - a push that does not fit is refused and counted, so
// order survives and the newest sample of that lane is the one lost - while the
// state and command lanes keep only their newest entry, so a burst can never
// hide the latest sample behind an older one.
class ClientTransport {
 public:
  // Stores the immutable configuration snapshot; the referenced configuration
  // must outlive the transport. No DDS entity is created before Start().
  explicit ClientTransport(const configuration::ApplicationConfig &config);
  ~ClientTransport();

  ClientTransport(const ClientTransport &) = delete;
  ClientTransport &operator=(const ClientTransport &) = delete;

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
  //
  // The shipped application process has no consumer for this: it constructs the
  // transport, Starts it once and lets the destructor Stop() it, and a start
  // failure is already an exception. It exists for a caller that supervises the
  // transport's own lifetime (a restart policy, a test).
  bool started() const;

  // Hands one request to the transport thread. A stop request (Damping,
  // ZeroTorque, Reset) goes to its own lane, so a full ordinary queue can never
  // hold one back and a burst of targets can never displace one; every other
  // operation shares the ordinary lane, whose order carries meaning. Never
  // blocks, never touches DDS and never allocates; a request that does not fit,
  // and one handed over while the transport is not running, is refused, counted
  // and reported by returning false.
  bool PublishRequest(const protocol::ExecutionRequest &request);

  // Hands one target frame to the newest-entry command lane. A frame that
  // supersedes an unread one is counted as dropped, and so are a frame that
  // cannot be stored at all and one handed over while the transport is not
  // running.
  bool PublishCommand(const protocol::JointCommand &command);

  // Hands one heartbeat to the queue. A full lane refuses the newest heartbeat
  // and counts it, because only the state machine can tell the owner's sample
  // from a foreign or reordered one; a heartbeat handed over while the
  // transport is not running is refused and counted as well.
  bool PublishHeartbeat(const protocol::SupervisorHeartbeat &heartbeat);

  // Drains the inbound lanes into the owned event batch and returns it. Only
  // the entries below the returned count are written, and the batch describes
  // this one call: an empty take costs one pass over the empty result lane and
  // one try of the state lane. The reference stays valid until the next call.
  // Called only by the supervision thread.
  const ClientEvents &TakeEvents();

  // The counters below are the transport's own evidence for a test or a future
  // health line: nothing in the shipped application process reads them today
  // (its status line reports the behaviour and the session, not the transport's
  // drop counts), and no counter is decremented or reset.
  //
  // Execution results dropped because the inbound result queue was full.
  std::uint64_t dropped_result_count() const;
  // Robot states dropped because a newer sample superseded them: the state lane
  // keeps one entry, so a slow consumer loses the superseded ones and always
  // gets the newest.
  std::uint64_t dropped_state_count() const;
  // Requests dropped, from the ordinary and the stop lane, including every one
  // handed over while the transport was not running.
  std::uint64_t dropped_request_count() const;
  // Joint commands dropped because a newer command superseded an unread one,
  // because none could be stored, or because the transport was not running.
  std::uint64_t dropped_command_count() const;
  // Heartbeats dropped because the heartbeat lane was full when the sample was
  // handed over, or because the transport was not running.
  std::uint64_t dropped_heartbeat_count() const;
  // Inbound samples that failed FromWire validation and were discarded whole.
  std::uint64_t inbound_rejected_count() const;
  // Reads that failed and ended the transport loop, counted separately so that
  // a malformed sample can be told apart from a broken endpoint.
  std::uint64_t inbound_read_failure_count() const;
  // Requests that went to the stop lane, counted apart from the ordinary lane:
  // the lane is what makes a stop independent of ordinary traffic, and this is
  // the only way a caller can see which lane took a request.
  std::uint64_t stop_lane_publish_count() const;

 private:
  struct Impl;

  // The request lanes, the result lane and the heartbeat lane are FIFO, so
  // their order survives and a full queue refuses the newest entry instead of
  // displacing an older one; the state and command lanes keep only the newest
  // entry, so a burst can never hide the latest sample behind an older one.
  using RequestChannel =
      BoundedChannel<protocol::ExecutionRequest, kClientRequestCapacity>;
  using StopRequestChannel =
      BoundedChannel<protocol::ExecutionRequest, kClientStopRequestCapacity>;
  using CommandChannel = LatestValueChannel<protocol::JointCommand>;
  using HeartbeatChannel =
      BoundedChannel<protocol::SupervisorHeartbeat, kClientHeartbeatCapacity>;
  using ResultChannel =
      BoundedChannel<runtime::Received<protocol::ExecutionResult>,
                     kClientResultCapacity>;
  // The state lane carries the host receive stamp with the value, so the
  // supervision thread judges the age of a state against the moment the
  // transport read it and not against the moment it looked.
  using StateChannel =
      LatestValueChannel<runtime::Received<protocol::RobotState>>;

  const configuration::ApplicationConfig &config_;
  RequestChannel requests_;
  StopRequestChannel stop_requests_;
  CommandChannel commands_;
  HeartbeatChannel heartbeats_;
  ResultChannel results_;
  StateChannel states_;
  ClientEvents events_;
  // Inbound scratch of the supervision thread, kept out of the take so an empty
  // take neither clears the batch's state nor builds a twenty-four kilobyte
  // frame on the stack.
  runtime::Received<protocol::RobotState> state_scratch_{};
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> started_{false};
  std::atomic<std::uint64_t> stop_lane_published_{0};
  std::atomic<std::uint64_t> dropped_results_{0};
  std::atomic<std::uint64_t> dropped_states_{0};
  std::atomic<std::uint64_t> dropped_requests_{0};
  std::atomic<std::uint64_t> dropped_commands_{0};
  std::atomic<std::uint64_t> dropped_heartbeats_{0};
  std::atomic<std::uint64_t> inbound_rejected_{0};
  std::atomic<std::uint64_t> inbound_read_failures_{0};
};

}  // namespace irmv::transport
