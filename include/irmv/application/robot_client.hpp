#pragma once

#include <array>
#include <cstddef>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

// The supervisor identity of one process run: a random sixteen byte id that is
// never the all-zero "none" value. The first byte is forced into 0x01..0x7f so
// it is neither zero nor the top of its range; the id is at the same time the
// band of request ids a RobotClient uses (its first eight bytes, big-endian),
// and a band that started inside the upper half of the sequence space would
// hardly ever have room for the requests of a session.
//
// When the system entropy source is unusable the identity falls back to a value
// mixed from the process id, the host clocks and a process-local counter: it is
// not random then, but it is still unique per process and never empty, the way
// the runtime's own identity falls back. That fallback is also why this library
// depends on POSIX (getpid) like the rest of this phase.
protocol::Id MakeSupervisorId();

// What a RobotClient needs from a transport. The client never sees a DDS type,
// a reader or a writer: every send goes through this interface, so a test can
// record the exact bytes a request would put on the wire. ClientTransport is
// the production implementation.
class ClientSink {
 public:
  virtual ~ClientSink() = default;

  // Hands one request to the transport. Both lanes together: the transport
  // routes a stop to its own lane. A refusal is counted by the transport and
  // returns false; the client stays pending and retries.
  virtual bool PublishRequest(const protocol::ExecutionRequest &request) = 0;

  // Hands one target frame to the newest-value command lane.
  virtual bool PublishCommand(const protocol::JointCommand &command) = 0;

  // Hands one heartbeat to the heartbeat lane.
  virtual bool PublishHeartbeat(
      const protocol::SupervisorHeartbeat &heartbeat) = 0;
};

// Outcome of the pending request, edge triggered: Poll reports Confirmed,
// Rejected, TimedOut or Unknown on the call that first observes it, then None
// until the next BeginRequest.
enum class RequestOutcome {
  None = 0,
  Pending,
  Confirmed,
  Rejected,
  TimedOut,
  Unknown,
};

// One Poll result: the outcome of the pending request plus the trusted state
// snapshot. A snapshot is trusted only while it belongs to the instance and
// session the client is bound to; losing the link, the instance or the session
// drops it, so a target generator can never build on a dead state.
struct ClientView {
  // Serial of the Poll call that produced this view; the first call reports 1
  // and every later call a strictly larger value.
  //
  // The outcome below is edge triggered - it is reported on the one call that
  // first observes it and then never again - while a state snapshot stays
  // meaningful for as long as it is fresh. A caller that shows one view to
  // several decisions (the supervision loop runs one Step per input event and
  // then one more Step on the same poll) therefore has to be able to tell the
  // two apart: only the first decision that sees a given serial may act on the
  // outcome of that poll, and the behaviour FSM consumes the serial for exactly
  // that.
  protocol::Sequence poll_seq{0};
  RequestOutcome outcome{RequestOutcome::None};
  // Reason of a Rejected outcome; None otherwise.
  protocol::RejectReason reject_reason{protocol::RejectReason::None};
  // Latest trusted state; unset while state_known is false.
  protocol::RobotState state{};
  bool state_known{false};
  // Receive age of the snapshot at most supervision.state_timeout_us.
  bool state_fresh{false};
  // AcquireSession confirmed and the session still live.
  bool session_owned{false};
  // The lease, the instance or the session was lost; set until a new session
  // is confirmed.
  bool session_lost{false};
  // Another runtime instance was observed while one was bound; set until a new
  // session is confirmed on the new instance.
  bool runtime_changed{false};
};

// The official supervisor client: the frozen request/answer and target rules of
// docs/runtime-protocol.md, without a single DDS type. It is DDS-free and links
// only the protocol and the configuration; its one platform dependency is the
// POSIX process id MakeSupervisorId() falls back to when entropy is
// unavailable.
//
// The client is a pure function of what it is told and of the times it is
// given: it reads no clock, allocates nothing, never blocks on the transport
// beyond a bounded Publish and never throws. Only the supervision thread may
// call the decision API (the Observe* entries and Poll are not thread safe
// against each other); the transport hands it what the transport thread
// received.
//
// One ordinary request is pending at a time, and a stop (Damping, ZeroTorque,
// Reset) may preempt it with a strictly larger request id. While a request is
// pending the identical payload is resent on the retry period and the request
// expires at its final deadline. A request is confirmed only by an Applied
// result that answers it and a state published at or after that result which
// agrees on instance, session, execution state, output action, epoch and
// producer; a state that contradicts the answer refuses the confirmation for
// good, so a late answer after a fault or a stop can never revive a behaviour.
class RobotClient {
 public:
  // `sink` and `config` must outlive the client. `supervisor_id` is fixed for
  // the life of the process; MakeSupervisorId() is the production source of it.
  // The all-zero id is refused as an identity: a client constructed with it can
  // acquire nothing and publishes nothing.
  RobotClient(ClientSink &sink, const configuration::ApplicationConfig &config,
              protocol::Id supervisor_id);

  RobotClient(const RobotClient &) = delete;
  RobotClient &operator=(const RobotClient &) = delete;

  // Feeds one received state with its host receive stamp. A state whose
  // protocol version is not the frozen one is ignored; so is a state that
  // cannot be the instance the client talks to, except that such a state
  // invalidates the instance it contradicts.
  void ObserveState(const protocol::RobotState &state, protocol::TimeUs rx_us);

  // Feeds one received result with its host receive stamp. A result that does
  // not answer the pending request is ignored; a matching Rejected result
  // resolves it at once, a matching Applied result only with a witness state.
  void ObserveResult(const protocol::ExecutionResult &result,
                     protocol::TimeUs rx_us);

  // Starts one request and sends its first copy. Refused when the client has no
  // identity, when an ordinary request is already pending, when the operation
  // is not part of the frozen set, when an AcquireSession is asked for with a
  // live session or without a matching instance observed, when any other
  // operation is asked for without a live session, when a Tracking request
  // carries no usable first frame, and when the request id would overflow.
  //
  // The routing identity of a Tracking request (protocol version, runtime id,
  // session id, request id, stamp) is completed in the client's own stored copy
  // of `first_command`, which is what every resend repeats; the caller's frame
  // is only read. SendCommand below is the one that mutates the caller's frame.
  bool BeginRequest(protocol::Operation operation, protocol::TimeUs now_us,
                    protocol::Id producer_id = protocol::kNoId,
                    const protocol::JointCommand *first_command = nullptr);

  // Starts a Reset request for the latched fault `fault_id`.
  bool BeginReset(protocol::Sequence fault_id, protocol::TimeUs now_us);

  // Advances the pending request and reports the current view. Resends the
  // identical payload on the retry period, expires the request at its final
  // deadline, and reports a link loss it can only notice while polling.
  ClientView Poll(protocol::TimeUs now_us);

  // Completes the routing identity of a generator-built frame (protocol
  // version, runtime id, session id, send stamp) and publishes it. It writes
  // those fields into the frame the caller passed in - unlike BeginRequest,
  // which only reads its first frame and completes the client's own copy - so
  // the caller's frame is the one that went on the wire. Refused without a live
  // session and when the frame's joint count or layout does not match the
  // configuration. The producer id, the epoch, the command sequence, the source
  // state sequence and the joint values stay the generator's.
  bool SendCommand(protocol::JointCommand *command, protocol::TimeUs now_us);

  // Publishes one heartbeat with the next strictly increasing sequence.
  // Refused without a live session.
  bool SendHeartbeat(protocol::TimeUs now_us);

  // The next authorization watermark this client may propose: strictly greater
  // than the watermark it has seen and than everything it proposed before, so a
  // value is never reused.
  protocol::Sequence next_epoch() const;

  // Session and instance the client is bound to; both the all-zero id while
  // none is known.
  const protocol::Id &session_id() const;
  const protocol::Id &runtime_id() const;

  // True while a confirmed session is live.
  bool session_owned() const;

  // The pending request, if any: its request id, and how often its payload has
  // left the client (zero while nothing is pending).
  bool request_pending() const;
  protocol::Sequence pending_request_id() const;
  std::size_t retry_count() const;

  // The authorization watermark the pending request proposes, and 0 while
  // nothing is pending or the pending request proposes none. Read-only, for a
  // caller that built the first frame of a tracking request and has to agree
  // with the epoch that request is asking for.
  protocol::Sequence pending_proposed_epoch() const;

  // Forgets the pending request and an outcome of it that no Poll has reported
  // yet: the exit path gives up on an answer instead of waiting for one.
  void AbandonRequest();

 private:
  // Projection around the write position: the oldest observation sits at
  // `history_next_ - history_count_`. At least the last eight states are kept
  // so a state that arrived before its result can still confirm it.
  static constexpr std::size_t kStateHistoryCapacity = 8;

  struct StateObservation {
    protocol::RobotState state{};
  };

  bool Begin(protocol::Operation operation, protocol::TimeUs now_us,
             protocol::Id producer_id,
             const protocol::JointCommand *first_command,
             protocol::Sequence fault_id);
  bool IdentityValid() const;
  bool LiveSession() const;
  bool SameInstance(const protocol::RobotState &state) const;
  bool StateFresh(protocol::TimeUs now_us) const;
  const StateObservation &HistoryAt(std::size_t offset) const;
  void RecordHistory(const protocol::RobotState &state);
  bool MatchesPending(const protocol::ExecutionResult &result,
                      protocol::TimeUs rx_us) const;
  void EvaluatePending();
  void Resolve(RequestOutcome outcome, protocol::RejectReason reason);
  void LoseSession();
  void DropSnapshot();
  void ResendPending(protocol::TimeUs now_us);

  ClientSink &sink_;
  const configuration::ApplicationConfig &config_;
  const protocol::Id supervisor_id_;

  // Instance the client is bound to. `instance_known_` is true only for an
  // observed state whose runtime id is not empty and whose config and layout
  // digests are the configuration's own.
  bool instance_known_{false};
  protocol::Id runtime_id_{};
  bool runtime_changed_{false};

  // Session state. The watermark is the highest epoch a trusted state or a
  // confirmed result carried; the proposal is the highest epoch this client
  // asked for, so no value is proposed twice.
  bool session_owned_{false};
  bool session_lost_{false};
  protocol::Id session_id_{};
  protocol::Sequence epoch_watermark_{0};
  protocol::Sequence proposed_epoch_{0};
  protocol::Sequence heartbeat_seq_{0};

  // Trusted snapshot and the arrival-ordered history behind it.
  std::array<StateObservation, kStateHistoryCapacity> history_{};
  std::size_t history_count_{0};
  std::size_t history_next_{0};
  protocol::RobotState latest_state_{};
  protocol::TimeUs latest_state_rx_us_{0};
  bool state_known_{false};

  // The one pending request, its original payload kept for bounded resends.
  bool pending_{false};
  protocol::ExecutionRequest pending_request_{};
  protocol::TimeUs pending_first_send_us_{0};
  protocol::TimeUs pending_last_send_us_{0};
  std::size_t pending_sends_{0};
  bool confirmation_refused_{false};
  protocol::ExecutionResult pending_result_{};
  bool pending_result_valid_{false};

  // Request ids start at the band of the supervisor identity and only ever
  // increase; the last sequence value is reserved, because an id nothing can
  // follow cannot be answered in order.
  protocol::Sequence next_request_id_{0};

  // Outcome to report exactly once by the next Poll.
  RequestOutcome report_outcome_{RequestOutcome::None};
  protocol::RejectReason report_reason_{protocol::RejectReason::None};

  // Serial of the Poll calls, so one view can be recognized as the same poll.
  protocol::Sequence poll_seq_{0};
};

// A stop is exactly the transport's stop lane (Damping, ZeroTorque, Reset), and
// it is what may preempt a pending request. The frozen definition lives in
// irmv::transport, which the DDS-free application library does not link, so
// this mirror is pinned against it by application_client_test.
bool IsStopOperation(protocol::Operation operation);

}  // namespace irmv::application
