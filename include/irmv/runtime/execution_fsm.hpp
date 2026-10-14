#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/command_gate.hpp"
#include "irmv/runtime/execution_context.hpp"
#include "irmv/runtime/safety.hpp"

namespace irmv::runtime {

// Guarded execution state machine: Boot, ZeroTorque, Damping, Tracking and
// Fault, with request idempotency and write-confirmed results.
//
// One Tick consumes one feedback snapshot, one bounded input batch and one
// backend write path. The FSM reads no clock, allocates nothing, never blocks
// and never throws: every timestamp and every packet arrives from the caller.
// It evaluates feedback safety before requests, processes stop requests before
// ordinary requests, and only confirms an applied request after the torque
// write implementing it was accepted. A refused write never leaves the machine
// in a torque-commanding state.
//
// The FSM stores a const reference to the immutable configuration snapshot; the
// referenced config must outlive the FSM. All other state is fixed storage.
//
// Why ProcessHeartbeats discarded one heartbeat. One counter per reason lets
// the runtime's logs tell a version-skewed or foreign peer apart from a plain
// replay of an already accepted sequence, which one aggregate count cannot.
enum class HeartbeatReject : std::size_t {
  ProtocolVersion = 0,
  RuntimeId,
  SessionId,
  SupervisorId,
  StaleSequence,
};
inline constexpr std::size_t kHeartbeatRejectCount = 5;

class ExecutionFsm {
 public:
  ExecutionFsm(const configuration::RuntimeConfig &config,
               protocol::Id runtime_id);

  ExecutionFsm(const ExecutionFsm &) = delete;
  ExecutionFsm &operator=(const ExecutionFsm &) = delete;

  // Runs one control tick. `feedback` must be the snapshot read for this tick;
  // the backend is used for Write only.
  TickResult Tick(protocol::TimeUs now_us,
                  const protocol::FeedbackSnapshot &feedback,
                  const TickInput &input, hardware::Backend &backend);

  protocol::ExecutionState execution_state() const { return state_; }

  protocol::OutputAction output_action() const { return output_action_; }

  protocol::Id runtime_id() const { return runtime_id_; }

  protocol::Id session_id() const { return session_id_; }

  protocol::Id producer_id() const { return producer_id_; }

  // Authorization watermark currently published; never reused, never wrapped.
  protocol::Sequence epoch() const { return epoch_; }

  // Zero while no fault is latched.
  protocol::Sequence fault_id() const { return fault_id_; }

  protocol::Sequence tick() const { return tick_; }

  // Read-only view of the accepted target the command gate holds: the frame the
  // Tracking output is recomputed from on every tick, and the transport receive
  // stamp of that frame. The test-only output trace reads them; nothing in the
  // control path changes behaviour because of them. No target is held outside
  // Tracking.
  bool has_target() const { return gate_.has_target(); }

  const protocol::JointCommand &current_target() const {
    return gate_.current_target();
  }

  protocol::TimeUs target_rx_us() const {
    return gate_.last_valid_command_rx_us();
  }

  // ---- heartbeat chain (supervision lease) -------------------------------
  //
  // The FSM end of the chain: what one Tick's ProcessHeartbeats did with the
  // batch the transport read. The sequence and the monotonic tick stamp of the
  // last accepted heartbeat tie this end to the transport's own counters. The
  // control path reads none of them; the runtime prints them on its status and
  // fault lines.

  // Heartbeats that passed every identity check and carried a strictly newer
  // sequence, thereby refreshing the lease.
  std::uint64_t heartbeats_accepted_count() const {
    return heartbeats_accepted_;
  }

  // Heartbeats the loop discarded, over every reason below.
  std::uint64_t heartbeats_rejected_count() const;

  // Discarded heartbeats of one reason.
  std::uint64_t heartbeat_rejection_count(HeartbeatReject reason) const;

  // Sequence of the newest accepted heartbeat, 0 while none was accepted.
  protocol::Sequence last_accepted_heartbeat_seq() const {
    return last_accepted_heartbeat_seq_;
  }

  // Tick time when the lease was last refreshed by an accepted heartbeat, 0
  // while none was accepted. Unlike the internal lease stamp this is not
  // cleared when the session expires, so the SupervisorTimeout line can report
  // the silence that ended the lease.
  protocol::TimeUs last_accepted_heartbeat_us() const {
    return last_accepted_heartbeat_us_;
  }

 private:
  // Per-tick data shared with the per-state implementations in
  // src/runtime/states/*.cpp.
  struct TickScope {
    protocol::TimeUs now_us{0};
    const protocol::FeedbackSnapshot *feedback{nullptr};
    const SafetyReport *report{nullptr};
    TickResult *out{nullptr};
  };

  // A result produced by this tick, held back until the write is confirmed.
  struct PendingResult {
    protocol::ExecutionResult result;
    bool applied{false};
    // The answer is the stored answer of the session's request record, so the
    // confirmation step must write the confirmed status back into it.
    bool recorded{false};
  };

  // Last handled request of the current session: identity and payload of the
  // request together with the answer that was given.
  struct RequestRecord {
    bool valid{false};
    protocol::Operation operation{protocol::Operation::AcquireSession};
    protocol::Sequence request_id{0};
    protocol::Id supervisor_id{};
    protocol::Sequence expected_epoch{0};
    protocol::Sequence proposed_epoch{0};
    protocol::Id producer_id{};
    protocol::Sequence fault_id{0};
    protocol::JointCommand first_command{};
    bool has_command{false};
    protocol::ExecutionResult result{};
  };

  // Deduplication metadata of the current session bootstrap, kept while the
  // lease is healthy so a lost handshake reply stays retryable.
  struct AcquireRecord {
    bool valid{false};
    protocol::Id supervisor_id{};
    protocol::Sequence request_id{0};
    // Tick that answered the bootstrap request, so a duplicate inside the same
    // batch is answered once.
    protocol::Sequence answered_tick{0};
    protocol::ExecutionResult result{};
  };

  // ---- tick phases (execution_fsm.cpp) -----------------------------------

  void ProcessCommands(TickScope &scope, const TickInput &input);

  // Stop requests (Damping, ZeroTorque, Reset) first, then normal requests
  // (AcquireSession, Tracking), each in receive order.
  void ProcessRequests(TickScope &scope, const TickInput &input);

  void ProcessHeartbeats(TickScope &scope, const TickInput &input);

  // Counts one discarded heartbeat under its reason.
  void RejectHeartbeat(HeartbeatReject reason);

  // Falls back to Damping when the accepted target expired while tracking.
  void ExpireTracking(TickScope &scope);

  void PublishState(TickScope &scope);

  // ---- request handling --------------------------------------------------

  void HandleRequest(TickScope &scope,
                     const Received<protocol::ExecutionRequest> &packet);
  void HandleAcquireSession(TickScope &scope,
                            const Received<protocol::ExecutionRequest> &packet);
  void HandleStop(TickScope &scope,
                  const Received<protocol::ExecutionRequest> &packet);
  void HandleReset(TickScope &scope,
                   const Received<protocol::ExecutionRequest> &packet);
  void HandleTracking(TickScope &scope,
                      const Received<protocol::ExecutionRequest> &packet);

  // The runtime's own stop: no supervisor identity, no request record, no
  // answer, and never a way out of a latched fault. It reaches zero torque from
  // every state that allows a ZeroTorque stop.
  void HandleLocalStop(TickScope &scope);

  protocol::ExecutionResult MakeResult(
      const protocol::ExecutionRequest &request, protocol::TimeUs now_us) const;

  void QueueApplied(TickScope &scope,
                    const protocol::ExecutionRequest &request);

  // Answers a request that passed identity and idempotency checks. The answer
  // is remembered as the last handled request of the session.
  void QueueRejected(TickScope &scope,
                     const protocol::ExecutionRequest &request,
                     protocol::RejectReason reason);

  // Answers a request that failed identity, version or idempotency checks:
  // those never overwrite the session's request record.
  void QueueUnrecordedRejection(TickScope &scope,
                                const protocol::ExecutionRequest &request,
                                protocol::RejectReason reason);

  // Starts an empty request record for a newly issued session, dropping both
  // the confirmed record and any answer staged by the previous session.
  void StartRequestRecord();

  void StageRequestAnswer(const protocol::ExecutionRequest &request,
                          const protocol::ExecutionResult &result);

  bool PayloadMatches(const protocol::ExecutionRequest &request,
                      const RequestRecord &record) const;

  // Releases the answers of this tick. An answer is Applied only when the write
  // implementing the tick's output was accepted and the committed state, output
  // action, epoch and producer still match the answer; otherwise it is
  // downgraded, and only a confirmed answer becomes the session's request
  // record.
  void CommitPendingResults(TickScope &scope, bool write_accepted);

  bool ImplementsAnswer(const protocol::ExecutionResult &answer) const;

  // ---- events and results ------------------------------------------------

  void EmitResult(TickScope &scope, const protocol::ExecutionResult &result);

  void EmitEvent(
      TickScope &scope, EventCode code, protocol::ExecutionState from,
      protocol::ExecutionState to,
      protocol::RejectReason reject_reason = protocol::RejectReason::None,
      protocol::FaultReason fault_reason = protocol::FaultReason::None,
      protocol::StopReason stop_reason = protocol::StopReason::None,
      protocol::Sequence request_id = 0);

  // ---- state plumbing ----------------------------------------------------

  void EnterState(protocol::ExecutionState state, TickScope &scope);
  void LeaveState(protocol::ExecutionState state, TickScope &scope);
  void SetState(protocol::ExecutionState next, TickScope &scope,
                EventCode code);
  hardware::TorqueOutput ComputeStateOutput(TickScope &scope);
  hardware::TorqueOutput ZeroTorqueOutput() const;
  protocol::RejectReason CheckStateAllows(protocol::ExecutionState state,
                                          protocol::Operation operation) const;

  // ---- authorization, session and supervision ----------------------------

  // Drops the gate context, the accepted target and the producer identity
  // without touching the authorization watermark.
  void ClearMotion(TickScope &scope);

  // Clears the motion authorization and advances the authorization watermark so
  // that the revoked epoch can never be used again. Returns false and latches
  // ProtocolExhausted when the watermark cannot advance without wrapping.
  bool RevokeMotion(TickScope &scope);

  // Advances the authorization watermark by one, latches ProtocolExhausted and
  // returns false when it cannot advance without wrapping.
  bool AdvanceWatermark(TickScope &scope);

  protocol::Id GenerateSessionId();

  bool LeaseValid(protocol::TimeUs now_us) const;

  void IssueSession(TickScope &scope,
                    const protocol::ExecutionRequest &request);

  // Ends the supervision lease: clears the session, revokes the motion epoch
  // and stops Tracking.
  void ExpireSession(TickScope &scope);

  // ---- continuity --------------------------------------------------------

  bool CheckContinuity(TickScope &scope,
                       const protocol::JointCommand &command) const;

  double ExpectedCurrentTorque(const protocol::FeedbackSnapshot &feedback,
                               std::size_t index) const;

  // ---- per-state implementation, one file per state ----------------------

  // boot.cpp
  void EnterBoot(TickScope &scope);
  void LeaveBoot(TickScope &scope);
  bool BootReady(TickScope &scope) const;
  hardware::TorqueOutput StepBoot(TickScope &scope);
  protocol::RejectReason BootAllows(protocol::Operation operation) const;

  // zero_torque.cpp
  void EnterZeroTorque(TickScope &scope);
  void LeaveZeroTorque(TickScope &scope);
  hardware::TorqueOutput StepZeroTorque(TickScope &scope);
  protocol::RejectReason ZeroTorqueAllows(protocol::Operation operation) const;

  // damping.cpp
  void EnterDamping(TickScope &scope);
  void LeaveDamping(TickScope &scope);
  hardware::TorqueOutput StepDamping(TickScope &scope);
  protocol::RejectReason DampingAllows(protocol::Operation operation) const;

  // tracking.cpp
  void EnterTracking(TickScope &scope);
  void LeaveTracking(TickScope &scope);
  hardware::TorqueOutput StepTracking(TickScope &scope);
  protocol::RejectReason TrackingAllows(protocol::Operation operation) const;

  // fault.cpp
  void EnterFault(TickScope &scope);
  void LeaveFault(TickScope &scope);
  hardware::TorqueOutput StepFault(TickScope &scope);
  protocol::RejectReason FaultAllows(protocol::Operation operation) const;

  // Latches a fault from a non-faulting state. An already latched fault is
  // never overwritten: the attempt is still reported as an event.
  void LatchFault(TickScope &scope, protocol::FaultReason reason,
                  EventCode code, bool damping_eligible,
                  const protocol::FaultReport &report);

  // Re-evaluates fault output eligibility and the recovery window.
  void UpdateFaultEligibility(TickScope &scope);

  // One tick counts as healthy for the recovery window when the measured state
  // is fully usable and the torque write path accepted the last attempt.
  bool RecoveryTickHealthy(TickScope &scope) const;

  // Handles a refused torque write: latches WriteFailed, or permanently
  // degrades the output of an already latched fault to zero torque.
  void DegradeForWriteFailure(TickScope &scope, std::uint32_t error_code);

  // Highest damping eligibility a latched fault may hold: the measured state
  // must still be fully usable. This is deliberately independent of the fault
  // flag of a safety report, because it also decides whether a fault latched
  // while the measured state was healthy keeps commanding damping torque.
  static bool DampingEligible(const SafetyReport &report);

  static protocol::FaultReport MakeFaultReport(protocol::FaultReason reason,
                                               protocol::TimeUs now_us,
                                               std::uint32_t error_code,
                                               const char *message);

  const configuration::RuntimeConfig &config_;
  protocol::Id runtime_id_;
  CommandGate gate_;

  protocol::Sequence tick_{0};
  protocol::Sequence state_seq_{0};
  bool entered_{false};
  // A stop or Reset accepted in this tick. A motion request may not undo it
  // while the same tick is still being processed, so the priority the
  // processing order gives a stop survives the ordinary request lane that
  // follows it. The flag belongs to one tick: the next tick authorizes the same
  // frame again.
  bool stop_applied_this_tick_{false};

  protocol::ExecutionState state_{protocol::ExecutionState::Boot};
  protocol::OutputAction output_action_{protocol::OutputAction::ZeroTorque};

  bool boot_started_{false};
  protocol::TimeUs boot_start_us_{0};
  bool boot_window_started_{false};
  protocol::TimeUs boot_window_start_us_{0};
  // Which required inputs have delivered a frame at least once. The boot grace
  // window only excuses an input that was never observed, so a joint or the
  // required IMU that goes missing after its first frame faults immediately.
  FeedbackObservation observed_{};

  protocol::Id session_id_{};
  protocol::Id session_supervisor_id_{};
  protocol::TimeUs last_heartbeat_rx_us_{0};
  protocol::Sequence last_heartbeat_seq_{0};
  bool heartbeat_seen_{false};

  // Heartbeat-chain evidence of ProcessHeartbeats: cumulative counts, the
  // newest accepted sequence and the tick time of that acceptance. The last two
  // are deliberately not cleared by ExpireSession, so the SupervisorTimeout
  // line can name the sequence and the silence that ended the lease.
  std::uint64_t heartbeats_accepted_{0};
  std::array<std::uint64_t, kHeartbeatRejectCount> heartbeat_rejections_{};
  protocol::Sequence last_accepted_heartbeat_seq_{0};
  protocol::TimeUs last_accepted_heartbeat_us_{0};

  protocol::Id producer_id_{};
  protocol::Sequence epoch_{0};
  protocol::Sequence last_applied_command_seq_{0};

  protocol::Sequence fault_counter_{0};
  protocol::Sequence fault_id_{0};
  protocol::FaultReason fault_reason_{protocol::FaultReason::None};
  protocol::FaultReport fault_report_{};
  bool fault_damping_eligible_{false};
  bool fault_output_degraded_{false};
  bool recovery_window_started_{false};
  protocol::TimeUs recovery_window_start_us_{0};
  bool recovery_ready_{false};

  protocol::StopReason stop_reason_{protocol::StopReason::None};

  // Accepted primary write of the previous tick; a refused write is what
  // permits recovery readiness and what degrades a latched fault.
  bool last_write_accepted_{true};

  // Answers of the previous ticks that survived their confirmation, and the
  // answer staged by the current tick that becomes the record only once the
  // write commits. A staged answer is never replayable.
  RequestRecord request_record_{};
  RequestRecord staged_record_{};
  AcquireRecord acquire_record_{};

  std::array<PendingResult, kRequestBatchCapacity + kStopRequestBatchCapacity>
      pending_{};
  std::size_t pending_count_{0};

  std::uint64_t dropped_results_{0};
  std::uint64_t dropped_events_{0};
};

}  // namespace irmv::runtime
