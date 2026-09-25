#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <utility>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/fake_backend.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/execution_fsm.hpp"

namespace irmv::test {

using configuration::RuntimeConfig;
using protocol::ExecutionRequest;
using protocol::ExecutionResult;
using protocol::ExecutionState;
using protocol::Id;
using protocol::JointCommand;
using protocol::Operation;
using protocol::Sequence;
using protocol::SupervisorHeartbeat;
using protocol::TimeUs;
using runtime::ExecutionFsm;
using runtime::kCommandBatchCapacity;
using runtime::kHeartbeatBatchCapacity;
using runtime::kRequestBatchCapacity;
using runtime::kStopRequestBatchCapacity;
using runtime::Received;
using runtime::TickInput;
using runtime::TickResult;

inline constexpr const char *kFixtureConfigPath = "config/sdk.json";

// Sentinel for "let the fixture derive this value": the next request id, the
// next epoch, the next command sequence or the automatically increasing
// heartbeat sequence.
inline constexpr Sequence kAutoSequence = 0;

// The fixture drives its own simulated clock; the absolute value only has to
// stay far away from zero so that relative deadlines are visible.
inline constexpr TimeUs kFixtureStartUs = 1'000'000;

inline Id MakeId(std::uint8_t seed) {
  Id id{};
  id.fill(seed);
  return id;
}

inline Id FixtureRuntimeId() { return MakeId(0x11); }
inline Id FixtureSupervisorId() { return MakeId(0x22); }
inline Id FixtureOtherSupervisorId() { return MakeId(0x23); }
inline Id FixtureProducerId() { return MakeId(0x33); }
inline Id FixtureSecondProducerId() { return MakeId(0x44); }
inline Id FixtureForeignId() { return MakeId(0x99); }

// Feedback template: two joints at the configured control frequency with a
// static position and a small non-zero velocity, so damping and tracking
// outputs are distinguishable from zero torque. The receive stamps are
// refreshed from the simulated clock on every tick, which keeps the snapshot
// coherent with the tick that consumes it.
inline constexpr double kFixtureQ[2] = {0.5, -0.25};
inline constexpr double kFixtureDq[2] = {0.2, -0.1};
inline constexpr double kFixtureFrequencyHz = 500.0;

// The 32 bit error code from the plan: a driver error must survive as a full
// unsigned value.
inline constexpr std::uint32_t kDriverErrorCode = 0x80000000u;

// Slices one request result out of a tick. Deriving from the protocol payload
// keeps the plan example (`f.Request(...).rejected()`) readable while callers
// still receive a protocol::ExecutionResult.
struct FixtureResult : public ExecutionResult {
  FixtureResult() = default;

  explicit FixtureResult(const ExecutionResult &result)
      : ExecutionResult(result) {}

  bool applied() const { return status == protocol::ResultStatus::Applied; }

  bool rejected() const { return status == protocol::ResultStatus::Rejected; }
};

// Deterministic driver for ExecutionFsm: it owns the configuration, the fake
// backend, the simulated clock and the input queues, and it only ever drives
// the FSM through the public Tick interface. Nothing here reads FSM internals
// or the wall clock, sleeps or uses threads.
class RuntimeFixture {
 public:
  // Loads the repository fixture configuration; tests that need a test-only
  // configuration change fields on the returned value before construction.
  static RuntimeConfig LoadFixtureConfig(
      const std::filesystem::path &config_path = kFixtureConfigPath) {
    return configuration::LoadConfig(config_path);
  }

  explicit RuntimeFixture(configuration::RuntimeConfig configuration)
      : config(std::move(configuration)),
        backend(this->config),
        fsm_(this->config, FixtureRuntimeId()) {
    backend.Initialize();
    ResetFeedback();
  }

  explicit RuntimeFixture(
      const std::filesystem::path &config_path = kFixtureConfigPath)
      : RuntimeFixture(configuration::LoadConfig(config_path)) {}

  // Public state required by the plan: the configuration the FSM references
  // and the backend whose write acceptance the tests toggle.
  configuration::RuntimeConfig config;
  hardware::FakeBackend backend;

  // ---- clock and tick driving -------------------------------------------

  TimeUs now_us() const { return now_us_; }

  TimeUs ControlPeriodUs() const { return config.control_period_us; }

  const protocol::RobotState &state() const { return last_tick_.state; }

  const FixtureResult &last_result() const { return last_result_; }

  const TickResult &last_tick() const { return last_tick_; }

  // Runs exactly one Tick at the current simulated time with coherent feedback
  // and the queued inputs.
  const TickResult &Step() {
    TickInput input{};
    input.local_stop_requested = std::exchange(local_stop_pending_, false);
    AppendQueuedHeartbeats(input);
    AppendQueuedRequests(input);
    AppendAutoCommand(input);
    AppendQueuedCommands(input);
    UpdateFeedbackStamps();
    backend.Inject(feedback_);
    const protocol::FeedbackSnapshot snapshot = backend.Read(now_us_);
    last_tick_ = fsm_.Tick(now_us_, snapshot, input, backend);
    return last_tick_;
  }

  // Asks the state machine for the runtime's own ZeroTorque stop in the next
  // tick: the trusted local stop of the leaving process, not a supervisor
  // request. Every received request of that tick is still answered.
  void RequestLocalStop() { local_stop_pending_ = true; }

  // Steps Tick in control_period_us increments until the requested duration has
  // elapsed. The simulated clock never moves backwards.
  void Advance(TimeUs duration_us) {
    TimeUs remaining = duration_us;
    const TimeUs period =
        config.control_period_us == 0 ? 1 : config.control_period_us;
    while (remaining > 0) {
      const TimeUs step = remaining < period ? remaining : period;
      now_us_ += step;
      Step();
      remaining -= step;
    }
  }

  // Runs healthy ticks until the FSM leaves Boot. The bound keeps a broken
  // implementation from hanging the test process.
  std::size_t BootToZeroTorque(std::size_t max_ticks = 4096) {
    for (std::size_t tick = 0; tick < max_ticks; ++tick) {
      if (state().execution_state == ExecutionState::ZeroTorque) {
        return tick;
      }
      Advance(config.control_period_us);
    }
    return max_ticks;
  }

  // ---- input queues ------------------------------------------------------

  // Queues one execution request for the next tick. The stop queue is the
  // dedicated lane for Damping and ZeroTorque operations.
  void QueueRequest(const ExecutionRequest &request, TimeUs rx_us = 0,
                    bool stop_queue = false) {
    const TimeUs stamp = rx_us == 0 ? now_us_ : rx_us;
    if (stop_queue) {
      if (pending_stop_request_count_ < kStopRequestBatchCapacity) {
        pending_stop_requests_[pending_stop_request_count_++] = {request,
                                                                 stamp};
      }
      return;
    }
    if (pending_request_count_ < kRequestBatchCapacity) {
      pending_requests_[pending_request_count_++] = {request, stamp};
    }
  }

  void QueueCommand(const JointCommand &command, TimeUs rx_us = 0) {
    if (pending_command_count_ < kCommandBatchCapacity) {
      pending_commands_[pending_command_count_++] = {
          command, rx_us == 0 ? now_us_ : rx_us};
    }
  }

  // Queues one supervised heartbeat for the next tick. kAutoSequence uses the
  // fixture's running sequence, any other value is sent verbatim (which lets a
  // test repeat or reorder heartbeats). The identity is taken from the latest
  // published state, so a session acquired during the next tick is not yet
  // visible here.
  void QueueHeartbeat(Sequence heartbeat_seq = kAutoSequence,
                      TimeUs rx_us = 0) {
    if (pending_heartbeat_count_ >= kHeartbeatBatchCapacity) {
      return;
    }
    SupervisorHeartbeat heartbeat;
    heartbeat.protocol_version = protocol::kProtocolVersion;
    heartbeat.runtime_id = FixtureRuntimeId();
    heartbeat.session_id = state().session_id;
    heartbeat.supervisor_id = FixtureSupervisorId();
    heartbeat.heartbeat_seq =
        heartbeat_seq == kAutoSequence ? ++heartbeat_seq_ : heartbeat_seq;
    heartbeat.stamp_us = rx_us == 0 ? now_us_ : rx_us;
    pending_heartbeats_[pending_heartbeat_count_++] = {
        heartbeat, rx_us == 0 ? now_us_ : rx_us};
  }

  // Queues one raw heartbeat for foreign identity and sequence tests.
  void QueueRawHeartbeat(const SupervisorHeartbeat &heartbeat,
                         TimeUs rx_us = 0) {
    if (pending_heartbeat_count_ < kHeartbeatBatchCapacity) {
      pending_heartbeats_[pending_heartbeat_count_++] = {
          heartbeat, rx_us == 0 ? now_us_ : rx_us};
    }
  }

  void SetHeartbeatEnabled(bool enabled) { heartbeat_enabled_ = enabled; }

  // Forces the sequence of the automatic heartbeat. A fixed sequence sends
  // duplicates; kAutoSequence resumes a strictly increasing sequence.
  void SetHeartbeatSequence(Sequence heartbeat_seq) {
    heartbeat_sequence_fixed_ = heartbeat_seq != kAutoSequence;
    heartbeat_seq_ = heartbeat_seq;
  }

  // Controls the automatic command feed: while disabled the accepted target
  // receives nothing and its deadlines expire.
  void SetCommandEnabled(bool enabled) { command_enabled_ = enabled; }

  // Number of published states the automatic command refers to as its source
  // state. Larger values age the source state without aging the packet.
  void SetSourceStateLagTicks(Sequence ticks) { source_lag_ticks_ = ticks; }

  // ---- request builders --------------------------------------------------

  // Builds a full first frame for a tracking authorization: it sits on the
  // current measured state, has finite gains and is therefore acceptable for
  // the entry continuity guards unless the caller changes it.
  JointCommand MakeFirstFrame(Id producer_id, Sequence epoch = kAutoSequence,
                              Sequence command_seq = kAutoSequence,
                              Sequence source_state_seq = kAutoSequence) const {
    JointCommand command;
    command.protocol_version = protocol::kProtocolVersion;
    command.runtime_id = FixtureRuntimeId();
    command.session_id = state().session_id;
    command.producer_id = producer_id;
    command.epoch = epoch == kAutoSequence ? state().epoch + 1 : epoch;
    command.command_seq =
        command_seq == kAutoSequence ? next_command_seq_ + 1 : command_seq;
    command.source_state_seq = source_state_seq == kAutoSequence
                                   ? LatestSourceStateSeq()
                                   : source_state_seq;
    command.stamp_us = now_us_;
    static_cast<void>(command.layout_id.Assign(config.layout_id));
    command.joints.count = static_cast<std::uint32_t>(config.joint_count());
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      command.joints.q[index] = feedback_.joints[index].q;
      command.joints.dq[index] = 0.0;
      command.joints.tau_ff[index] = 0.0;
      command.joints.kp[index] = 1.0;
      command.joints.kd[index] = 0.5;
    }
    return command;
  }

  // Builds a complete request for the current context. Zero epochs mean "the
  // current watermark" and "the next epoch"; request_id 0 means "the next
  // request id".
  ExecutionRequest MakeRequest(Operation operation,
                               Sequence request_id = kAutoSequence,
                               Id producer_id = FixtureProducerId(),
                               Id supervisor_id = FixtureSupervisorId(),
                               Sequence expected_epoch = kAutoSequence,
                               Sequence proposed_epoch = kAutoSequence) {
    ExecutionRequest request;
    request.protocol_version = protocol::kProtocolVersion;
    request.supervisor_id = supervisor_id;
    request.request_id =
        request_id == kAutoSequence ? next_request_id_ + 1 : request_id;
    request.session_id = operation == Operation::AcquireSession
                             ? protocol::kNoId
                             : state().session_id;
    request.runtime_id = FixtureRuntimeId();
    request.expected_epoch =
        expected_epoch == kAutoSequence ? state().epoch : expected_epoch;
    request.proposed_epoch = proposed_epoch == kAutoSequence
                                 ? request.expected_epoch + 1
                                 : proposed_epoch;
    request.producer_id = producer_id;
    request.fault_id = state().fault_id;
    request.operation = operation;
    if (operation == Operation::Tracking) {
      request.first_command =
          MakeFirstFrame(producer_id, request.proposed_epoch);
    }
    next_request_id_ = request.request_id;
    if (operation == Operation::Tracking) {
      last_frame_ = request.first_command;
      has_frame_ = true;
      next_command_seq_ = request.first_command.command_seq;
    }
    return request;
  }

  // ---- single tick conveniences -----------------------------------------

  FixtureResult Send(const ExecutionRequest &request, bool stop_queue = false) {
    QueueRequest(request, 0, stop_queue);
    Step();
    last_result_ = ResultFor(last_tick_, request.request_id);
    return last_result_;
  }

  FixtureResult Acquire(Sequence request_id = kAutoSequence,
                        Id supervisor_id = FixtureSupervisorId()) {
    return Send(MakeRequest(Operation::AcquireSession, request_id,
                            FixtureProducerId(), supervisor_id));
  }

  FixtureResult Request(Operation operation,
                        Sequence request_id = kAutoSequence) {
    return Send(MakeRequest(operation, request_id),
                !IsNormalOperation(operation));
  }

  FixtureResult StopRequest(Operation operation,
                            Sequence request_id = kAutoSequence) {
    return Send(MakeRequest(operation, request_id), true);
  }

  FixtureResult StartTracking(Sequence request_id = kAutoSequence,
                              Id producer_id = FixtureProducerId()) {
    return Send(MakeRequest(Operation::Tracking, request_id, producer_id));
  }

  FixtureResult ResetRequest(Sequence request_id = kAutoSequence,
                             Sequence fault_id = kAutoSequence) {
    ExecutionRequest request = MakeRequest(Operation::Reset, request_id);
    if (fault_id != kAutoSequence) {
      request.fault_id = fault_id;
    }
    return Send(request, true);
  }

  // AcquireSession and Tracking travel in the normal request lane; both stop
  // operations and Reset travel in the dedicated stop lane.
  static bool IsNormalOperation(Operation operation) {
    return operation == Operation::AcquireSession ||
           operation == Operation::Tracking;
  }

  // Slices the result answering one request out of a tick. The returned value
  // keeps request_id 0 when the tick carried no such result.
  static FixtureResult ResultFor(const TickResult &tick, Sequence request_id) {
    for (std::uint32_t index = 0; index < tick.result_count; ++index) {
      if (tick.results[index].request_id == request_id) {
        return FixtureResult(tick.results[index]);
      }
    }
    return FixtureResult{};
  }

  // ---- feedback control --------------------------------------------------

  protocol::FeedbackSnapshot &mutable_feedback() { return feedback_; }

  void ResetFeedback() {
    feedback_ = protocol::FeedbackSnapshot{};
    feedback_.snapshot_seq = 0;
    for (std::size_t index = 0;
         index < config.joint_count() && index < protocol::kMaxJoints;
         ++index) {
      protocol::JointFeedback &joint = feedback_.joints[index];
      joint.valid = true;
      joint.connected = true;
      joint.q = index < 2 ? kFixtureQ[index] : 0.0;
      joint.dq = index < 2 ? kFixtureDq[index] : 0.0;
      joint.tau_est = 0.0;
      joint.freq_hz = kFixtureFrequencyHz;
      joint.error_code = 0;
    }
    feedback_.imu.valid = true;
    UpdateFeedbackStamps();
  }

  void SetDriverError(std::uint32_t error_code = kDriverErrorCode) {
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      feedback_.joints[index].error_code = error_code;
    }
  }

  void ClearDriverError() { SetDriverError(0); }

  void SetJointVelocity(double velocity) {
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      feedback_.joints[index].dq = velocity;
    }
  }

  void SetJointFrequency(double freq_hz) {
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      feedback_.joints[index].freq_hz = freq_hz;
    }
  }

  void SetJointValid(bool valid) {
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      feedback_.joints[index].valid = valid;
    }
  }

  // While the IMU is not fresh the fixture leaves its receive stamp where it
  // was, which makes the IMU sample stale without touching the joints.
  void SetImuFresh(bool fresh) { imu_fresh_ = fresh; }

  // Reports the snapshot a backend that has never delivered a frame reports:
  // every sample at its all-default value, unobserved and unconnected, so the
  // tick that consumes it sees no observation at all. The stamps of an absent
  // snapshot are deliberately not refreshed. Clearing the flag is the first
  // frame arriving: the healthy template and its stamps come back with it.
  void SetFeedbackAbsent(bool absent) {
    feedback_absent_ = absent;
    if (!absent) {
      ResetFeedback();
    }
  }

  // Overrides the joint count of the injected snapshot, which lets a test feed
  // an incomplete measured state. Zero restores the configured joint count.
  void SetFeedbackCount(std::uint32_t count) {
    feedback_count_override_ = count;
  }

  // Ages every measured receive stamp by the requested amount without moving
  // the simulated clock, which makes the snapshot stale.
  void AgeFeedback(TimeUs age_us) {
    const TimeUs base = now_us_ > age_us ? now_us_ - age_us : 0;
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      feedback_.joints[index].rx_stamp_us = base;
    }
    feedback_.imu.rx_stamp_us = base;
  }

  const JointCommand &last_frame() const { return last_frame_; }

  bool has_frame() const { return has_frame_; }

 private:
  Sequence LatestSourceStateSeq() const {
    const Sequence newest = last_tick_.state.state_seq;
    if (source_lag_ticks_ == 0) {
      return newest + 1;
    }
    return newest > source_lag_ticks_ ? newest - source_lag_ticks_ : 0;
  }

  void UpdateFeedbackStamps() {
    feedback_.count = feedback_count_override_ != 0
                          ? feedback_count_override_
                          : static_cast<std::uint32_t>(config.joint_count());
    feedback_.stamp_us = now_us_;
    feedback_.snapshot_seq = ++feedback_seq_;
    if (feedback_absent_) {
      for (std::size_t index = 0;
           index < config.joint_count() && index < protocol::kMaxJoints;
           ++index) {
        feedback_.joints[index] = protocol::JointFeedback{};
      }
      feedback_.imu = protocol::ImuFeedback{};
      return;
    }
    for (std::size_t index = 0;
         index < config.joint_count() && index < protocol::kMaxJoints;
         ++index) {
      feedback_.joints[index].rx_stamp_us = now_us_;
    }
    feedback_.imu.rx_stamp_us =
        imu_fresh_ ? now_us_ : feedback_.imu.rx_stamp_us;
  }

  void AppendQueuedHeartbeats(TickInput &input) {
    if (heartbeat_enabled_) {
      SupervisorHeartbeat heartbeat;
      heartbeat.protocol_version = protocol::kProtocolVersion;
      heartbeat.runtime_id = FixtureRuntimeId();
      heartbeat.session_id = state().session_id;
      heartbeat.supervisor_id = FixtureSupervisorId();
      heartbeat.heartbeat_seq =
          heartbeat_sequence_fixed_ ? heartbeat_seq_ : ++heartbeat_seq_;
      heartbeat.stamp_us = now_us_;
      if (input.heartbeat_count < kHeartbeatBatchCapacity) {
        input.heartbeats[input.heartbeat_count++] = {heartbeat, now_us_};
      }
    }
    for (std::size_t index = 0; index < pending_heartbeat_count_ &&
                                input.heartbeat_count < kHeartbeatBatchCapacity;
         ++index) {
      input.heartbeats[input.heartbeat_count++] = pending_heartbeats_[index];
    }
    pending_heartbeat_count_ = 0;
  }

  void AppendQueuedRequests(TickInput &input) {
    for (std::size_t index = 0;
         index < pending_request_count_ && index < kRequestBatchCapacity;
         ++index) {
      input.requests[index] = pending_requests_[index];
    }
    input.request_count = pending_request_count_ < kRequestBatchCapacity
                              ? pending_request_count_
                              : kRequestBatchCapacity;
    pending_request_count_ = 0;

    for (std::size_t index = 0; index < pending_stop_request_count_ &&
                                index < kStopRequestBatchCapacity;
         ++index) {
      input.stop_requests[index] = pending_stop_requests_[index];
    }
    input.stop_request_count =
        pending_stop_request_count_ < kStopRequestBatchCapacity
            ? pending_stop_request_count_
            : kStopRequestBatchCapacity;
    pending_stop_request_count_ = 0;
  }

  void AppendQueuedCommands(TickInput &input) {
    for (std::size_t index = 0; index < pending_command_count_ &&
                                input.command_count < kCommandBatchCapacity;
         ++index) {
      input.commands[input.command_count++] = pending_commands_[index];
    }
    pending_command_count_ = 0;
  }

  void AppendAutoCommand(TickInput &input) {
    if (!command_enabled_ || !has_frame_ ||
        state().producer_id == protocol::kNoId) {
      return;
    }
    JointCommand command = last_frame_;
    command.command_seq = ++next_command_seq_;
    command.source_state_seq = LatestSourceStateSeq();
    command.runtime_id = FixtureRuntimeId();
    command.session_id = state().session_id;
    command.producer_id = state().producer_id;
    command.epoch = state().epoch;
    command.stamp_us = now_us_;
    if (input.command_count < kCommandBatchCapacity) {
      input.commands[input.command_count++] = {command, now_us_};
    }
  }

  protocol::FeedbackSnapshot feedback_{};
  TickResult last_tick_{};
  FixtureResult last_result_{};
  JointCommand last_frame_{};
  bool has_frame_{false};

  TimeUs now_us_{kFixtureStartUs};
  Sequence feedback_seq_{0};
  Sequence next_request_id_{0};
  Sequence next_command_seq_{0};
  Sequence heartbeat_seq_{0};
  Sequence source_lag_ticks_{1};
  bool heartbeat_enabled_{true};
  bool heartbeat_sequence_fixed_{false};
  bool command_enabled_{true};
  bool imu_fresh_{true};
  bool feedback_absent_{false};
  bool local_stop_pending_{false};
  std::uint32_t feedback_count_override_{0};

  std::array<Received<ExecutionRequest>, kRequestBatchCapacity>
      pending_requests_{};
  std::size_t pending_request_count_{0};
  std::array<Received<ExecutionRequest>, kStopRequestBatchCapacity>
      pending_stop_requests_{};
  std::size_t pending_stop_request_count_{0};
  std::array<Received<JointCommand>, kCommandBatchCapacity> pending_commands_{};
  std::size_t pending_command_count_{0};
  std::array<Received<SupervisorHeartbeat>, kHeartbeatBatchCapacity>
      pending_heartbeats_{};
  std::size_t pending_heartbeat_count_{0};

  ExecutionFsm fsm_;
};

}  // namespace irmv::test
