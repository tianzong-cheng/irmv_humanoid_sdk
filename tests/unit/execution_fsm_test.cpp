#include "irmv/runtime/execution_fsm.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/runtime/execution_context.hpp"
#include "support/check.hpp"
#include "support/runtime_fixture.hpp"

namespace {

using irmv::configuration::RuntimeConfig;
using irmv::hardware::TorqueOutput;
using irmv::protocol::ExecutionRequest;
using irmv::protocol::ExecutionState;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::Operation;
using irmv::protocol::OutputAction;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::Sequence;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::runtime::EventCode;
using irmv::runtime::HeartbeatReject;
using irmv::runtime::RuntimeEvent;
using irmv::runtime::TickResult;
using irmv::test::FixtureResult;
using irmv::test::RuntimeFixture;

const ExecutionState kBoot = ExecutionState::Boot;
const ExecutionState kZeroTorque = ExecutionState::ZeroTorque;
const ExecutionState kDamping = ExecutionState::Damping;
const ExecutionState kTracking = ExecutionState::Tracking;
const ExecutionState kFault = ExecutionState::Fault;

const OutputAction kActionZeroTorque = OutputAction::ZeroTorque;
const OutputAction kActionDamping = OutputAction::Damping;
const OutputAction kActionTracking = OutputAction::Tracking;

// The damping torque of the default feedback template: -kd * dq.
constexpr double kDampingTorque0 = -2.0 * 0.2;
constexpr double kDampingTorque1 = -2.0 * -0.1;

// ---- helpers --------------------------------------------------------------

const RuntimeEvent *FindEvent(const TickResult &tick, EventCode code) {
  for (std::uint32_t index = 0; index < tick.event_count; ++index) {
    if (tick.events[index].code == code) {
      return &tick.events[index];
    }
  }
  return nullptr;
}

int CountEvents(const TickResult &tick, EventCode code) {
  int count = 0;
  for (std::uint32_t index = 0; index < tick.event_count; ++index) {
    if (tick.events[index].code == code) {
      ++count;
    }
  }
  return count;
}

bool IsZeroTorque(const TorqueOutput &output) {
  for (std::size_t index = 0; index < output.count; ++index) {
    if (output.torque[index] != 0.0) {
      return false;
    }
  }
  return true;
}

// Steps the fixture one control period at a time until the machine reaches the
// requested state or the budget is exhausted; returns the ticks spent.
std::size_t TicksUntil(RuntimeFixture &fixture, ExecutionState wanted,
                       std::size_t max_ticks = 4096) {
  std::size_t ticks = 0;
  while (fixture.state().execution_state != wanted && ticks < max_ticks) {
    fixture.Advance(fixture.ControlPeriodUs());
    ++ticks;
  }
  return ticks;
}

// ---- (a) the plan example ------------------------------------------------

int CheckPlanExample() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Tracking).rejected());
  CHECK(f.Request(Operation::Damping).applied());
  f.backend.SetWriteAccepted(false);
  CHECK(f.StartTracking().rejected());
  CHECK(f.state().execution_state == ExecutionState::Fault);

  // The rejected tracking request never reported success: the refused write
  // turned it into a WriteFailed rejection.
  CHECK(f.last_result().reason == RejectReason::WriteFailed);
  CHECK(f.last_result().status == ResultStatus::Rejected);
  return 0;
}

// ---- (b) normal path -----------------------------------------------------

int CheckNormalPath() {
  RuntimeFixture f;
  CHECK(f.BootToZeroTorque() > 0);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(f.state().epoch == 0);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.last_tick().output_valid);
  CHECK(IsZeroTorque(f.last_tick().output));

  const FixtureResult acquired = f.Acquire();
  CHECK(acquired.applied());
  CHECK(acquired.execution_state == kZeroTorque);
  CHECK(acquired.session_id == f.state().session_id);
  const Id session = f.state().session_id;
  CHECK(session != irmv::protocol::kNoId);
  // AcquireSession never grants a motion epoch.
  CHECK(f.state().epoch == 0);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(CountEvents(f.last_tick(), EventCode::SessionAcquired) == 1);

  const FixtureResult damped = f.Request(Operation::Damping);
  CHECK(damped.applied());
  CHECK(damped.execution_state == kDamping);
  CHECK(damped.output_action == kActionDamping);
  CHECK(damped.epoch == 1);
  CHECK(damped.session_id == session);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().output_action == kActionDamping);
  CHECK(f.state().epoch == 1);
  CHECK(f.state().stop_reason == StopReason::Requested);
  CHECK(f.last_tick().output.torque[0] == kDampingTorque0);
  CHECK(f.last_tick().output.torque[1] == kDampingTorque1);
  CHECK(CountEvents(f.last_tick(), EventCode::StopApplied) == 1);

  const FixtureResult tracking = f.StartTracking();
  CHECK(tracking.applied());
  CHECK(tracking.execution_state == kTracking);
  CHECK(tracking.output_action == kActionTracking);
  CHECK(tracking.epoch == 2);
  CHECK(tracking.producer_id == irmv::test::FixtureProducerId());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().output_action == kActionTracking);
  CHECK(f.state().producer_id == irmv::test::FixtureProducerId());
  CHECK(f.state().epoch == 2);
  CHECK(f.state().session_id == session);
  CHECK(f.state().stop_reason == StopReason::None);
  CHECK(f.state().last_applied_command_seq > 0);
  CHECK(CountEvents(f.last_tick(), EventCode::AuthorizationGranted) == 1);

  // Fresh commands and heartbeats keep the target owned and alive.
  f.Advance(50'000);
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().epoch == 2);
  CHECK(f.last_tick().output.torque[0] == -0.1);

  const FixtureResult stopped = f.StopRequest(Operation::Damping);
  CHECK(stopped.applied());
  CHECK(stopped.execution_state == kDamping);
  CHECK(stopped.epoch == 3);
  CHECK(stopped.output_action == kActionDamping);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == 3);
  CHECK(f.state().stop_reason == StopReason::Requested);
  CHECK(f.last_tick().output.torque[0] == kDampingTorque0);

  const FixtureResult zero = f.StopRequest(Operation::ZeroTorque);
  CHECK(zero.applied());
  CHECK(zero.execution_state == kZeroTorque);
  CHECK(zero.epoch == 4);
  CHECK(zero.output_action == kActionZeroTorque);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(f.state().epoch == 4);
  CHECK(f.last_tick().output_valid);
  CHECK(IsZeroTorque(f.last_tick().output));
  return 0;
}

// ---- (c) write failure ---------------------------------------------------

int CheckWriteFailure() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());

  f.backend.SetWriteAccepted(false);
  const FixtureResult result = f.StartTracking();
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::WriteFailed);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id != 0);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(!f.last_tick().write_result.accepted);
  CHECK(f.last_tick().output_valid);

  // The refused write is followed by a zero torque attempt in the same tick,
  // and no result of the tick reports success.
  CHECK(f.backend.LastOutput().has_value());
  CHECK(IsZeroTorque(*f.backend.LastOutput()));
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(f.last_tick().result_count > 0);
  for (std::uint32_t index = 0; index < f.last_tick().result_count; ++index) {
    CHECK(f.last_tick().results[index].status == ResultStatus::Rejected);
    CHECK(f.last_tick().results[index].reason == RejectReason::WriteFailed);
  }
  const RuntimeEvent *latched =
      FindEvent(f.last_tick(), EventCode::FaultLatched);
  CHECK(latched != nullptr);
  CHECK(latched->fault_reason == irmv::protocol::FaultReason::WriteFailed);
  CHECK(latched->to == kFault);

  // The latch does not clear when writes work again, and it keeps zero torque.
  f.backend.SetWriteAccepted(true);
  f.Advance(10'000);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(f.last_tick().write_result.accepted);
  CHECK(IsZeroTorque(f.last_tick().output));
  return 0;
}

int CheckIncompleteSnapshotFaults() {
  RuntimeFixture f;
  f.SetFeedbackCount(1);
  f.Advance(f.ControlPeriodUs());

  CHECK(f.state().execution_state == kFault);
  const RuntimeEvent *latched =
      FindEvent(f.last_tick(), EventCode::FaultLatched);
  CHECK(latched != nullptr);
  CHECK(latched->fault_reason == irmv::protocol::FaultReason::FeedbackInvalid);
  // The reason survives even though an incomplete snapshot carries no detail.
  CHECK(f.state().faults.count == 1);
  CHECK(f.state().faults.details[0].reason ==
        irmv::protocol::FaultReason::FeedbackInvalid);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(f.state().feedback.count == 1);
  return 0;
}

// ---- (d) boot rules ------------------------------------------------------

int CheckBootWindow() {
  RuntimeFixture f;
  // The window is boot.healthy_window_us = 100000 us of consecutive healthy
  // ticks; at 2000 us per tick it completes on the 51st tick, because the
  // window starts on the first healthy tick.
  const std::size_t ticks = TicksUntil(f, kZeroTorque);
  CHECK(ticks == 51);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(FindEvent(f.last_tick(), EventCode::BootCompleted) != nullptr);
  return 0;
}

int CheckBootWindowRestart() {
  RuntimeFixture f;
  f.Advance(40 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kBoot);

  // One unhealthy tick restarts the window without faulting (still inside the
  // frequency grace), so the 40 healthy ticks before it are discarded.
  f.SetJointFrequency(300.0);
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kBoot);
  CHECK(f.state().healthy);
  f.SetJointFrequency(irmv::test::kFixtureFrequencyHz);

  // 45 further healthy ticks are not enough any more.
  f.Advance(45 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kBoot);

  const std::size_t ticks = TicksUntil(f, kZeroTorque);
  CHECK(ticks == 6);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(FindEvent(f.last_tick(), EventCode::BootCompleted) != nullptr);
  return 0;
}

int CheckBootDeadline() {
  RuntimeConfig config = RuntimeFixture::LoadFixtureConfig();
  // Test-only configuration: a healthy window longer than the deadline is the
  // only way a deadline expiry is observable, because after the frequency grace
  // an unusable measured state faults first.
  config.boot.healthy_window_us = 10 * config.boot.deadline_us;
  RuntimeFixture f(std::move(config));
  const std::size_t ticks = TicksUntil(f, kFault, 2000);
  CHECK(ticks == 1002);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id != 0);
  const RuntimeEvent *event = FindEvent(f.last_tick(), EventCode::BootFailed);
  CHECK(event != nullptr);
  CHECK(event->fault_reason == irmv::protocol::FaultReason::BootFailed);
  CHECK(f.state().output_action == kActionZeroTorque);
  return 0;
}

int CheckBootObservedFaults() {
  {  // Velocity
    RuntimeFixture f;
    f.SetJointVelocity(11.0);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    const RuntimeEvent *event =
        FindEvent(f.last_tick(), EventCode::FaultLatched);
    CHECK(event != nullptr);
    CHECK(event->fault_reason ==
          irmv::protocol::FaultReason::JointVelocityExceeded);
  }
  {  // Position
    RuntimeFixture f;
    f.mutable_feedback().joints[1].q = -2.5;
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    const RuntimeEvent *event =
        FindEvent(f.last_tick(), EventCode::FaultLatched);
    CHECK(event != nullptr);
    CHECK(event->fault_reason ==
          irmv::protocol::FaultReason::JointPositionExceeded);
  }
  {  // Driver error, high bit of a 32 bit code preserved
    RuntimeFixture f;
    f.SetDriverError();
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    const RuntimeEvent *event =
        FindEvent(f.last_tick(), EventCode::FaultLatched);
    CHECK(event != nullptr);
    CHECK(event->fault_reason == irmv::protocol::FaultReason::DriverError);
    CHECK(f.state().faults.count > 0);
    bool found_code = false;
    for (std::uint32_t index = 0; index < f.state().faults.count; ++index) {
      if (f.state().faults.details[index].error_code ==
          irmv::test::kDriverErrorCode) {
        found_code = true;
      }
    }
    CHECK(found_code);
  }
  return 0;
}

int CheckBootRequiresImu() {
  RuntimeConfig config = RuntimeFixture::LoadFixtureConfig();
  config.imu_required = true;
  {
    RuntimeFixture f(config);
    f.SetImuFresh(false);
    f.Advance(60 * f.ControlPeriodUs());
    // A stale IMU neither faults during the grace nor completes the window.
    CHECK(f.state().execution_state == kBoot);
  }
  {
    RuntimeFixture f(config);
    f.mutable_feedback().imu.valid = false;
    f.Advance(f.ControlPeriodUs());
    // An observed invalid IMU faults immediately, grace or not.
    CHECK(f.state().execution_state == kFault);
    const RuntimeEvent *event =
        FindEvent(f.last_tick(), EventCode::FaultLatched);
    CHECK(event != nullptr);
    CHECK(event->fault_reason == irmv::protocol::FaultReason::ImuInvalid);
  }
  {
    RuntimeFixture f(config);
    CHECK(f.BootToZeroTorque() > 0);
    CHECK(f.state().execution_state == kZeroTorque);
  }
  return 0;
}

// ---- (e) request idempotency --------------------------------------------

int CheckRequestIdempotency() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  const Sequence watermark = f.state().epoch;

  const Sequence id_first = 10;
  const ExecutionRequest request = f.MakeRequest(Operation::Damping, id_first);
  const FixtureResult first = f.Send(request, true);
  CHECK(first.applied());
  CHECK(f.state().epoch == watermark + 1);
  const Sequence applied_watermark = f.state().epoch;
  const TimeUs first_stamp = first.stamp_us;

  // Same request id and payload: the stored answer is repeated once more and
  // nothing is applied a second time.
  const FixtureResult replay = f.Send(request, true);
  CHECK(replay.applied());
  CHECK(replay.stamp_us == first_stamp);
  CHECK(replay.epoch == first.epoch);
  CHECK(f.last_tick().result_count == 1);
  CHECK(f.state().epoch == applied_watermark);

  // Same request id, different payload: conflict, and the stored record stays.
  ExecutionRequest conflict = f.MakeRequest(Operation::Damping, id_first);
  conflict.producer_id = irmv::test::FixtureSecondProducerId();
  const FixtureResult conflict_result = f.Send(conflict, true);
  CHECK(conflict_result.rejected());
  CHECK(conflict_result.reason == RejectReason::RequestConflict);
  CHECK(f.state().epoch == applied_watermark);

  const FixtureResult repeated = f.Send(request, true);
  CHECK(repeated.applied());
  CHECK(repeated.stamp_us == first_stamp);

  // A smaller request id is stale and changes nothing.
  const ExecutionRequest stale =
      f.MakeRequest(Operation::Damping, id_first - 1);
  const FixtureResult stale_result = f.Send(stale, true);
  CHECK(stale_result.rejected());
  CHECK(stale_result.reason == RejectReason::StaleRequest);
  CHECK(f.state().epoch == applied_watermark);

  // A larger request id is handled normally.
  const FixtureResult next = f.Request(Operation::Damping, id_first + 1);
  CHECK(next.applied());
  CHECK(f.state().epoch == applied_watermark + 1);
  // Only the most recent request is remembered.
  const FixtureResult now_stale = f.Send(request, true);
  CHECK(now_stale.reason == RejectReason::StaleRequest);
  return 0;
}

// ---- (f), (g) stop ordering and the authorization watermark ---------------

int CheckStopWinsInSameTick() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  CHECK(f.state().execution_state == kTracking);
  const Sequence tracking_epoch = f.state().epoch;

  // The stop lane is processed first, so the start loses the epoch context it
  // was built for even though both arrive in the same tick. The stop carries
  // the smaller request id so the start is not classified as stale instead.
  const ExecutionRequest stop = f.MakeRequest(Operation::Damping);
  const ExecutionRequest start =
      f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                    irmv::test::FixtureSecondProducerId());
  f.QueueRequest(start);
  f.QueueRequest(stop, 0, true);
  f.Step();

  const FixtureResult stop_result =
      RuntimeFixture::ResultFor(f.last_tick(), stop.request_id);
  const FixtureResult start_result =
      RuntimeFixture::ResultFor(f.last_tick(), start.request_id);
  CHECK(stop_result.applied());
  CHECK(stop_result.execution_state == kDamping);
  CHECK(start_result.rejected());
  CHECK(start_result.reason == RejectReason::EpochMismatch);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == tracking_epoch + 1);

  // The stop advanced the watermark: a later request carrying the old context
  // is rejected despite its larger request id.
  const ExecutionRequest late = f.MakeRequest(
      Operation::Tracking, stop.request_id + 100,
      irmv::test::FixtureSecondProducerId(), irmv::test::FixtureSupervisorId(),
      tracking_epoch, tracking_epoch + 1);
  const FixtureResult late_result = f.Send(late);
  CHECK(late_result.rejected());
  CHECK(late_result.reason == RejectReason::EpochMismatch);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().epoch == tracking_epoch + 1);

  // The same frame with the current watermark is accepted.
  CHECK(f.StartTracking(0, irmv::test::FixtureSecondProducerId()).applied());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().producer_id == irmv::test::FixtureSecondProducerId());
  return 0;
}

// ---- (n) velocity violation overrides every state ------------------------

int CheckVelocityFaultOverridesEveryState() {
  {  // Boot
    RuntimeFixture f;
    f.SetJointVelocity(11.0);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
  }
  {  // ZeroTorque
    RuntimeFixture f;
    f.BootToZeroTorque();
    f.Acquire();
    f.SetJointVelocity(11.0);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
  }
  {  // Damping
    RuntimeFixture f;
    f.BootToZeroTorque();
    f.Acquire();
    CHECK(f.Request(Operation::Damping).applied());
    f.SetJointVelocity(11.0);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
  }
  {  // Tracking
    RuntimeFixture f;
    f.BootToZeroTorque();
    f.Acquire();
    CHECK(f.Request(Operation::Damping).applied());
    CHECK(f.StartTracking().applied());
    f.SetJointVelocity(11.0);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    CHECK(f.state().producer_id == irmv::protocol::kNoId);
  }
  return 0;
}

// ---- (o) a fault and a request in the same tick --------------------------

int CheckFaultBeatsRequestInSameTick() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  const Sequence watermark = f.state().epoch;

  const ExecutionRequest start = f.MakeRequest(Operation::Tracking);
  f.QueueRequest(start);
  // An observed position violation in the very same tick.
  f.mutable_feedback().joints[0].q = 2.5;
  f.Step();

  const FixtureResult result =
      RuntimeFixture::ResultFor(f.last_tick(), start.request_id);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::InvalidTransition);
  CHECK(result.producer_id == irmv::protocol::kNoId);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == watermark);
  // The output follows the fault rules, never the tracking request.
  CHECK(f.state().output_action == kActionDamping);
  CHECK(f.last_tick().output.torque[0] == kDampingTorque0);
  CHECK(f.last_tick().output.torque[1] == kDampingTorque1);
  return 0;
}

// ---- (h) session takeover -----------------------------------------------

int CheckSessionTakeover() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  const FixtureResult first = f.Acquire(7);
  CHECK(first.applied());
  const Id session = f.state().session_id;
  CHECK(session != irmv::protocol::kNoId);
  CHECK(first.session_id == session);
  CHECK(CountEvents(f.last_tick(), EventCode::SessionAcquired) == 1);

  // A repeated bootstrap request with the same id reads the issued session back
  // instead of issuing a new one.
  const FixtureResult retry = f.Acquire(7);
  CHECK(retry.applied());
  CHECK(retry.session_id == session);
  CHECK(retry.stamp_us == first.stamp_us);
  CHECK(f.state().session_id == session);
  CHECK(f.state().epoch == 0);
  CHECK(CountEvents(f.last_tick(), EventCode::SessionAcquired) == 0);

  // Another supervisor cannot take a healthy session over.
  const FixtureResult busy =
      f.Acquire(8, irmv::test::FixtureOtherSupervisorId());
  CHECK(busy.rejected());
  CHECK(busy.reason == RejectReason::SessionBusy);
  CHECK(f.state().session_id == session);

  // Neither can the owning supervisor with a different request id.
  CHECK(f.Acquire(9).reason == RejectReason::SessionBusy);
  CHECK(f.Acquire(7, irmv::test::FixtureOtherSupervisorId()).reason ==
        RejectReason::SessionBusy);

  // Once the lease is over, the next acquire issues a fresh session.
  f.SetHeartbeatEnabled(false);
  f.Advance(f.config.timeouts.supervisor_us + f.ControlPeriodUs());
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().execution_state == kZeroTorque);
  const FixtureResult taken =
      f.Acquire(20, irmv::test::FixtureOtherSupervisorId());
  CHECK(taken.applied());
  CHECK(taken.session_id != session);
  CHECK(taken.session_id != irmv::protocol::kNoId);
  CHECK(f.state().session_id == taken.session_id);
  return 0;
}

// ---- (i) supervision lease ----------------------------------------------

int CheckSupervisionLease();

int CheckSupervisionLease() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Id session = f.state().session_id;
  const Sequence epoch = f.state().epoch;

  // Fresh heartbeats and fresh commands keep tracking alive well past both the
  // command and the source-state timeout.
  const std::uint64_t accepted_after_tracking =
      f.fsm().heartbeats_accepted_count();
  const std::uint64_t rejected_after_tracking =
      f.fsm().heartbeats_rejected_count();
  f.Advance(4 * f.config.timeouts.command_us);
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().session_id == session);
  CHECK(f.state().epoch == epoch);
  // The fresh owner heartbeats refreshed the lease, and the last acceptance is
  // the tick that just ran, so the age of that acceptance is zero.
  CHECK(f.fsm().heartbeats_accepted_count() > accepted_after_tracking);
  CHECK(f.fsm().last_accepted_heartbeat_seq() > 0);
  CHECK(f.now_us() - f.fsm().last_accepted_heartbeat_us() == 0);

  // Repeating an already accepted heartbeat sequence never refreshes the lease,
  // so the session expires exactly supervisor_us after the last accepted one.
  // The rejection is counted under its own reason while acceptance stands
  // still.
  const std::uint64_t accepted_before_repeat =
      f.fsm().heartbeats_accepted_count();
  const std::uint64_t stale_before =
      f.fsm().heartbeat_rejection_count(HeartbeatReject::StaleSequence);
  f.SetHeartbeatSequence(1);
  f.Advance(50 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.fsm().heartbeats_accepted_count() == accepted_before_repeat);
  CHECK(f.fsm().heartbeat_rejection_count(HeartbeatReject::StaleSequence) >
        stale_before);
  CHECK(f.fsm().heartbeats_rejected_count() > rejected_after_tracking);
  const std::size_t ticks = TicksUntil(f, kDamping, 400);
  CHECK(ticks == 101);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().stop_reason == StopReason::SupervisorTimeout);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch + 1);
  CHECK(f.state().output_action == kActionDamping);
  CHECK(f.last_tick().output.torque[0] == kDampingTorque0);
  CHECK(FindEvent(f.last_tick(), EventCode::SessionExpired) != nullptr);
  CHECK(FindEvent(f.last_tick(), EventCode::SupervisorTimeout) != nullptr);

  // A heartbeat for the expired session is ignored and a request carrying the
  // expired session is rejected: the lease cannot be revived.
  irmv::protocol::SupervisorHeartbeat stale_heartbeat;
  stale_heartbeat.protocol_version = irmv::protocol::kProtocolVersion;
  stale_heartbeat.runtime_id = irmv::test::FixtureRuntimeId();
  stale_heartbeat.session_id = session;
  stale_heartbeat.supervisor_id = irmv::test::FixtureSupervisorId();
  stale_heartbeat.heartbeat_seq = 1000;
  stale_heartbeat.stamp_us = f.now_us();
  const std::uint64_t session_rejects =
      f.fsm().heartbeat_rejection_count(HeartbeatReject::SessionId);
  f.QueueRawHeartbeat(stale_heartbeat);
  ExecutionRequest stale = f.MakeRequest(Operation::Damping);
  stale.session_id = session;
  const FixtureResult stale_result = f.Send(stale, true);
  CHECK(stale_result.rejected());
  CHECK(stale_result.reason == RejectReason::WrongSession);
  // The heartbeat for the expired session names an id the machine no longer
  // holds, so it is counted as a session rejection and refreshes nothing.
  CHECK(f.fsm().heartbeat_rejection_count(HeartbeatReject::SessionId) >
        session_rejects);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().execution_state == kDamping);
  return 0;
}

// ---- (j) command and source-state timeouts -------------------------------

int CheckCommandTimeout() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Id session = f.state().session_id;
  const Sequence epoch = f.state().epoch;
  const JointCommand accepted = f.last_frame();

  // The command feed stops; a late duplicate of an older packet is queued once
  // and must not extend anything.
  f.SetCommandEnabled(false);
  f.Advance(30 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);
  f.QueueCommand(accepted);
  const std::size_t ticks = TicksUntil(f, kDamping, 400);
  CHECK(ticks == 21);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().stop_reason == StopReason::CommandTimeout);
  CHECK(f.state().session_id == session);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch + 1);
  CHECK(f.state().output_action == kActionDamping);
  CHECK(FindEvent(f.last_tick(), EventCode::CommandTimeout) != nullptr);
  CHECK(FindEvent(f.last_tick(), EventCode::AuthorizationRevoked) != nullptr);

  // The healthy session still lets the same supervisor take over again.
  CHECK(f.StartTracking().applied());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().epoch == epoch + 2);
  return 0;
}

int CheckSourceStateTimeout() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  const Id session = f.state().session_id;
  // Enough published states to reference one that is already 140 ms old.
  f.Advance(100 * f.ControlPeriodUs());

  // The first frame still references a source state inside source_state_us
  // (150 ms) while the command timeout (100 ms) has just been refreshed by the
  // request itself. No later packet may replace the target, so the source-state
  // deadline is what expires first.
  f.SetSourceStateLagTicks(70);
  f.SetCommandEnabled(false);
  CHECK(f.StartTracking().applied());
  const Sequence epoch = f.state().epoch;

  const std::size_t ticks = TicksUntil(f, kDamping, 200);
  // The source state is 140 ms old when the target is accepted: the fifth tick
  // reaches exactly source_state_us, which is still fresh, so the sixth tick is
  // the first that expires the target.
  CHECK(ticks == 6);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().stop_reason == StopReason::SourceStateTimeout);
  CHECK(f.state().session_id == session);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch + 1);
  CHECK(f.state().output_action == kActionDamping);
  CHECK(FindEvent(f.last_tick(), EventCode::SourceStateTimeout) != nullptr);
  return 0;
}

// ---- (k) first-frame continuity -----------------------------------------

int CheckContinuityGuards() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Id producer = f.state().producer_id;
  const Sequence epoch = f.state().epoch;

  // Position delta
  {
    ExecutionRequest jump =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                      irmv::test::FixtureSecondProducerId());
    jump.first_command.joints.q[1] += 0.11;
    const FixtureResult result = f.Send(jump);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::Discontinuity);
  }
  // Velocity delta
  {
    ExecutionRequest jump =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                      irmv::test::FixtureSecondProducerId());
    jump.first_command.joints.dq[0] = irmv::test::kFixtureDq[0] + 0.51;
    const FixtureResult result = f.Send(jump);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::Discontinuity);
  }
  // Torque delta
  {
    ExecutionRequest jump =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                      irmv::test::FixtureSecondProducerId());
    jump.first_command.joints.tau_ff[0] = 10.0;
    const FixtureResult result = f.Send(jump);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::Discontinuity);
  }

  // Every rejection left the authorized context untouched: same producer, same
  // watermark, same target, and the owner's commands still hold the target.
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().producer_id == producer);
  CHECK(f.state().epoch == epoch);
  f.Advance(10 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().producer_id == producer);
  CHECK(f.state().epoch == epoch);

  // A compliant new source switches atomically: the same tick commands the new
  // gains and the new producer owns the new epoch.
  ExecutionRequest compliant =
      f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                    irmv::test::FixtureSecondProducerId());
  compliant.first_command.joints.kp[0] = 0.0;
  compliant.first_command.joints.kd[0] = 2.0;
  const FixtureResult switched = f.Send(compliant);
  CHECK(switched.applied());
  CHECK(switched.producer_id == irmv::test::FixtureSecondProducerId());
  CHECK(switched.epoch == epoch + 1);
  CHECK(f.state().producer_id == irmv::test::FixtureSecondProducerId());
  CHECK(f.state().epoch == epoch + 1);
  CHECK(f.state().last_applied_command_seq ==
        compliant.first_command.command_seq);
  CHECK(f.last_tick().output.torque[0] == 2.0 * -irmv::test::kFixtureDq[0]);
  CHECK(CountEvents(f.last_tick(), EventCode::AuthorizationGranted) == 1);
  return 0;
}

// ---- (p) foreign packets -------------------------------------------------

int CheckForeignPacketsChangeNothing() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Id session = f.state().session_id;
  const Id producer = f.state().producer_id;
  const Sequence epoch = f.state().epoch;

  const JointCommand owned = f.last_frame();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // Foreign instance, foreign session, foreign epoch and a foreign producer:
  // all dropped by the gate, none of them faults the owner or refreshes a
  // deadline.
  {
    JointCommand foreign = owned;
    foreign.runtime_id = irmv::test::FixtureForeignId();
    foreign.joints.q[0] = nan;
    f.QueueCommand(foreign);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kTracking);
    CHECK(f.state().producer_id == producer);
  }
  {
    JointCommand foreign = owned;
    foreign.session_id = irmv::test::FixtureForeignId();
    foreign.joints.dq[0] = nan;
    f.QueueCommand(foreign);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kTracking);
  }
  {
    JointCommand foreign = owned;
    foreign.epoch = epoch + 5;
    foreign.joints.kp[0] = nan;
    f.QueueCommand(foreign);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kTracking);
  }
  {
    JointCommand foreign = owned;
    foreign.producer_id = irmv::test::FixtureForeignId();
    foreign.joints.kd[0] = -1.0;
    f.QueueCommand(foreign);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kTracking);
  }

  // A request from a foreign instance or with a foreign session changes
  // nothing either.
  ExecutionRequest other_runtime = f.MakeRequest(Operation::Damping);
  other_runtime.runtime_id = irmv::test::FixtureForeignId();
  CHECK(f.Send(other_runtime, true).reason == RejectReason::WrongInstance);
  ExecutionRequest other_session = f.MakeRequest(Operation::ZeroTorque);
  other_session.session_id = irmv::test::FixtureForeignId();
  CHECK(f.Send(other_session, true).reason == RejectReason::WrongSession);
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().session_id == session);
  CHECK(f.state().producer_id == producer);
  CHECK(f.state().epoch == epoch);

  // The owner's command deadline kept its own timeline.
  f.SetCommandEnabled(false);
  const std::size_t ticks = TicksUntil(f, kDamping, 400);
  CHECK(ticks == 51);
  CHECK(f.state().stop_reason == StopReason::CommandTimeout);
  CHECK(f.state().epoch == epoch + 1);
  return 0;
}

// ---- (l) fault output degradation latch ----------------------------------

int CheckFaultOutputDegradation() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  f.mutable_feedback().joints[0].dq = 11.0;
  f.Advance(f.ControlPeriodUs());

  // A velocity violation alone leaves the measured state usable, so the fault
  // keeps damping.
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().output_action == kActionDamping);
  CHECK(f.state().damping_eligible);
  CHECK(f.last_tick().output.torque[0] == -20.0);
  CHECK(f.last_tick().output.torque[1] == kDampingTorque1);

  // A driver error removes the damping context of this fault for good.
  f.SetDriverError();
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(!f.state().damping_eligible);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(FindEvent(f.last_tick(), EventCode::FaultOutputDegraded) != nullptr);
  CHECK(CountEvents(f.last_tick(), EventCode::FaultOutputDegraded) == 1);

  // The degradation is a latch: even a fully recovered measured state keeps
  // commanding zero torque until the fault is reset.
  f.ClearDriverError();
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.mutable_feedback().joints[1].dq = irmv::test::kFixtureDq[1];
  f.Advance(10 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().healthy);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(!f.state().damping_eligible);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(CountEvents(f.last_tick(), EventCode::FaultOutputDegraded) == 0);
  return 0;
}

int CheckWriteFailureDegradesLatchedFault() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  f.mutable_feedback().joints[0].dq = 11.0;
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().output_action == kActionDamping);
  CHECK(f.state().damping_eligible);

  // The send path fails while a damping-eligible fault is latched: the fault
  // degrades permanently and only the zero torque fallback is attempted.
  f.backend.SetWriteAccepted(false);
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  CHECK(!f.state().damping_eligible);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(!f.last_tick().write_result.accepted);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(FindEvent(f.last_tick(), EventCode::FaultOutputDegraded) != nullptr);

  // Even with a working send path and a recovered measured state the fault
  // keeps commanding zero torque.
  f.backend.SetWriteAccepted(true);
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.Advance(10 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().healthy);
  CHECK(f.last_tick().write_result.accepted);
  CHECK(!f.state().damping_eligible);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(CountEvents(f.last_tick(), EventCode::FaultOutputDegraded) == 0);
  return 0;
}

// ---- (m) recovery window and Reset ---------------------------------------

int CheckRecoveryWindowAndReset() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  f.SetJointVelocity(11.0);
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  const Sequence fault_id = f.state().fault_id;
  CHECK(fault_id != 0);
  const Sequence watermark = f.state().epoch;

  // The measured state recovers; the recovery window starts on the first
  // healthy tick, so an immediate reset is refused.
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.mutable_feedback().joints[1].dq = irmv::test::kFixtureDq[1];
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().healthy);
  const FixtureResult early = f.ResetRequest(0, fault_id);
  CHECK(early.rejected());
  CHECK(early.reason == RejectReason::RecoveryNotReady);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id == fault_id);
  CHECK(f.state().epoch == watermark);

  // A violation inside the window restarts it.
  f.SetJointVelocity(11.0);
  f.Advance(3 * f.ControlPeriodUs());
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.mutable_feedback().joints[1].dq = irmv::test::kFixtureDq[1];
  f.Advance(45 * f.ControlPeriodUs());
  CHECK(f.ResetRequest(0, fault_id).reason == RejectReason::RecoveryNotReady);
  f.Advance(10 * f.ControlPeriodUs());

  // The reset must match the instance, the session and the latched fault id.
  ExecutionRequest wrong_instance = f.MakeRequest(Operation::Reset);
  wrong_instance.fault_id = fault_id;
  wrong_instance.runtime_id = irmv::test::FixtureForeignId();
  CHECK(f.Send(wrong_instance, true).reason == RejectReason::WrongInstance);
  ExecutionRequest wrong_session = f.MakeRequest(Operation::Reset);
  wrong_session.fault_id = fault_id;
  wrong_session.session_id = irmv::test::FixtureForeignId();
  CHECK(f.Send(wrong_session, true).reason == RejectReason::WrongSession);
  const FixtureResult mismatched = f.ResetRequest(0, fault_id + 1);
  CHECK(mismatched.rejected());
  CHECK(mismatched.reason == RejectReason::FaultMismatch);
  CHECK(f.state().execution_state == kFault);

  // The matching reset succeeds and only reaches ZeroTorque.
  const FixtureResult reset = f.ResetRequest(0, fault_id);
  CHECK(reset.applied());
  CHECK(reset.execution_state == kZeroTorque);
  CHECK(reset.fault_id == 0);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().fault_id == 0);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == watermark + 1);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(IsZeroTorque(f.last_tick().output));
  CHECK(FindEvent(f.last_tick(), EventCode::ResetApplied) != nullptr);
  CHECK(CountEvents(f.last_tick(), EventCode::RecoveryReady) == 0);

  // The cleared latch does not come back and the machine restarts normally.
  f.Advance(10 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().fault_id == 0);
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.state().execution_state == kDamping);
  return 0;
}

// ---- (q) authorization epoch exhaustion ----------------------------------

int CheckEpochWatermarkExhaustion() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());

  // The last representable epoch is a legal authorization: it is greater than
  // the current watermark and never wrapped.
  const Sequence last_epoch = std::numeric_limits<Sequence>::max();
  const ExecutionRequest request = f.MakeRequest(
      Operation::Tracking, irmv::test::kAutoSequence,
      irmv::test::FixtureProducerId(), irmv::test::FixtureSupervisorId(),
      f.state().epoch, last_epoch);
  const FixtureResult tracking = f.Send(request);
  CHECK(tracking.applied());
  CHECK(tracking.epoch == last_epoch);
  CHECK(f.state().epoch == last_epoch);
  CHECK(f.state().execution_state == kTracking);

  // Every stop has to advance the watermark, which cannot be done without
  // wrapping: the runtime latches a protocol fault and refuses the request.
  const FixtureResult stop = f.StopRequest(Operation::Damping);
  CHECK(stop.rejected());
  CHECK(stop.reason == RejectReason::EpochMismatch);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id != 0);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == last_epoch);
  CHECK(IsZeroTorque(f.last_tick().output));
  const RuntimeEvent *latched =
      FindEvent(f.last_tick(), EventCode::FaultLatched);
  CHECK(latched != nullptr);
  CHECK(latched->fault_reason ==
        irmv::protocol::FaultReason::ProtocolExhausted);
  CHECK(FindEvent(f.last_tick(), EventCode::ProtocolExhausted) != nullptr);
  CHECK(f.state().faults.details[0].reason ==
        irmv::protocol::FaultReason::ProtocolExhausted);

  // The instance is finished: a reset cannot advance the watermark either.
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.Advance(60 * f.ControlPeriodUs());
  const FixtureResult reset = f.ResetRequest(0, f.state().fault_id);
  CHECK(reset.rejected());
  CHECK(reset.reason == RejectReason::EpochMismatch);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().epoch == last_epoch);
  return 0;
}

// ---- state rules, publication and bounded capacity ------------------------

int CheckStateRulesAndPublication() {
  RuntimeFixture f;
  // AcquireSession is legal during Boot; motion and reset are not.
  const FixtureResult acquire = f.Acquire();
  CHECK(acquire.applied());
  CHECK(f.state().execution_state == kBoot);
  CHECK(f.state().session_id != irmv::protocol::kNoId);
  CHECK(f.state().epoch == 0);
  CHECK(f.Request(Operation::Damping).rejected());
  CHECK(f.last_result().reason == RejectReason::InvalidTransition);
  CHECK(f.Request(Operation::Tracking).rejected());
  CHECK(f.last_result().reason == RejectReason::InvalidTransition);
  CHECK(f.Request(Operation::Reset).rejected());
  CHECK(f.last_result().reason == RejectReason::InvalidTransition);
  CHECK(f.state().execution_state == kBoot);

  // The published state describes exactly this tick.
  CHECK(f.state().stamp_us == f.now_us());
  CHECK(f.state().state_seq != 0);
  CHECK(f.state().runtime_id == irmv::test::FixtureRuntimeId());
  CHECK(f.state().session_id == acquire.session_id);
  CHECK(f.state().feedback.stamp_us == f.now_us());
  CHECK(f.state().feedback.count == 2);
  CHECK(f.state().feedback.joints[0].q == irmv::test::kFixtureQ[0]);
  CHECK(f.state().feedback.joints[1].dq == irmv::test::kFixtureDq[1]);
  CHECK(f.state().config_id.View() == f.config.config_id);
  CHECK(f.state().layout_id.View() == f.config.layout_id);
  CHECK(f.state().healthy);
  CHECK(f.state().faults.count == 0);

  CHECK(f.BootToZeroTorque() > 0);
  CHECK(f.state().execution_state == kZeroTorque);

  // A request without a valid session and a reset outside a fault are refused.
  ExecutionRequest no_session = f.MakeRequest(Operation::Damping);
  no_session.session_id = irmv::protocol::kNoId;
  CHECK(f.Send(no_session, true).reason == RejectReason::WrongSession);
  CHECK(f.Request(Operation::Reset).reason == RejectReason::InvalidTransition);

  ExecutionRequest bad_version = f.MakeRequest(Operation::Damping);
  bad_version.protocol_version = 999;
  CHECK(f.Send(bad_version, true).reason == RejectReason::VersionMismatch);
  ExecutionRequest bad_operation = f.MakeRequest(Operation::Damping);
  bad_operation.operation = static_cast<Operation>(99);
  CHECK(f.Send(bad_operation, true).reason == RejectReason::Malformed);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().epoch == 0);
  return 0;
}

int CheckOwnedInvalidCommandLatches() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Sequence epoch = f.state().epoch;

  // A frame from the authorized owner that violates the entry limits is a
  // command fault, not a silent drop.
  JointCommand bad = f.last_frame();
  bad.command_seq = f.state().last_applied_command_seq + 1000;
  bad.source_state_seq = f.state().state_seq;
  bad.runtime_id = irmv::test::FixtureRuntimeId();
  bad.session_id = f.state().session_id;
  bad.producer_id = f.state().producer_id;
  bad.epoch = f.state().epoch;
  bad.joints.q[0] = 5.0;
  f.QueueCommand(bad);
  f.Advance(f.ControlPeriodUs());

  CHECK(f.state().execution_state == kFault);
  const RuntimeEvent *latched =
      FindEvent(f.last_tick(), EventCode::FaultLatched);
  CHECK(latched != nullptr);
  CHECK(latched->fault_reason == irmv::protocol::FaultReason::CommandInvalid);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch);
  // The measured state is healthy, so this fault keeps damping.
  CHECK(f.state().output_action == kActionDamping);
  CHECK(f.last_tick().output.torque[0] == kDampingTorque0);
  return 0;
}

int CheckBoundedResultCapacity() {
  RuntimeFixture f;
  f.BootToZeroTorque();

  // One tick carrying the full stop lane and a full ordinary lane: forty
  // answers are produced while only kResultCapacity fit.
  for (int index = 0; index < 8; ++index) {
    f.QueueRequest(f.MakeRequest(Operation::Damping, 1000 + index), 0, true);
  }
  for (int index = 0; index < 32; ++index) {
    f.QueueRequest(f.MakeRequest(Operation::AcquireSession, 2000 + index));
  }
  f.Step();

  CHECK(f.last_tick().result_count == irmv::runtime::kResultCapacity);
  CHECK(f.last_tick().dropped_result_count == 8);
  // The stop lane was answered before the session existed.
  CHECK(RuntimeFixture::ResultFor(f.last_tick(), 1000).reason ==
        RejectReason::WrongSession);
  // The session was still issued and is published, even though the answers that
  // did not fit were dropped.
  CHECK(f.state().session_id != irmv::protocol::kNoId);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(FindEvent(f.last_tick(), EventCode::SessionAcquired) != nullptr);
  return 0;
}

// ---- fix round 1: write-confirmed answers --------------------------------

// F1: a duplicate of the same request inside one tick must be answered once, by
// the answer that is confirmed after the write, never by a provisional Applied.
int CheckSameTickDuplicateIsConfirmedOnce() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());

  const ExecutionRequest start = f.MakeRequest(Operation::Tracking);
  f.backend.SetWriteAccepted(false);
  f.QueueRequest(start);
  f.QueueRequest(start);
  f.Step();

  std::uint32_t matching = 0;
  for (std::uint32_t index = 0; index < f.last_tick().result_count; ++index) {
    if (f.last_tick().results[index].request_id == start.request_id) {
      ++matching;
    }
  }
  CHECK(matching == 1);
  const FixtureResult result =
      RuntimeFixture::ResultFor(f.last_tick(), start.request_id);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::WriteFailed);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);

  // The stored record carries the confirmed answer, not a provisional success.
  f.backend.SetWriteAccepted(true);
  const FixtureResult replay = f.Send(start);
  CHECK(replay.rejected());
  CHECK(replay.reason == RejectReason::WriteFailed);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);

  // A recorded rejection is deduplicated the same way.
  {
    RuntimeFixture g;
    g.BootToZeroTorque();
    g.Acquire();
    const ExecutionRequest reset = g.MakeRequest(Operation::Reset, 9);
    g.QueueRequest(reset, 0, true);
    g.QueueRequest(reset, 0, true);
    g.Step();
    std::uint32_t reset_matches = 0;
    for (std::uint32_t index = 0; index < g.last_tick().result_count; ++index) {
      if (g.last_tick().results[index].request_id == reset.request_id) {
        ++reset_matches;
      }
    }
    CHECK(reset_matches == 1);
    const FixtureResult reset_result =
        RuntimeFixture::ResultFor(g.last_tick(), reset.request_id);
    CHECK(reset_result.reason == RejectReason::InvalidTransition);
    CHECK(g.state().execution_state == kZeroTorque);
  }
  return 0;
}

// F2 (a): a tracking grant superseded by an owner command fault in the same
// tick is not confirmed.
int CheckTrackingSupersededByCommandFault() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Sequence epoch = f.state().epoch;

  const ExecutionRequest start =
      f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                    irmv::test::FixtureSecondProducerId());
  f.QueueRequest(start);
  // A frame of the new authorization that fails the entry limits.
  JointCommand bad = start.first_command;
  bad.command_seq = start.first_command.command_seq + 1;
  bad.joints.q[0] = 5.0;
  f.QueueCommand(bad);
  f.Step();

  const FixtureResult result =
      RuntimeFixture::ResultFor(f.last_tick(), start.request_id);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::InvalidTransition);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch + 1);
  CHECK(FindEvent(f.last_tick(), EventCode::AuthorizationGranted) != nullptr);
  CHECK(FindEvent(f.last_tick(), EventCode::AuthorizationRevoked) != nullptr);
  const RuntimeEvent *latched =
      FindEvent(f.last_tick(), EventCode::FaultLatched);
  CHECK(latched != nullptr);
  CHECK(latched->fault_reason == irmv::protocol::FaultReason::CommandInvalid);
  return 0;
}

// F2 (b): a tracking grant superseded by the supervision lease expiry in the
// same tick is not confirmed.
int CheckTrackingSupersededBySupervisorTimeout() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());

  f.SetHeartbeatEnabled(false);
  f.Advance(150 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);

  // The grant and the lease expiry land in the same tick.
  const ExecutionRequest start =
      f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                    irmv::test::FixtureSecondProducerId());
  f.QueueRequest(start);
  f.Advance(f.ControlPeriodUs());

  const FixtureResult result =
      RuntimeFixture::ResultFor(f.last_tick(), start.request_id);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::InvalidTransition);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().stop_reason == StopReason::SupervisorTimeout);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == start.proposed_epoch + 1);
  return 0;
}

// F2 (c): requests that survive their tick are still confirmed as Applied.
int CheckConfirmedStopStaysApplied() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();

  const ExecutionRequest damping = f.MakeRequest(Operation::Damping);
  const FixtureResult damping_result = f.Send(damping, true);
  CHECK(damping_result.applied());
  CHECK(damping_result.reason == RejectReason::None);
  CHECK(f.state().execution_state == kDamping);

  const ExecutionRequest zero = f.MakeRequest(Operation::ZeroTorque);
  const FixtureResult zero_result = f.Send(zero, true);
  CHECK(zero_result.applied());
  CHECK(zero_result.reason == RejectReason::None);
  CHECK(f.state().execution_state == kZeroTorque);

  // The confirmed answers stay replayable and the older one stays stale.
  CHECK(f.Send(zero, true).applied());
  CHECK(f.Send(damping, true).reason == RejectReason::StaleRequest);
  return 0;
}

// F3: the continuity guard compares the new frame against the torque of the
// current context, not against zero.
int CheckContinuityOldContextTorque() {
  {  // Damping: the reference is the damping output -kd * dq.
    RuntimeFixture f;
    f.BootToZeroTorque();
    f.Acquire();
    CHECK(f.Request(Operation::Damping).applied());
    CHECK(f.last_tick().output.torque[0] == kDampingTorque0);

    ExecutionRequest frame =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                      irmv::test::FixtureSecondProducerId());
    for (std::size_t index = 0; index < 2; ++index) {
      frame.first_command.joints.q[index] = irmv::test::kFixtureQ[index];
      frame.first_command.joints.dq[index] = irmv::test::kFixtureDq[index];
    }
    // A target equal to the measured state has no PD term, so the expected new
    // torque is exactly tau_ff = 5.0: |5.0 - (-0.4)| = 5.4 > 5 while
    // |5.0 - 0.0| = 5.0 <= 5.
    frame.first_command.joints.tau_ff[0] = 5.0;
    const FixtureResult result = f.Send(frame);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::Discontinuity);
    CHECK(f.state().execution_state == kDamping);
    CHECK(f.state().producer_id == irmv::protocol::kNoId);
  }
  {  // Tracking: the reference is the accepted target's unclamped torque.
    RuntimeFixture f;
    f.BootToZeroTorque();
    f.Acquire();
    CHECK(f.Request(Operation::Damping).applied());

    ExecutionRequest first = f.MakeRequest(Operation::Tracking);
    for (std::size_t index = 0; index < 2; ++index) {
      first.first_command.joints.q[index] = irmv::test::kFixtureQ[index];
      first.first_command.joints.kp[index] = 0.0;
      first.first_command.joints.kd[index] = 4.0;
    }
    CHECK(f.Send(first).applied());
    CHECK(f.state().execution_state == kTracking);
    // The accepted target holds 4 * (0 - dq): -0.8 for joint 0, +0.4 for joint
    // 1 (the damping reference for joint 0 is only -0.4).
    CHECK(f.last_tick().output.torque[0] == -0.8);

    ExecutionRequest frame =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence,
                      irmv::test::FixtureSecondProducerId());
    for (std::size_t index = 0; index < 2; ++index) {
      frame.first_command.joints.q[index] = irmv::test::kFixtureQ[index];
      frame.first_command.joints.dq[index] = irmv::test::kFixtureDq[index];
      frame.first_command.joints.kp[index] = 0.0;
      frame.first_command.joints.kd[index] = 0.0;
    }
    // Joint 0: |4.5 - (-0.8)| = 5.3 > 5, while |4.5 - 0.0| = 4.5 and
    // |4.5 - (-0.4)| = 4.9 both stay inside the bound.
    frame.first_command.joints.tau_ff[0] = 4.5;
    const FixtureResult result = f.Send(frame);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::Discontinuity);
    CHECK(f.state().execution_state == kTracking);
    CHECK(f.state().producer_id == irmv::test::FixtureProducerId());
  }
  return 0;
}

// G1: an identical request from another supervisor is not the owner's replay:
// the session binding refuses it before the request record is consulted, so the
// owner's stored answer stays untouched.
int CheckForeignSupervisorIsNotTheOwnersReplay() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();

  const ExecutionRequest owned = f.MakeRequest(Operation::Damping, 42);
  const FixtureResult owned_result = f.Send(owned, true);
  CHECK(owned_result.applied());
  const Sequence watermark = f.state().epoch;

  // The same id and payload from another supervisor is refused as a foreign
  // request of the owner's session, and nothing is applied a second time.
  ExecutionRequest foreign = owned;
  foreign.supervisor_id = irmv::test::FixtureOtherSupervisorId();
  const FixtureResult foreign_result = f.Send(foreign, true);
  CHECK(foreign_result.rejected());
  CHECK(foreign_result.reason == RejectReason::WrongSession);
  CHECK(f.state().epoch == watermark);

  // The owning supervisor still reads its own answer back.
  const FixtureResult replay = f.Send(owned, true);
  CHECK(replay.applied());
  CHECK(replay.stamp_us == owned_result.stamp_us);
  CHECK(f.state().epoch == watermark);
  return 0;
}

// G2: a bootstrap request carries no session.
int CheckAcquireRequiresEmptySession() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  const FixtureResult issued = f.Acquire(7);
  CHECK(issued.applied());
  CHECK(issued.session_id != irmv::protocol::kNoId);

  // The retry keeps carrying an empty session and still reads the session back.
  const FixtureResult retry = f.Acquire(7);
  CHECK(retry.applied());
  CHECK(retry.session_id == issued.session_id);
  CHECK(f.state().session_id == issued.session_id);

  // A non-empty session is malformed for a bootstrap request.
  ExecutionRequest bad = f.MakeRequest(Operation::AcquireSession, 8);
  bad.session_id = issued.session_id;
  const FixtureResult result = f.Send(bad);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::Malformed);
  CHECK(f.state().session_id == issued.session_id);
  CHECK(f.state().epoch == 0);

  // The same bootstrap packet twice in one batch is answered once.
  {
    RuntimeFixture g;
    g.BootToZeroTorque();
    const ExecutionRequest acquire =
        g.MakeRequest(Operation::AcquireSession, 12);
    g.QueueRequest(acquire);
    g.QueueRequest(acquire);
    g.Step();
    std::uint32_t matches = 0;
    for (std::uint32_t index = 0; index < g.last_tick().result_count; ++index) {
      if (g.last_tick().results[index].request_id == acquire.request_id) {
        ++matches;
      }
    }
    CHECK(matches == 1);
    CHECK(
        RuntimeFixture::ResultFor(g.last_tick(), acquire.request_id).applied());
    CHECK(g.state().session_id != irmv::protocol::kNoId);
    CHECK(CountEvents(g.last_tick(), EventCode::SessionAcquired) == 1);
  }
  return 0;
}

// G3: only the supervisor that owns the session can refresh its lease.
int CheckHeartbeatSupervisorOwnership() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Id session = f.state().session_id;
  const Sequence epoch = f.state().epoch;

  f.SetHeartbeatEnabled(false);
  f.Advance(100 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);
  irmv::protocol::SupervisorHeartbeat foreign;
  foreign.protocol_version = irmv::protocol::kProtocolVersion;
  foreign.runtime_id = irmv::test::FixtureRuntimeId();
  foreign.session_id = session;
  foreign.supervisor_id = irmv::test::FixtureOtherSupervisorId();
  foreign.heartbeat_seq = 99'999;
  foreign.stamp_us = f.now_us();
  const std::uint64_t foreign_before =
      f.fsm().heartbeat_rejection_count(HeartbeatReject::SupervisorId);
  const std::uint64_t accepted_before = f.fsm().heartbeats_accepted_count();
  f.QueueRawHeartbeat(foreign);

  // A foreign heartbeat with the right session and a much larger sequence
  // refreshes nothing: the lease still lapses supervisor_us after the last
  // owner heartbeat, not after the foreign one. The refusal is counted as a
  // supervisor rejection and acceptance does not move.
  const std::size_t ticks = TicksUntil(f, kDamping, 400);
  CHECK(ticks == 51);
  CHECK(f.fsm().heartbeat_rejection_count(HeartbeatReject::SupervisorId) >
        foreign_before);
  CHECK(f.fsm().heartbeats_accepted_count() == accepted_before);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().stop_reason == StopReason::SupervisorTimeout);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch + 1);
  return 0;
}

// The cadence of accepted heartbeats: a gap longer than the machine's late
// threshold moves the worst-case interval and the late count, while a gap
// inside it moves neither. The runtime does not parse the application's
// heartbeat period, so its threshold is half the lease - the tightest budget it
// can prove - and the test feeds a gap on each side of it.
int CheckHeartbeatCadence() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.state().session_id != irmv::protocol::kNoId);
  CHECK(f.state().execution_state == kZeroTorque);
  // From here the automatic feed is off: only the heartbeats this test queues
  // are judged, each on its own tick.
  f.SetHeartbeatEnabled(false);

  const TimeUs late_threshold_us = f.config.timeouts.supervisor_us / 2;

  // The first heartbeat of the session only starts the cadence: it has no
  // predecessor of the same session, so no interval is recorded.
  f.QueueHeartbeat();
  f.Step();
  CHECK(f.fsm().worst_accepted_heartbeat_gap_us() == 0);
  CHECK(f.fsm().heartbeats_late_count() == 0);

  // A gap beyond the threshold is one late interval and the new worst case,
  // while the lease itself (the session) survives it.
  const TimeUs first_us = f.now_us();
  const TimeUs gap_us = late_threshold_us + 5 * f.ControlPeriodUs();
  f.Advance(gap_us);
  CHECK(f.state().session_id != irmv::protocol::kNoId);
  f.QueueHeartbeat();
  f.Step();
  CHECK(f.now_us() - first_us == gap_us);
  CHECK(f.fsm().worst_accepted_heartbeat_gap_us() == gap_us);
  CHECK(f.fsm().heartbeats_late_count() == 1);

  // A gap inside the threshold moves neither number.
  const TimeUs worst_before = f.fsm().worst_accepted_heartbeat_gap_us();
  const std::uint64_t late_before = f.fsm().heartbeats_late_count();
  f.Advance(10 * f.ControlPeriodUs());
  f.QueueHeartbeat();
  f.Step();
  CHECK(f.state().session_id != irmv::protocol::kNoId);
  CHECK(f.fsm().worst_accepted_heartbeat_gap_us() == worst_before);
  CHECK(f.fsm().heartbeats_late_count() == late_before);
  return 0;
}

// A receive batch that mixes an owner heartbeat with one the state machine has
// to discard must still refresh the lease: the entries of a batch are judged in
// receive order, and a foreign or reordered sample never costs the owner
// heartbeat its tick. The lane above hands the batch over as a FIFO, so this is
// the contract the machine is written against.
int CheckMixedHeartbeatBatchRefreshesLease() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Id session = f.state().session_id;

  // The automatic feed stops; from here on only the queued batches carry
  // heartbeats. Half a lease has already lapsed when the first batch arrives.
  f.SetHeartbeatEnabled(false);
  f.Advance(50 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);

  irmv::protocol::SupervisorHeartbeat owner;
  owner.protocol_version = irmv::protocol::kProtocolVersion;
  owner.runtime_id = irmv::test::FixtureRuntimeId();
  owner.session_id = session;
  owner.supervisor_id = irmv::test::FixtureSupervisorId();
  owner.heartbeat_seq = 5000;
  owner.stamp_us = f.now_us();

  irmv::protocol::SupervisorHeartbeat foreign = owner;
  foreign.supervisor_id = irmv::test::FixtureOtherSupervisorId();
  foreign.heartbeat_seq = owner.heartbeat_seq + 1000;

  // One batch: the owner heartbeat first, a foreign one behind it.
  f.QueueRawHeartbeat(owner);
  f.QueueRawHeartbeat(foreign);
  f.Step();
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().session_id == session);
  CHECK(f.state().producer_id == irmv::test::FixtureProducerId());

  // The lease runs from the owner heartbeat of that batch: half of it later the
  // session is still alive, and it lapses exactly one lease after it rather
  // than being shortened by the foreign entry.
  f.Advance(50 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);
  CHECK(TicksUntil(f, kDamping, 400) == 101);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().stop_reason == StopReason::SupervisorTimeout);

  // A reordered pair is the same rule inside one batch: the newer owner
  // heartbeat refreshes the lease and the older one behind it refreshes
  // nothing, and certainly undoes nothing.
  RuntimeFixture g;
  g.BootToZeroTorque();
  g.Acquire();
  CHECK(g.Request(Operation::Damping).applied());
  CHECK(g.StartTracking().applied());
  const Id second_session = g.state().session_id;
  g.SetHeartbeatEnabled(false);
  g.Advance(50 * g.ControlPeriodUs());

  irmv::protocol::SupervisorHeartbeat newer = owner;
  newer.session_id = second_session;
  newer.heartbeat_seq = 7000;
  newer.stamp_us = g.now_us();
  irmv::protocol::SupervisorHeartbeat older = newer;
  older.heartbeat_seq = newer.heartbeat_seq - 1;

  g.QueueRawHeartbeat(newer);
  g.QueueRawHeartbeat(older);
  g.Step();
  CHECK(g.state().execution_state == kTracking);
  CHECK(g.state().session_id == second_session);
  g.Advance(50 * g.ControlPeriodUs());
  CHECK(g.state().execution_state == kTracking);
  CHECK(TicksUntil(g, kDamping, 400) == 101);
  CHECK(g.state().session_id == irmv::protocol::kNoId);
  CHECK(g.state().stop_reason == StopReason::SupervisorTimeout);
  return 0;
}

// G4: a Reset cannot win against a fresh safety observation in the same tick.
int CheckResetWithFreshSafetyObservation() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  f.mutable_feedback().joints[0].dq = 11.0;
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  const Sequence fault_id = f.state().fault_id;
  const Sequence watermark = f.state().epoch;

  // The measured state recovers and the recovery window completes.
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.Advance(60 * f.ControlPeriodUs());

  // A reset arriving in the tick of a fresh violation is refused: the safety
  // observation restarts the recovery window before the request is considered.
  ExecutionRequest reset = f.MakeRequest(Operation::Reset);
  reset.fault_id = fault_id;
  f.QueueRequest(reset, 0, true);
  f.mutable_feedback().joints[0].dq = 11.0;
  f.Step();
  const FixtureResult refused =
      RuntimeFixture::ResultFor(f.last_tick(), reset.request_id);
  CHECK(refused.rejected());
  CHECK(refused.reason == RejectReason::RecoveryNotReady);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id == fault_id);
  CHECK(f.state().epoch == watermark);

  // Without the fresh violation the same reset succeeds, which shows the
  // refusal came from the observation and not from a permanently unready state.
  f.mutable_feedback().joints[0].dq = irmv::test::kFixtureDq[0];
  f.Advance(60 * f.ControlPeriodUs());
  const FixtureResult applied = f.ResetRequest(0, fault_id);
  CHECK(applied.applied());
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().fault_id == 0);
  return 0;
}

// A session never inherits the staged answer of the session it replaced: a
// request of the dying session staged in the takeover tick must not become the
// new session's record.
int CheckTakeoverDoesNotInheritStagedAnswer() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  const FixtureResult first = f.Acquire(10);
  CHECK(first.applied());
  const Id old_session = f.state().session_id;
  CHECK(old_session != irmv::protocol::kNoId);

  // The lease is left to lapse, but not yet expired by a tick: the takeover
  // lands in the same tick as the last packet of the dying session.
  f.SetHeartbeatEnabled(false);
  f.Advance(150 * f.ControlPeriodUs());
  CHECK(f.state().session_id == old_session);
  const Sequence old_watermark = f.state().epoch;

  ExecutionRequest stop = f.MakeRequest(Operation::Damping, 100);
  stop.session_id = old_session;
  const ExecutionRequest takeover =
      f.MakeRequest(Operation::AcquireSession, 11);
  f.QueueRequest(stop, 0, true);
  f.QueueRequest(takeover);
  f.Advance(f.ControlPeriodUs());

  CHECK(
      RuntimeFixture::ResultFor(f.last_tick(), takeover.request_id).applied());
  const Id new_session = f.state().session_id;
  CHECK(new_session != irmv::protocol::kNoId);
  CHECK(new_session != old_session);

  // The new session's very first request is answered on its own behalf, not as
  // a stale packet of the session it replaced.
  const ExecutionRequest fresh = f.MakeRequest(Operation::Damping, 3);
  const FixtureResult fresh_result = f.Send(fresh, true);
  CHECK(fresh_result.request_id == fresh.request_id);
  CHECK(fresh_result.reason != RejectReason::StaleRequest);
  CHECK(fresh_result.applied());
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().session_id == new_session);

  // A packet of the dying session stays refused, and an old epoch context is
  // still refused on the epoch guards.
  ExecutionRequest stale_session = f.MakeRequest(Operation::Damping, 101);
  stale_session.session_id = old_session;
  const FixtureResult stale_session_result = f.Send(stale_session, true);
  CHECK(stale_session_result.rejected());
  CHECK(stale_session_result.reason == RejectReason::WrongSession);

  const ExecutionRequest stale_epoch = f.MakeRequest(
      Operation::Tracking, 102, irmv::test::FixtureProducerId(),
      irmv::test::FixtureSupervisorId(), old_watermark, old_watermark + 1);
  const FixtureResult stale_epoch_result = f.Send(stale_epoch);
  CHECK(stale_epoch_result.rejected());
  CHECK(stale_epoch_result.reason == RejectReason::EpochMismatch);
  CHECK(f.state().session_id == new_session);
  return 0;
}

// ---- the runtime's own stop ------------------------------------------------

// A local stop is the trusted signal of the leaving process, not a request: it
// needs no supervisor identity and no session, answers nobody, never advances
// the session's request record and never displaces a received stop.
int CheckLocalStopIsNotARequest() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  const ExecutionRequest stop = f.MakeRequest(Operation::Damping, 100);
  CHECK(f.Send(stop, true).applied());
  CHECK(f.state().execution_state == kDamping);
  const Sequence damped_epoch = f.state().epoch;

  // It reaches zero torque from Damping, revokes the watermark and leaves the
  // request record exactly where it was.
  f.RequestLocalStop();
  f.Step();
  const TickResult &local = f.last_tick();
  CHECK(local.state.execution_state == kZeroTorque);
  CHECK(local.state.epoch == damped_epoch + 1);
  CHECK(local.state.output_action == kActionZeroTorque);
  CHECK(IsZeroTorque(local.output));
  CHECK(local.state.stop_reason == StopReason::Requested);
  CHECK(local.state.producer_id == irmv::protocol::kNoId);
  CHECK(local.result_count == 0);
  CHECK(CountEvents(local, EventCode::StopApplied) == 1);
  CHECK(CountEvents(local, EventCode::RequestRejected) == 0);

  // The record is untouched: the supervisor's next id is a new request, not one
  // the runtime's own stop consumed. ZeroTorque holds, so the idempotent stop
  // is still allowed.
  const FixtureResult next =
      f.Send(f.MakeRequest(Operation::ZeroTorque, 101), true);
  CHECK(next.applied());
  CHECK(next.reason == RejectReason::None);
  CHECK(f.state().execution_state == kZeroTorque);

  // Zero torque already holds, so repeating the runtime's own stop changes
  // nothing: no watermark, no event and no result.
  const Sequence zero_epoch = f.state().epoch;
  f.RequestLocalStop();
  f.Step();
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().epoch == zero_epoch);
  CHECK(CountEvents(f.last_tick(), EventCode::StopApplied) == 0);

  // Boot and Fault are never left by the runtime itself: the startup path and a
  // latched fault decide their own output.
  {
    RuntimeFixture boot;
    boot.RequestLocalStop();
    boot.Step();
    CHECK(boot.state().execution_state == kBoot);
    CHECK(boot.last_tick().result_count == 0);
    CHECK(CountEvents(boot.last_tick(), EventCode::StopApplied) == 0);
    CHECK(boot.BootToZeroTorque() > 0);
    CHECK(boot.state().execution_state == kZeroTorque);
  }
  {
    RuntimeFixture fault;
    fault.BootToZeroTorque();
    fault.Acquire();
    fault.SetJointVelocity(15.0);
    fault.Advance(fault.ControlPeriodUs());
    CHECK(fault.state().execution_state == kFault);
    const Sequence fault_id = fault.state().fault_id;
    const OutputAction action = fault.state().output_action;
    fault.RequestLocalStop();
    fault.Step();
    CHECK(fault.state().execution_state == kFault);
    CHECK(fault.state().fault_id == fault_id);
    CHECK(fault.state().output_action == action);
    CHECK(CountEvents(fault.last_tick(), EventCode::StopApplied) == 0);
  }
  return 0;
}

// A stop received in the tick of the local stop is still answered and still
// applied; the local stop then decides the state the tick commits, so the
// received answer is confirmed against that state like every other answer.
int CheckLocalStopKeepsInboundStop() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Sequence epoch = f.state().epoch;

  const ExecutionRequest received = f.MakeRequest(Operation::Damping, 200);
  f.QueueRequest(received, 0, true);
  f.RequestLocalStop();
  f.Step();

  const TickResult &tick = f.last_tick();
  const FixtureResult received_result = RuntimeFixture::ResultFor(tick, 200);
  CHECK(received_result.request_id == received.request_id);
  CHECK(tick.result_count == 1);
  CHECK(received_result.rejected());
  CHECK(received_result.reason == RejectReason::InvalidTransition);
  CHECK(tick.state.execution_state == kZeroTorque);
  CHECK(tick.state.epoch == epoch + 2);
  CHECK(tick.state.producer_id == irmv::protocol::kNoId);
  CHECK(tick.state.output_action == kActionZeroTorque);
  CHECK(IsZeroTorque(tick.output));
  CHECK(CountEvents(tick, EventCode::StopApplied) == 2);
  CHECK(CountEvents(tick, EventCode::AuthorizationRevoked) == 1);
  CHECK(CountEvents(tick, EventCode::RequestRejected) == 0);
  return 0;
}

// H1: the session is bound to the supervisor it was issued to, so a request
// that only names the published session is refused whatever it asks for, and
// without touching the owner's request record.
int CheckForeignSupervisorCannotUseSession() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  const Id owner = irmv::test::FixtureSupervisorId();
  const Id intruder = irmv::test::FixtureOtherSupervisorId();
  const Id producer = irmv::test::FixtureProducerId();

  const ExecutionRequest owned =
      f.MakeRequest(Operation::Damping, 100, producer, owner);
  const FixtureResult owned_result = f.Send(owned, true);
  CHECK(owned_result.applied());
  CHECK(f.state().execution_state == kDamping);
  const Sequence stop_watermark = f.state().epoch;

  // A start that echoes the owner's session and carries a valid first frame is
  // still refused: the authorization does not change, no tracking torque is
  // executed, and the epoch is not burned.
  const ExecutionRequest tracking =
      f.MakeRequest(Operation::Tracking, 1'000, producer, intruder);
  FixtureResult result = f.Send(tracking);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::WrongSession);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().epoch == stop_watermark);
  CHECK(f.state().producer_id == irmv::protocol::kNoId);
  CHECK(f.last_tick().output.torque[0] == kDampingTorque0);
  CHECK(CountEvents(f.last_tick(), EventCode::AuthorizationGranted) == 0);

  // A stop from the intruder changes nothing either.
  const ExecutionRequest zero =
      f.MakeRequest(Operation::ZeroTorque, 1'001, producer, intruder);
  result = f.Send(zero, true);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::WrongSession);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().epoch == stop_watermark);

  // An id below the owner's newest one is refused as a foreign request, not as
  // a stale request of the owner: identity is checked before the record.
  const ExecutionRequest lower =
      f.MakeRequest(Operation::Damping, 50, producer, intruder);
  const FixtureResult lower_result = f.Send(lower, true);
  CHECK(lower_result.rejected());
  CHECK(lower_result.reason == RejectReason::WrongSession);
  CHECK(f.state().epoch == stop_watermark);

  // The owner's record is untouched, so its answer is still replayable.
  const FixtureResult replay = f.Send(owned, true);
  CHECK(replay.applied());
  CHECK(replay.stamp_us == owned_result.stamp_us);

  // The owner is served normally afterwards, and with the very id the intruder
  // used: a refused request never entered the session's record.
  const ExecutionRequest owned_start =
      f.MakeRequest(Operation::Tracking, 1'000, producer, owner);
  result = f.Send(owned_start);
  CHECK(result.applied());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().producer_id == producer);
  const Sequence tracking_watermark = f.state().epoch;
  const double tracking_torque0 = f.last_tick().output.torque[0];

  // A foreign stop while the owner tracks: refused, and the executed output is
  // still the owner's target.
  const ExecutionRequest foreign_stop =
      f.MakeRequest(Operation::Damping, 1'002, producer, intruder);
  result = f.Send(foreign_stop, true);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::WrongSession);
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().epoch == tracking_watermark);
  CHECK(f.state().producer_id == producer);
  CHECK(f.last_tick().output.torque[0] == tracking_torque0);

  const ExecutionRequest owned_stop =
      f.MakeRequest(Operation::Damping, 1'004, producer, owner);
  result = f.Send(owned_stop, true);
  CHECK(result.applied());
  CHECK(f.state().execution_state == kDamping);

  // A latched fault is not cleared by a foreign Reset, and the owner's request
  // that follows is answered by the state machine - which proves the refused
  // Reset did not become the session's newest handled request.
  f.SetJointVelocity(15.0);
  f.Advance(f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  const Sequence fault_id = f.state().fault_id;
  ExecutionRequest foreign_reset =
      f.MakeRequest(Operation::Reset, 1'003, producer, intruder);
  foreign_reset.fault_id = fault_id;
  result = f.Send(foreign_reset, true);
  CHECK(result.rejected());
  CHECK(result.reason == RejectReason::WrongSession);
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id == fault_id);

  const ExecutionRequest owned_after =
      f.MakeRequest(Operation::Damping, 1'005, producer, owner);
  result = f.Send(owned_after, true);
  CHECK(result.reason == RejectReason::InvalidTransition);
  CHECK(f.state().execution_state == kFault);
  return 0;
}

// One stop and one start forced into the same control tick. The start carries
// the epoch the accepted stop of that tick produces and proposes the next one,
// with a fresh continuous frame, so only same-tick stop priority can refuse it.
struct StopStartTick {
  const TickResult *tick{nullptr};
  FixtureResult stop{};
  FixtureResult start{};
  Sequence start_id{0};
};

StopStartTick SendStopAndStart(RuntimeFixture &f, Operation stop_operation,
                               Sequence stop_id, Sequence start_id,
                               Id start_producer) {
  const Sequence epoch = f.state().epoch;
  const ExecutionRequest stop = f.MakeRequest(stop_operation, stop_id);
  const ExecutionRequest start =
      f.MakeRequest(Operation::Tracking, start_id, start_producer,
                    irmv::test::FixtureSupervisorId(), epoch + 1, epoch + 2);
  f.QueueRequest(start);
  f.QueueRequest(stop, 0, true);
  f.Step();
  StopStartTick result;
  result.tick = &f.last_tick();
  result.stop = RuntimeFixture::ResultFor(*result.tick, stop_id);
  result.start = RuntimeFixture::ResultFor(*result.tick, start_id);
  result.start_id = start_id;
  return result;
}

// I1: a stop accepted in a tick owns that tick; a start that arrives with the
// epoch the stop produced cannot authorize motion in the same tick.
int CheckSameTickStopBeatsStart() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Sequence epoch = f.state().epoch;

  const StopStartTick damping_stop = SendStopAndStart(
      f, Operation::Damping, 200, 201, irmv::test::FixtureSecondProducerId());
  CHECK(damping_stop.stop.applied());
  CHECK(damping_stop.stop.execution_state == kDamping);
  CHECK(damping_stop.start.rejected());
  CHECK(damping_stop.start.reason == RejectReason::InvalidTransition);
  CHECK(damping_stop.tick->state.execution_state == kDamping);
  CHECK(damping_stop.tick->state.epoch == epoch + 1);
  CHECK(damping_stop.tick->state.producer_id == irmv::protocol::kNoId);
  CHECK(damping_stop.tick->state.output_action == kActionDamping);
  CHECK(damping_stop.tick->output.torque[0] == kDampingTorque0);
  CHECK(CountEvents(*damping_stop.tick, EventCode::AuthorizationGranted) == 0);

  // The rule owns one tick only: the same frame with a newer request id is
  // authorized on the next tick, without an extra Damping excursion.
  ExecutionRequest retry =
      f.MakeRequest(Operation::Tracking, damping_stop.start_id + 1,
                    irmv::test::FixtureSecondProducerId());
  const FixtureResult retry_result = f.Send(retry);
  CHECK(retry_result.applied());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().producer_id == irmv::test::FixtureSecondProducerId());

  // A ZeroTorque stop of the same tick wins in the same way and leaves the
  // machine at zero torque.
  const StopStartTick zero_stop = SendStopAndStart(
      f, Operation::ZeroTorque, 210, 211, irmv::test::FixtureProducerId());
  CHECK(zero_stop.stop.applied());
  CHECK(zero_stop.start.rejected());
  CHECK(zero_stop.start.reason == RejectReason::InvalidTransition);
  CHECK(zero_stop.tick->state.execution_state == kZeroTorque);
  CHECK(zero_stop.tick->state.output_action == kActionZeroTorque);
  CHECK(IsZeroTorque(zero_stop.tick->output));
  CHECK(zero_stop.tick->state.producer_id == irmv::protocol::kNoId);

  // Repeated stops all apply and the start still loses.
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Sequence repeated_epoch = f.state().epoch;
  {
    const Sequence stop_id = 220;
    const Sequence start_id = 222;
    const ExecutionRequest first = f.MakeRequest(Operation::Damping, stop_id);
    const ExecutionRequest second =
        f.MakeRequest(Operation::ZeroTorque, stop_id + 1);
    const ExecutionRequest start = f.MakeRequest(
        Operation::Tracking, start_id, irmv::test::FixtureSecondProducerId(),
        irmv::test::FixtureSupervisorId(), repeated_epoch + 2,
        repeated_epoch + 3);
    f.QueueRequest(start);
    f.QueueRequest(first, 0, true);
    f.QueueRequest(second, 0, true);
    f.Step();
    const TickResult &tick = f.last_tick();
    // Both stops of the tick are answered. The tick commits zero torque, so the
    // newer stop is Applied and the older one is answered as not implemented by
    // the committed state - the write-confirmation rule every answer obeys,
    // exactly as a start superseded by a later request of its own tick is.
    CHECK(RuntimeFixture::ResultFor(tick, stop_id).request_id == stop_id);
    CHECK(RuntimeFixture::ResultFor(tick, stop_id).rejected());
    CHECK(RuntimeFixture::ResultFor(tick, stop_id).reason ==
          RejectReason::InvalidTransition);
    CHECK(RuntimeFixture::ResultFor(tick, stop_id + 1).applied());
    const FixtureResult start_result =
        RuntimeFixture::ResultFor(tick, start_id);
    CHECK(start_result.rejected());
    CHECK(start_result.reason == RejectReason::InvalidTransition);
    CHECK(tick.state.execution_state == kZeroTorque);
    CHECK(tick.state.epoch == repeated_epoch + 2);
    CHECK(tick.state.producer_id == irmv::protocol::kNoId);
    CHECK(IsZeroTorque(tick.output));
    CHECK(CountEvents(tick, EventCode::AuthorizationGranted) == 0);
  }

  return 0;
}

// I2: only a stop that was accepted owns the tick. Refused traffic must not
// steal a legitimate start of the same tick.
int CheckRefusedStopDoesNotOwnTheTick() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  CHECK(f.StartTracking().applied());
  const Sequence epoch = f.state().epoch;

  // A foreign supervisor's stop in the stop lane is refused; the owner's start
  // in the ordinary lane of the same tick is authorized.
  {
    ExecutionRequest foreign =
        f.MakeRequest(Operation::Damping, 300, irmv::test::FixtureProducerId(),
                      irmv::test::FixtureOtherSupervisorId());
    const ExecutionRequest start = f.MakeRequest(
        Operation::Tracking, 301, irmv::test::FixtureSecondProducerId(),
        irmv::test::FixtureSupervisorId(), epoch, epoch + 1);
    f.QueueRequest(start);
    f.QueueRequest(foreign, 0, true);
    f.Step();
    const TickResult &tick = f.last_tick();
    const FixtureResult foreign_result =
        RuntimeFixture::ResultFor(tick, foreign.request_id);
    CHECK(foreign_result.rejected());
    CHECK(foreign_result.reason == RejectReason::WrongSession);
    const FixtureResult start_result =
        RuntimeFixture::ResultFor(tick, start.request_id);
    CHECK(start_result.applied());
    CHECK(tick.state.execution_state == kTracking);
    CHECK(tick.state.producer_id == irmv::test::FixtureSecondProducerId());
    CHECK(tick.state.epoch == epoch + 1);
    CHECK(CountEvents(tick, EventCode::StopApplied) == 0);
  }

  // A stop that was already applied in an earlier tick is answered from the
  // record when it is replayed, so it does not own the replaying tick either.
  {
    const ExecutionRequest stop =
        f.MakeRequest(Operation::Damping, 310, irmv::test::FixtureProducerId(),
                      irmv::test::FixtureSupervisorId());
    const FixtureResult applied = f.Send(stop, true);
    CHECK(applied.applied());
    CHECK(f.state().execution_state == kDamping);
    const Sequence damped_epoch = f.state().epoch;
    const TimeUs applied_stamp = applied.stamp_us;

    const ExecutionRequest start = f.MakeRequest(
        Operation::Tracking, 312, irmv::test::FixtureProducerId(),
        irmv::test::FixtureSupervisorId(), damped_epoch, damped_epoch + 1);
    f.QueueRequest(start);
    f.QueueRequest(stop, 0, true);
    f.Step();
    const TickResult &tick = f.last_tick();
    const FixtureResult replay =
        RuntimeFixture::ResultFor(tick, stop.request_id);
    CHECK(replay.applied());
    CHECK(replay.stamp_us == applied_stamp);
    const FixtureResult start_result =
        RuntimeFixture::ResultFor(tick, start.request_id);
    CHECK(start_result.applied());
    CHECK(tick.state.execution_state == kTracking);
    CHECK(tick.state.epoch == damped_epoch + 1);
    CHECK(CountEvents(tick, EventCode::StopApplied) == 0);
  }
  return 0;
}

// J1: the producer that already owns the motion authorization may move to a new
// behavior epoch through the ordinary guards, without a Damping stop in
// between. The old epoch is invalid the moment the switch is committed.
int CheckSameProducerNewEpoch() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();
  CHECK(f.Request(Operation::Damping).applied());
  const Id producer = irmv::test::FixtureProducerId();
  CHECK(f.StartTracking(irmv::test::kAutoSequence, producer).applied());
  const Sequence epoch = f.state().epoch;
  CHECK(f.state().execution_state == kTracking);
  // kd * (0 - dq) on the frame that sits on the measured state.
  CHECK(f.last_tick().output.torque[0] == -0.1);
  // The automatic command feed would keep publishing the frame the fixture
  // built, which is not the frame under test here.
  f.SetCommandEnabled(false);

  // The same component enters a new execution stage: fresh, continuous frame,
  // expected watermark E and proposed epoch E + 2. The moved target is a dyadic
  // fraction, so the expected torque is exact.
  constexpr double kMovedDelta = 0.0625;
  const double expected_torque =
      1.0 * kMovedDelta + 0.5 * (0.0 - irmv::test::kFixtureDq[0]);
  ExecutionRequest next =
      f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence, producer,
                    irmv::test::FixtureSupervisorId(), epoch, epoch + 2);
  next.first_command.joints.q[0] += kMovedDelta;
  const FixtureResult next_result = f.Send(next);
  CHECK(next_result.applied());
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().producer_id == producer);
  CHECK(f.state().epoch == epoch + 2);
  // kp * kMovedDelta + kd * (0 - dq) with kp = 1, kd = 0.5.
  CHECK(f.last_tick().output.torque[0] == expected_torque);

  // The identical replay is answered from the record: no second authorization
  // and no second epoch.
  {
    const FixtureResult replay = f.Send(next);
    CHECK(replay.applied());
    CHECK(replay.stamp_us == next_result.stamp_us);
    CHECK(f.state().epoch == epoch + 2);
  }

  // The old epoch stops being executable at once: a frame carrying it is
  // dropped by the gate, without a fault and without touching the new target.
  {
    const JointCommand stale = f.MakeFirstFrame(producer, epoch);
    f.QueueCommand(stale);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kTracking);
    CHECK(f.state().epoch == epoch + 2);
    CHECK(f.state().producer_id == producer);
    CHECK(f.last_tick().output.torque[0] == expected_torque);
  }

  // Rejected switches: a repeated epoch, a discontinuous frame and an expired
  // source state.
  {
    ExecutionRequest same_epoch =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence, producer,
                      irmv::test::FixtureSupervisorId(), epoch + 2, epoch + 2);
    FixtureResult result = f.Send(same_epoch);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::EpochMismatch);
  }
  {
    ExecutionRequest jump =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence, producer,
                      irmv::test::FixtureSupervisorId(), epoch + 2, epoch + 3);
    jump.first_command.joints.q[0] += 0.2;
    const FixtureResult result = f.Send(jump);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::Discontinuity);
  }
  {
    f.SetSourceStateLagTicks(100);
    ExecutionRequest old_source =
        f.MakeRequest(Operation::Tracking, irmv::test::kAutoSequence, producer,
                      irmv::test::FixtureSupervisorId(), epoch + 2, epoch + 3);
    const FixtureResult result = f.Send(old_source);
    f.SetSourceStateLagTicks(1);
    CHECK(result.rejected());
    CHECK(result.reason == RejectReason::StaleSource);
  }

  // Every rejection left the running authorization exactly as it was.
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().epoch == epoch + 2);
  CHECK(f.state().producer_id == producer);
  CHECK(f.last_tick().output.torque[0] == expected_torque);
  return 0;
}

// ---- boot: waiting for the first frame ------------------------------------

// A backend that has not delivered a frame yet must not latch a fault: the boot
// grace window waits, commands zero torque and reports neither health nor a
// fault, and the first frame that does arrive completes the self-check.
int CheckBootWaitsForFirstFrame() {
  RuntimeFixture f;
  f.SetFeedbackAbsent(true);
  for (int tick = 0; tick < 5; ++tick) {
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kBoot);
    CHECK(f.state().fault_id == 0);
    CHECK(!f.state().healthy);
    CHECK(f.state().output_action == kActionZeroTorque);
    CHECK(IsZeroTorque(f.last_tick().output));
  }
  CHECK(CountEvents(f.last_tick(), EventCode::FaultLatched) == 0);
  CHECK(CountEvents(f.last_tick(), EventCode::BootFailed) == 0);

  // The first frame arrives inside the grace window: nothing was latched, so
  // the ordinary self-check completes and the machine reaches ZeroTorque.
  f.SetFeedbackAbsent(false);
  CHECK(f.BootToZeroTorque() > 0);
  CHECK(f.state().execution_state == kZeroTorque);
  CHECK(f.state().healthy);
  CHECK(f.state().fault_id == 0);
  return 0;
}

// A machine that never receives a frame faults when the grace window expires,
// with the safety reason and not a fabricated measurement.
int CheckBootWithoutFirstFrameFaults() {
  RuntimeFixture f;
  f.SetFeedbackAbsent(true);
  f.Advance(f.config.boot.frequency_grace_us + 2 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kFault);
  CHECK(f.state().fault_id != 0);
  CHECK(CountEvents(f.last_tick(), EventCode::FaultLatched) == 1);
  CHECK(f.state().output_action == kActionZeroTorque);
  CHECK(!f.state().healthy);
  // The grace window expires long before the boot deadline of the shipped
  // configuration, so what turned the wait into a fault is invalid feedback and
  // not a boot failure.
  CHECK(f.config.boot.frequency_grace_us < f.config.boot.deadline_us);
  CHECK(f.state().faults.count >= 1);
  CHECK(f.state().faults.details[0].reason ==
        irmv::protocol::FaultReason::FeedbackInvalid);
  return 0;
}

// A frame that already arrived is an observation: the boot grace window never
// excuses a sample that goes missing afterwards, so a joint or the required IMU
// that stops delivering faults at once instead of waiting out the window. A
// machine that never received a first frame keeps waiting, and every boot tick
// commands zero torque.
int CheckBootObservedThenLostFaults() {
  {  // A joint that delivered a frame and then reverts to the absent shape.
    RuntimeFixture f;
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kBoot);
    CHECK(f.state().healthy);
    CHECK(f.state().fault_id == 0);
    CHECK(IsZeroTorque(f.last_tick().output));

    f.SetJointAbsent(0, true);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    CHECK(f.state().fault_id != 0);
    CHECK(CountEvents(f.last_tick(), EventCode::FaultLatched) == 1);
    CHECK(!f.state().healthy);
    CHECK(f.state().faults.count >= 1);
    CHECK(f.state().faults.details[0].reason ==
          irmv::protocol::FaultReason::FeedbackInvalid);
    CHECK(f.state().faults.details[0].joint_index == 0);
    CHECK(f.state().output_action == kActionZeroTorque);
    CHECK(IsZeroTorque(f.last_tick().output));
  }

  {  // The required IMU that delivered a frame and then goes missing.
    RuntimeConfig config = RuntimeFixture::LoadFixtureConfig();
    config.imu_required = true;
    RuntimeFixture f(std::move(config));
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kBoot);
    CHECK(f.state().healthy);
    CHECK(IsZeroTorque(f.last_tick().output));

    f.SetImuAbsent(true);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    CHECK(f.state().fault_id != 0);
    CHECK(CountEvents(f.last_tick(), EventCode::FaultLatched) == 1);
    CHECK(f.state().faults.count >= 1);
    CHECK(f.state().faults.details[0].reason ==
          irmv::protocol::FaultReason::ImuInvalid);
    // The measured joints are still usable, so this fault keeps the damping
    // output; only the joints' own invalidity degrades it to zero torque.
    CHECK(f.state().output_action == kActionDamping);
  }

  {  // Never observed: the same absent sample waits, and an observation that
     // arrives later ends the excuse inside the same grace window.
    RuntimeFixture f;
    f.SetJointAbsent(1, true);
    f.Advance(5 * f.ControlPeriodUs());
    CHECK(f.state().execution_state == kBoot);
    CHECK(f.state().fault_id == 0);
    CHECK(!f.state().healthy);
    CHECK(IsZeroTorque(f.last_tick().output));

    f.SetJointAbsent(1, false);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kBoot);
    CHECK(f.state().healthy);
    CHECK(IsZeroTorque(f.last_tick().output));

    f.SetJointAbsent(1, true);
    f.Advance(f.ControlPeriodUs());
    CHECK(f.state().execution_state == kFault);
    CHECK(f.state().faults.details[0].joint_index == 1);
    CHECK(IsZeroTorque(f.last_tick().output));
  }
  return 0;
}

}  // namespace

int main() {
  CHECK(CheckPlanExample() == 0);
  CHECK(CheckIncompleteSnapshotFaults() == 0);
  CHECK(CheckNormalPath() == 0);
  CHECK(CheckWriteFailure() == 0);
  CHECK(CheckBootWindow() == 0);
  CHECK(CheckBootWindowRestart() == 0);
  CHECK(CheckBootDeadline() == 0);
  CHECK(CheckBootObservedFaults() == 0);
  CHECK(CheckBootRequiresImu() == 0);
  CHECK(CheckRequestIdempotency() == 0);
  CHECK(CheckStopWinsInSameTick() == 0);
  CHECK(CheckVelocityFaultOverridesEveryState() == 0);
  CHECK(CheckFaultBeatsRequestInSameTick() == 0);
  CHECK(CheckSessionTakeover() == 0);
  CHECK(CheckSupervisionLease() == 0);
  CHECK(CheckCommandTimeout() == 0);
  CHECK(CheckSourceStateTimeout() == 0);
  CHECK(CheckContinuityGuards() == 0);
  CHECK(CheckForeignPacketsChangeNothing() == 0);
  CHECK(CheckFaultOutputDegradation() == 0);
  CHECK(CheckWriteFailureDegradesLatchedFault() == 0);
  CHECK(CheckRecoveryWindowAndReset() == 0);
  CHECK(CheckEpochWatermarkExhaustion() == 0);
  CHECK(CheckStateRulesAndPublication() == 0);
  CHECK(CheckOwnedInvalidCommandLatches() == 0);
  CHECK(CheckBoundedResultCapacity() == 0);
  CHECK(CheckSameTickDuplicateIsConfirmedOnce() == 0);
  CHECK(CheckTrackingSupersededByCommandFault() == 0);
  CHECK(CheckTrackingSupersededBySupervisorTimeout() == 0);
  CHECK(CheckConfirmedStopStaysApplied() == 0);
  CHECK(CheckContinuityOldContextTorque() == 0);
  CHECK(CheckForeignSupervisorIsNotTheOwnersReplay() == 0);
  CHECK(CheckAcquireRequiresEmptySession() == 0);
  CHECK(CheckHeartbeatSupervisorOwnership() == 0);
  CHECK(CheckHeartbeatCadence() == 0);
  CHECK(CheckMixedHeartbeatBatchRefreshesLease() == 0);
  CHECK(CheckResetWithFreshSafetyObservation() == 0);
  CHECK(CheckTakeoverDoesNotInheritStagedAnswer() == 0);
  CHECK(CheckLocalStopIsNotARequest() == 0);
  CHECK(CheckLocalStopKeepsInboundStop() == 0);
  CHECK(CheckForeignSupervisorCannotUseSession() == 0);
  CHECK(CheckSameTickStopBeatsStart() == 0);
  CHECK(CheckRefusedStopDoesNotOwnTheTick() == 0);
  CHECK(CheckSameProducerNewEpoch() == 0);
  CHECK(CheckBootWaitsForFirstFrame() == 0);
  CHECK(CheckBootWithoutFirstFrameFaults() == 0);
  CHECK(CheckBootObservedThenLostFaults() == 0);
  return 0;
}
