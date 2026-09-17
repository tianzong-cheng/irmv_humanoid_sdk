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
  f.Advance(4 * f.config.timeouts.command_us);
  CHECK(f.state().execution_state == kTracking);
  CHECK(f.state().session_id == session);
  CHECK(f.state().epoch == epoch);

  // Repeating an already accepted heartbeat sequence never refreshes the lease,
  // so the session expires exactly supervisor_us after the last accepted one.
  f.SetHeartbeatSequence(1);
  f.Advance(50 * f.ControlPeriodUs());
  CHECK(f.state().execution_state == kTracking);
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
  f.QueueRawHeartbeat(stale_heartbeat);
  ExecutionRequest stale = f.MakeRequest(Operation::Damping);
  stale.session_id = session;
  const FixtureResult stale_result = f.Send(stale, true);
  CHECK(stale_result.rejected());
  CHECK(stale_result.reason == RejectReason::WrongSession);
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

// G1: the payload key includes the supervisor identity.
int CheckPayloadKeyIncludesSupervisor() {
  RuntimeFixture f;
  f.BootToZeroTorque();
  f.Acquire();

  const ExecutionRequest owned = f.MakeRequest(Operation::Damping, 42);
  const FixtureResult owned_result = f.Send(owned, true);
  CHECK(owned_result.applied());
  const Sequence watermark = f.state().epoch;

  // The same id and payload from another supervisor is not the owner's replay.
  ExecutionRequest foreign = owned;
  foreign.supervisor_id = irmv::test::FixtureOtherSupervisorId();
  const FixtureResult foreign_result = f.Send(foreign, true);
  CHECK(foreign_result.rejected());
  CHECK(foreign_result.reason == RejectReason::RequestConflict);
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
  f.QueueRawHeartbeat(foreign);

  // A foreign heartbeat with the right session and a much larger sequence
  // refreshes nothing: the lease still lapses supervisor_us after the last
  // owner heartbeat, not after the foreign one.
  const std::size_t ticks = TicksUntil(f, kDamping, 400);
  CHECK(ticks == 51);
  CHECK(f.state().execution_state == kDamping);
  CHECK(f.state().stop_reason == StopReason::SupervisorTimeout);
  CHECK(f.state().session_id == irmv::protocol::kNoId);
  CHECK(f.state().epoch == epoch + 1);
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

// ---- the runtime's own stop request ---------------------------------------

// A tick driver that hands the state machine a caller-built input. The runtime
// fixture owns its own tick loop; these checks have to control the stop lane of
// one single tick exactly, so they own the input instead.
class InputDriver {
 public:
  InputDriver()
      : config(RuntimeFixture::LoadFixtureConfig()),
        backend(config),
        fsm(config, irmv::test::FixtureRuntimeId()) {
    backend.Initialize();
    feedback_.count = static_cast<std::uint32_t>(config.joint_count());
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      irmv::protocol::JointFeedback &joint = feedback_.joints[index];
      joint.valid = true;
      joint.connected = true;
      joint.q = irmv::test::kFixtureQ[index];
      joint.dq = irmv::test::kFixtureDq[index];
      joint.freq_hz = irmv::test::kFixtureFrequencyHz;
    }
  }

  TimeUs now_us() const { return now_us_; }

  const irmv::protocol::RobotState &state() const { return last_tick_.state; }

  // Runs healthy ticks one control period apart until `duration_us` elapsed.
  void Advance(TimeUs duration_us) {
    const TimeUs period =
        config.control_period_us == 0 ? 1 : config.control_period_us;
    TimeUs remaining = duration_us;
    while (remaining > 0) {
      const TimeUs step = remaining < period ? remaining : period;
      now_us_ += step;
      Tick(irmv::runtime::TickInput{});
      remaining -= step;
    }
  }

  // Runs exactly one tick with caller feedback and caller input.
  const TickResult &Tick(const irmv::runtime::TickInput &input) {
    ++feedback_.snapshot_seq;
    feedback_.stamp_us = now_us_;
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      feedback_.joints[index].rx_stamp_us = now_us_;
    }
    backend.Inject(feedback_);
    last_tick_ = fsm.Tick(now_us_, backend.Read(now_us_), input, backend);
    return last_tick_;
  }

  RuntimeConfig config;
  irmv::hardware::FakeBackend backend;
  irmv::runtime::ExecutionFsm fsm;

 private:
  irmv::protocol::FeedbackSnapshot feedback_{};
  TickResult last_tick_{};
  TimeUs now_us_{irmv::test::kFixtureStartUs};
};

ExecutionRequest MakeLaneRequest(const irmv::runtime::ExecutionFsm &fsm,
                                 const irmv::protocol::RobotState &state,
                                 Operation operation, Sequence request_id) {
  ExecutionRequest request;
  request.protocol_version = irmv::protocol::kProtocolVersion;
  request.supervisor_id = irmv::test::FixtureSupervisorId();
  request.request_id = request_id;
  request.session_id = operation == Operation::AcquireSession
                           ? irmv::protocol::kNoId
                           : state.session_id;
  request.runtime_id = fsm.runtime_id();
  request.expected_epoch = state.epoch;
  request.proposed_epoch = state.epoch + 1;
  request.producer_id = irmv::test::FixtureProducerId();
  request.fault_id = state.fault_id;
  request.operation = operation;
  return request;
}

// The runtime's own stop must not crowd out a stop that arrived in the same
// tick: the local request is appended to the stop lane, and a full lane defers
// it instead of overwriting a received stop.
int CheckRuntimeStopKeepsInboundStop() {
  InputDriver driver;
  driver.Advance(200'000);
  CHECK(driver.state().execution_state == kZeroTorque);

  irmv::runtime::TickInput bootstrap{};
  bootstrap.requests[0] = {MakeLaneRequest(driver.fsm, driver.state(),
                                           Operation::AcquireSession, 10),
                           driver.now_us()};
  bootstrap.request_count = 1;
  driver.Tick(bootstrap);
  CHECK(driver.state().session_id != irmv::protocol::kNoId);

  // The supervisor asks for Damping in the very tick in which the runtime asks
  // for its own stop.
  const Sequence inbound_id = 11;
  irmv::runtime::TickInput input{};
  input.stop_requests[0] = {MakeLaneRequest(driver.fsm, driver.state(),
                                            Operation::Damping, inbound_id),
                            driver.now_us()};
  input.stop_request_count = 1;
  CHECK(irmv::runtime::AppendRuntimeStopRequest(&input, driver.fsm,
                                                driver.now_us()));
  CHECK(input.stop_request_count == 2);
  CHECK(input.stop_requests[0].value.request_id == inbound_id);
  CHECK(input.stop_requests[0].value.operation == Operation::Damping);
  const Sequence local_id = input.stop_requests[1].value.request_id;

  const TickResult &tick = driver.Tick(input);
  // Both stops of the tick were answered, the received one first: with the
  // received request overwritten, the tick produced a single answer.
  CHECK(tick.result_count == 2);
  CHECK(RuntimeFixture::ResultFor(tick, inbound_id).request_id == inbound_id);
  CHECK(RuntimeFixture::ResultFor(tick, local_id).request_id == local_id);
  CHECK(RuntimeFixture::ResultFor(tick, local_id).applied());
  CHECK(tick.state.execution_state == kZeroTorque);
  CHECK(CountEvents(tick, EventCode::StopApplied) == 2);

  // A full stop lane never loses a received stop: the local request is refused
  // and the eight inbound requests stay where they are.
  irmv::runtime::TickInput full{};
  for (std::size_t index = 0; index < irmv::runtime::kStopRequestBatchCapacity;
       ++index) {
    full.stop_requests[index] = {
        MakeLaneRequest(driver.fsm, driver.state(), Operation::ZeroTorque,
                        100 + index),
        driver.now_us()};
  }
  full.stop_request_count = irmv::runtime::kStopRequestBatchCapacity;
  CHECK(!irmv::runtime::AppendRuntimeStopRequest(&full, driver.fsm,
                                                 driver.now_us()));
  CHECK(full.stop_request_count == irmv::runtime::kStopRequestBatchCapacity);
  for (std::size_t index = 0; index < irmv::runtime::kStopRequestBatchCapacity;
       ++index) {
    CHECK(full.stop_requests[index].value.request_id == 100 + index);
  }
  const TickResult &full_tick = driver.Tick(full);
  CHECK(full_tick.result_count == irmv::runtime::kStopRequestBatchCapacity);
  return 0;
}

// The stop the runtime issues for itself must be answered, never classified as
// stale, even after a supervisor that already used large request ids.
int CheckRuntimeStopIsNeverStale() {
  InputDriver driver;
  driver.Advance(200'000);
  CHECK(driver.state().execution_state == kZeroTorque);

  constexpr Sequence kLargeRequestId = 1'000'000;
  irmv::runtime::TickInput bootstrap{};
  bootstrap.requests[0] = {
      MakeLaneRequest(driver.fsm, driver.state(), Operation::AcquireSession,
                      kLargeRequestId),
      driver.now_us()};
  bootstrap.request_count = 1;
  driver.Tick(bootstrap);
  CHECK(driver.state().session_id != irmv::protocol::kNoId);
  CHECK(driver.fsm.last_handled_request_id() == kLargeRequestId);

  // The supervisor parks the machine in Damping with an even larger id.
  const Sequence damping_id = kLargeRequestId + 1;
  irmv::runtime::TickInput damping{};
  damping.stop_requests[0] = {MakeLaneRequest(driver.fsm, driver.state(),
                                              Operation::Damping, damping_id),
                              driver.now_us()};
  damping.stop_request_count = 1;
  driver.Tick(damping);
  CHECK(driver.state().execution_state == kDamping);
  CHECK(driver.fsm.last_handled_request_id() == damping_id);

  // The runtime's own stop carries an id strictly greater than everything the
  // session handled, so it is applied instead of answered as a stale request.
  irmv::runtime::TickInput stop{};
  CHECK(irmv::runtime::AppendRuntimeStopRequest(&stop, driver.fsm,
                                                driver.now_us()));
  CHECK(stop.stop_request_count == 1);
  const Sequence local_id = stop.stop_requests[0].value.request_id;
  CHECK(stop.stop_requests[0].value.session_id == driver.state().session_id);

  const TickResult &tick = driver.Tick(stop);
  CHECK(RuntimeFixture::ResultFor(tick, local_id).applied());
  CHECK(tick.state.execution_state == kZeroTorque);
  CHECK(CountEvents(tick, EventCode::RequestRejected) == 0);
  CHECK(FindEvent(tick, EventCode::StopApplied) != nullptr);
  CHECK(local_id > damping_id);
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
  CHECK(CheckPayloadKeyIncludesSupervisor() == 0);
  CHECK(CheckAcquireRequiresEmptySession() == 0);
  CHECK(CheckHeartbeatSupervisorOwnership() == 0);
  CHECK(CheckResetWithFreshSafetyObservation() == 0);
  CHECK(CheckTakeoverDoesNotInheritStagedAnswer() == 0);
  CHECK(CheckRuntimeStopKeepsInboundStop() == 0);
  CHECK(CheckRuntimeStopIsNeverStale() == 0);
  return 0;
}
