// Order-of-arrival tests for the test-only confirmation helper: a request is
// only proven when an Applied result and a later-published agreeing state both
// identify it. No clock reads: every stamp is explicit.
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "irmv/protocol/types.hpp"
#include "support/check.hpp"
#include "support/confirmation.hpp"

namespace {

using irmv::protocol::ExecutionRequest;
using irmv::protocol::ExecutionResult;
using irmv::protocol::ExecutionState;
using irmv::protocol::Id;
using irmv::protocol::kNoId;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::Operation;
using irmv::protocol::OutputAction;
using irmv::protocol::RejectReason;
using irmv::protocol::ResultStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;
using irmv::test::ConfirmationExpectations;
using irmv::test::ConfirmationLog;
using irmv::test::ConfirmationOrder;
using irmv::test::ConfirmationStatus;
using irmv::test::TrackedRequest;

constexpr Sequence kRequestId = 0x1122334455ULL;
constexpr Sequence kOtherRequestId = 0x6655443322ULL;
constexpr Sequence kSecondRequestId = 0x99AA99AA99ULL;
constexpr Sequence kEpoch = 5;
constexpr TimeUs kSendUs = 1'000'000;
constexpr TimeUs kResultStampUs = kSendUs + 500;
constexpr TimeUs kResultRxUs = kResultStampUs + 200;
constexpr TimeUs kStateRxUs = kResultStampUs + 400;
constexpr std::size_t kJointCount = 2;

Id MakeId(std::uint8_t seed) {
  Id id{};
  id.fill(seed);
  return id;
}

Id RuntimeId() { return MakeId(0x11); }
Id SupervisorId() { return MakeId(0x22); }
Id ProducerId() { return MakeId(0x33); }
Id SessionId() { return MakeId(0x44); }
Id OtherSessionId() { return MakeId(0x55); }

// A Tracking request carries a full first command, so a retry has payload
// fields to agree on beyond the header.
ExecutionRequest MakeRequest(Sequence request_id, Operation operation,
                             const Id &session_id = SessionId()) {
  ExecutionRequest request{};
  request.supervisor_id = SupervisorId();
  request.runtime_id = RuntimeId();
  request.request_id = request_id;
  request.session_id = session_id;
  request.operation = operation;
  if (operation == Operation::Tracking) {
    request.first_command.protocol_version = kProtocolVersion;
    request.first_command.runtime_id = RuntimeId();
    request.first_command.session_id = session_id;
    request.first_command.producer_id = ProducerId();
    request.first_command.epoch = kEpoch;
    request.first_command.command_seq = 7;
    request.first_command.source_state_seq = 11;
    request.first_command.layout_id.Assign("test_layout");
    request.first_command.joints.count = kJointCount;
    request.first_command.joints.q[0] = 0.5;
    request.first_command.joints.q[1] = -0.25;
    request.first_command.joints.dq[0] = 0.2;
    request.first_command.joints.kp[0] = 20.0;
    request.first_command.joints.kp[1] = 20.0;
    request.first_command.joints.kd[0] = 1.0;
    request.first_command.joints.kd[1] = 1.0;
  }
  return request;
}

// The Applied answer the runtime publishes for `request` in the tick stamped
// `stamp_us`. A bootstrap request (no session yet) is granted a session.
ExecutionResult MakeResult(const ExecutionRequest &request, TimeUs stamp_us) {
  ExecutionResult result{};
  result.supervisor_id = request.supervisor_id;
  result.runtime_id = request.runtime_id;
  result.request_id = request.request_id;
  result.session_id =
      request.session_id == kNoId ? SessionId() : request.session_id;
  result.execution_state = ExecutionState::Tracking;
  result.output_action = OutputAction::Tracking;
  result.producer_id = ProducerId();
  result.epoch = kEpoch;
  result.stamp_us = stamp_us;
  result.status = ResultStatus::Applied;
  return result;
}

RobotState MakeState(const ExecutionResult &result, TimeUs stamp_us) {
  RobotState state{};
  state.runtime_id = result.runtime_id;
  state.session_id = result.session_id;
  state.producer_id = result.producer_id;
  state.epoch = result.epoch;
  state.execution_state = result.execution_state;
  state.output_action = result.output_action;
  state.stamp_us = stamp_us;
  return state;
}

RobotState MakeFaultState(const ExecutionResult &result, TimeUs stamp_us) {
  RobotState state = MakeState(result, stamp_us);
  state.execution_state = ExecutionState::Fault;
  state.output_action = OutputAction::ZeroTorque;
  return state;
}

// A result only answers the request it identifies: same request id, runtime and
// supervisor, and a session the tracked request can accept. Anything else must
// leave the entry pending even when a state agrees with it.
int MismatchedResultStaysPending(const ExecutionResult &result) {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);
  log.ObserveResult(result, kResultRxUs);
  log.ObserveState(MakeState(result, result.stamp_us), kResultRxUs + 100);

  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Pending);
  CHECK(entry->order == ConfirmationOrder::None);
  CHECK(!log.confirmed(kRequestId));
  CHECK(!log.answered(kRequestId));
  CHECK(log.result_of(kRequestId) == nullptr);
  CHECK(log.results().size() == 1);
  CHECK(log.results()[0].result.runtime_id == result.runtime_id);
  CHECK(log.results()[0].rx_us == kResultRxUs);
  CHECK(log.states().size() == 1);
  CHECK(log.states()[0].state.stamp_us == result.stamp_us);
  return 0;
}

int ResultThenStateConfirms() {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);
  CHECK(log.Find(kRequestId) != nullptr);
  CHECK(log.Find(kRequestId)->send_count == 1);
  CHECK(log.Find(kRequestId)->first_send_us == kSendUs);

  const ExecutionResult result = MakeResult(request, kResultStampUs);
  log.ObserveResult(result, kResultRxUs);
  const TrackedRequest *answered = log.Find(kRequestId);
  CHECK(answered != nullptr);
  CHECK(answered->status == ConfirmationStatus::Pending);
  CHECK(answered->order == ConfirmationOrder::None);
  CHECK(!log.confirmed(kRequestId));
  CHECK(log.answered(kRequestId));
  CHECK(log.results().size() == 1);
  CHECK(log.results()[0].order == 1);
  CHECK(log.results()[0].rx_us == kResultRxUs);
  CHECK(log.results()[0].result.status == ResultStatus::Applied);
  CHECK(log.result_of(kRequestId) == &log.results()[0]);

  log.ObserveState(MakeState(result, kResultStampUs), kStateRxUs);
  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Confirmed);
  CHECK(entry->order == ConfirmationOrder::ResultFirst);
  CHECK(entry->confirmed_session == SessionId());
  CHECK(log.confirmed(kRequestId));
  CHECK(log.states().size() == 1);
  CHECK(log.states()[0].order == 2);
  CHECK(log.states()[0].rx_us == kStateRxUs);
  CHECK(log.states()[0].state.stamp_us == kResultStampUs);
  CHECK(log.states()[0].state.execution_state == ExecutionState::Tracking);
  CHECK(!log.confirmed(kOtherRequestId));

  std::cout << "ok result_then_state_confirms\n";
  return 0;
}

int StateThenResultConfirms() {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);

  const ExecutionResult result = MakeResult(request, kResultStampUs);
  log.ObserveState(MakeState(result, kResultStampUs), kResultStampUs + 100);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Pending);
  CHECK(log.states().size() == 1);
  CHECK(log.states()[0].order == 1);
  CHECK(log.states()[0].rx_us == kResultStampUs + 100);

  log.ObserveResult(result, kResultStampUs + 300);
  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Confirmed);
  CHECK(entry->order == ConfirmationOrder::StateFirst);
  CHECK(entry->confirmed_session == SessionId());
  CHECK(log.confirmed(kRequestId));
  CHECK(log.results().size() == 1);
  CHECK(log.results()[0].order == 2);
  CHECK(log.results()[0].rx_us == kResultStampUs + 300);
  CHECK(log.results()[0].result.stamp_us == kResultStampUs);

  // The witness is the first state that arrived at or after the answer, so a
  // later disagreeing state that was already on hand loses to it.
  ConfirmationLog witness_log;
  witness_log.Track(request, kSendUs);
  const ExecutionResult answer = MakeResult(request, kResultStampUs);
  witness_log.ObserveState(MakeState(answer, kResultStampUs),
                           kResultStampUs + 50);
  RobotState latched = MakeState(answer, kResultStampUs + 100);
  latched.execution_state = ExecutionState::Fault;
  witness_log.ObserveState(latched, kResultStampUs + 150);
  witness_log.ObserveResult(answer, kResultStampUs + 300);
  const TrackedRequest *witnessed = witness_log.Find(kRequestId);
  CHECK(witnessed != nullptr);
  CHECK(witnessed->status == ConfirmationStatus::Confirmed);
  CHECK(witnessed->order == ConfirmationOrder::StateFirst);
  CHECK(witness_log.states().size() == 2);

  std::cout << "ok state_then_result_confirms\n";
  return 0;
}

int FaultBeforeLateAppliedDoesNotConfirm() {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);

  // The fault is published in a later tick than the one that answered the
  // request, but it arrives first; the late answer cannot outrun it.
  const TimeUs fault_us = kResultStampUs + 1'000;
  const ExecutionResult late = MakeResult(request, kResultStampUs);
  log.ObserveState(MakeFaultState(late, fault_us), fault_us + 20);
  log.ObserveResult(late, fault_us + 100);

  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Superseded);
  CHECK(!log.confirmed(kRequestId));
  CHECK(log.answered(kRequestId));
  CHECK(log.results().size() == 1);
  CHECK(log.results()[0].result.stamp_us == kResultStampUs);
  CHECK(log.results()[0].rx_us == fault_us + 100);
  CHECK(log.states().size() == 1);
  CHECK(log.states()[0].state.execution_state == ExecutionState::Fault);
  CHECK(log.states()[0].state.stamp_us == fault_us);

  // A Rejected answer speaks for the request but never confirms it.
  ConfirmationLog rejected_log;
  rejected_log.Track(request, kSendUs);
  ExecutionResult rejected = MakeResult(request, kResultStampUs);
  rejected.status = ResultStatus::Rejected;
  rejected.reason = RejectReason::SessionBusy;
  rejected_log.ObserveResult(rejected, kResultRxUs);
  rejected_log.ObserveState(MakeState(rejected, rejected.stamp_us), kStateRxUs);
  const TrackedRequest *rejected_entry = rejected_log.Find(kRequestId);
  CHECK(rejected_entry != nullptr);
  CHECK(rejected_entry->status == ConfirmationStatus::Pending);
  CHECK(rejected_log.answered(kRequestId));
  CHECK(!rejected_log.confirmed(kRequestId));
  CHECK(rejected_entry->confirmed_session == kNoId);
  CHECK(rejected_log.results().size() == 1);
  CHECK(rejected_log.results()[0].result.status == ResultStatus::Rejected);

  std::cout << "ok fault_before_late_applied_does_not_confirm\n";
  return 0;
}

int StateOlderThanResultDoesNotConfirm() {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);

  // The only state seen was published before the tick that answered the
  // request, so it witnesses nothing about the answer.
  const TimeUs older_us = kResultStampUs - 500;
  const ExecutionResult result = MakeResult(request, kResultStampUs);
  log.ObserveResult(result, kResultRxUs);
  log.ObserveState(MakeState(result, older_us), kStateRxUs);

  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Pending);
  CHECK(entry->order == ConfirmationOrder::None);
  CHECK(!log.confirmed(kRequestId));
  CHECK(log.answered(kRequestId));
  CHECK(log.states().size() == 1);
  CHECK(log.states()[0].state.stamp_us == older_us);
  CHECK(log.states()[0].order == 2);

  // A processing stamp newer than the arrival would be impossible, so the
  // answer is not trusted even though a state agrees with it.
  ConfirmationLog late_log;
  late_log.Track(request, kSendUs);
  const ExecutionResult future = MakeResult(request, kResultRxUs + 100);
  late_log.ObserveResult(future, kResultRxUs);
  late_log.ObserveState(MakeState(future, future.stamp_us), kStateRxUs);
  CHECK(late_log.Find(kRequestId)->status == ConfirmationStatus::Pending);
  CHECK(!late_log.confirmed(kRequestId));
  CHECK(late_log.answered(kRequestId));

  // An answer stamped before the request was ever sent cannot answer it either.
  ConfirmationLog stale_log;
  stale_log.Track(request, kSendUs);
  const ExecutionResult stale = MakeResult(request, kSendUs - 100);
  stale_log.ObserveResult(stale, kResultRxUs);
  stale_log.ObserveState(MakeState(stale, stale.stamp_us), kStateRxUs);
  CHECK(stale_log.Find(kRequestId)->status == ConfirmationStatus::Pending);
  CHECK(!stale_log.confirmed(kRequestId));
  CHECK(stale_log.answered(kRequestId));

  std::cout << "ok state_older_than_result_does_not_confirm\n";
  return 0;
}

int NewerRequestCancelsPending() {
  ConfirmationLog log;
  const ExecutionRequest first = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(first, kSendUs);
  const ExecutionResult first_result = MakeResult(first, kResultStampUs);
  log.ObserveResult(first_result, kResultRxUs);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Pending);

  // A stop is a new request: it withdraws the pending confirmation.
  const ExecutionRequest stop =
      MakeRequest(kSecondRequestId, Operation::ZeroTorque, SessionId());
  log.Track(stop, kSendUs + 1'000);

  CHECK(log.tracked().size() == 2);
  const TrackedRequest *withdrawn = log.Find(kRequestId);
  CHECK(withdrawn != nullptr);
  CHECK(withdrawn->status == ConfirmationStatus::Superseded);
  CHECK(withdrawn->order == ConfirmationOrder::None);
  const TrackedRequest *pending = log.Find(kSecondRequestId);
  CHECK(pending != nullptr);
  CHECK(pending->status == ConfirmationStatus::Pending);
  CHECK(pending->send_count == 1);
  CHECK(pending->first_send_us == kSendUs + 1'000);

  // The state that agrees with the withdrawn request cannot revive it.
  log.ObserveState(MakeState(first_result, first_result.stamp_us), kStateRxUs);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Superseded);
  CHECK(!log.confirmed(kRequestId));
  CHECK(log.Find(kSecondRequestId)->status == ConfirmationStatus::Pending);
  CHECK(log.states().size() == 1);

  std::cout << "ok newer_request_cancels_pending\n";
  return 0;
}

int NewEpochSupersedesLateApplied() {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);
  const ExecutionResult result = MakeResult(request, kResultStampUs);
  log.ObserveResult(result, kResultRxUs);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Pending);

  // The first state at or after the answer shows a newer epoch: the session
  // agreement is broken and stays broken.
  RobotState newer_epoch = MakeState(result, kResultStampUs + 100);
  newer_epoch.epoch = kEpoch + 1;
  log.ObserveState(newer_epoch, kResultStampUs + 120);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Superseded);

  log.ObserveState(MakeState(result, kResultStampUs), kStateRxUs);
  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Superseded);
  CHECK(entry->order == ConfirmationOrder::None);
  CHECK(entry->confirmed_session == kNoId);
  CHECK(!log.confirmed(kRequestId));
  CHECK(log.states().size() == 2);
  CHECK(log.states()[0].state.epoch == kEpoch + 1);
  CHECK(log.states()[1].state.epoch == kEpoch);

  std::cout << "ok new_epoch_supersedes_late_applied\n";
  return 0;
}

int MismatchedIdentityDoesNotConfirm() {
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);

  ExecutionResult other_runtime = MakeResult(request, kResultStampUs);
  other_runtime.runtime_id = MakeId(0x99);
  CHECK(MismatchedResultStaysPending(other_runtime) == 0);

  ExecutionResult other_supervisor = MakeResult(request, kResultStampUs);
  other_supervisor.supervisor_id = MakeId(0x98);
  CHECK(MismatchedResultStaysPending(other_supervisor) == 0);

  ExecutionResult other_session = MakeResult(request, kResultStampUs);
  other_session.session_id = OtherSessionId();
  CHECK(MismatchedResultStaysPending(other_session) == 0);

  ExecutionResult other_request = MakeResult(request, kResultStampUs);
  other_request.request_id = kOtherRequestId;
  CHECK(MismatchedResultStaysPending(other_request) == 0);

  // A bootstrap request has no session yet and accepts the one it is granted,
  // while a result carrying no session at all answers nothing.
  const ExecutionRequest acquire =
      MakeRequest(kSecondRequestId, Operation::AcquireSession, kNoId);
  ConfirmationLog bootstrap;
  bootstrap.Track(acquire, kSendUs);
  const ExecutionResult granted = MakeResult(acquire, kResultStampUs);
  bootstrap.ObserveResult(granted, kResultRxUs);
  bootstrap.ObserveState(MakeState(granted, granted.stamp_us), kStateRxUs);
  const TrackedRequest *granted_entry = bootstrap.Find(kSecondRequestId);
  CHECK(granted_entry != nullptr);
  CHECK(granted_entry->status == ConfirmationStatus::Confirmed);
  CHECK(granted_entry->confirmed_session == SessionId());
  CHECK(bootstrap.answered(kSecondRequestId));

  ConfirmationLog session_less;
  session_less.Track(acquire, kSendUs);
  ExecutionResult no_session = MakeResult(acquire, kResultStampUs);
  no_session.session_id = kNoId;
  session_less.ObserveResult(no_session, kResultRxUs);
  session_less.ObserveState(MakeState(no_session, no_session.stamp_us),
                            kStateRxUs);
  const TrackedRequest *session_less_entry = session_less.Find(kSecondRequestId);
  CHECK(session_less_entry != nullptr);
  CHECK(session_less_entry->status == ConfirmationStatus::Pending);
  CHECK(!session_less.confirmed(kSecondRequestId));
  CHECK(!session_less.answered(kSecondRequestId));
  CHECK(session_less.results().size() == 1);

  std::cout << "ok mismatched_identity_does_not_confirm\n";
  return 0;
}

int MismatchedExpectationDoesNotConfirm() {
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  ConfirmationExpectations expectations;
  expectations.execution_state = ExecutionState::Tracking;
  expectations.epoch = kEpoch;

  ConfirmationLog log;
  log.Track(request, kSendUs, expectations);
  const TrackedRequest *tracked = log.Find(kRequestId);
  CHECK(tracked != nullptr);
  CHECK(tracked->expectations.execution_state.has_value());
  CHECK(*tracked->expectations.epoch == kEpoch);

  ExecutionResult result = MakeResult(request, kResultStampUs);
  result.epoch = kEpoch + 1;
  log.ObserveResult(result, kResultRxUs);
  log.ObserveState(MakeState(result, result.stamp_us), kStateRxUs);

  const TrackedRequest *entry = log.Find(kRequestId);
  CHECK(entry != nullptr);
  CHECK(entry->status == ConfirmationStatus::Pending);
  CHECK(!log.confirmed(kRequestId));
  CHECK(log.answered(kRequestId));
  CHECK(log.results().size() == 1);
  CHECK(log.results()[0].result.epoch == kEpoch + 1);
  CHECK(log.states().size() == 1);

  // Control: the same expectation against a matching answer does confirm, so
  // the case above fails on the epoch and not on expectations being ignored.
  ConfirmationLog agreeing;
  agreeing.Track(request, kSendUs, expectations);
  const ExecutionResult matching = MakeResult(request, kResultStampUs);
  agreeing.ObserveResult(matching, kResultRxUs);
  agreeing.ObserveState(MakeState(matching, matching.stamp_us), kStateRxUs);
  CHECK(agreeing.Find(kRequestId)->status == ConfirmationStatus::Confirmed);

  std::cout << "ok mismatched_expectation_does_not_confirm\n";
  return 0;
}

int RetryKeepsPendingAndConfirmsWithStoredStamp() {
  ConfirmationLog log;
  const ExecutionRequest request = MakeRequest(kRequestId, Operation::Tracking);
  log.Track(request, kSendUs);

  const TimeUs retry_us = kSendUs + 2'000;
  log.Track(request, retry_us);
  CHECK(log.tracked().size() == 1);
  const TrackedRequest *retried = log.Find(kRequestId);
  CHECK(retried != nullptr);
  CHECK(retried->send_count == 2);
  CHECK(retried->first_send_us == kSendUs);
  CHECK(retried->last_send_us == retry_us);
  CHECK(retried->status == ConfirmationStatus::Pending);
  CHECK(retried->order == ConfirmationOrder::None);

  // The payload comparison ignores first_command.stamp_us, which the client
  // refreshes on every send: the third send is still the same request.
  ExecutionRequest restamped = request;
  restamped.first_command.stamp_us = kSendUs + 1'500;
  log.Track(restamped, retry_us + 10);
  CHECK(log.tracked().size() == 1);
  CHECK(log.Find(kRequestId)->send_count == 3);

  // The runtime replays the answer it stored for the first send.
  const ExecutionResult stored = MakeResult(request, kResultStampUs);
  log.ObserveResult(stored, kResultRxUs + 2'000);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Pending);
  CHECK(log.answered(kRequestId));
  CHECK(log.result_of(kRequestId) != nullptr);
  CHECK(log.result_of(kRequestId)->result.stamp_us == kResultStampUs);

  log.ObserveState(MakeState(stored, stored.stamp_us), kStateRxUs + 2'000);
  const TrackedRequest *confirmed = log.Find(kRequestId);
  CHECK(confirmed != nullptr);
  CHECK(confirmed->status == ConfirmationStatus::Confirmed);
  CHECK(confirmed->order == ConfirmationOrder::ResultFirst);
  CHECK(confirmed->confirmed_session == SessionId());
  CHECK(log.confirmed(kRequestId));
  CHECK(log.states().size() == 1);

  // A repeated answer stays in the log raw, and result_of reports the latest
  // matching observation.
  const ExecutionResult duplicate = MakeResult(request, kResultStampUs + 100);
  log.ObserveResult(duplicate, kResultRxUs + 2'100);
  CHECK(log.results().size() == 2);
  CHECK(log.results()[1].order == 3);
  CHECK(log.result_of(kRequestId) == &log.results()[1]);
  CHECK(log.result_of(kRequestId)->result.stamp_us == kResultStampUs + 100);
  CHECK(log.confirmed(kRequestId));

  // A further identical send leaves the confirmed entry alone.
  log.Track(request, kSendUs + 5'000);
  CHECK(log.tracked().size() == 1);
  CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Confirmed);
  CHECK(log.Find(kRequestId)->send_count == 4);

  // A newer request withdraws pending confirmations only: the proven entry
  // stays proven.
  const ExecutionRequest newer =
      MakeRequest(kSecondRequestId, Operation::ZeroTorque, SessionId());
  log.Track(newer, kSendUs + 6'000);
  CHECK(log.tracked().size() == 2);
  const TrackedRequest *proven = log.Find(kRequestId);
  CHECK(proven != nullptr);
  CHECK(proven->status == ConfirmationStatus::Confirmed);
  CHECK(proven->confirmed_session == SessionId());
  CHECK(log.confirmed(kRequestId));
  CHECK(log.Find(kSecondRequestId)->status == ConfirmationStatus::Pending);

  // The same request id with a changed payload is a different request: the
  // pending entry is withdrawn and a fresh one replaces it.
  ConfirmationLog conflicting_log;
  const ExecutionRequest stop =
      MakeRequest(kOtherRequestId, Operation::ZeroTorque, SessionId());
  conflicting_log.Track(stop, kSendUs);
  ExecutionRequest changed = stop;
  changed.proposed_epoch = kEpoch;
  conflicting_log.Track(changed, kSendUs + 1'000);
  CHECK(conflicting_log.tracked().size() == 2);
  CHECK(conflicting_log.tracked()[0].status == ConfirmationStatus::Superseded);
  CHECK(conflicting_log.tracked()[0].send_count == 1);
  const TrackedRequest *replacement = conflicting_log.Find(kOtherRequestId);
  CHECK(replacement == &conflicting_log.tracked()[1]);
  CHECK(replacement->status == ConfirmationStatus::Pending);
  CHECK(replacement->send_count == 1);
  CHECK(replacement->first_send_us == kSendUs + 1'000);
  CHECK(replacement->request.proposed_epoch == kEpoch);

  // The payload comparison covers the whole fixed-size joint arrays, not only
  // the entries below joints.count.
  ConfirmationLog payload_log;
  payload_log.Track(request, kSendUs);
  ExecutionRequest beyond_count = request;
  beyond_count.first_command.joints.q[kJointCount] = 1.0;
  payload_log.Track(beyond_count, kSendUs + 1'000);
  CHECK(payload_log.tracked().size() == 2);
  CHECK(payload_log.tracked()[0].status == ConfirmationStatus::Superseded);
  CHECK(payload_log.tracked()[1].status == ConfirmationStatus::Pending);
  CHECK(payload_log.tracked()[1].send_count == 1);

  // The same request id for a different operation is a different request.
  ConfirmationLog operation_log;
  operation_log.Track(request, kSendUs);
  ExecutionRequest other_operation = request;
  other_operation.operation = Operation::ZeroTorque;
  operation_log.Track(other_operation, kSendUs + 1'000);
  CHECK(operation_log.tracked().size() == 2);
  CHECK(operation_log.tracked()[0].status == ConfirmationStatus::Superseded);
  CHECK(operation_log.tracked()[1].status == ConfirmationStatus::Pending);
  CHECK(operation_log.tracked()[1].send_count == 1);

  std::cout << "ok retry_keeps_pending_and_confirms_with_stored_stamp\n";
  return 0;
}


// Every clause of the agreement between an answer and the state it publishes has
// to be able to refuse a confirmation on its own: a witness that differs in
// exactly one field is what pins one clause down, and the untouched witness is
// the control that shows the case fails on that clause alone.
int WitnessMismatchDoesNotConfirm() {
  struct WitnessCase {
    const char *field;
    void (*mutate)(RobotState &state);
  };
  const WitnessCase cases[] = {
      {"runtime_id", [](RobotState &state) { state.runtime_id = MakeId(0x99); }},
      {"session_id",
       [](RobotState &state) { state.session_id = OtherSessionId(); }},
      {"producer_id", [](RobotState &state) { state.producer_id = MakeId(0x77); }},
      {"output_action",
       [](RobotState &state) { state.output_action = OutputAction::Damping; }},
  };
  for (const WitnessCase &witness_case : cases) {
    ConfirmationLog log;
    const ExecutionRequest request =
        MakeRequest(kRequestId, Operation::Tracking);
    ConfirmationExpectations expectations;
    expectations.execution_state = ExecutionState::Tracking;
    expectations.output_action = OutputAction::Tracking;
    expectations.producer_id = ProducerId();
    expectations.session_id = SessionId();
    log.Track(request, kSendUs, expectations);
    const ExecutionResult result = MakeResult(request, kResultStampUs);
    log.ObserveResult(result, kResultRxUs);
    CHECK(log.Find(kRequestId)->status == ConfirmationStatus::Pending);

    RobotState witness = MakeState(result, result.stamp_us);
    witness_case.mutate(witness);
    log.ObserveState(witness, kStateRxUs);

    const TrackedRequest *entry = log.Find(kRequestId);
    CHECK(entry != nullptr);
    if (entry->status != ConfirmationStatus::Superseded) {
      std::cerr << __FILE__ << ':' << __LINE__
                << ": a witness with a different " << witness_case.field
                << " confirmed the request\n";
      return 1;
    }
    CHECK(!log.confirmed(kRequestId));
    CHECK(entry->order == ConfirmationOrder::None);
    CHECK(log.states().size() == 1);

    ConfirmationLog agreeing;
    agreeing.Track(request, kSendUs, expectations);
    agreeing.ObserveResult(result, kResultRxUs);
    agreeing.ObserveState(MakeState(result, result.stamp_us), kStateRxUs);
    CHECK(agreeing.Find(kRequestId)->status == ConfirmationStatus::Confirmed);
  }

  std::cout << "ok witness_mismatch_does_not_confirm\n";
  return 0;
}

// Same for the expectations: each clause has to keep a request pending against an
// answer that would otherwise confirm it, and the matching answer is the control.
int ExpectationMismatchDoesNotConfirm() {
  struct ExpectationCase {
    const char *field;
    void (*set)(ConfirmationExpectations &expectations);
  };
  const ExpectationCase cases[] = {
      {"execution_state",
       [](ConfirmationExpectations &expectations) {
         expectations.execution_state = ExecutionState::Damping;
       }},
      {"output_action",
       [](ConfirmationExpectations &expectations) {
         expectations.output_action = OutputAction::Damping;
       }},
      {"producer_id",
       [](ConfirmationExpectations &expectations) {
         expectations.producer_id = MakeId(0x77);
       }},
      {"session_id",
       [](ConfirmationExpectations &expectations) {
         expectations.session_id = OtherSessionId();
       }},
      {"epoch",
       [](ConfirmationExpectations &expectations) {
         expectations.epoch = kEpoch + 1;
       }},
  };
  for (const ExpectationCase &expectation_case : cases) {
    const ExecutionRequest request =
        MakeRequest(kRequestId, Operation::Tracking);
    const ExecutionResult result = MakeResult(request, kResultStampUs);

    ConfirmationExpectations expectations;
    expectation_case.set(expectations);
    ConfirmationLog log;
    log.Track(request, kSendUs, expectations);
    log.ObserveResult(result, kResultRxUs);
    log.ObserveState(MakeState(result, result.stamp_us), kStateRxUs);
    const TrackedRequest *entry = log.Find(kRequestId);
    CHECK(entry != nullptr);
    if (entry->status != ConfirmationStatus::Pending) {
      std::cerr << __FILE__ << ':' << __LINE__ << ": expectation "
                << expectation_case.field
                << " did not keep the request pending\n";
      return 1;
    }
    CHECK(!log.confirmed(kRequestId));
    CHECK(log.answered(kRequestId));

    ConfirmationLog agreeing;
    agreeing.Track(request, kSendUs);
    agreeing.ObserveResult(result, kResultRxUs);
    agreeing.ObserveState(MakeState(result, result.stamp_us), kStateRxUs);
    CHECK(agreeing.Find(kRequestId)->status == ConfirmationStatus::Confirmed);
  }

  std::cout << "ok expectation_mismatch_does_not_confirm\n";
  return 0;
}

}  // namespace

int main() {
  CHECK(ResultThenStateConfirms() == 0);
  CHECK(StateThenResultConfirms() == 0);
  CHECK(FaultBeforeLateAppliedDoesNotConfirm() == 0);
  CHECK(StateOlderThanResultDoesNotConfirm() == 0);
  CHECK(NewerRequestCancelsPending() == 0);
  CHECK(NewEpochSupersedesLateApplied() == 0);
  CHECK(MismatchedIdentityDoesNotConfirm() == 0);
  CHECK(MismatchedExpectationDoesNotConfirm() == 0);
  CHECK(WitnessMismatchDoesNotConfirm() == 0);
  CHECK(ExpectationMismatchDoesNotConfirm() == 0);
  CHECK(RetryKeepsPendingAndConfirmsWithStoredStamp() == 0);
  return 0;
}
