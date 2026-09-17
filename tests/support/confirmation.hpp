#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "irmv/protocol/types.hpp"

namespace irmv::test {

// Values the confirmed outcome must show. Unset members are not checked.
struct ConfirmationExpectations {
  std::optional<protocol::ExecutionState> execution_state;
  std::optional<protocol::OutputAction> output_action;
  std::optional<protocol::Sequence> epoch;
  std::optional<protocol::Id> producer_id;
  std::optional<protocol::Id> session_id;  // when the session is known up front
};

enum class ConfirmationStatus { Pending, Confirmed, Superseded };

// Which observation arrived first and therefore decided the confirmation.
enum class ConfirmationOrder { None, StateFirst, ResultFirst };

struct TrackedRequest {
  protocol::ExecutionRequest request;
  ConfirmationExpectations expectations;
  std::size_t send_count{0};  // 1 for the first send, +1 per identical retry
  protocol::TimeUs first_send_us{0};
  protocol::TimeUs last_send_us{0};
  ConfirmationStatus status{ConfirmationStatus::Pending};
  ConfirmationOrder order{ConfirmationOrder::None};
  protocol::Id confirmed_session{protocol::kNoId};  // 0 while unconfirmed
};

struct ResultObservation {
  protocol::ExecutionResult result;
  protocol::TimeUs rx_us{0};
  std::size_t order{0};  // submission index across both observation kinds
};

struct StateObservation {
  protocol::RobotState state;
  protocol::TimeUs rx_us{0};
  std::size_t order{0};
};

// Test-only log of the requests a client sent and the runtime traffic it saw.
// An entry is only confirmed when an Applied result identifies the request and
// the first state published at or after that result agrees with it, so a latched
// fault, a revoked session, a newer epoch or another instance cannot be mistaken
// for an answer. Entries are never removed: confirmation and supersession are
// terminal, which keeps a stale answer from re-arming a withdrawn request.
class ConfirmationLog {
 public:
  void Track(const protocol::ExecutionRequest &request, protocol::TimeUs send_us,
             const ConfirmationExpectations &expectations = {}) {
    const std::size_t existing = FindIndex(request.request_id);
    if (existing != tracked_.size()) {
      if (SamePayload(tracked_[existing].request, request)) {
        TrackedRequest &entry = tracked_[existing];
        entry.send_count += 1;
        entry.last_send_us = send_us;
        return;
      }
      tracked_[existing].status = ConfirmationStatus::Superseded;
    }
    for (TrackedRequest &entry : tracked_) {
      if (entry.status == ConfirmationStatus::Pending) {
        entry.status = ConfirmationStatus::Superseded;
      }
    }
    TrackedRequest fresh;
    fresh.request = request;
    fresh.expectations = expectations;
    fresh.send_count = 1;
    fresh.first_send_us = send_us;
    fresh.last_send_us = send_us;
    tracked_.push_back(fresh);
  }

  void ObserveResult(const protocol::ExecutionResult &result,
                     protocol::TimeUs rx_us) {
    ResultObservation observation;
    observation.result = result;
    observation.rx_us = rx_us;
    observation.order = ++observations_;
    results_.push_back(observation);
    EvaluatePending();
  }

  void ObserveState(const protocol::RobotState &state, protocol::TimeUs rx_us) {
    StateObservation observation;
    observation.state = state;
    observation.rx_us = rx_us;
    observation.order = ++observations_;
    states_.push_back(observation);
    EvaluatePending();
  }

  const std::vector<TrackedRequest> &tracked() const { return tracked_; }
  const std::vector<ResultObservation> &results() const { return results_; }
  const std::vector<StateObservation> &states() const { return states_; }

  // Latest tracked entry with this request id; nullptr when never tracked.
  const TrackedRequest *Find(protocol::Sequence request_id) const {
    const std::size_t index = FindIndex(request_id);
    return index == tracked_.size() ? nullptr : &tracked_[index];
  }

  bool confirmed(protocol::Sequence request_id) const {
    const TrackedRequest *entry = Find(request_id);
    return entry != nullptr && entry->status == ConfirmationStatus::Confirmed;
  }

  // True when an identity-matching result was observed, whatever its status.
  bool answered(protocol::Sequence request_id) const {
    const TrackedRequest *entry = Find(request_id);
    return entry != nullptr && LatestResult(entry->request) != nullptr;
  }

  // Matching result observation of a request id, the latest one; nullptr when none.
  const ResultObservation *result_of(protocol::Sequence request_id) const {
    const TrackedRequest *entry = Find(request_id);
    return entry == nullptr ? nullptr : LatestResult(entry->request);
  }

 private:
  std::size_t FindIndex(protocol::Sequence request_id) const {
    for (std::size_t i = tracked_.size(); i > 0; --i) {
      if (tracked_[i - 1].request.request_id == request_id) {
        return i - 1;
      }
    }
    return tracked_.size();
  }

  // A result answers a tracked request only when it carries the same request
  // id, runtime and supervisor, and a non-zero session the request accepts: a
  // bootstrap request (no session yet) accepts the session it is granted.
  static bool SameIdentity(const protocol::ExecutionRequest &request,
                           const protocol::ExecutionResult &result) {
    if (result.request_id != request.request_id ||
        result.runtime_id != request.runtime_id ||
        result.supervisor_id != request.supervisor_id) {
      return false;
    }
    if (result.session_id == protocol::kNoId) {
      return false;
    }
    return request.session_id == protocol::kNoId ||
           result.session_id == request.session_id;
  }

  const ResultObservation *LatestResult(
      const protocol::ExecutionRequest &request) const {
    for (std::size_t i = results_.size(); i > 0; --i) {
      if (SameIdentity(request, results_[i - 1].result)) {
        return &results_[i - 1];
      }
    }
    return nullptr;
  }

  // The first state published at or after the answer; observations are appended
  // in arrival order, so the earliest qualifying state comes first.
  const StateObservation *WitnessState(protocol::TimeUs result_stamp_us) const {
    for (const StateObservation &observation : states_) {
      if (observation.state.stamp_us >= result_stamp_us) {
        return &observation;
      }
    }
    return nullptr;
  }

  static bool Agrees(const protocol::RobotState &state,
                     const protocol::ExecutionResult &result) {
    return state.runtime_id == result.runtime_id &&
           state.session_id == result.session_id &&
           state.execution_state == result.execution_state &&
           state.output_action == result.output_action &&
           state.epoch == result.epoch &&
           state.producer_id == result.producer_id;
  }

  static bool Holds(const ConfirmationExpectations &expectations,
                    const protocol::ExecutionResult &result) {
    if (expectations.execution_state.has_value() &&
        *expectations.execution_state != result.execution_state) {
      return false;
    }
    if (expectations.output_action.has_value() &&
        *expectations.output_action != result.output_action) {
      return false;
    }
    if (expectations.epoch.has_value() && *expectations.epoch != result.epoch) {
      return false;
    }
    if (expectations.producer_id.has_value() &&
        *expectations.producer_id != result.producer_id) {
      return false;
    }
    if (expectations.session_id.has_value() &&
        *expectations.session_id != result.session_id) {
      return false;
    }
    return true;
  }

  // Field by field: ExecutionRequest is not comparable as a whole, and a retry
  // is defined as the same operation and header, with the same command only for
  // tracking.
  static bool SamePayload(const protocol::ExecutionRequest &left,
                          const protocol::ExecutionRequest &right) {
    if (left.operation != right.operation ||
        left.supervisor_id != right.supervisor_id ||
        left.runtime_id != right.runtime_id ||
        left.session_id != right.session_id ||
        left.expected_epoch != right.expected_epoch ||
        left.proposed_epoch != right.proposed_epoch ||
        left.producer_id != right.producer_id ||
        left.fault_id != right.fault_id) {
      return false;
    }
    if (left.operation != protocol::Operation::Tracking) {
      return true;
    }
    return SameCommand(left.first_command, right.first_command);
  }

  static bool SameCommand(const protocol::JointCommand &left,
                          const protocol::JointCommand &right) {
    if (left.protocol_version != right.protocol_version ||
        left.runtime_id != right.runtime_id ||
        left.session_id != right.session_id ||
        left.producer_id != right.producer_id || left.epoch != right.epoch ||
        left.command_seq != right.command_seq ||
        left.source_state_seq != right.source_state_seq ||
        left.layout_id.View() != right.layout_id.View() ||
        left.joints.count != right.joints.count) {
      return false;
    }
    for (std::size_t i = 0; i < protocol::kMaxJoints; ++i) {
      if (left.joints.q[i] != right.joints.q[i] ||
          left.joints.dq[i] != right.joints.dq[i] ||
          left.joints.tau_ff[i] != right.joints.tau_ff[i] ||
          left.joints.kp[i] != right.joints.kp[i] ||
          left.joints.kd[i] != right.joints.kd[i]) {
        return false;
      }
    }
    return true;
  }

  void EvaluatePending() {
    for (TrackedRequest &entry : tracked_) {
      if (entry.status == ConfirmationStatus::Pending) {
        Evaluate(entry);
      }
    }
  }

  void Evaluate(TrackedRequest &entry) {
    const ResultObservation *result = LatestResult(entry.request);
    if (result == nullptr) {
      return;
    }
    // A stamp older than the first send or newer than the arrival cannot
    // answer this request; a Rejected answer speaks without confirming.
    if (result->result.status != protocol::ResultStatus::Applied ||
        result->result.stamp_us < entry.first_send_us ||
        result->result.stamp_us > result->rx_us) {
      return;
    }
    if (!Holds(entry.expectations, result->result)) {
      return;
    }
    const StateObservation *witness = WitnessState(result->result.stamp_us);
    if (witness == nullptr) {
      return;
    }
    if (!Agrees(witness->state, result->result)) {
      entry.status = ConfirmationStatus::Superseded;
      return;
    }
    entry.status = ConfirmationStatus::Confirmed;
    entry.confirmed_session = result->result.session_id;
    entry.order = witness->order < result->order ? ConfirmationOrder::StateFirst
                                                : ConfirmationOrder::ResultFirst;
  }

  std::vector<TrackedRequest> tracked_;
  std::vector<ResultObservation> results_;
  std::vector<StateObservation> states_;
  std::size_t observations_{0};
};

}  // namespace irmv::test
