#include "irmv/application/producer_supervisor.hpp"

#include <cmath>
#include <cstddef>

namespace irmv::application {
namespace {

bool SameId(const protocol::Id &left, const protocol::Id &right) {
  return left == right;
}

bool IsEmpty(const protocol::Id &id) { return SameId(id, protocol::kNoId); }

// One complete, usable frame: every joint of this deployment, in its layout,
// with finite values and non-negative gains. The runtime applies its own
// continuity and limit guards on top; this is the shape a candidate must have
// before the application is willing to offer it as the first frame of an
// authorization.
bool UsableFrame(const protocol::JointCommand &frame,
                 const configuration::ApplicationConfig &config) {
  if (frame.joints.count != config.joint_count()) {
    return false;
  }
  if (frame.layout_id.View() != config.layout_id()) {
    return false;
  }
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    const double values[] = {frame.joints.q[index], frame.joints.dq[index],
                             frame.joints.tau_ff[index], frame.joints.kp[index],
                             frame.joints.kd[index]};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    if (frame.joints.kp[index] < 0.0 || frame.joints.kd[index] < 0.0) {
      return false;
    }
  }
  return true;
}

}  // namespace

ProducerSupervisor::ProducerSupervisor(
    const configuration::ApplicationConfig &config)
    : config_(config) {}

bool ProducerSupervisor::Select(
    const protocol::ProducerAnnouncement &announcement) {
  if (announcement.protocol_version != protocol::kProducerProtocolVersion) {
    failure_ = ProducerStop::Incompatible;
    return false;
  }
  if (announcement.status != protocol::ProducerStatus::Ready ||
      announcement.reason != protocol::ProducerFailureReason::None) {
    failure_ = ProducerStop::Incompatible;
    return false;
  }
  if (IsEmpty(announcement.producer_process_id)) {
    // A producer without a process identity cannot be told apart from another
    // one, and its replies could not be associated with it.
    failure_ = ProducerStop::Incompatible;
    return false;
  }
  if (announcement.layout_id.View() != config_.layout_id()) {
    // A producer that commands another layout would produce frames this
    // deployment's runtime refuses; it is not a candidate at all.
    failure_ = ProducerStop::Incompatible;
    return false;
  }
  if (announcement.target_period_us == 0 ||
      announcement.target_period_us >= config_.runtime.timeouts.command_us) {
    // A producer slower than the runtime's own target deadline could never keep
    // a context alive: every frame would arrive after the one before it
    // expired.
    failure_ = ProducerStop::Incompatible;
    return false;
  }

  if (have_producer_ &&
      SameId(producer_process_id_, announcement.producer_process_id)) {
    // A producer that announces itself again - a renewal while it waits for a
    // preparation, or for an authorization - is the producer already selected.
    // The announcement is not a new registration: rebuilding the selection here
    // would drop the preparation or the candidate this producer is waiting on,
    // and a producer that renews on its own schedule would then never be
    // prepared at all.
    failure_ = ProducerStop::None;
    return true;
  }

  const bool replacing = have_producer_;
  if (replacing) {
    // The process that is being replaced is told to stop, and its preparation
    // and candidate belong to it and are dropped: nothing of a producer that is
    // no longer selected may reach the machine.
    stop_notice_ = protocol::StopNotice{};
    stop_notice_.producer_process_id = producer_process_id_;
    stop_notice_.producer_id = producer_id_;
    stop_notice_.epoch = authorized_epoch_;
    stop_notice_.reason = protocol::StopReason::None;
    stop_notice_pending_ = true;
    failure_ = ProducerStop::Replaced;
  } else {
    failure_ = ProducerStop::None;
  }

  have_producer_ = true;
  producer_process_id_ = announcement.producer_process_id;
  producer_id_ = announcement.producer_process_id;
  layout_id_ = announcement.layout_id;
  target_period_us_ = announcement.target_period_us;
  preparation_ = Preparation{};
  have_candidate_ = false;
  candidate_ = protocol::JointCommand{};
  return true;
}

bool ProducerSupervisor::Prepare(protocol::Id runtime_id,
                                 protocol::Id session_id,
                                 const protocol::VelocityIntentMessage &intent,
                                 protocol::TimeUs now_us,
                                 protocol::TimeUs prepare_timeout_us,
                                 protocol::PrepareRequest *request) {
  if (request == nullptr || !have_producer_ || preparation_.outstanding) {
    return false;
  }
  // The preparation number never repeats within one registration: a stale reply
  // of an earlier preparation can therefore never be mistaken for an answer to
  // this one. Zero is not a number a preparation is ever given.
  if (next_prepare_id_ == 0) {
    next_prepare_id_ = 1;
  }
  protocol::PrepareRequest prepared;
  prepared.producer_process_id = producer_process_id_;
  prepared.prepare_id = next_prepare_id_++;
  prepared.runtime_id = runtime_id;
  prepared.session_id = session_id;
  prepared.producer_id = producer_id_;
  // The epoch is not known before the behaviour proposes one; the frame the
  // producer returns carries zero until the behaviour stamps the authorization
  // it asks for. What this request fixes is *which* producer, *which*
  // preparation and *which* session the answer belongs to.
  prepared.epoch = 0;
  prepared.target_period_us = target_period_us_;
  prepared.intent = intent;

  preparation_.outstanding = true;
  preparation_.id = prepared.prepare_id;
  preparation_.runtime_id = runtime_id;
  preparation_.session_id = session_id;
  preparation_.deadline_us = now_us + prepare_timeout_us;
  have_candidate_ = false;
  failure_ = ProducerStop::None;
  *request = prepared;
  return true;
}

ProducerOutcome ProducerSupervisor::Observe(
    const protocol::PrepareResult &result, const protocol::RobotState &state,
    protocol::TimeUs now_us) {
  if (!preparation_.outstanding) {
    return ProducerOutcome::Ignored;
  }
  // The association key, whole: a result of another process instance, another
  // preparation or another session belongs to another run and changes nothing
  // here.
  if (!SameId(result.producer_process_id, producer_process_id_) ||
      result.prepare_id != preparation_.id ||
      !SameId(result.runtime_id, preparation_.runtime_id) ||
      !SameId(result.session_id, preparation_.session_id)) {
    return ProducerOutcome::Ignored;
  }
  if (now_us >= preparation_.deadline_us) {
    // The answer arrived after the deadline the caller gave the preparation:
    // the frame it carries was produced for a state this deployment has long
    // left, so it is refused here rather than handed to a caller that would
    // have to remember to expire the preparation first. The result is not
    // dropped silently - the preparation ends with the reason its deadline
    // gives - and a later result of the same key finds no preparation at all.
    preparation_ = Preparation{};
    have_candidate_ = false;
    failure_ = ProducerStop::PrepareTimeout;
    return ProducerOutcome::Failed;
  }
  preparation_.outstanding = false;

  if (result.status != protocol::ProducerStatus::Ready) {
    failure_ = ProducerStop::PrepareFailed;
    have_candidate_ = false;
    return ProducerOutcome::Failed;
  }

  // The frame must have been generated from a state the application can see: a
  // source sequence ahead of the newest trusted state describes an observation
  // this deployment never published, and one that is zero was never attached to
  // one.
  if (result.first_command.source_state_seq == 0 ||
      result.first_command.source_state_seq > state.state_seq) {
    failure_ = ProducerStop::PrepareFailed;
    have_candidate_ = false;
    return ProducerOutcome::Failed;
  }
  if (!UsableFrame(result.first_command, config_)) {
    failure_ = ProducerStop::PrepareFailed;
    have_candidate_ = false;
    return ProducerOutcome::Failed;
  }

  candidate_ = result.first_command;
  // The authorization is not known yet: the behaviour stamps the producer, the
  // epoch and the command sequence when it proposes the Tracking request, so
  // the producer cannot pin an authorization the application never asked for.
  candidate_.producer_id = producer_id_;
  candidate_.epoch = 0;
  candidate_.command_seq = 0;
  candidate_.layout_id.Assign(config_.layout_id());
  have_candidate_ = true;
  failure_ = ProducerStop::None;
  return ProducerOutcome::Candidate;
}

bool ProducerSupervisor::ExpirePreparation(protocol::TimeUs now_us) {
  if (!preparation_.outstanding || now_us < preparation_.deadline_us) {
    return false;
  }
  preparation_ = Preparation{};
  have_candidate_ = false;
  failure_ = ProducerStop::PrepareTimeout;
  return true;
}

bool ProducerSupervisor::TakeStopNotice(protocol::StopNotice *notice) {
  if (notice == nullptr || !stop_notice_pending_) {
    return false;
  }
  *notice = stop_notice_;
  stop_notice_pending_ = false;
  return true;
}

protocol::AuthorizationNotice ProducerSupervisor::Authorized(
    protocol::Id runtime_id, protocol::Id session_id, protocol::Id producer_id,
    protocol::Sequence epoch, protocol::Sequence first_command_seq) {
  protocol::AuthorizationNotice notice;
  notice.producer_process_id = producer_process_id_;
  notice.prepare_id = preparation_.id;
  notice.runtime_id = runtime_id;
  notice.session_id = session_id;
  notice.producer_id = producer_id;
  notice.epoch = epoch;
  notice.first_command_seq = first_command_seq;
  authorized_epoch_ = epoch;
  return notice;
}

protocol::StopNotice ProducerSupervisor::Revoke(ProducerStop reason) {
  protocol::StopNotice notice;
  notice.producer_process_id = producer_process_id_;
  notice.producer_id = producer_id_;
  notice.epoch = authorized_epoch_;
  notice.reason = protocol::StopReason::None;
  preparation_ = Preparation{};
  have_candidate_ = false;
  if (reason != ProducerStop::Replaced) {
    // A replacement already queued its own stop notice for the old process;
    // this revocation must not overwrite it with a second one.
    stop_notice_ = notice;
    stop_notice_pending_ = have_producer_;
    failure_ = reason;
  }
  return notice;
}

void ProducerSupervisor::Forget() {
  have_producer_ = false;
  producer_process_id_ = protocol::kNoId;
  producer_id_ = protocol::kNoId;
  preparation_ = Preparation{};
  have_candidate_ = false;
  candidate_ = protocol::JointCommand{};
  failure_ = ProducerStop::None;
  stop_notice_pending_ = false;
  authorized_epoch_ = 0;
}

}  // namespace irmv::application
