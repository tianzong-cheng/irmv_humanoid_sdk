// The producer supervisor: which producer is selected, what a preparation
// result does to that selection, and which candidate may be offered to the
// behaviour as the first frame of an authorization.
//
// It is a pure decision component, so every rule is asserted here without DDS:
// the association key of a preparation, a stale reply of another process or
// another session, a producer that commands another layout, a frame that is not
// one this deployment can send, a replacement that drops the previous
// producer's work, and the notices both directions carry.
#include "irmv/application/producer_supervisor.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/producer.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::ProducerOutcome;
using irmv::application::ProducerStop;
using irmv::application::ProducerSupervisor;
using irmv::configuration::ApplicationConfig;
using irmv::configuration::LoadApplicationConfig;
using irmv::protocol::AuthorizationNotice;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::PrepareRequest;
using irmv::protocol::PrepareResult;
using irmv::protocol::ProducerAnnouncement;
using irmv::protocol::ProducerFailureReason;
using irmv::protocol::ProducerStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::StopNotice;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::protocol::VelocityIntentMessage;

constexpr TimeUs kNowUs = 1000;

Id MakeId(std::uint8_t seed) {
  Id id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

const ApplicationConfig &Config() {
  static const ApplicationConfig config =
      LoadApplicationConfig("config/sdk.json");
  return config;
}

ProducerAnnouncement MakeAnnouncement(const ApplicationConfig &config,
                                      std::uint8_t seed) {
  ProducerAnnouncement announcement;
  announcement.producer_process_id = MakeId(seed);
  announcement.runtime_id = MakeId(0x60);
  announcement.layout_id.Assign(config.layout_id());
  announcement.target_period_us = 20000;
  announcement.reads_robot_state = true;
  announcement.status = ProducerStatus::Ready;
  return announcement;
}

RobotState MakeState(Sequence state_seq) {
  RobotState state;
  state.state_seq = state_seq;
  state.stamp_us = kNowUs;
  state.feedback.count = static_cast<std::uint32_t>(Config().joint_count());
  state.feedback.imu.valid = true;
  return state;
}

JointCommand MakeFrame(const ApplicationConfig &config,
                       Sequence source_state_seq) {
  JointCommand frame;
  frame.layout_id.Assign(config.layout_id());
  frame.source_state_seq = source_state_seq;
  frame.joints.count = static_cast<std::uint32_t>(config.joint_count());
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    frame.joints.q[index] = 0.01 * index;
    frame.joints.dq[index] = 0.0;
    frame.joints.tau_ff[index] = 0.0;
    frame.joints.kp[index] = 30.0;
    frame.joints.kd[index] = 2.0;
  }
  return frame;
}

VelocityIntentMessage MakeIntent() {
  VelocityIntentMessage intent;
  intent.producer_process_id = MakeId(0x01);
  intent.seq = 1;
  intent.forward = 0.3;
  intent.valid_for_us = 500000;
  return intent;
}

PrepareResult MakeResult(const ProducerAnnouncement &announcement,
                         const PrepareRequest &request,
                         Sequence source_state_seq) {
  PrepareResult result;
  result.producer_process_id = announcement.producer_process_id;
  result.prepare_id = request.prepare_id;
  result.runtime_id = request.runtime_id;
  result.session_id = request.session_id;
  result.producer_id = request.producer_id;
  result.status = ProducerStatus::Ready;
  result.first_command = MakeFrame(Config(), source_state_seq);
  return result;
}

// Selects one producer and opens one preparation. Both steps must succeed; the
// caller checks the request it gets back.
bool PrepareOne(ProducerSupervisor *supervisor,
                const ProducerAnnouncement &announcement,
                PrepareRequest *request) {
  if (!supervisor->Select(announcement)) {
    return false;
  }
  return supervisor->Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                             1000000, request);
}

}  // namespace

// A producer is accepted only when the deployment can use it at all.
int Selection() {
  const ApplicationConfig &config = Config();
  ProducerSupervisor supervisor(config);

  ProducerAnnouncement announcement = MakeAnnouncement(config, 0x01);
  CHECK(supervisor.Select(announcement));
  CHECK(supervisor.selected());
  CHECK(supervisor.producer_process_id() == announcement.producer_process_id);
  // The producer is authorized under its own process identity.
  CHECK(supervisor.producer_id() == announcement.producer_process_id);
  CHECK(supervisor.failure() == ProducerStop::None);

  // Another protocol version describes an interface this build cannot speak.
  ProducerAnnouncement wrong_version = MakeAnnouncement(config, 0x02);
  wrong_version.protocol_version = 2;
  CHECK(!supervisor.Select(wrong_version));
  CHECK(supervisor.failure() == ProducerStop::Incompatible);

  // A producer that reports itself failed is not a candidate.
  ProducerAnnouncement failed = MakeAnnouncement(config, 0x03);
  failed.status = ProducerStatus::Failed;
  failed.reason = ProducerFailureReason::Internal;
  CHECK(!supervisor.Select(failed));

  // An empty process identity could not be told apart from another producer.
  ProducerAnnouncement anonymous = MakeAnnouncement(config, 0x04);
  anonymous.producer_process_id = irmv::protocol::kNoId;
  CHECK(!supervisor.Select(anonymous));

  // Another layout would produce frames this deployment's runtime refuses.
  ProducerAnnouncement other_layout = MakeAnnouncement(config, 0x05);
  other_layout.layout_id.Assign("another-layout");
  CHECK(!supervisor.Select(other_layout));

  // A producer slower than the runtime's own target deadline could never keep a
  // context alive.
  ProducerAnnouncement slow = MakeAnnouncement(config, 0x06);
  slow.target_period_us = config.runtime.timeouts.command_us;
  CHECK(!supervisor.Select(slow));
  slow.target_period_us = 0;
  CHECK(!supervisor.Select(slow));

  // The first selection is still the selected one: a refused announcement
  // changes nothing.
  CHECK(supervisor.selected());
  CHECK(supervisor.producer_process_id() == announcement.producer_process_id);

  // A renewal by the selected producer - the same process announcing itself
  // again while it waits to be prepared - is that same selection: the
  // preparation in flight and the candidate it produced survive it. A producer
  // that renews on its own cadence would otherwise never be prepared, because
  // every renewal would drop what it was waiting on.
  ProducerSupervisor renewing(config);
  PrepareRequest prepared;
  CHECK(PrepareOne(&renewing, announcement, &prepared));
  CHECK(renewing.preparing());
  CHECK(renewing.Select(announcement));
  CHECK(renewing.preparing());
  CHECK(renewing.prepare_id() == prepared.prepare_id);
  CHECK(renewing.producer_process_id() == announcement.producer_process_id);
  const RobotState state = MakeState(10);
  CHECK(renewing.Observe(MakeResult(announcement, prepared, 9), state,
                         kNowUs) == ProducerOutcome::Candidate);
  CHECK(renewing.Select(announcement));
  CHECK(renewing.have_candidate());
  CHECK(renewing.candidate().source_state_seq == 9);
  // ... and it is not a replacement: no stop notice is owed for it.
  StopNotice renewal_notice;
  CHECK(!renewing.TakeStopNotice(&renewal_notice));
  return 0;
}

// One preparation at a time, with a number that never repeats and an answer
// that is only looked at when it belongs to it.
int Preparation() {
  const ApplicationConfig &config = Config();
  ProducerSupervisor supervisor(config);
  const ProducerAnnouncement announcement = MakeAnnouncement(config, 0x01);

  // Nothing can be prepared before a producer is selected.
  PrepareRequest request;
  CHECK(!supervisor.Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                            1000000, &request));

  CHECK(PrepareOne(&supervisor, announcement, &request));
  CHECK(request.prepare_id != 0);
  CHECK(request.producer_process_id == announcement.producer_process_id);
  CHECK(request.producer_id == announcement.producer_process_id);
  CHECK(request.epoch == 0);
  CHECK(request.target_period_us == announcement.target_period_us);
  CHECK(request.intent.seq == 1);
  CHECK(supervisor.preparing());

  // A second preparation while the first is outstanding is refused.
  PrepareRequest second;
  CHECK(!supervisor.Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                            1000000, &second));

  // A reply of another process, another preparation or another session is not
  // an answer to this one, and it changes nothing at all.
  const RobotState state = MakeState(10);
  PrepareResult stranger = MakeResult(announcement, request, 9);
  stranger.producer_process_id = MakeId(0x40);
  CHECK(supervisor.Observe(stranger, state, kNowUs) ==
        ProducerOutcome::Ignored);
  CHECK(supervisor.preparing());

  PrepareResult other_preparation = MakeResult(announcement, request, 9);
  other_preparation.prepare_id = request.prepare_id + 5;
  CHECK(supervisor.Observe(other_preparation, state, kNowUs) ==
        ProducerOutcome::Ignored);
  CHECK(supervisor.preparing());

  PrepareResult other_session = MakeResult(announcement, request, 9);
  other_session.session_id = MakeId(0x50);
  CHECK(supervisor.Observe(other_session, state, kNowUs) ==
        ProducerOutcome::Ignored);
  CHECK(supervisor.preparing());

  // The right answer is accepted, and the frame is the candidate.
  const PrepareResult answer = MakeResult(announcement, request, 9);
  CHECK(supervisor.Observe(answer, state, kNowUs) ==
        ProducerOutcome::Candidate);
  CHECK(!supervisor.preparing());
  CHECK(supervisor.have_candidate());
  CHECK(supervisor.candidate().joints.count == config.joint_count());
  CHECK(supervisor.candidate().source_state_seq == 9);
  // The authorization is not the producer's to pin: the behaviour stamps the
  // producer, the epoch and the sequence when it proposes the request.
  CHECK(supervisor.candidate().producer_id == announcement.producer_process_id);
  CHECK(supervisor.candidate().epoch == 0);
  CHECK(supervisor.candidate().command_seq == 0);

  // The next preparation gets a number this one never had.
  PrepareRequest next;
  CHECK(supervisor.Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                           1000000, &next));
  CHECK(next.prepare_id != request.prepare_id);
  CHECK(next.prepare_id > request.prepare_id);

  // A preparation that is not answered inside its deadline expires, once, and
  // the producer stays registered.
  CHECK(!supervisor.ExpirePreparation(kNowUs + 1000000 - 1));
  CHECK(supervisor.ExpirePreparation(kNowUs + 1000000));
  CHECK(!supervisor.ExpirePreparation(kNowUs + 1000000));
  CHECK(!supervisor.preparing());
  CHECK(supervisor.failure() == ProducerStop::PrepareTimeout);
  CHECK(supervisor.selected());

  // The answer of an expired preparation is refused by Observe itself, without
  // anyone having expired it first: the frame it carries was produced for a
  // state this deployment has long left, so it is not a candidate.
  PrepareRequest stale;
  CHECK(supervisor.Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                           1000000, &stale));
  const PrepareResult late = MakeResult(announcement, stale, 9);
  CHECK(supervisor.Observe(late, state, kNowUs + 1000000) ==
        ProducerOutcome::Failed);
  CHECK(!supervisor.preparing());
  CHECK(!supervisor.have_candidate());
  CHECK(supervisor.failure() == ProducerStop::PrepareTimeout);
  // ... and a second copy of it changes nothing, because there is no
  // preparation left to answer.
  CHECK(supervisor.Observe(late, state, kNowUs + 1000000 + 1) ==
        ProducerOutcome::Ignored);

  // The stop notice is handed over once and only once. Nothing above queued
  // one, so the revocation that owes it is what this checks.
  StopNotice notice;
  CHECK(!supervisor.TakeStopNotice(nullptr));
  CHECK(!supervisor.have_stop_notice());
  static_cast<void>(supervisor.Revoke(ProducerStop::Stopped));
  CHECK(supervisor.have_stop_notice());
  CHECK(supervisor.TakeStopNotice(&notice));
  CHECK(!supervisor.have_stop_notice());
  CHECK(!supervisor.TakeStopNotice(&notice));
  CHECK(notice.producer_id == announcement.producer_process_id);
  return 0;
}

// A frame this deployment cannot send is a failed preparation, never a
// candidate.
int Usability() {
  const ApplicationConfig &config = Config();
  ProducerSupervisor supervisor(config);
  const ProducerAnnouncement announcement = MakeAnnouncement(config, 0x01);
  PrepareRequest request;
  CHECK(PrepareOne(&supervisor, announcement, &request));
  const RobotState state = MakeState(10);

  PrepareResult refused = MakeResult(announcement, request, 9);
  refused.status = ProducerStatus::Failed;
  refused.reason = ProducerFailureReason::PrepareFailed;
  CHECK(supervisor.Observe(refused, state, kNowUs) == ProducerOutcome::Failed);
  CHECK(!supervisor.have_candidate());
  CHECK(supervisor.failure() == ProducerStop::PrepareFailed);

  // A frame generated from a state this deployment never published.
  PrepareRequest second;
  CHECK(PrepareOne(&supervisor, announcement, &second));
  PrepareResult ahead = MakeResult(announcement, second, state.state_seq + 1);
  CHECK(supervisor.Observe(ahead, state, kNowUs) == ProducerOutcome::Failed);

  // A frame attached to no state at all.
  PrepareRequest third;
  CHECK(PrepareOne(&supervisor, announcement, &third));
  PrepareResult unattached = MakeResult(announcement, third, 0);
  CHECK(supervisor.Observe(unattached, state, kNowUs) ==
        ProducerOutcome::Failed);

  // A frame of another joint count.
  PrepareRequest fourth;
  CHECK(PrepareOne(&supervisor, announcement, &fourth));
  PrepareResult wrong_count = MakeResult(announcement, fourth, 9);
  wrong_count.first_command.joints.count = config.joint_count() + 1;
  CHECK(supervisor.Observe(wrong_count, state, kNowUs) ==
        ProducerOutcome::Failed);

  // A frame of another layout.
  PrepareRequest fifth;
  CHECK(PrepareOne(&supervisor, announcement, &fifth));
  PrepareResult wrong_layout = MakeResult(announcement, fifth, 9);
  wrong_layout.first_command.layout_id.Assign("another-layout");
  CHECK(supervisor.Observe(wrong_layout, state, kNowUs) ==
        ProducerOutcome::Failed);

  // A frame with a value that is not finite, and one with a negative gain.
  PrepareRequest sixth;
  CHECK(PrepareOne(&supervisor, announcement, &sixth));
  PrepareResult non_finite = MakeResult(announcement, sixth, 9);
  non_finite.first_command.joints.q[0] =
      std::numeric_limits<double>::quiet_NaN();
  CHECK(supervisor.Observe(non_finite, state, kNowUs) ==
        ProducerOutcome::Failed);

  PrepareRequest seventh;
  CHECK(PrepareOne(&supervisor, announcement, &seventh));
  PrepareResult negative_gain = MakeResult(announcement, seventh, 9);
  negative_gain.first_command.joints.kd[0] = -1.0;
  CHECK(supervisor.Observe(negative_gain, state, kNowUs) ==
        ProducerOutcome::Failed);

  // None of them left a candidate behind.
  CHECK(!supervisor.have_candidate());
  return 0;
}

// A second producer replaces the first: the replaced one is told to stop, and
// nothing of its work survives.
//
// Whether a replacement may be offered at all is the caller's decision - the
// supervision loop takes a registration only while the application is idle in
// Ready with nothing armed, and drops the candidate the behaviour held for the
// replaced producer - so this pins what the supervisor itself owes the caller:
// the new selection, the stop notice for the old one, and no survivor of the
// old one's preparation or candidate.
int Replacement() {
  const ApplicationConfig &config = Config();
  ProducerSupervisor supervisor(config);
  const ProducerAnnouncement first = MakeAnnouncement(config, 0x01);
  PrepareRequest request;
  CHECK(PrepareOne(&supervisor, first, &request));
  const RobotState state = MakeState(10);
  const PrepareResult answer = MakeResult(first, request, 9);
  CHECK(supervisor.Observe(answer, state, kNowUs) ==
        ProducerOutcome::Candidate);
  CHECK(supervisor.have_candidate());

  const ProducerAnnouncement second = MakeAnnouncement(config, 0x02);
  CHECK(supervisor.Select(second));
  CHECK(supervisor.failure() == ProducerStop::Replaced);
  CHECK(supervisor.producer_process_id() == second.producer_process_id);
  // The candidate of the replaced producer is gone, and so is its preparation.
  CHECK(!supervisor.have_candidate());
  CHECK(!supervisor.preparing());
  // The replaced producer is told to stop.
  CHECK(supervisor.have_stop_notice());
  CHECK(supervisor.stop_notice().producer_process_id ==
        first.producer_process_id);

  // A result of the replaced producer is no longer associated with anything.
  const PrepareResult late = MakeResult(first, request, 9);
  CHECK(supervisor.Observe(late, state, kNowUs) == ProducerOutcome::Ignored);
  CHECK(!supervisor.have_candidate());
  return 0;
}

// The two notices the application sends, and what a revocation leaves behind.
int Notices() {
  const ApplicationConfig &config = Config();
  ProducerSupervisor supervisor(config);
  const ProducerAnnouncement announcement = MakeAnnouncement(config, 0x01);
  PrepareRequest request;
  CHECK(PrepareOne(&supervisor, announcement, &request));

  const AuthorizationNotice notice = supervisor.Authorized(
      MakeId(0x70), MakeId(0x71), announcement.producer_process_id, 42, 7);
  CHECK(notice.producer_process_id == announcement.producer_process_id);
  CHECK(notice.prepare_id == request.prepare_id);
  CHECK(notice.runtime_id == MakeId(0x70));
  CHECK(notice.session_id == MakeId(0x71));
  CHECK(notice.producer_id == announcement.producer_process_id);
  CHECK(notice.epoch == 42);
  CHECK(notice.first_command_seq == 7);

  // A revocation ends the preparation and the candidate and names the context
  // it ends.
  const StopNotice stop = supervisor.Revoke(ProducerStop::Stopped);
  CHECK(stop.producer_process_id == announcement.producer_process_id);
  CHECK(stop.epoch == 42);
  CHECK(!supervisor.preparing());
  CHECK(!supervisor.have_candidate());

  // The producer stays registered: it may be prepared again for the next
  // authorization, with a new preparation number.
  PrepareRequest again;
  CHECK(supervisor.Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                           1000000, &again));
  CHECK(again.prepare_id != request.prepare_id);

  // Forgetting drops the producer: a restarted process must register again.
  supervisor.Forget();
  CHECK(!supervisor.selected());
  PrepareRequest forgotten;
  CHECK(!supervisor.Prepare(MakeId(0x70), MakeId(0x71), MakeIntent(), kNowUs,
                            1000000, &forgotten));
  return 0;
}

int main() {
  int failures = 0;
  failures += Selection();
  failures += Preparation();
  failures += Usability();
  failures += Replacement();
  failures += Notices();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
