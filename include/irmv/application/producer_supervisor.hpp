#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/producer.hpp"
#include "irmv/protocol/types.hpp"

// The application's side of the external producer selection: which producer is
// registered, what it was asked to prepare, what came back and whether the
// result may be offered to the behaviour as the first frame of a locomotion
// authorization.
//
// It is a pure function of the messages and the states it is handed - it reads
// no clock, allocates nothing, performs no I/O and never throws - so every rule
// about a stale reply, a second producer, a restarted process or a first frame
// that is not usable is testable without DDS, and the supervision loop is the
// only thing that touches the transport.
//
// The authority stays where it belongs: this component decides *which* producer
// the application would use and *what* candidate it produced. Nothing here
// grants execution - the runtime does, through the Tracking request the
// behaviour sends - and nothing here can keep a context alive after the runtime
// withdrew it.
namespace irmv::application {

// Why a producer stopped being the selected one. It is what the application
// reports and what the stop notice carries.
enum class ProducerStop {
  None = 0,
  // The application ended the context itself (a stop, a fault, the exit).
  Stopped,
  // The producer reported that it could not prepare.
  PrepareFailed,
  // The producer did not answer the preparation inside its deadline.
  PrepareTimeout,
  // Another producer registered and replaced this one.
  Replaced,
  // The producer's announcement is not one this deployment can use.
  Incompatible,
};

// What one preparation result did to the supervisor.
enum class ProducerOutcome {
  // Not for this preparation, or already superseded: nothing changed.
  Ignored,
  // A first frame this supervisor accepts as the candidate of the pending
  // preparation; the behaviour may offer it to a Tracking request.
  Candidate,
  // The producer refused, or its frame is not usable. The preparation is over.
  Failed,
};

class ProducerSupervisor {
 public:
  // `config` must outlive the supervisor.
  explicit ProducerSupervisor(const configuration::ApplicationConfig &config);

  ProducerSupervisor(const ProducerSupervisor &) = delete;
  ProducerSupervisor &operator=(const ProducerSupervisor &) = delete;

  // Registers one producer from its announcement. It is accepted when it
  // reports itself Ready, when its protocol version, layout and target period
  // are ones this deployment can use, and when it is not the producer already
  // selected. A second producer replaces the first: the replaced one's
  // preparation and candidate are dropped - a frame it produced belongs to a
  // process that is no longer selected - and the caller sends the stop notice
  // the supervisor returns.
  //
  // The producer is authorized under its own process identity, so the id the
  // frames carry is traceable to the process that sent them and no entropy is
  // needed to invent one.
  bool Select(const protocol::ProducerAnnouncement &announcement);

  // True when a producer is registered and no preparation is outstanding.
  bool selected() const { return have_producer_; }

  // Opens one preparation of `prepare_id` for the selected producer. The
  // request carries the identity the application authorizes under, the
  // preparation number and the intent the producer must prepare for. Refused
  // when no producer is selected, when a preparation is already outstanding, or
  // when the identifier would be zero or reused.
  bool Prepare(protocol::Id runtime_id, protocol::Id session_id,
               const protocol::VelocityIntentMessage &intent,
               protocol::TimeUs now_us, protocol::TimeUs prepare_timeout_us,
               protocol::PrepareRequest *request);

  // Consumes one preparation result. Only a result whose association key
  // matches the outstanding preparation is looked at at all; the frame it
  // carries is accepted only when the state it was built from is fresh, when
  // the frame has this deployment's layout and joint count, and when every
  // value in it is finite. Anything else fails the preparation rather than
  // producing a candidate.
  ProducerOutcome Observe(const protocol::PrepareResult &result,
                          const protocol::RobotState &state,
                          protocol::TimeUs now_us);

  // The candidate the last accepted result produced, in the order the behaviour
  // wants it: the frame to put into the Tracking request.
  const protocol::JointCommand &candidate() const { return candidate_; }
  bool have_candidate() const { return have_candidate_; }

  // Why the last preparation ended, if it ended badly. Cleared by the next
  // preparation.
  ProducerStop failure() const { return failure_; }
  // True when a preparation is outstanding.
  bool preparing() const { return preparation_.outstanding; }

  // Gives up on an outstanding preparation that was not answered inside its
  // deadline. Returns true when it just expired, so the caller can report the
  // cause once.
  bool Expired(protocol::TimeUs now_us) const;

  // The notice to send once the runtime applied the context the candidate was
  // offered for. It carries the preparation the candidate belongs to and the
  // identity the runtime granted, so a producer can never mistake an
  // authorization for another one.
  protocol::AuthorizationNotice Authorized(
      protocol::Id runtime_id, protocol::Id session_id,
      protocol::Id producer_id, protocol::Sequence epoch,
      protocol::Sequence first_command_seq);

  // Ends the preparation and the candidate, and returns the notice to send. The
  // producer stays registered: the application may prepare it again for the
  // next authorization. A stop notice with no producer is not sent.
  protocol::StopNotice Revoke(ProducerStop reason);
  bool have_stop_notice() const { return stop_notice_pending_; }
  const protocol::StopNotice &stop_notice() const { return stop_notice_; }

  // Forgets the selected producer as well: a producer that restarts must
  // register, prepare and be authorized again from the beginning.
  void Forget();

  const protocol::Id &producer_process_id() const {
    return producer_process_id_;
  }
  const protocol::Id &producer_id() const { return producer_id_; }
  protocol::Sequence prepare_id() const { return preparation_.id; }

 private:
  // The association key of one preparation, as the interface freezes it.
  struct Preparation {
    bool outstanding{false};
    protocol::Sequence id{0};
    protocol::Id runtime_id{};
    protocol::Id session_id{};
    protocol::TimeUs deadline_us{0};
  };

  const configuration::ApplicationConfig &config_;
  bool have_producer_{false};
  protocol::Id producer_process_id_{};
  protocol::Id producer_id_{};
  protocol::Text64 layout_id_;
  protocol::TimeUs target_period_us_{0};
  Preparation preparation_{};
  protocol::Sequence next_prepare_id_{1};

  bool have_candidate_{false};
  protocol::JointCommand candidate_{};
  ProducerStop failure_{ProducerStop::None};

  bool stop_notice_pending_{false};
  protocol::StopNotice stop_notice_{};
  // The epoch the runtime granted for the last authorization of this producer,
  // so a stop notice names the context it ends. Zero while none was granted.
  protocol::Sequence authorized_epoch_{0};
};

}  // namespace irmv::application
