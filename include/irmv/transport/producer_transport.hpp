#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/producer.hpp"
#include "irmv/transport/bounded_channel.hpp"

// The application side of the external producer interface: the six endpoints of
// protocol/producer.idl, on one transport thread.
//
// Thread model, identical to ClientTransport: one thread owns every DDS reader
// and writer and carries every conversion, so the supervision thread never
// performs a DDS call. Both directions cross fixed-capacity channels between
// exactly those two threads - the supervision thread publishes and takes, the
// transport thread takes and publishes - and the transport thread waits on a
// WaitSet over its two readers with a bounded ten millisecond timeout, which
// keeps Stop() responsive and bounds how long an outbound sample can wait.
//
// Every lane here is FIFO, including the outbound ones. A lifecycle message is
// not a newest-value stream: a superseded preparation, notice or stop is
// exactly the message whose loss leaves the two sides disagreeing about what is
// prepared or authorized. The one exception is the velocity intent, which is
// the newest-value lane the interface declares - an older intent is worthless
// the moment a newer one exists - so a full lane there drops the newest sample,
// counted, and never displaces an older one.
//
// Nothing waits for a peer and nothing blocks on a full channel: every refusal,
// drop or rejected sample is counted, and an endpoint that cannot be created is
// an exception from Start().
namespace irmv::transport {

// Capacity of the inbound lanes: a burst of lifecycle messages a slow
// supervision iteration has not drained yet. Announcements and preparation
// results are rare - one per producer and one per preparation - so eight is a
// generous bound that still refuses rather than grows.
inline constexpr std::size_t kProducerInboundCapacity = 8;
// Capacity of the outbound lifecycle lanes. The application sends one
// preparation, at most one authorization and at most one stop per context, so
// the lane only absorbs the difference between the loop that produces them and
// the transport thread that writes them.
inline constexpr std::size_t kProducerOutboundCapacity = 8;

// One drain of the inbound lanes. It describes only that one call; the drop
// counters of the transport are cumulative accessors.
struct ProducerEvents {
  std::array<protocol::ProducerAnnouncement, kProducerInboundCapacity>
      announcements{};
  std::size_t announcement_count{0};
  std::array<protocol::PrepareResult, kProducerInboundCapacity>
      prepare_results{};
  std::size_t prepare_result_count{0};
};

class ProducerTransport {
 public:
  // Stores the configuration snapshot; the referenced configuration must
  // outlive the transport. No DDS entity is created before Start().
  explicit ProducerTransport(const configuration::ApplicationConfig &config);
  ~ProducerTransport();

  ProducerTransport(const ProducerTransport &) = delete;
  ProducerTransport &operator=(const ProducerTransport &) = delete;

  // Creates the participant, the six endpoints and the transport thread.
  // Idempotent; throws dds::core::Error when any of them cannot be created.
  void Start();

  // Asks the transport thread to stop and joins it. Idempotent, safe on a
  // transport that was never started, and also called by the destructor.
  void Stop();

  bool started() const;

  // Hands one preparation request to the transport thread. Never blocks, never
  // touches DDS and never allocates; a request that does not fit, or one handed
  // over while the transport is not running, is refused, counted and reported
  // by returning false.
  bool PublishPrepareRequest(const protocol::PrepareRequest &request);

  // Hands one authorization notice to its lane. It is only ever sent after the
  // runtime applied the context, so losing it delays a producer's permission to
  // publish rather than granting one - and the runtime refuses a frame of a
  // withdrawn epoch regardless.
  bool PublishAuthorization(const protocol::AuthorizationNotice &notice);

  // Hands one stop notice to its lane.
  bool PublishStop(const protocol::StopNotice &notice);

  // Hands the newest velocity intent to the newest-value lane. Publishing a
  // newer intent while an older one is unread replaces it and reports the loss
  // through the drop counter: an older intent carries no information at all.
  bool PublishVelocityIntent(const protocol::VelocityIntentMessage &intent);

  // Drains the inbound lanes into the owned batch and returns it. The batch
  // describes this one call; the reference stays valid until the next call.
  // Called only by the supervision thread.
  const ProducerEvents &TakeEvents();

  // Announcements dropped because the inbound lane was full.
  std::uint64_t dropped_announcement_count() const;
  // Preparation results dropped because the inbound lane was full.
  std::uint64_t dropped_prepare_result_count() const;
  // Outbound lifecycle messages dropped because their lane was full (or because
  // the transport was not running), per lane.
  std::uint64_t dropped_prepare_request_count() const;
  std::uint64_t dropped_authorization_count() const;
  std::uint64_t dropped_stop_count() const;
  // Velocity intents dropped: a newer one superseded an unread one, none could
  // be stored, or the transport was not running.
  std::uint64_t dropped_velocity_intent_count() const;
  // Inbound samples that failed the codec and were discarded whole.
  std::uint64_t inbound_rejected_count() const;
  // Reads that failed and ended the transport loop, counted apart from a
  // rejected sample so a broken endpoint can be told from a malformed one.
  std::uint64_t inbound_read_failure_count() const;

 private:
  struct Impl;

  using AnnouncementChannel =
      BoundedChannel<protocol::ProducerAnnouncement, kProducerInboundCapacity>;
  using PrepareResultChannel =
      BoundedChannel<protocol::PrepareResult, kProducerInboundCapacity>;
  using PrepareRequestChannel =
      BoundedChannel<protocol::PrepareRequest, kProducerOutboundCapacity>;
  using AuthorizationChannel =
      BoundedChannel<protocol::AuthorizationNotice, kProducerOutboundCapacity>;
  using StopChannel =
      BoundedChannel<protocol::StopNotice, kProducerOutboundCapacity>;
  using IntentChannel = LatestValueChannel<protocol::VelocityIntentMessage>;

  const configuration::ApplicationConfig &config_;
  AnnouncementChannel announcements_;
  PrepareResultChannel prepare_results_;
  PrepareRequestChannel prepare_requests_;
  AuthorizationChannel authorizations_;
  StopChannel stops_;
  IntentChannel intents_;
  ProducerEvents events_;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> started_{false};
  std::atomic<std::uint64_t> dropped_announcements_{0};
  std::atomic<std::uint64_t> dropped_prepare_results_{0};
  std::atomic<std::uint64_t> dropped_prepare_requests_{0};
  std::atomic<std::uint64_t> dropped_authorizations_{0};
  std::atomic<std::uint64_t> dropped_stops_{0};
  std::atomic<std::uint64_t> dropped_intents_{0};
  std::atomic<std::uint64_t> inbound_rejected_{0};
  std::atomic<std::uint64_t> inbound_read_failures_{0};
};

}  // namespace irmv::transport
