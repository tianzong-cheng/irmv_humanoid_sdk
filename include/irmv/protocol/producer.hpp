#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "irmv/protocol/types.hpp"

// The external producer interface: the six messages an external policy process
// and the application exchange, and the frozen endpoints of them.
//
// A producer is a target generator, not a second supervisor. It never acquires
// a session, never requests a mode, never resets and never publishes a
// supervision heartbeat: the application selects it, prepares it, asks the
// runtime to authorize the context it produced and tells it when that context
// ends. The runtime remains the only authority that grants or withdraws
// execution, so a producer that never receives a stop notice - or that keeps
// publishing after one - is still refused the moment it uses a withdrawn epoch.
//
// The joint-level target itself is not a new message: a producer publishes
// `JointCommand` on the existing execution topic, exactly like the local policy
// and the initialization generator. These six messages only carry the lifecycle
// around it.
namespace irmv::protocol {

// The version of this interface, frozen with the topics below.
inline constexpr std::uint32_t kProducerProtocolVersion = 1;

// Whether a producer can be used. A failed producer reports a reason instead of
// staying silent: an application that never hears from its producer must not be
// able to tell "not ready yet" from "will never be ready".
enum class ProducerStatus : std::uint32_t {
  Ready = 0,
  Failed = 1,
};

enum class ProducerFailureReason : std::uint32_t {
  None = 0,
  // Another producer is already selected for this runtime instance.
  Busy = 1,
  // The announced version, layout or period is not one this application can
  // use.
  Incompatible = 2,
  // The producer could not produce a frame for the preparation it was given.
  PrepareFailed = 3,
  Internal = 4,
};

// One producer process declaring itself, with the runtime instance it observed
// and the layout it would command, so an incompatible producer is refused
// before anything is prepared.
struct ProducerAnnouncement {
  std::uint32_t protocol_version{kProducerProtocolVersion};
  Id producer_process_id{};
  Id runtime_id{};
  Text64 layout_id;
  TimeUs target_period_us{0};
  bool reads_robot_state{false};
  ProducerStatus status{ProducerStatus::Ready};
  ProducerFailureReason reason{ProducerFailureReason::None};
  Text256 detail;
};

// The body-frame velocity the producer should track: metres per second forward
// and to the left, radians per second around the base z axis, with the instant
// it was sent and how long it stays valid. The sequence increases with every
// message, and a producer that sees no fresh intent inside the validity period
// tracks zero instead of reusing the last one.
struct VelocityIntentMessage {
  std::uint32_t protocol_version{kProducerProtocolVersion};
  Id producer_process_id{};
  Sequence seq{0};
  double forward{0.0};
  double lateral{0.0};
  double yaw{0.0};
  TimeUs send_stamp_us{0};
  TimeUs valid_for_us{0};
};

// The application asking one producer to prepare a frame for `prepare_id`. The
// intent travels with the request: a producer prepares for the velocity it is
// asked to track, never for the last one it happened to see.
struct PrepareRequest {
  std::uint32_t protocol_version{kProducerProtocolVersion};
  Id producer_process_id{};
  Sequence prepare_id{0};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  Sequence epoch{0};
  TimeUs target_period_us{0};
  VelocityIntentMessage intent;
};

// The producer's answer: Ready with the complete first frame, or Failed with a
// reason. The first frame is what the application puts into the Tracking
// request, so it carries every joint and the identity the producer generated it
// under.
struct PrepareResult {
  std::uint32_t protocol_version{kProducerProtocolVersion};
  Id producer_process_id{};
  Sequence prepare_id{0};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  Sequence epoch{0};
  ProducerStatus status{ProducerStatus::Failed};
  ProducerFailureReason reason{ProducerFailureReason::None};
  Text256 detail;
  JointCommand first_command;
};

// The application telling a producer that the context it prepared is live: the
// authorization the runtime granted, the preparation it belongs to and the
// sequence of the first frame the runtime adopted. From here the producer
// publishes strictly increasing frames.
struct AuthorizationNotice {
  std::uint32_t protocol_version{kProducerProtocolVersion};
  Id producer_process_id{};
  Sequence prepare_id{0};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  Sequence epoch{0};
  Sequence first_command_seq{0};
};

// The application telling a producer to stop. It is a notice and not the reason
// a producer loses its rights: the runtime withdraws the epoch on its own, and
// a producer that never receives this message is refused all the same the
// moment it publishes a frame of a withdrawn epoch.
struct StopNotice {
  std::uint32_t protocol_version{kProducerProtocolVersion};
  Id producer_process_id{};
  Id runtime_id{};
  Id session_id{};
  Id producer_id{};
  Sequence epoch{0};
  StopReason reason{StopReason::None};
};

// The six endpoints, with the prefix and the QoS the interface freezes. The
// five lifecycle messages are reliable, volatile, keep-last eight with a finite
// blocking time: a lost lifecycle message would leave the two sides disagreeing
// about what is prepared or authorized, and the loss is worth waiting for. The
// velocity intent is the opposite - a newest-value stream whose older samples
// are worthless - so it is best effort, volatile, keep last one.
inline constexpr TopicDescriptor kProducerAnnouncementTopic{
    "irmv/v1/producer/announcement",
    {Reliability::Reliable, Durability::Volatile, 8, 10'000}};
inline constexpr TopicDescriptor kProducerPrepareRequestTopic{
    "irmv/v1/producer/prepare_request",
    {Reliability::Reliable, Durability::Volatile, 8, 10'000}};
inline constexpr TopicDescriptor kProducerPrepareResultTopic{
    "irmv/v1/producer/prepare_result",
    {Reliability::Reliable, Durability::Volatile, 8, 10'000}};
inline constexpr TopicDescriptor kProducerAuthorizationTopic{
    "irmv/v1/producer/authorization",
    {Reliability::Reliable, Durability::Volatile, 8, 10'000}};
inline constexpr TopicDescriptor kProducerStopTopic{
    "irmv/v1/producer/stop",
    {Reliability::Reliable, Durability::Volatile, 8, 10'000}};
inline constexpr TopicDescriptor kProducerVelocityIntentTopic{
    "irmv/v1/producer/velocity_intent",
    {Reliability::BestEffort, Durability::Volatile, 1, 0}};

// Every endpoint of the interface, for a test that walks the frozen list and
// for a supervisor that has to say which one lost a sample.
inline constexpr std::array<TopicDescriptor, 6> kProducerTopics = {
    kProducerAnnouncementTopic,  kProducerPrepareRequestTopic,
    kProducerPrepareResultTopic, kProducerAuthorizationTopic,
    kProducerStopTopic,          kProducerVelocityIntentTopic};

}  // namespace irmv::protocol
