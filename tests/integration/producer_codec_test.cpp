// The external producer interface: the six messages round-trip field by field,
// every malformed one is refused whole, and the six endpoints really carry the
// QoS the interface freezes - read back from live DDS endpoints, not from the
// descriptor that describes them.
//
// The producer's lifecycle QoS is the opposite of its velocity-intent QoS on
// purpose: a lost lifecycle message leaves the two sides disagreeing about what
// is prepared or authorized, while an older velocity intent is worthless the
// moment a newer one exists. This is the entry that pins both.
#include "irmv/transport/producer_codec.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <dds/dds.hpp>
#include <iostream>
#include <string>
#include <thread>

#include "irmv/protocol/producer.hpp"
#include "irmv/protocol/types.hpp"
#include "producer.hpp"
#include "support/check.hpp"

namespace {

namespace wire = irmv::protocol::wire;
namespace producer_codec = irmv::transport::producer_codec;

using irmv::protocol::AuthorizationNotice;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::kProducerAnnouncementTopic;
using irmv::protocol::kProducerAuthorizationTopic;
using irmv::protocol::kProducerPrepareRequestTopic;
using irmv::protocol::kProducerPrepareResultTopic;
using irmv::protocol::kProducerStopTopic;
using irmv::protocol::kProducerTopics;
using irmv::protocol::kProducerVelocityIntentTopic;
using irmv::protocol::PrepareRequest;
using irmv::protocol::PrepareResult;
using irmv::protocol::ProducerAnnouncement;
using irmv::protocol::ProducerFailureReason;
using irmv::protocol::ProducerStatus;
using irmv::protocol::Sequence;
using irmv::protocol::StopNotice;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::protocol::TopicDescriptor;
using irmv::protocol::VelocityIntentMessage;

Id MakeId(std::uint8_t seed) {
  Id id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

bool SameId(const Id &left, const Id &right) { return left == right; }

// A frame that is complete enough for the codec: the joint rules themselves are
// the execution codec's subject, not this one's.
JointCommand MakeCommand(std::uint32_t joint_count, Sequence epoch,
                         Sequence command_seq) {
  JointCommand command;
  command.producer_id = MakeId(0x30);
  command.epoch = epoch;
  command.command_seq = command_seq;
  command.source_state_seq = 7;
  command.layout_id.Assign("layout");
  command.joints.count = joint_count;
  for (std::uint32_t index = 0; index < joint_count; ++index) {
    command.joints.q[index] = 0.1 * index;
    command.joints.dq[index] = 0.01 * index;
    command.joints.kp[index] = 50.0;
    command.joints.kd[index] = 2.0;
  }
  return command;
}

VelocityIntentMessage MakeIntent(Sequence seq, TimeUs valid_for_us) {
  VelocityIntentMessage intent;
  intent.producer_process_id = MakeId(0x01);
  intent.seq = seq;
  intent.forward = 0.4;
  intent.lateral = -0.1;
  intent.yaw = 0.2;
  intent.send_stamp_us = 1234;
  intent.valid_for_us = valid_for_us;
  return intent;
}

// ---- round trips ----------------------------------------------------------

int AnnouncementRoundTrip() {
  ProducerAnnouncement source;
  source.producer_process_id = MakeId(0x01);
  source.runtime_id = MakeId(0x02);
  source.layout_id.Assign("fake-layout");
  source.target_period_us = 20000;
  source.reads_robot_state = true;
  source.status = ProducerStatus::Failed;
  source.reason = ProducerFailureReason::Incompatible;
  source.detail.Assign("layout mismatch");

  wire::ProducerAnnouncement encoded;
  producer_codec::ToWire(source, encoded);
  ProducerAnnouncement decoded;
  CHECK(producer_codec::FromWire(encoded, decoded));
  CHECK(SameId(decoded.producer_process_id, source.producer_process_id));
  CHECK(SameId(decoded.runtime_id, source.runtime_id));
  CHECK(decoded.layout_id.View() == "fake-layout");
  CHECK(decoded.target_period_us == 20000);
  CHECK(decoded.reads_robot_state);
  CHECK(decoded.status == ProducerStatus::Failed);
  CHECK(decoded.reason == ProducerFailureReason::Incompatible);
  CHECK(decoded.detail.View() == "layout mismatch");
  return 0;
}

int IntentRoundTrip() {
  const VelocityIntentMessage source = MakeIntent(11, 500000);
  wire::VelocityIntentMessage encoded;
  producer_codec::ToWire(source, encoded);
  VelocityIntentMessage decoded;
  CHECK(producer_codec::FromWire(encoded, decoded));
  CHECK(SameId(decoded.producer_process_id, source.producer_process_id));
  CHECK(decoded.seq == 11);
  CHECK(decoded.forward == 0.4);
  CHECK(decoded.lateral == -0.1);
  CHECK(decoded.yaw == 0.2);
  CHECK(decoded.send_stamp_us == 1234);
  CHECK(decoded.valid_for_us == 500000);
  return 0;
}

int PrepareRoundTrip() {
  PrepareRequest request;
  request.producer_process_id = MakeId(0x01);
  request.prepare_id = 42;
  request.runtime_id = MakeId(0x02);
  request.session_id = MakeId(0x03);
  request.producer_id = MakeId(0x04);
  request.epoch = 9;
  request.target_period_us = 20000;
  request.intent = MakeIntent(3, 500000);

  wire::PrepareRequest encoded_request;
  producer_codec::ToWire(request, encoded_request);
  PrepareRequest decoded_request;
  CHECK(producer_codec::FromWire(encoded_request, decoded_request));
  CHECK(decoded_request.prepare_id == 42);
  CHECK(decoded_request.epoch == 9);
  CHECK(SameId(decoded_request.session_id, request.session_id));
  CHECK(decoded_request.intent.seq == 3);
  CHECK(decoded_request.intent.valid_for_us == 500000);

  PrepareResult result;
  result.producer_process_id = MakeId(0x01);
  result.prepare_id = 42;
  result.runtime_id = MakeId(0x02);
  result.session_id = MakeId(0x03);
  result.producer_id = MakeId(0x04);
  result.epoch = 9;
  result.status = ProducerStatus::Ready;
  result.first_command = MakeCommand(2, 9, 1);

  wire::PrepareResult encoded_result;
  producer_codec::ToWire(result, encoded_result);
  PrepareResult decoded_result;
  CHECK(producer_codec::FromWire(encoded_result, decoded_result));
  CHECK(decoded_result.status == ProducerStatus::Ready);
  CHECK(decoded_result.prepare_id == 42);
  CHECK(decoded_result.first_command.joints.count == 2);
  CHECK(decoded_result.first_command.epoch == 9);
  CHECK(decoded_result.first_command.joints.kp[1] == 50.0);
  CHECK(decoded_result.first_command.layout_id.View() == "layout");
  return 0;
}

int NoticeRoundTrip() {
  AuthorizationNotice authorization;
  authorization.producer_process_id = MakeId(0x01);
  authorization.prepare_id = 42;
  authorization.runtime_id = MakeId(0x02);
  authorization.session_id = MakeId(0x03);
  authorization.producer_id = MakeId(0x04);
  authorization.epoch = 9;
  authorization.first_command_seq = 1;

  wire::AuthorizationNotice encoded_authorization;
  producer_codec::ToWire(authorization, encoded_authorization);
  AuthorizationNotice decoded_authorization;
  CHECK(producer_codec::FromWire(encoded_authorization, decoded_authorization));
  CHECK(decoded_authorization.prepare_id == 42);
  CHECK(decoded_authorization.epoch == 9);
  CHECK(decoded_authorization.first_command_seq == 1);

  StopNotice stop;
  stop.producer_process_id = MakeId(0x01);
  stop.runtime_id = MakeId(0x02);
  stop.session_id = MakeId(0x03);
  stop.producer_id = MakeId(0x04);
  stop.epoch = 9;
  stop.reason = StopReason::Requested;

  wire::StopNotice encoded_stop;
  producer_codec::ToWire(stop, encoded_stop);
  StopNotice decoded_stop;
  CHECK(producer_codec::FromWire(encoded_stop, decoded_stop));
  CHECK(decoded_stop.epoch == 9);
  CHECK(decoded_stop.reason == StopReason::Requested);
  return 0;
}

// ---- refusals -------------------------------------------------------------

int RefusedMessages() {
  ProducerAnnouncement announcement;
  announcement.producer_process_id = MakeId(0x01);
  wire::ProducerAnnouncement encoded_announcement;
  producer_codec::ToWire(announcement, encoded_announcement);
  encoded_announcement.protocol_version(2);
  ProducerAnnouncement decoded_announcement;
  CHECK(!producer_codec::FromWire(encoded_announcement, decoded_announcement));
  encoded_announcement.protocol_version(
      irmv::protocol::kProducerProtocolVersion);
  encoded_announcement.status(static_cast<wire::ProducerStatus>(99));
  CHECK(!producer_codec::FromWire(encoded_announcement, decoded_announcement));
  // Ready and failed at the same time describes nothing.
  encoded_announcement.status(
      static_cast<wire::ProducerStatus>(ProducerStatus::Ready));
  encoded_announcement.reason(static_cast<wire::ProducerFailureReason>(
      ProducerFailureReason::Internal));
  CHECK(!producer_codec::FromWire(encoded_announcement, decoded_announcement));
  // A text count above the wire capacity is not a prefix of the text.
  encoded_announcement.reason(
      static_cast<wire::ProducerFailureReason>(ProducerFailureReason::None));
  encoded_announcement.detail().count(9999);
  CHECK(!producer_codec::FromWire(encoded_announcement, decoded_announcement));

  wire::VelocityIntentMessage encoded_intent;
  producer_codec::ToWire(MakeIntent(1, 500000), encoded_intent);
  VelocityIntentMessage decoded_intent;
  CHECK(producer_codec::FromWire(encoded_intent, decoded_intent));
  encoded_intent.valid_for_us(0);
  CHECK(!producer_codec::FromWire(encoded_intent, decoded_intent));
  encoded_intent.valid_for_us(500000);
  encoded_intent.protocol_version(0);
  CHECK(!producer_codec::FromWire(encoded_intent, decoded_intent));

  // A preparation message without a preparation number attaches to nothing.
  PrepareRequest request;
  request.producer_process_id = MakeId(0x01);
  request.prepare_id = 1;
  request.intent = MakeIntent(1, 500000);
  wire::PrepareRequest encoded_request;
  producer_codec::ToWire(request, encoded_request);
  PrepareRequest decoded_request;
  CHECK(producer_codec::FromWire(encoded_request, decoded_request));
  encoded_request.prepare_id(0);
  CHECK(!producer_codec::FromWire(encoded_request, decoded_request));
  encoded_request.prepare_id(1);
  encoded_request.intent().valid_for_us(0);
  CHECK(!producer_codec::FromWire(encoded_request, decoded_request));

  PrepareResult result;
  result.producer_process_id = MakeId(0x01);
  result.prepare_id = 1;
  result.status = ProducerStatus::Ready;
  result.first_command = MakeCommand(2, 3, 1);
  wire::PrepareResult encoded_result;
  producer_codec::ToWire(result, encoded_result);
  PrepareResult decoded_result;
  CHECK(producer_codec::FromWire(encoded_result, decoded_result));
  encoded_result.prepare_id(0);
  CHECK(!producer_codec::FromWire(encoded_result, decoded_result));
  encoded_result.prepare_id(1);
  // Ready with no frame is an authorization the application could not carry.
  encoded_result.first_command().joints().count(0);
  CHECK(!producer_codec::FromWire(encoded_result, decoded_result));
  // The nested frame obeys the execution rules: a count above the wire capacity
  // is refused by the execution codec.
  encoded_result.first_command().joints().count(irmv::protocol::kMaxJoints + 1);
  CHECK(!producer_codec::FromWire(encoded_result, decoded_result));
  encoded_result.first_command().joints().count(2);

  AuthorizationNotice authorization;
  authorization.producer_process_id = MakeId(0x01);
  authorization.prepare_id = 1;
  wire::AuthorizationNotice encoded_authorization;
  producer_codec::ToWire(authorization, encoded_authorization);
  AuthorizationNotice decoded_authorization;
  CHECK(producer_codec::FromWire(encoded_authorization, decoded_authorization));
  encoded_authorization.prepare_id(0);
  CHECK(
      !producer_codec::FromWire(encoded_authorization, decoded_authorization));

  wire::StopNotice encoded_stop;
  producer_codec::ToWire(StopNotice{}, encoded_stop);
  StopNotice decoded_stop;
  CHECK(producer_codec::FromWire(encoded_stop, decoded_stop));
  encoded_stop.reason(static_cast<wire::StopReason>(999));
  CHECK(!producer_codec::FromWire(encoded_stop, decoded_stop));
  return 0;
}

// ---- the frozen endpoints -------------------------------------------------

int FrozenTopics() {
  const std::array<const char *, 6> names = {
      "irmv/v1/producer/announcement",   "irmv/v1/producer/prepare_request",
      "irmv/v1/producer/prepare_result", "irmv/v1/producer/authorization",
      "irmv/v1/producer/stop",           "irmv/v1/producer/velocity_intent"};
  CHECK(kProducerTopics.size() == names.size());
  for (std::size_t index = 0; index < names.size(); ++index) {
    CHECK(kProducerTopics[index].name == names[index]);
  }
  const auto lifecycle = [](const TopicDescriptor &descriptor) {
    return descriptor.qos.reliability ==
               irmv::protocol::Reliability::Reliable &&
           descriptor.qos.durability == irmv::protocol::Durability::Volatile &&
           descriptor.qos.history_depth == 8 &&
           descriptor.qos.max_blocking_time_us == 10000;
  };
  CHECK(lifecycle(kProducerAnnouncementTopic));
  CHECK(lifecycle(kProducerPrepareRequestTopic));
  CHECK(lifecycle(kProducerPrepareResultTopic));
  CHECK(lifecycle(kProducerAuthorizationTopic));
  CHECK(lifecycle(kProducerStopTopic));
  CHECK(kProducerVelocityIntentTopic.qos.reliability ==
        irmv::protocol::Reliability::BestEffort);
  CHECK(kProducerVelocityIntentTopic.qos.durability ==
        irmv::protocol::Durability::Volatile);
  CHECK(kProducerVelocityIntentTopic.qos.history_depth == 1);
  CHECK(kProducerVelocityIntentTopic.qos.max_blocking_time_us == 0);
  return 0;
}

std::uint32_t Domain() {
  if (const char *override_domain = std::getenv("IRMV_TEST_DDS_DOMAIN")) {
    return static_cast<std::uint32_t>(
        std::strtoul(override_domain, nullptr, 10));
  }
  return 221;
}

// The same interface binding the running transport uses, so a machine with a
// second network card cannot answer for a peer. An empty value lets CycloneDDS
// choose, which is what the transport does for an empty `dds.interface`.
std::string InterfaceConfig() {
  const char *interface = std::getenv("IRMV_TEST_DDS_INTERFACE");
  const std::string name = interface != nullptr && *interface != '\0'
                               ? std::string(interface)
                               : std::string("lo");
  return "<CycloneDDS><Domain><General><Interfaces><NetworkInterface "
         "name=\"" +
         name +
         "\" priority=\"default\"/></Interfaces></General></Domain>"
         "</CycloneDDS>";
}

}  // namespace

// The six endpoints as real DDS endpoints: the policies read back from a writer
// and a reader built with the frozen descriptors are the ones the interface
// declares, and a lifecycle message and a velocity intent really travel.
int main() {
  CHECK(AnnouncementRoundTrip() == 0);
  CHECK(IntentRoundTrip() == 0);
  CHECK(PrepareRoundTrip() == 0);
  CHECK(NoticeRoundTrip() == 0);
  CHECK(RefusedMessages() == 0);
  CHECK(FrozenTopics() == 0);

  const std::uint32_t domain = Domain();
  dds::domain::DomainParticipant participant(
      domain, dds::domain::qos::DomainParticipantQos(), nullptr,
      dds::core::status::StatusMask::none(), InterfaceConfig());
  dds::pub::Publisher publisher(participant);
  dds::sub::Subscriber subscriber(participant);

  // A lifecycle endpoint: reliable, volatile, keep last eight, finite blocking.
  {
    const dds::topic::Topic<wire::PrepareRequest> topic(
        participant, "irmv/v1/producer/prepare_request");
    dds::pub::DataWriter<wire::PrepareRequest> writer(
        publisher, topic,
        producer_codec::ToWriterQos(kProducerPrepareRequestTopic.qos));
    dds::sub::DataReader<wire::PrepareRequest> reader(
        subscriber, topic,
        producer_codec::ToReaderQos(kProducerPrepareRequestTopic.qos));
    const auto writer_reliability =
        writer.qos().policy<dds::core::policy::Reliability>();
    const auto writer_durability =
        writer.qos().policy<dds::core::policy::Durability>();
    const auto writer_history =
        writer.qos().policy<dds::core::policy::History>();
    const auto reader_reliability =
        reader.qos().policy<dds::core::policy::Reliability>();
    const auto reader_durability =
        reader.qos().policy<dds::core::policy::Durability>();
    const auto reader_history =
        reader.qos().policy<dds::core::policy::History>();
    CHECK(writer_reliability.kind() ==
          dds::core::policy::ReliabilityKind::RELIABLE);
    CHECK(writer_durability.kind() ==
          dds::core::policy::DurabilityKind::VOLATILE);
    CHECK(writer_history.kind() == dds::core::policy::HistoryKind::KEEP_LAST);
    CHECK(writer_history.depth() == 8);
    CHECK(writer_reliability.max_blocking_time() ==
          dds::core::Duration::from_microsecs(10000));
    CHECK(reader_reliability.kind() ==
          dds::core::policy::ReliabilityKind::RELIABLE);
    CHECK(reader_durability.kind() ==
          dds::core::policy::DurabilityKind::VOLATILE);
    CHECK(reader_history.kind() == dds::core::policy::HistoryKind::KEEP_LAST);
    CHECK(reader_history.depth() == 8);

    PrepareRequest request;
    request.producer_process_id = MakeId(0x01);
    request.prepare_id = 7;
    request.epoch = 3;
    request.intent = MakeIntent(1, 500000);
    wire::PrepareRequest sample;
    producer_codec::ToWire(request, sample);
    writer.write(sample);

    bool received = false;
    for (int attempt = 0; attempt < 200 && !received; ++attempt) {
      const auto samples = reader.select()
                               .max_samples(8)
                               .state(dds::sub::status::DataState::any())
                               .take();
      for (const auto &candidate : samples) {
        if (!candidate.info().valid()) {
          continue;
        }
        PrepareRequest decoded;
        if (producer_codec::FromWire(candidate.data(), decoded)) {
          CHECK(decoded.prepare_id == 7);
          CHECK(decoded.epoch == 3);
          CHECK(decoded.intent.seq == 1);
          received = true;
          break;
        }
      }
      if (!received) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
    CHECK(received);
  }

  // A velocity intent endpoint: best effort, volatile, keep last one, and no
  // blocking time at all - a newest-value stream never waits for a reader.
  {
    const dds::topic::Topic<wire::VelocityIntentMessage> topic(
        participant, "irmv/v1/producer/velocity_intent");
    dds::pub::DataWriter<wire::VelocityIntentMessage> writer(
        publisher, topic,
        producer_codec::ToWriterQos(kProducerVelocityIntentTopic.qos));
    const auto reliability =
        writer.qos().policy<dds::core::policy::Reliability>();
    const auto durability =
        writer.qos().policy<dds::core::policy::Durability>();
    const auto history = writer.qos().policy<dds::core::policy::History>();
    CHECK(reliability.kind() ==
          dds::core::policy::ReliabilityKind::BEST_EFFORT);
    CHECK(durability.kind() == dds::core::policy::DurabilityKind::VOLATILE);
    CHECK(history.kind() == dds::core::policy::HistoryKind::KEEP_LAST);
    CHECK(history.depth() == 1);
  }

  std::cout << "producer_codec_test: ok\n";
  return 0;
}
