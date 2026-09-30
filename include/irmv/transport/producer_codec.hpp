#pragma once

#include <dds/dds.hpp>

#include "irmv/protocol/producer.hpp"
#include "producer.hpp"

// Sample and endpoint conversion between the producer lifecycle types in
// irmv/protocol/producer.hpp and the generated wire types from
// protocol/producer.idl, plus the six endpoints themselves.
//
// The two directions have the same contract as the execution codec: ToWire
// copies an already validated core value and clamps text to the wire capacity,
// while FromWire trusts nothing and reports a payload it cannot accept by
// returning false with `target` untouched, so a rejected message is never
// partially applied.
//
// On top of the structural rules, FromWire enforces the invariants that make a
// lifecycle message mean anything at all:
//   * `protocol_version` must be the frozen one - a message of another version
//     cannot be interpreted, and guessing would be worse than refusing it;
//   * a preparation number is never zero, because zero is what a default
//     constructed message carries and a result or a notice without a
//     preparation would attach to nothing;
//   * a velocity intent must have a non-zero validity period and a sequence,
//     so an intent can never be "valid forever" or reuse the previous one.
//
// Which producer, preparation, session or epoch a message is *currently*
// allowed to name is not decided here: that is the supervisor's state, not the
// shape of the message.
namespace irmv::transport::producer_codec {

void ToWire(const protocol::ProducerAnnouncement &source,
            protocol::wire::ProducerAnnouncement &target);
void ToWire(const protocol::VelocityIntentMessage &source,
            protocol::wire::VelocityIntentMessage &target);
void ToWire(const protocol::PrepareRequest &source,
            protocol::wire::PrepareRequest &target);
void ToWire(const protocol::PrepareResult &source,
            protocol::wire::PrepareResult &target);
void ToWire(const protocol::AuthorizationNotice &source,
            protocol::wire::AuthorizationNotice &target);
void ToWire(const protocol::StopNotice &source,
            protocol::wire::StopNotice &target);

bool FromWire(const protocol::wire::ProducerAnnouncement &source,
              protocol::ProducerAnnouncement &target);
bool FromWire(const protocol::wire::VelocityIntentMessage &source,
              protocol::VelocityIntentMessage &target);
bool FromWire(const protocol::wire::PrepareRequest &source,
              protocol::PrepareRequest &target);
bool FromWire(const protocol::wire::PrepareResult &source,
              protocol::PrepareResult &target);
bool FromWire(const protocol::wire::AuthorizationNotice &source,
              protocol::AuthorizationNotice &target);
bool FromWire(const protocol::wire::StopNotice &source,
              protocol::StopNotice &target);

// The endpoint policies of one of the six frozen descriptors, so a test can
// read back from a real endpoint exactly what the interface declares.
dds::pub::qos::DataWriterQos ToWriterQos(const protocol::TopicQos &qos);
dds::sub::qos::DataReaderQos ToReaderQos(const protocol::TopicQos &qos);

}  // namespace irmv::transport::producer_codec
