#pragma once

#include <dds/dds.hpp>

#include "execution.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::transport::codec {

// Sample and endpoint conversion between the frozen protocol types in
// irmv/protocol/types.hpp and the generated wire types from
// protocol/execution.idl. Every wire struct is built from fixed IDL arrays plus
// an explicit count, so neither direction allocates.
//
// ToWire is the outbound path: the core types are already validated, so it only
// copies, clamping text to the wire capacity. FromWire is the inbound path and
// trusts nothing: it validates counts, enum values and text lengths, and a
// payload that does not fit is reported by returning false with `target`
// untouched, so that a rejected message is never partially applied.
//
// Validation rules, derived from what irmv::protocol already accepts:
//   * JointVector.count in [1, kMaxJoints]: ValidateCommandShape requires
//     exactly the configured joint count, which is at least one.
//   * FeedbackSnapshot.count and FaultReport.count in [0, kMaxJoints]: an empty
//     snapshot and a healthy state with no faults are both representable, and
//     the core classifies a count mismatch as invalid feedback.
//   * every enum value passes protocol::IsValid.
//   * Text64/Text256 counts stay within the wire capacity and the bytes are
//     copied through BoundedText::Assign, including the full 64/256 byte
//     boundary.
void ToWire(const protocol::JointCommand &source,
            protocol::wire::JointCommand &target);
void ToWire(const protocol::ExecutionRequest &source,
            protocol::wire::ExecutionRequest &target);
void ToWire(const protocol::ExecutionResult &source,
            protocol::wire::ExecutionResult &target);
void ToWire(const protocol::SupervisorHeartbeat &source,
            protocol::wire::SupervisorHeartbeat &target);
void ToWire(const protocol::RobotState &source, protocol::wire::RobotState &target);

bool FromWire(const protocol::wire::JointCommand &source,
              protocol::JointCommand &target);
bool FromWire(const protocol::wire::ExecutionRequest &source,
              protocol::ExecutionRequest &target);
bool FromWire(const protocol::wire::ExecutionResult &source,
              protocol::ExecutionResult &target);
bool FromWire(const protocol::wire::SupervisorHeartbeat &source,
              protocol::SupervisorHeartbeat &target);
bool FromWire(const protocol::wire::RobotState &source,
              protocol::RobotState &target);

// Endpoint policies for a frozen channel descriptor. Both endpoints of a
// channel receive the same reliability, history depth, durability and finite
// resource limits, so the policies read back from an endpoint built with these
// helpers are the ones the frozen descriptors declare.
dds::pub::qos::DataWriterQos ToWriterQos(const protocol::TopicQos &qos);
dds::sub::qos::DataReaderQos ToReaderQos(const protocol::TopicQos &qos);

}  // namespace irmv::transport::codec
