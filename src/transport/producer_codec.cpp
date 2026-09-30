#include "irmv/transport/producer_codec.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "irmv/protocol/validation.hpp"
#include "irmv/transport/codec.hpp"

namespace irmv::transport::producer_codec {
namespace {

using protocol::ProducerFailureReason;
using protocol::ProducerStatus;

// Wire counts are unsigned and can be anything; the core text is fixed. A count
// above the capacity is clamped on the way out (the core value is already
// validated) and rejected on the way in.
std::size_t ClampCount(std::uint32_t count, std::size_t capacity) {
  return count > capacity ? capacity : count;
}

template <typename WireText, typename ProtocolText>
void ToWireText(const ProtocolText &source, WireText &target) {
  const std::size_t capacity = target.bytes().size();
  const std::size_t count = ClampCount(source.count, capacity);
  target.count(static_cast<std::uint32_t>(count));
  target.bytes().fill(0);
  std::copy_n(source.bytes.begin(), count, target.bytes().begin());
}

template <typename ProtocolText, typename WireText>
bool FromWireText(const WireText &source, ProtocolText &target) {
  const std::size_t count = source.count();
  if (count > target.bytes.size()) {
    return false;
  }
  target.count = static_cast<std::uint32_t>(count);
  std::copy_n(source.bytes().begin(), count, target.bytes.begin());
  return true;
}

// One of the two producer enumerations, checked against the frozen set before
// it is written into a core value. The wire type is generated as a scoped enum
// of its own, so the value is carried through its underlying type and the
// frozen set is what decides.
template <typename ProtocolEnum, typename WireEnum>
bool FromWireEnum(WireEnum raw, ProtocolEnum &target) {
  const auto value = static_cast<std::uint32_t>(
      static_cast<std::underlying_type_t<WireEnum>>(raw));
  const ProtocolEnum candidate = static_cast<ProtocolEnum>(value);
  if (!protocol::IsValid(candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

// ---- outbound -------------------------------------------------------------

void ToWireIntent(const protocol::VelocityIntentMessage &source,
                  protocol::wire::VelocityIntentMessage &target) {
  target.protocol_version(source.protocol_version);
  target.producer_process_id(source.producer_process_id);
  target.seq(source.seq);
  target.forward(source.forward);
  target.lateral(source.lateral);
  target.yaw(source.yaw);
  target.send_stamp_us(source.send_stamp_us);
  target.valid_for_us(source.valid_for_us);
}

// ---- inbound --------------------------------------------------------------

bool ReadIntent(const protocol::wire::VelocityIntentMessage &source,
                protocol::VelocityIntentMessage &target) {
  protocol::VelocityIntentMessage candidate;
  candidate.protocol_version = source.protocol_version();
  if (candidate.protocol_version != protocol::kProducerProtocolVersion) {
    return false;
  }
  candidate.producer_process_id = source.producer_process_id();
  candidate.seq = source.seq();
  // An intent without a validity period would be valid forever, which is the
  // one thing a newest-value stream must never be.
  candidate.valid_for_us = source.valid_for_us();
  if (candidate.valid_for_us == 0) {
    return false;
  }
  candidate.forward = source.forward();
  candidate.lateral = source.lateral();
  candidate.yaw = source.yaw();
  candidate.send_stamp_us = source.send_stamp_us();
  target = candidate;
  return true;
}

}  // namespace

void ToWire(const protocol::ProducerAnnouncement &source,
            protocol::wire::ProducerAnnouncement &target) {
  target.protocol_version(source.protocol_version);
  target.producer_process_id(source.producer_process_id);
  target.runtime_id(source.runtime_id);
  ToWireText(source.layout_id, target.layout_id());
  target.target_period_us(source.target_period_us);
  target.reads_robot_state(source.reads_robot_state);
  target.status(static_cast<protocol::wire::ProducerStatus>(source.status));
  target.reason(
      static_cast<protocol::wire::ProducerFailureReason>(source.reason));
  ToWireText(source.detail, target.detail());
}

void ToWire(const protocol::VelocityIntentMessage &source,
            protocol::wire::VelocityIntentMessage &target) {
  ToWireIntent(source, target);
}

void ToWire(const protocol::PrepareRequest &source,
            protocol::wire::PrepareRequest &target) {
  target.protocol_version(source.protocol_version);
  target.producer_process_id(source.producer_process_id);
  target.prepare_id(source.prepare_id);
  target.runtime_id(source.runtime_id);
  target.session_id(source.session_id);
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.target_period_us(source.target_period_us);
  ToWireIntent(source.intent, target.intent());
}

void ToWire(const protocol::PrepareResult &source,
            protocol::wire::PrepareResult &target) {
  target.protocol_version(source.protocol_version);
  target.producer_process_id(source.producer_process_id);
  target.prepare_id(source.prepare_id);
  target.runtime_id(source.runtime_id);
  target.session_id(source.session_id);
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.status(static_cast<protocol::wire::ProducerStatus>(source.status));
  target.reason(
      static_cast<protocol::wire::ProducerFailureReason>(source.reason));
  ToWireText(source.detail, target.detail());
  codec::ToWire(source.first_command, target.first_command());
}

void ToWire(const protocol::AuthorizationNotice &source,
            protocol::wire::AuthorizationNotice &target) {
  target.protocol_version(source.protocol_version);
  target.producer_process_id(source.producer_process_id);
  target.prepare_id(source.prepare_id);
  target.runtime_id(source.runtime_id);
  target.session_id(source.session_id);
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.first_command_seq(source.first_command_seq);
}

void ToWire(const protocol::StopNotice &source,
            protocol::wire::StopNotice &target) {
  target.protocol_version(source.protocol_version);
  target.producer_process_id(source.producer_process_id);
  target.runtime_id(source.runtime_id);
  target.session_id(source.session_id);
  target.producer_id(source.producer_id);
  target.epoch(source.epoch);
  target.reason(static_cast<protocol::wire::StopReason>(source.reason));
}

bool FromWire(const protocol::wire::ProducerAnnouncement &source,
              protocol::ProducerAnnouncement &target) {
  protocol::ProducerAnnouncement candidate;
  candidate.protocol_version = source.protocol_version();
  if (candidate.protocol_version != protocol::kProducerProtocolVersion) {
    return false;
  }
  candidate.producer_process_id = source.producer_process_id();
  candidate.runtime_id = source.runtime_id();
  if (!FromWireText(source.layout_id(), candidate.layout_id)) {
    return false;
  }
  candidate.target_period_us = source.target_period_us();
  candidate.reads_robot_state = source.reads_robot_state();
  if (!FromWireEnum(source.status(), candidate.status)) {
    return false;
  }
  if (!FromWireEnum(source.reason(), candidate.reason)) {
    return false;
  }
  if (candidate.status == ProducerStatus::Ready &&
      candidate.reason != ProducerFailureReason::None) {
    // A producer that reports itself ready and failed at the same time is
    // describing nothing an application could act on.
    return false;
  }
  if (!FromWireText(source.detail(), candidate.detail)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::VelocityIntentMessage &source,
              protocol::VelocityIntentMessage &target) {
  protocol::VelocityIntentMessage candidate;
  if (!ReadIntent(source, candidate)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::PrepareRequest &source,
              protocol::PrepareRequest &target) {
  protocol::PrepareRequest candidate;
  candidate.protocol_version = source.protocol_version();
  if (candidate.protocol_version != protocol::kProducerProtocolVersion) {
    return false;
  }
  candidate.producer_process_id = source.producer_process_id();
  candidate.prepare_id = source.prepare_id();
  if (candidate.prepare_id == 0) {
    return false;
  }
  candidate.runtime_id = source.runtime_id();
  candidate.session_id = source.session_id();
  candidate.producer_id = source.producer_id();
  candidate.epoch = source.epoch();
  candidate.target_period_us = source.target_period_us();
  if (!ReadIntent(source.intent(), candidate.intent)) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::PrepareResult &source,
              protocol::PrepareResult &target) {
  protocol::PrepareResult candidate;
  candidate.protocol_version = source.protocol_version();
  if (candidate.protocol_version != protocol::kProducerProtocolVersion) {
    return false;
  }
  candidate.producer_process_id = source.producer_process_id();
  candidate.prepare_id = source.prepare_id();
  if (candidate.prepare_id == 0) {
    return false;
  }
  candidate.runtime_id = source.runtime_id();
  candidate.session_id = source.session_id();
  candidate.producer_id = source.producer_id();
  candidate.epoch = source.epoch();
  if (!FromWireEnum(source.status(), candidate.status)) {
    return false;
  }
  if (!FromWireEnum(source.reason(), candidate.reason)) {
    return false;
  }
  if (candidate.status == ProducerStatus::Ready &&
      candidate.reason != ProducerFailureReason::None) {
    return false;
  }
  if (!FromWireText(source.detail(), candidate.detail)) {
    return false;
  }
  if (!codec::FromWire(source.first_command(), candidate.first_command)) {
    return false;
  }
  // A ready preparation without a usable first frame would ask the application
  // to authorize a context it cannot send; a failed one carries no frame that
  // means anything.
  if (candidate.status == ProducerStatus::Ready &&
      candidate.first_command.joints.count == 0) {
    return false;
  }
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::AuthorizationNotice &source,
              protocol::AuthorizationNotice &target) {
  protocol::AuthorizationNotice candidate;
  candidate.protocol_version = source.protocol_version();
  if (candidate.protocol_version != protocol::kProducerProtocolVersion) {
    return false;
  }
  candidate.producer_process_id = source.producer_process_id();
  candidate.prepare_id = source.prepare_id();
  if (candidate.prepare_id == 0) {
    return false;
  }
  candidate.runtime_id = source.runtime_id();
  candidate.session_id = source.session_id();
  candidate.producer_id = source.producer_id();
  candidate.epoch = source.epoch();
  candidate.first_command_seq = source.first_command_seq();
  target = candidate;
  return true;
}

bool FromWire(const protocol::wire::StopNotice &source,
              protocol::StopNotice &target) {
  protocol::StopNotice candidate;
  candidate.protocol_version = source.protocol_version();
  if (candidate.protocol_version != protocol::kProducerProtocolVersion) {
    return false;
  }
  candidate.producer_process_id = source.producer_process_id();
  candidate.runtime_id = source.runtime_id();
  candidate.session_id = source.session_id();
  candidate.producer_id = source.producer_id();
  candidate.epoch = source.epoch();
  if (!FromWireEnum(source.reason(), candidate.reason)) {
    return false;
  }
  target = candidate;
  return true;
}

dds::pub::qos::DataWriterQos ToWriterQos(const protocol::TopicQos &qos) {
  return codec::ToWriterQos(qos);
}

dds::sub::qos::DataReaderQos ToReaderQos(const protocol::TopicQos &qos) {
  return codec::ToReaderQos(qos);
}

}  // namespace irmv::transport::producer_codec
