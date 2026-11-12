#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <string>

#include "irmv/simulation/codec.hpp"
#include "support/check.hpp"

static bool forbid_allocation = false;
void* operator new(std::size_t n) {
  if (forbid_allocation) std::abort();
  if (auto p = std::malloc(n)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
using namespace irmv::simulation;
MessageIdentity GoldenMessageIdentity() {
  MessageIdentity v;
  v.kind = static_cast<MessageKind>(1);
  for (std::size_t i = 0; i < v.runtime_id.size(); ++i) v.runtime_id[i] = i + 1;
  for (std::size_t i = 0; i < v.supervisor_id.size(); ++i)
    v.supervisor_id[i] = i + 1;
  for (std::size_t i = 0; i < v.session_id.size(); ++i) v.session_id[i] = i + 1;
  for (std::size_t i = 0; i < v.producer_id.size(); ++i)
    v.producer_id[i] = i + 1;
  v.request_id = 0x0102030405060708ULL;
  v.epoch = 0x0102030405060708ULL;
  v.proposed_epoch = 0x0102030405060708ULL;
  v.sequence = 0x0102030405060708ULL;
  v.source_state_seq = 0x0102030405060708ULL;
  v.fault_id = 0x0102030405060708ULL;
  v.stamp_us = 0x0102030405060708ULL;
  v.operation = 0x01020304U;
  v.status = 0x01020304U;
  v.reason = 0x01020304U;
  v.execution_state = 0x01020304U;
  v.output_action = 0x01020304U;
  for (std::size_t i = 0; i < v.content_digest.size(); ++i)
    v.content_digest[i] = i + 1;
  return v;
}
Hello GoldenHello() {
  Hello v;
  v.role = static_cast<Role>(1);
  for (std::size_t i = 0; i < v.config_digest.size(); ++i)
    v.config_digest[i] = i + 1;
  for (std::size_t i = 0; i < v.application_digest.size(); ++i)
    v.application_digest[i] = i + 1;
  for (std::size_t i = 0; i < v.engine_digest.size(); ++i)
    v.engine_digest[i] = i + 1;
  for (std::size_t i = 0; i < v.runtime_id.size(); ++i) v.runtime_id[i] = i + 1;
  v.physics_dt_us = 0x0102030405060708ULL;
  v.control_dt_us = 0x0102030405060708ULL;
  v.physics_dt_us = 1000;
  v.control_dt_us = 2000;
  return v;
}
Feedback GoldenFeedback() {
  Feedback v;
  v.physics_step = 0x0102030405060708ULL;
  v.engine_time_s = 1.25;
  v.q.count = 2;
  v.q.values[0] = 1.25;
  v.q.values[1] = -2.5;
  v.dq.count = 2;
  v.dq.values[0] = 1.25;
  v.dq.values[1] = -2.5;
  v.tau.count = 2;
  v.tau.values[0] = 1.25;
  v.tau.values[1] = -2.5;
  for (std::size_t i = 0; i < v.imu_quaternion.size(); ++i)
    v.imu_quaternion[i] = (i + 1) * 0.125;
  for (std::size_t i = 0; i < v.imu_angular_velocity.size(); ++i)
    v.imu_angular_velocity[i] = (i + 1) * 0.125;
  for (std::size_t i = 0; i < v.imu_specific_force.size(); ++i)
    v.imu_specific_force[i] = (i + 1) * 0.125;
  for (std::size_t i = 0; i < v.base_position.size(); ++i)
    v.base_position[i] = (i + 1) * 0.125;
  for (std::size_t i = 0; i < v.base_velocity.size(); ++i)
    v.base_velocity[i] = (i + 1) * 0.125;
  v.valid = true;
  v.imu_valid = true;
  return v;
}
Torque GoldenTorque() {
  Torque v;
  v.write_id = 0x0102030405060708ULL;
  v.accepted = true;
  v.state_seq = 0x0102030405060708ULL;
  v.command = GoldenMessageIdentity();
  v.tau.count = 2;
  v.tau.values[0] = 1.25;
  v.tau.values[1] = -2.5;
  return v;
}
Manifest GoldenManifest() {
  Manifest v;
  v.sender = static_cast<Role>(1);
  v.message = GoldenMessageIdentity();
  return v;
}
Ack GoldenAck() {
  Ack v;
  v.consumer = static_cast<Role>(1);
  v.disposition = static_cast<Disposition>(1);
  v.manifest = GoldenManifest();
  return v;
}
ReleaseArm GoldenReleaseArm() {
  ReleaseArm v;
  v.tracking_request = GoldenMessageIdentity();
  v.first_command = GoldenMessageIdentity();
  v.applied_result = GoldenMessageIdentity();
  v.tracking_state = GoldenMessageIdentity();
  return v;
}
LeaveReady GoldenLeaveReady() {
  LeaveReady v;
  v.stop_request = GoldenMessageIdentity();
  v.stop_result = GoldenMessageIdentity();
  v.stopped_state = GoldenMessageIdentity();
  return v;
}
LeaveCommitted GoldenLeaveCommitted() {
  LeaveCommitted v;
  v.proof = GoldenLeaveReady();
  v.authorization_revoked = true;
  v.session_closed = true;
  v.heartbeat_resolved = true;
  return v;
}
Abort GoldenAbort() {
  Abort v;
  v.code = 0x01020304U;
  v.reason.Assign(std::string_view("stop\0proof", 10));
  return v;
}
AppliedToStep GoldenAppliedToStep() {
  AppliedToStep v;
  v.write_id = 0x0102030405060708ULL;
  v.physics_step_begin = 0x0102030405060708ULL;
  v.physics_step_end = 0x0102030405060708ULL;
  return v;
}
PhaseSeal GoldenPhaseSeal() {
  PhaseSeal seal;
  seal.sender = Role::Runtime;
  seal.manifest_count = 33;
  for (std::size_t i = 0; i < seal.manifest_digest.size(); ++i)
    seal.manifest_digest[i] = i + 1;
  return seal;
}
PhaseAck GoldenPhaseAck() {
  return {Role::Application, Disposition::BusinessConsumed, GoldenPhaseSeal()};
}
Payload GoldenPayload(FrameType kind) {
  switch (kind) {
    case FrameType::Hello:
      return GoldenHello();
    case FrameType::Feedback:
      return GoldenFeedback();
    case FrameType::Torque:
      return GoldenTorque();
    case FrameType::Manifest:
      return GoldenManifest();
    case FrameType::Ack:
      return GoldenAck();
    case FrameType::ReleaseArm:
      return GoldenReleaseArm();
    case FrameType::LeaveReady:
      return GoldenLeaveReady();
    case FrameType::LeaveCommitted:
      return GoldenLeaveCommitted();
    case FrameType::Abort:
      return GoldenAbort();
    case FrameType::AppliedToStep:
      return GoldenAppliedToStep();
    case FrameType::PhaseSeal:
      return GoldenPhaseSeal();
    case FrameType::PhaseAck:
      return GoldenPhaseAck();
  }
  return Hello{};
}
int main(int argc, char** argv) {
  ByteBuffer wire;
  Frame frame;
  if (argc == 3 && std::string(argv[1]) == "fixture") {
    frame.payload = GoldenPayload(static_cast<FrameType>(std::stoi(argv[2])));
    for (std::size_t i = 0; i < frame.identity.run_id.size(); ++i)
      frame.identity.run_id[i] = i + 1;
    frame.identity.control_step = 0x010203;
    frame.identity.phase = 0x01020304;
    frame.identity.logical_us = 1 + 0x010203ULL * 2000;
    CHECK(Encode(frame, &wire));
  } else if (argc == 2) {
    const std::string hex = argv[1];
    CHECK(hex.size() / 2 <= kMaxPacketBytes);
    wire.size = hex.size() / 2;
    for (std::size_t i = 0; i < wire.size; ++i)
      wire.bytes[i] = std::stoul(hex.substr(2 * i, 2), nullptr, 16);
    CHECK(Decode({wire.bytes.data(), wire.size}, &frame));
    ByteBuffer encoded;
    CHECK(Encode(frame, &encoded));
    CHECK(encoded.size == wire.size);
    CHECK(encoded.bytes == wire.bytes);
  } else {
    for (unsigned kind = 1; kind <= 12; ++kind) {
      frame.payload = GoldenPayload(static_cast<FrameType>(kind));
      forbid_allocation = true;
      CHECK(Encode(frame, &wire));
      Frame copy;
      CHECK(Decode({wire.bytes.data(), wire.size}, &copy));
      for (std::size_t n = 0; n < wire.size; ++n)
        CHECK(!Decode({wire.bytes.data(), n}, &copy));
      forbid_allocation = false;
    }
    CHECK(kMaxPhaseManifestEntries == 64);
    for (std::uint32_t count : {0U, 33U, 64U, 65U}) {
      auto seal = GoldenPhaseSeal();
      seal.manifest_count = count;
      frame.payload = seal;
      CHECK(Encode(frame, &wire) == (count <= 64));
      frame.payload =
          PhaseAck{Role::Application, Disposition::BusinessConsumed, seal};
      CHECK(Encode(frame, &wire) == (count <= 64));
    }
    frame.physics_dt_us = 3;
    CHECK(!Encode(frame, &wire));
    frame.physics_dt_us = 1000;
    frame.identity.control_step = std::numeric_limits<std::uint64_t>::max();
    CHECK(!Encode(frame, &wire));
    frame = Frame{};
    auto hello = Hello{};
    hello.physics_dt_us = 2;
    frame.payload = hello;
    CHECK(!Encode(frame, &wire));
    auto expected = Hello{};
    expected.engine_digest[0] = 1;
    CHECK(!MatchesHello(Hello{}, expected));
    frame.payload = Manifest{};
    std::get<Manifest>(frame.payload).message.kind =
        static_cast<MessageKind>(0);
    CHECK(!Encode(frame, &wire));
    frame.payload = GoldenPayload(FrameType::Feedback);
    std::get<Feedback>(frame.payload).engine_time_s =
        std::numeric_limits<double>::quiet_NaN();
    CHECK(!Encode(frame, &wire));
    CHECK(!Decode({wire.bytes.data(), kMaxPacketBytes + 1}, &frame));
    CHECK(!Decode({nullptr, 1}, &frame));
    CHECK(!Encode(frame, nullptr));
    return 0;
  }
  const char* digits = "0123456789abcdef";
  for (std::size_t i = 0; i < wire.size; ++i)
    std::cout << digits[wire.bytes[i] >> 4] << digits[wire.bytes[i] & 15];
  std::cout << '\n';
}
