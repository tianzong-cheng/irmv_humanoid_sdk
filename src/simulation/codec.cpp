#include "irmv/simulation/codec.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>
namespace irmv::simulation {
namespace {
template <class T>
bool EnumValid(T v) noexcept {
  const auto n = static_cast<std::uint32_t>(v);
  if constexpr (std::is_same_v<T, Role>) return n >= 1 && n <= 3;
  if constexpr (std::is_same_v<T, MessageKind>) return n >= 1 && n <= 5;
  if constexpr (std::is_same_v<T, Disposition>) return n >= 1 && n <= 2;
  return true;
}
struct Writer {
  ByteBuffer& b;
  bool ok{true};
  template <class T>
  void Integer(T v) noexcept {
    if (b.size + sizeof(T) > b.bytes.size()) {
      ok = false;
      return;
    }
    for (std::size_t i = 0; i < sizeof(T); ++i)
      b.bytes[b.size++] = static_cast<std::uint8_t>(v >> (i * 8));
  }
  template <class T>
  void operator()(const T& v) noexcept {
    if constexpr (std::is_enum_v<T>) {
      if (!EnumValid(v)) ok = false;
      Integer(static_cast<std::underlying_type_t<T>>(v));
    } else if constexpr (std::is_same_v<T, bool>)
      Integer<std::uint8_t>(v ? 1 : 0);
    else if constexpr (std::is_integral_v<T>)
      Integer(v);
    else if constexpr (std::is_same_v<T, double>) {
      static_assert(sizeof(double) == 8 &&
                    std::numeric_limits<double>::is_iec559);
      if (!std::isfinite(v)) ok = false;
      std::uint64_t bits;
      std::memcpy(&bits, &v, 8);
      Integer(bits);
    } else
      Visit(*this, v);
  }
  template <class T, std::size_t N>
  void operator()(const std::array<T, N>& v) noexcept {
    for (const auto& x : v) (*this)(x);
  }
  void operator()(const Joints& v) noexcept {
    (*this)(v.count);
    if (v.count > kMaxJoints) {
      ok = false;
      return;
    }
    for (std::size_t i = 0; i < v.count; ++i) (*this)(v.values[i]);
  }
  void operator()(const protocol::Text256& v) noexcept {
    (*this)(v.count);
    if (v.count > 256) {
      ok = false;
      return;
    }
    for (std::size_t i = 0; i < v.count; ++i)
      Integer<std::uint8_t>(static_cast<std::uint8_t>(v.bytes[i]));
  }
};
struct Reader {
  ByteView b;
  std::size_t pos{0};
  bool ok{true};
  template <class T>
  T Integer() noexcept {
    if (pos > b.size || sizeof(T) > b.size - pos) {
      ok = false;
      return 0;
    }
    T v = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i)
      v |= static_cast<T>(b.data[pos++]) << (i * 8);
    return v;
  }
  template <class T>
  void operator()(T& v) noexcept {
    if constexpr (std::is_enum_v<T>) {
      v = static_cast<T>(Integer<std::underlying_type_t<T>>());
      if (!EnumValid(v)) ok = false;
    } else if constexpr (std::is_same_v<T, bool>) {
      auto x = Integer<std::uint8_t>();
      if (x > 1) ok = false;
      v = x != 0;
    } else if constexpr (std::is_integral_v<T>)
      v = Integer<T>();
    else if constexpr (std::is_same_v<T, double>) {
      auto bits = Integer<std::uint64_t>();
      std::memcpy(&v, &bits, 8);
      if (!std::isfinite(v)) ok = false;
    } else
      Visit(*this, v);
  }
  template <class T, std::size_t N>
  void operator()(std::array<T, N>& v) noexcept {
    for (auto& x : v) (*this)(x);
  }
  void operator()(Joints& v) noexcept {
    (*this)(v.count);
    if (v.count > kMaxJoints) {
      ok = false;
      return;
    }
    for (std::size_t i = 0; i < v.count; ++i) (*this)(v.values[i]);
  }
  void operator()(protocol::Text256& v) noexcept {
    (*this)(v.count);
    if (v.count > 256) {
      ok = false;
      return;
    }
    for (std::size_t i = 0; i < v.count; ++i)
      v.bytes[i] = static_cast<char>(Integer<std::uint8_t>());
  }
};
template <class A, class V>
void VisitMessageIdentity(A& a, V& v) noexcept {
  a(v.kind);
  a(v.runtime_id);
  a(v.supervisor_id);
  a(v.session_id);
  a(v.producer_id);
  a(v.request_id);
  a(v.epoch);
  a(v.proposed_epoch);
  a(v.sequence);
  a(v.source_state_seq);
  a(v.fault_id);
  a(v.stamp_us);
  a(v.operation);
  a(v.status);
  a(v.reason);
  a(v.execution_state);
  a(v.output_action);
  a(v.content_digest);
}
template <class A>
void Visit(A& a, MessageIdentity& v) noexcept {
  VisitMessageIdentity(a, v);
}
template <class A>
void Visit(A& a, const MessageIdentity& v) noexcept {
  VisitMessageIdentity(a, v);
}
template <class A, class V>
void VisitHello(A& a, V& v) noexcept {
  a(v.role);
  a(v.config_digest);
  a(v.application_digest);
  a(v.engine_digest);
  a(v.runtime_id);
  a(v.physics_dt_us);
  a(v.control_dt_us);
}
template <class A>
void Visit(A& a, Hello& v) noexcept {
  VisitHello(a, v);
}
template <class A>
void Visit(A& a, const Hello& v) noexcept {
  VisitHello(a, v);
}
template <class A, class V>
void VisitFeedback(A& a, V& v) noexcept {
  a(v.physics_step);
  a(v.engine_time_s);
  a(v.q);
  a(v.dq);
  a(v.tau);
  a(v.imu_quaternion);
  a(v.imu_angular_velocity);
  a(v.imu_specific_force);
  a(v.base_position);
  a(v.base_velocity);
  a(v.valid);
  a(v.imu_valid);
}
template <class A>
void Visit(A& a, Feedback& v) noexcept {
  VisitFeedback(a, v);
}
template <class A>
void Visit(A& a, const Feedback& v) noexcept {
  VisitFeedback(a, v);
}
template <class A, class V>
void VisitTorque(A& a, V& v) noexcept {
  a(v.write_id);
  a(v.accepted);
  a(v.state_seq);
  a(v.command);
  a(v.tau);
}
template <class A>
void Visit(A& a, Torque& v) noexcept {
  VisitTorque(a, v);
}
template <class A>
void Visit(A& a, const Torque& v) noexcept {
  VisitTorque(a, v);
}
template <class A, class V>
void VisitManifest(A& a, V& v) noexcept {
  a(v.sender);
  a(v.message);
}
template <class A>
void Visit(A& a, Manifest& v) noexcept {
  VisitManifest(a, v);
}
template <class A>
void Visit(A& a, const Manifest& v) noexcept {
  VisitManifest(a, v);
}
template <class A, class V>
void VisitAck(A& a, V& v) noexcept {
  a(v.consumer);
  a(v.disposition);
  a(v.manifest);
}
template <class A>
void Visit(A& a, Ack& v) noexcept {
  VisitAck(a, v);
}
template <class A>
void Visit(A& a, const Ack& v) noexcept {
  VisitAck(a, v);
}
template <class A, class V>
void VisitReleaseArm(A& a, V& v) noexcept {
  a(v.tracking_request);
  a(v.first_command);
  a(v.applied_result);
  a(v.tracking_state);
}
template <class A>
void Visit(A& a, ReleaseArm& v) noexcept {
  VisitReleaseArm(a, v);
}
template <class A>
void Visit(A& a, const ReleaseArm& v) noexcept {
  VisitReleaseArm(a, v);
}
template <class A, class V>
void VisitLeaveReady(A& a, V& v) noexcept {
  a(v.stop_request);
  a(v.stop_result);
  a(v.stopped_state);
}
template <class A>
void Visit(A& a, LeaveReady& v) noexcept {
  VisitLeaveReady(a, v);
}
template <class A>
void Visit(A& a, const LeaveReady& v) noexcept {
  VisitLeaveReady(a, v);
}
template <class A, class V>
void VisitLeaveCommitted(A& a, V& v) noexcept {
  a(v.proof);
  a(v.authorization_revoked);
  a(v.session_closed);
  a(v.heartbeat_resolved);
}
template <class A>
void Visit(A& a, LeaveCommitted& v) noexcept {
  VisitLeaveCommitted(a, v);
}
template <class A>
void Visit(A& a, const LeaveCommitted& v) noexcept {
  VisitLeaveCommitted(a, v);
}
template <class A, class V>
void VisitAbort(A& a, V& v) noexcept {
  a(v.code);
  a(v.reason);
}
template <class A>
void Visit(A& a, Abort& v) noexcept {
  VisitAbort(a, v);
}
template <class A>
void Visit(A& a, const Abort& v) noexcept {
  VisitAbort(a, v);
}
template <class A, class V>
void VisitAppliedToStep(A& a, V& v) noexcept {
  a(v.write_id);
  a(v.physics_step_begin);
  a(v.physics_step_end);
}
template <class A>
void Visit(A& a, AppliedToStep& v) noexcept {
  VisitAppliedToStep(a, v);
}
template <class A>
void Visit(A& a, const AppliedToStep& v) noexcept {
  VisitAppliedToStep(a, v);
}
template <class A, class V>
void VisitPhaseSeal(A& a, V& v) noexcept {
  a(v.sender);
  a(v.manifest_count);
  a(v.manifest_digest);
}
template <class A>
void Visit(A& a, PhaseSeal& v) noexcept {
  VisitPhaseSeal(a, v);
}
template <class A>
void Visit(A& a, const PhaseSeal& v) noexcept {
  VisitPhaseSeal(a, v);
}
template <class A, class V>
void VisitPhaseAck(A& a, V& v) noexcept {
  a(v.consumer);
  a(v.disposition);
  a(v.seal);
}
template <class A>
void Visit(A& a, PhaseAck& v) noexcept {
  VisitPhaseAck(a, v);
}
template <class A>
void Visit(A& a, const PhaseAck& v) noexcept {
  VisitPhaseAck(a, v);
}
template <class A, class V>
void VisitInventoryReady(A& a, V& v) noexcept {
  a(v.consumer);
  a(v.seal);
}
template <class A>
void Visit(A& a, InventoryReady& v) noexcept {
  VisitInventoryReady(a, v);
}
template <class A>
void Visit(A& a, const InventoryReady& v) noexcept {
  VisitInventoryReady(a, v);
}
template <class A>
void Visit(A& a, PublishPermit& v) noexcept {
  a(v.seal);
}
template <class A>
void Visit(A& a, const PublishPermit& v) noexcept {
  a(v.seal);
}
template <class A, class V>
void VisitTickPermit(A& a, V& v) noexcept {
  a(v.role);
  a(v.stage);
}
template <class A>
void Visit(A& a, TickPermit& v) noexcept {
  VisitTickPermit(a, v);
}
template <class A>
void Visit(A& a, const TickPermit& v) noexcept {
  VisitTickPermit(a, v);
}
template <class A, class V>
void VisitFirstCommandProof(A& a, V& v) noexcept {
  a(v.request);
  a(v.command);
}
template <class A>
void Visit(A& a, FirstCommandProof& v) noexcept {
  VisitFirstCommandProof(a, v);
}
template <class A>
void Visit(A& a, const FirstCommandProof& v) noexcept {
  VisitFirstCommandProof(a, v);
}
template <class A, class V>
void VisitLifecycleRole(A& a, V& v) noexcept {
  a(v.role);
}
template <class A>
void Visit(A& a, ShutdownPermit& v) noexcept {
  VisitLifecycleRole(a, v);
}
template <class A>
void Visit(A& a, const ShutdownPermit& v) noexcept {
  VisitLifecycleRole(a, v);
}
template <class A>
void Visit(A& a, RuntimeOnlyPermit& v) noexcept {
  VisitLifecycleRole(a, v);
}
template <class A>
void Visit(A& a, const RuntimeOnlyPermit& v) noexcept {
  VisitLifecycleRole(a, v);
}
template <class A, class V>
void VisitShutdownCommitted(A& a, V& v) noexcept {
  a(v.write_id);
  a(v.state_seq);
}
template <class A>
void Visit(A& a, ShutdownCommitted& v) noexcept {
  VisitShutdownCommitted(a, v);
}
template <class A>
void Visit(A& a, const ShutdownCommitted& v) noexcept {
  VisitShutdownCommitted(a, v);
}
template <class A>
void Visit(A& a, ReleaseCancel& v) noexcept {
  VisitLifecycleRole(a, v);
}
template <class A>
void Visit(A& a, const ReleaseCancel& v) noexcept {
  VisitLifecycleRole(a, v);
}
bool Valid(const Frame& f) noexcept {
  const auto dt = f.control_dt_us;
  if (!dt || !f.physics_dt_us || dt % f.physics_dt_us ||
      f.identity.control_step >
          (std::numeric_limits<std::uint64_t>::max() - 1) / dt ||
      f.identity.logical_us != 1 + f.identity.control_step * dt)
    return false;
  if (auto p = std::get_if<ReleaseCancel>(&f.payload))
    return p->role >= Role::Coordinator && p->role <= Role::Application;
  if (auto p = std::get_if<ShutdownPermit>(&f.payload))
    return p->role >= Role::Coordinator && p->role <= Role::Application;
  if (auto p = std::get_if<RuntimeOnlyPermit>(&f.payload))
    return p->role >= Role::Coordinator && p->role <= Role::Application;
  if (auto h = std::get_if<Hello>(&f.payload))
    return h->physics_dt_us == f.physics_dt_us && h->control_dt_us == dt &&
           h->role >= Role::Coordinator && h->role <= Role::Application;
  if (auto b = std::get_if<Feedback>(&f.payload))
    return b->q.count == b->dq.count && b->q.count == b->tau.count;
  if (auto m = std::get_if<Manifest>(&f.payload))
    return m->sender >= Role::Coordinator && m->sender <= Role::Application &&
           m->message.kind >= MessageKind::Request &&
           m->message.kind <= MessageKind::Command;
  if (auto a = std::get_if<Ack>(&f.payload))
    return a->consumer >= Role::Coordinator &&
           a->consumer <= Role::Application &&
           a->disposition >= Disposition::Ingress &&
           a->disposition <= Disposition::BusinessConsumed &&
           a->manifest.sender >= Role::Coordinator &&
           a->manifest.sender <= Role::Application &&
           a->manifest.message.kind >= MessageKind::Request &&
           a->manifest.message.kind <= MessageKind::Command;
  if (auto seal = std::get_if<PhaseSeal>(&f.payload))
    return seal->manifest_count <= kMaxPhaseManifestEntries;
  if (auto ack = std::get_if<PhaseAck>(&f.payload))
    return ack->seal.manifest_count <= kMaxPhaseManifestEntries;
  if (auto ready = std::get_if<InventoryReady>(&f.payload))
    return ready->seal.manifest_count <= kMaxPhaseManifestEntries;
  if (auto permit = std::get_if<PublishPermit>(&f.payload))
    return permit->seal.manifest_count <= kMaxPhaseManifestEntries;
  if (auto permit = std::get_if<TickPermit>(&f.payload))
    return permit->stage == 1 || permit->stage == 2;
  if (auto proof = std::get_if<FirstCommandProof>(&f.payload))
    return proof->request.kind == MessageKind::Request &&
           proof->command.kind == MessageKind::Command;
  if (auto a = std::get_if<AppliedToStep>(&f.payload))
    return a->physics_step_end >= a->physics_step_begin;
  return true;
}
}  // namespace
bool MatchesHello(const Hello& a, const Hello& b) noexcept {
  return a.config_digest == b.config_digest &&
         a.application_digest == b.application_digest &&
         a.engine_digest == b.engine_digest && a.runtime_id == b.runtime_id &&
         a.physics_dt_us == b.physics_dt_us &&
         a.control_dt_us == b.control_dt_us;
}
bool Encode(const Frame& f, ByteBuffer* out) noexcept {
  if (!out || !Valid(f) || f.payload.valueless_by_exception()) return false;
  out->size = 0;
  Writer a{*out};
  a(std::uint32_t{0});
  a(kLocalProtocolVersion);
  a(static_cast<std::uint16_t>(f.payload.index() + 1));
  a(f.identity.run_id);
  a(f.identity.control_step);
  a(f.identity.phase);
  a(f.identity.logical_us);
  a(f.physics_dt_us);
  a(f.control_dt_us);
  std::visit([&a](const auto& v) { a(v); }, f.payload);
  if (!a.ok) {
    out->size = 0;
    return false;
  }
  auto length = static_cast<std::uint32_t>(out->size);
  for (unsigned i = 0; i < 4; ++i)
    out->bytes[i] = static_cast<std::uint8_t>(length >> (8 * i));
  return true;
}
bool Decode(ByteView b, Frame* out) noexcept {
  if (!out || !b.data || b.size > kMaxPacketBytes || b.size < 60) return false;
  Reader a{b};
  if (a.Integer<std::uint32_t>() != b.size ||
      a.Integer<std::uint16_t>() != kLocalProtocolVersion)
    return false;
  const auto kind = a.Integer<std::uint16_t>();
  Frame f;
  a(f.identity.run_id);
  a(f.identity.control_step);
  a(f.identity.phase);
  a(f.identity.logical_us);
  a(f.physics_dt_us);
  a(f.control_dt_us);
  switch (kind) {
    case 1: {
      Hello v;
      a(v);
      f.payload = v;
      break;
    }
    case 2: {
      Feedback v;
      a(v);
      f.payload = v;
      break;
    }
    case 3: {
      Torque v;
      a(v);
      f.payload = v;
      break;
    }
    case 4: {
      Manifest v;
      a(v);
      f.payload = v;
      break;
    }
    case 5: {
      Ack v;
      a(v);
      f.payload = v;
      break;
    }
    case 6: {
      ReleaseArm v;
      a(v);
      f.payload = v;
      break;
    }
    case 7: {
      LeaveReady v;
      a(v);
      f.payload = v;
      break;
    }
    case 8: {
      LeaveCommitted v;
      a(v);
      f.payload = v;
      break;
    }
    case 9: {
      Abort v;
      a(v);
      f.payload = v;
      break;
    }
    case 10: {
      AppliedToStep v;
      a(v);
      f.payload = v;
      break;
    }
    case 11: {
      PhaseSeal v;
      a(v);
      f.payload = v;
      break;
    }
    case 12: {
      PhaseAck v;
      a(v);
      f.payload = v;
      break;
    }
    case 13: {
      InventoryReady v;
      a(v);
      f.payload = v;
      break;
    }
    case 14: {
      PublishPermit v;
      a(v);
      f.payload = v;
      break;
    }
    case 15: {
      TickPermit v;
      a(v);
      f.payload = v;
      break;
    }
    case 16: {
      FirstCommandProof v;
      a(v);
      f.payload = v;
      break;
    }
    case 17: {
      ShutdownPermit v;
      a(v);
      f.payload = v;
      break;
    }
    case 18: {
      RuntimeOnlyPermit v;
      a(v);
      f.payload = v;
      break;
    }
    case 19: {
      ShutdownCommitted v;
      a(v);
      f.payload = v;
      break;
    }
    case 20: {
      ReleaseCancel v;
      a(v);
      f.payload = v;
      break;
    }
    default:
      return false;
  }
  if (!a.ok || a.pos != b.size || !Valid(f)) return false;
  *out = f;
  return true;
}
}  // namespace irmv::simulation
