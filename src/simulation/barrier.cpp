#include "irmv/simulation/barrier.hpp"

#include <cmath>
#include <cstring>
#include <type_traits>

#include "irmv/simulation/codec.hpp"
namespace irmv::simulation {
namespace {
constexpr std::array<std::uint32_t, 64> kRoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

std::uint32_t RotateRight(std::uint32_t value, std::uint32_t count) {
  return (value >> count) | (value << (32U - count));
}

class Hash {
 public:
  void Byte(std::uint8_t v) noexcept {
    block_[used_++] = v;
    ++size_;
    if (used_ == 64) {
      Block();
      used_ = 0;
    }
  }
  void Bytes(const std::uint8_t* p, std::size_t n) noexcept {
    for (std::size_t i = 0; i < n; ++i) Byte(p[i]);
  }
  Digest Finish() noexcept {
    auto bits = size_ * 8;
    Byte(0x80);
    while (used_ != 56) Byte(0);
    for (int i = 56; i >= 0; i -= 8) Byte(static_cast<std::uint8_t>(bits >> i));
    Digest d{};
    for (std::size_t i = 0; i < 8; ++i)
      for (int j = 0; j < 4; ++j) d[i * 4 + j] = hash_[i] >> (24 - j * 8);
    return d;
  }

 private:
  void Block() noexcept {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t offset = index * 4U;
      words[index] = (static_cast<std::uint32_t>(block_[offset]) << 24U) |
                     (static_cast<std::uint32_t>(block_[offset + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(block_[offset + 2U]) << 8U) |
                     static_cast<std::uint32_t>(block_[offset + 3U]);
    }
    for (std::size_t index = 16; index < words.size(); ++index) {
      const std::uint32_t s0 = RotateRight(words[index - 15U], 7U) ^
                               RotateRight(words[index - 15U], 18U) ^
                               (words[index - 15U] >> 3U);
      const std::uint32_t s1 = RotateRight(words[index - 2U], 17U) ^
                               RotateRight(words[index - 2U], 19U) ^
                               (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }

    std::uint32_t a = hash_[0];
    std::uint32_t b = hash_[1];
    std::uint32_t c = hash_[2];
    std::uint32_t d = hash_[3];
    std::uint32_t e = hash_[4];
    std::uint32_t f = hash_[5];
    std::uint32_t g = hash_[6];
    std::uint32_t h = hash_[7];
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t sum1 =
          RotateRight(e, 6U) ^ RotateRight(e, 11U) ^ RotateRight(e, 25U);
      const std::uint32_t choice = (e & f) ^ (~e & g);
      const std::uint32_t temporary1 =
          h + sum1 + choice + kRoundConstants[index] + words[index];
      const std::uint32_t sum0 =
          RotateRight(a, 2U) ^ RotateRight(a, 13U) ^ RotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }
    hash_[0] += a;
    hash_[1] += b;
    hash_[2] += c;
    hash_[3] += d;
    hash_[4] += e;
    hash_[5] += f;
    hash_[6] += g;
    hash_[7] += h;
  }
  std::array<std::uint8_t, 64> block_{};
  std::array<std::uint32_t, 8> hash_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                     0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                     0x1f83d9abU, 0x5be0cd19U};
  std::size_t used_{0};
  std::uint64_t size_{0};
};
struct Canonical {
  Hash hash;
  bool valid{true};
  template <class T>
  void Put(T v) noexcept {
    if constexpr (std::is_enum_v<T>)
      Put(static_cast<std::uint32_t>(v));
    else if constexpr (std::is_same_v<T, bool>)
      hash.Byte(v ? 1 : 0);
    else if constexpr (std::is_same_v<T, double>) {
      if (!std::isfinite(v)) valid = false;
      std::uint64_t bits;
      std::memcpy(&bits, &v, 8);
      Put(bits);
    } else
      for (std::size_t i = 0; i < sizeof(T); ++i) hash.Byte(v >> (8 * i));
  }
  template <class T, std::size_t N>
  void Put(const std::array<T, N>& a) noexcept {
    for (auto v : a) Put(v);
  }
  template <std::size_t N>
  void Put(const protocol::BoundedText<N>& t) noexcept {
    Put(t.count);
    if (t.count > N) {
      valid = false;
      return;
    }
    for (std::uint32_t i = 0; i < t.count; ++i) hash.Byte(t.bytes[i]);
  }
  template <class T, std::size_t N>
  void Active(const std::array<T, N>& a, std::uint32_t n) noexcept {
    if (n > N) {
      valid = false;
      return;
    }
    for (std::uint32_t i = 0; i < n; ++i) Put(a[i]);
  }
  void Put(const protocol::JointVector& v) noexcept {
    Put(v.count);
    Active(v.q, v.count);
    Active(v.dq, v.count);
    Active(v.tau_ff, v.count);
    Active(v.kp, v.count);
    Active(v.kd, v.count);
  }
  void Put(const protocol::JointCommand& v) noexcept {
    Put(v.protocol_version);
    Put(v.runtime_id);
    Put(v.session_id);
    Put(v.producer_id);
    Put(v.epoch);
    Put(v.command_seq);
    Put(v.source_state_seq);
    Put(v.stamp_us);
    Put(v.layout_id);
    Put(v.joints);
  }
  void Put(const protocol::ExecutionRequest& v) noexcept {
    Put(v.protocol_version);
    Put(v.supervisor_id);
    Put(v.request_id);
    Put(v.session_id);
    Put(v.runtime_id);
    Put(v.expected_epoch);
    Put(v.proposed_epoch);
    Put(v.producer_id);
    Put(v.fault_id);
    Put(v.operation);
    Put(v.first_command);
  }
  void Put(const protocol::ExecutionResult& v) noexcept {
    Put(v.protocol_version);
    Put(v.supervisor_id);
    Put(v.request_id);
    Put(v.session_id);
    Put(v.runtime_id);
    Put(v.execution_state);
    Put(v.output_action);
    Put(v.producer_id);
    Put(v.epoch);
    Put(v.fault_id);
    Put(v.stamp_us);
    Put(v.status);
    Put(v.reason);
  }
  void Put(const protocol::SupervisorHeartbeat& v) noexcept {
    Put(v.protocol_version);
    Put(v.supervisor_id);
    Put(v.session_id);
    Put(v.runtime_id);
    Put(v.heartbeat_seq);
    Put(v.stamp_us);
  }
  void Put(const protocol::JointFeedback& v) noexcept {
    Put(v.valid);
    Put(v.connected);
    Put(v.q);
    Put(v.dq);
    Put(v.tau_est);
    Put(v.rx_stamp_us);
    Put(v.freq_hz);
    Put(v.error_code);
  }
  void Put(const protocol::ImuFeedback& v) noexcept {
    Put(v.valid);
    Put(v.orientation_rpy);
    Put(v.angular_velocity);
    Put(v.linear_acceleration);
    Put(v.rx_stamp_us);
  }
  void Put(const protocol::FaultDetail& v) noexcept {
    Put(v.reason);
    Put(v.condition);
    Put(v.joint_index);
    Put(v.error_code);
    Put(v.observed_value);
    Put(v.limit);
    Put(v.sample_stamp_us);
    Put(v.detected_stamp_us);
    Put(v.output_action);
    Put(v.message);
  }
  void Put(const protocol::RobotState& v) noexcept {
    Put(v.protocol_version);
    Put(v.runtime_id);
    Put(v.session_id);
    Put(v.producer_id);
    Put(v.epoch);
    Put(v.state_seq);
    Put(v.stamp_us);
    Put(v.execution_state);
    Put(v.output_action);
    Put(v.stop_reason);
    Put(v.fault_id);
    Put(v.last_applied_command_seq);
    Put(v.healthy);
    Put(v.damping_eligible);
    Put(v.config_id);
    Put(v.layout_id);
    Put(v.feedback);
    Put(v.faults);
  }
  void Put(const protocol::FeedbackSnapshot& v) noexcept {
    Put(v.snapshot_seq);
    Put(v.stamp_us);
    Put(v.count);
    Active(v.joints, v.count);
    Put(v.imu);
  }
  void Put(const protocol::FaultReport& v) noexcept {
    Put(v.count);
    Active(v.details, v.count);
  }
};
bool Projection(const MessageIdentity& a, const MessageIdentity& b) noexcept {
  return a.kind == b.kind && a.runtime_id == b.runtime_id &&
         a.supervisor_id == b.supervisor_id && a.session_id == b.session_id &&
         a.producer_id == b.producer_id && a.request_id == b.request_id &&
         a.epoch == b.epoch && a.proposed_epoch == b.proposed_epoch &&
         a.sequence == b.sequence && a.source_state_seq == b.source_state_seq &&
         a.fault_id == b.fault_id && a.stamp_us == b.stamp_us &&
         a.operation == b.operation && a.status == b.status &&
         a.reason == b.reason && a.execution_state == b.execution_state &&
         a.output_action == b.output_action;
}
bool RolePair(Role sender, Role consumer) noexcept {
  return (sender == Role::Runtime && consumer == Role::Application) ||
         (sender == Role::Application && consumer == Role::Runtime);
}
Digest Inventory(const PhaseManifest& m) noexcept {
  Hash h;
  // Reuse the canonical codec's exact Manifest payload representation.
  ByteBuffer bytes;
  Frame f;
  f.identity = m.identity;
  f.physics_dt_us = m.physics_dt_us;
  f.control_dt_us = m.control_dt_us;
  for (std::uint32_t i = 0; i < m.count; ++i) {
    f.payload = Manifest{m.sender, m.messages[i]};
    if (!Encode(f, &bytes)) return {};
    h.Bytes(bytes.bytes.data() + 60, 180);
  }
  return h.Finish();
}
bool SameManifest(const PhaseManifest& a, const PhaseManifest& b) noexcept {
  if (!SamePhase(a.identity, b.identity) ||
      a.physics_dt_us != b.physics_dt_us ||
      a.control_dt_us != b.control_dt_us || a.sender != b.sender ||
      a.consumer != b.consumer || a.required != b.required ||
      a.count != b.count || a.seal.manifest_digest != b.seal.manifest_digest)
    return false;
  for (std::uint32_t i = 0; i < a.count; ++i)
    if (!SameIdentity(a.messages[i], b.messages[i])) return false;
  return true;
}
}  // namespace
bool SameIdentity(const MessageIdentity& a, const MessageIdentity& b) noexcept {
  return Projection(a, b) && a.content_digest == b.content_digest;
}
bool SamePhase(const StepIdentity& a, const StepIdentity& b) noexcept {
  return a.run_id == b.run_id && a.control_step == b.control_step &&
         a.phase == b.phase && a.logical_us == b.logical_us;
}
bool Identify(const protocol::JointCommand& v, MessageIdentity* out) noexcept {
  if (!out) return false;
  Canonical c;
  c.Put(v);
  if (!c.valid) return false;
  MessageIdentity m;
  m.kind = MessageKind::Command;
  m.runtime_id = v.runtime_id;
  m.session_id = v.session_id;
  m.producer_id = v.producer_id;
  m.epoch = v.epoch;
  m.source_state_seq = v.source_state_seq;
  m.stamp_us = v.stamp_us;
  m.sequence = static_cast<decltype(m.sequence)>(v.command_seq);
  m.content_digest = c.hash.Finish();
  *out = m;
  return true;
}
bool Identify(const protocol::ExecutionRequest& v,
              MessageIdentity* out) noexcept {
  if (!out) return false;
  Canonical c;
  c.Put(v);
  if (!c.valid) return false;
  MessageIdentity m;
  m.kind = MessageKind::Request;
  m.runtime_id = v.runtime_id;
  m.supervisor_id = v.supervisor_id;
  m.session_id = v.session_id;
  m.producer_id = v.producer_id;
  m.request_id = v.request_id;
  m.proposed_epoch = v.proposed_epoch;
  m.fault_id = v.fault_id;
  m.epoch = static_cast<decltype(m.epoch)>(v.expected_epoch);
  m.operation = static_cast<decltype(m.operation)>(v.operation);
  m.content_digest = c.hash.Finish();
  *out = m;
  return true;
}
bool Identify(const protocol::ExecutionResult& v,
              MessageIdentity* out) noexcept {
  if (!out) return false;
  Canonical c;
  c.Put(v);
  if (!c.valid) return false;
  MessageIdentity m;
  m.kind = MessageKind::Result;
  m.runtime_id = v.runtime_id;
  m.supervisor_id = v.supervisor_id;
  m.session_id = v.session_id;
  m.producer_id = v.producer_id;
  m.request_id = v.request_id;
  m.epoch = v.epoch;
  m.fault_id = v.fault_id;
  m.stamp_us = v.stamp_us;
  m.status = static_cast<decltype(m.status)>(v.status);
  m.reason = static_cast<decltype(m.reason)>(v.reason);
  m.execution_state =
      static_cast<decltype(m.execution_state)>(v.execution_state);
  m.output_action = static_cast<decltype(m.output_action)>(v.output_action);
  m.content_digest = c.hash.Finish();
  *out = m;
  return true;
}
bool Identify(const protocol::RobotState& v, MessageIdentity* out) noexcept {
  if (!out) return false;
  Canonical c;
  c.Put(v);
  if (!c.valid) return false;
  MessageIdentity m;
  m.kind = MessageKind::State;
  m.runtime_id = v.runtime_id;
  m.session_id = v.session_id;
  m.producer_id = v.producer_id;
  m.epoch = v.epoch;
  m.fault_id = v.fault_id;
  m.stamp_us = v.stamp_us;
  m.sequence = static_cast<decltype(m.sequence)>(v.state_seq);
  m.reason = static_cast<decltype(m.reason)>(v.stop_reason);
  m.execution_state =
      static_cast<decltype(m.execution_state)>(v.execution_state);
  m.output_action = static_cast<decltype(m.output_action)>(v.output_action);
  m.content_digest = c.hash.Finish();
  *out = m;
  return true;
}
bool Identify(const protocol::SupervisorHeartbeat& v,
              MessageIdentity* out) noexcept {
  if (!out) return false;
  Canonical c;
  c.Put(v);
  if (!c.valid) return false;
  MessageIdentity m;
  m.kind = MessageKind::Heartbeat;
  m.runtime_id = v.runtime_id;
  m.supervisor_id = v.supervisor_id;
  m.session_id = v.session_id;
  m.stamp_us = v.stamp_us;
  m.sequence = static_cast<decltype(m.sequence)>(v.heartbeat_seq);
  m.content_digest = c.hash.Finish();
  *out = m;
  return true;
}
bool PhaseManifest::Add(const MessageIdentity& m) noexcept {
  if (closed || count > kMaxPhaseManifestEntries) return false;
  for (std::uint32_t i = 0; i < count; ++i)
    if (Projection(messages[i], m)) return SameIdentity(messages[i], m);
  if (count == kMaxPhaseManifestEntries) return false;
  messages[count++] = m;
  return true;
}
bool PhaseManifest::Add(const Frame& f) noexcept {
  const auto* m = std::get_if<Manifest>(&f.payload);
  return m && SamePhase(identity, f.identity) &&
         physics_dt_us == f.physics_dt_us && control_dt_us == f.control_dt_us &&
         m->sender == sender && Add(m->message);
}
bool PhaseManifest::Close(const Frame& f) noexcept {
  const auto* s = std::get_if<PhaseSeal>(&f.payload);
  if (!s || !SamePhase(identity, f.identity) ||
      physics_dt_us != f.physics_dt_us || control_dt_us != f.control_dt_us)
    return false;
  auto candidate = *this;
  candidate.seal = *s;
  candidate.closed = true;
  if (!candidate.Valid()) return false;
  *this = candidate;
  return true;
}
bool PhaseManifest::Close() noexcept {
  if (closed) return Valid();
  if (count > kMaxPhaseManifestEntries) return false;
  seal = {sender, count, Inventory(*this)};
  closed = true;
  return Valid();
}
bool PhaseManifest::Valid() const noexcept {
  if (!closed || count > kMaxPhaseManifestEntries ||
      !RolePair(sender, consumer) || identity.run_id == protocol::kNoId ||
      identity.logical_us == 0 || physics_dt_us == 0 || control_dt_us == 0 ||
      control_dt_us % physics_dt_us != 0 || seal.sender != sender ||
      seal.manifest_count != count ||
      (required != Disposition::Ingress &&
       required != Disposition::BusinessConsumed))
    return false;
  for (std::uint32_t i = 0; i < count; ++i) {
    auto kind = messages[i].kind;
    if (sender == Role::Runtime
            ? (kind != MessageKind::State && kind != MessageKind::Result)
            : (kind != MessageKind::Request && kind != MessageKind::Command &&
               kind != MessageKind::Heartbeat))
      return false;
    for (std::uint32_t j = 0; j < i; ++j)
      if (Projection(messages[i], messages[j])) return false;
    Frame f;
    f.identity = identity;
    f.payload = Manifest{sender, messages[i]};
    ByteBuffer b;
    if (!Encode(f, &b)) return false;
  }
  return seal.manifest_digest == Inventory(*this);
}
bool Barrier::Begin(const PhaseManifest& m) noexcept {
  if (!m.Valid()) return false;
  const Phase* latest = nullptr;
  for (auto& p : phases_)
    if (p.used) {
      if (p.manifest.identity.run_id != m.identity.run_id) return false;
      if (p.manifest.identity.control_step == m.identity.control_step &&
          p.manifest.identity.phase == m.identity.phase) {
        if (!SameManifest(p.manifest, m)) {
          p.failed = true;
          return false;
        }
        return !p.failed;
      }
      if (!latest ||
          p.manifest.identity.control_step >
              latest->manifest.identity.control_step ||
          (p.manifest.identity.control_step ==
               latest->manifest.identity.control_step &&
           p.manifest.identity.phase > latest->manifest.identity.phase))
        latest = &p;
    }
  if (latest &&
      (m.identity.control_step < latest->manifest.identity.control_step ||
       (m.identity.control_step == latest->manifest.identity.control_step &&
        m.identity.phase <= latest->manifest.identity.phase) ||
       m.identity.logical_us < latest->manifest.identity.logical_us))
    return false;
  if (latest &&
      ((m.identity.control_step > latest->manifest.identity.control_step &&
        m.identity.logical_us <= latest->manifest.identity.logical_us) ||
       (m.identity.control_step == latest->manifest.identity.control_step &&
        m.identity.logical_us != latest->manifest.identity.logical_us)))
    return false;
  auto& p = phases_[next_];
  p = Phase{};
  p.used = true;
  p.manifest = m;
  next_ = (next_ + 1) % phases_.size();
  return true;
}
bool Barrier::Observe(const PhaseReceipt& r) noexcept {
  for (auto& p : phases_)
    if (p.used && SamePhase(p.manifest.identity, r.identity)) {
      if (p.failed || r.consumer != p.manifest.consumer ||
          r.physics_dt_us != p.manifest.physics_dt_us ||
          r.control_dt_us != p.manifest.control_dt_us)
        return false;
      for (std::uint32_t i = 0; i < p.manifest.count; ++i)
        if (Projection(p.manifest.messages[i], r.message)) {
          if (!SameIdentity(p.manifest.messages[i], r.message)) {
            p.failed = true;
            return false;
          }
          if (r.disposition == Disposition::Ingress)
            p.ingress[i] = true;
          else if (r.disposition == Disposition::BusinessConsumed &&
                   p.ingress[i])
            p.consumed[i] = true;
          else
            return false;
          return true;
        }
      p.failed = true;
      return false;
    }
  return false;
}
bool Barrier::MakeAck(const PhaseManifest& m, Frame* out) noexcept {
  if (!out || local_ != m.consumer || !m.Valid()) return false;
  for (auto& p : phases_)
    if (p.used && SameManifest(p.manifest, m) && !p.failed) {
      for (std::uint32_t i = 0; i < m.count; ++i)
        if (!p.ingress[i] ||
            (m.required == Disposition::BusinessConsumed && !p.consumed[i]))
          return false;
      *out = Frame{};
      out->identity = m.identity;
      out->physics_dt_us = m.physics_dt_us;
      out->control_dt_us = m.control_dt_us;
      out->payload = PhaseAck{local_, m.required, m.seal};
      p.acknowledged = true;
      return true;
    }
  return false;
}
bool Barrier::ObserveAck(const Frame& f, Role peer) noexcept {
  auto a = std::get_if<PhaseAck>(&f.payload);
  if (!a || a->consumer != peer) return false;
  for (auto& p : phases_)
    if (p.used && SamePhase(p.manifest.identity, f.identity)) {
      const auto& m = p.manifest;
      if (p.failed || peer != m.consumer || a->disposition != m.required ||
          f.physics_dt_us != m.physics_dt_us ||
          f.control_dt_us != m.control_dt_us || a->seal.sender != m.sender ||
          a->seal.manifest_count != m.count ||
          a->seal.manifest_digest != m.seal.manifest_digest)
        return false;
      p.acknowledged = true;
      return true;
    }
  return false;
}
bool Barrier::Complete(const PhaseManifest& m) const noexcept {
  if (!m.Valid()) return false;
  for (const auto& p : phases_)
    if (p.used && !p.failed && p.acknowledged && SameManifest(p.manifest, m))
      return true;
  return false;
}
}  // namespace irmv::simulation
