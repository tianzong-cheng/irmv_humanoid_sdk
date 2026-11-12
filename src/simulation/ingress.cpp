#include "irmv/simulation/ingress.hpp"
namespace irmv::simulation {
bool Ingress::ControlBegin(const PhaseManifest& m) noexcept {
  if (Failed() || m.consumer != consumer_ ||
      clock_.NowUs() != m.identity.logical_us || !m.Valid())
    return false;
  if (begun_ && SamePhase(current_.identity, m.identity))
    return barrier_.Begin(m);
  // Do not replace an unfinished phase, including an unacknowledged empty one.
  if (begun_ && !barrier_.Complete(current_)) return false;
  if (!barrier_.Begin(m) || !installs_.TryPush(m)) {
    failed_ = true;
    return false;
  }
  current_ = m;
  dequeued_.fill(false);
  begun_ = true;
  return true;
}
bool Ingress::ControlTakeReceipt(PhaseReceipt* r) noexcept {
  if (!r || Failed() || !receipts_.TryPop(*r)) return false;
  if (!barrier_.Observe(*r)) {
    failed_ = true;
    return false;
  }
  return true;
}
bool Ingress::ControlDequeued(const MessageIdentity& m) noexcept {
  if (!begun_ || Failed()) return false;
  for (std::uint32_t i = 0; i < current_.count; ++i)
    if (SameIdentity(m, current_.messages[i])) {
      dequeued_[i] = true;
      return true;
    }
  failed_ = true;
  return false;
}
bool Ingress::ControlProcessed(const MessageIdentity& m) noexcept {
  PhaseReceipt r;
  while (ControlTakeReceipt(&r)) {
  }
  if (!begun_ || Failed()) return false;
  for (std::uint32_t i = 0; i < current_.count; ++i)
    if (dequeued_[i] && SameIdentity(m, current_.messages[i])) {
      return barrier_.Observe({current_.identity, current_.physics_dt_us,
                               current_.control_dt_us, consumer_,
                               Disposition::BusinessConsumed, m});
    }
  return false;
}
bool Ingress::ControlAck(Frame* f) noexcept {
  PhaseReceipt r;
  while (ControlTakeReceipt(&r)) {
  }
  return !Failed() && begun_ && barrier_.MakeAck(current_, f);
}
void Ingress::WorkerUpdate() noexcept {
  PhaseManifest m;
  while (installs_.TryPop(m)) {
    active_ = next_;
    auto& c = cache_[active_];
    c = Cached{};
    c.used = true;
    c.manifest = m;
    next_ = (next_ + 1) % cache_.size();
    active_valid_ = true;
  }
}
int Ingress::WorkerCheck(const MessageIdentity& m,
                         protocol::TimeUs* stamp) noexcept {
  if (!stamp || Failed() || !active_valid_) return -1;
  auto& c = cache_[active_];
  for (std::uint32_t i = 0; i < c.manifest.count; ++i)
    if (SameIdentity(c.manifest.messages[i], m)) {
      if (c.seen[i]) return 0;
      // A previous phase's identical frozen sample cannot be executed again.
      for (std::size_t k = 0; k < cache_.size(); ++k)
        if (k != active_ && cache_[k].used)
          for (std::uint32_t j = 0; j < cache_[k].manifest.count; ++j)
            if (cache_[k].seen[j] &&
                SameIdentity(cache_[k].manifest.messages[j], m)) {
              failed_ = true;
              return -1;
            }
      if (clock_.NowUs() != c.manifest.identity.logical_us) {
        failed_ = true;
        return -1;
      }
      *stamp = c.manifest.identity.logical_us;
      return 1;
    }
  // Identical delayed retransmits are ignored, never re-stamped or re-enqueued.
  for (auto& old : cache_)
    if (old.used)
      for (std::uint32_t i = 0; i < old.manifest.count; ++i)
        if (old.seen[i] && SameIdentity(old.manifest.messages[i], m)) return 0;
  failed_ = true;
  return -1;
}
void Ingress::WorkerStored(const MessageIdentity& m, bool stored) noexcept {
  if (!stored || !active_valid_) {
    failed_ = true;
    return;
  }
  auto& c = cache_[active_];
  for (std::uint32_t i = 0; i < c.manifest.count; ++i)
    if (SameIdentity(c.manifest.messages[i], m)) {
      c.seen[i] = true;
      if (!receipts_.TryPush({c.manifest.identity, c.manifest.physics_dt_us,
                              c.manifest.control_dt_us, consumer_,
                              Disposition::Ingress, m}))
        failed_ = true;
      return;
    }
  failed_ = true;
}
}  // namespace irmv::simulation
