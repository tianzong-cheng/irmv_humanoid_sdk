#include "irmv/simulation/clock.hpp"

#include <limits>
#include <stdexcept>

namespace irmv::simulation {

static_assert(std::atomic<protocol::TimeUs>::is_always_lock_free,
              "logical time readers must never lock");

SimulationClock::SimulationClock(protocol::Id run_id, protocol::TimeUs dt)
    : run_id_(run_id), control_dt_us_(dt) {
  if (dt == 0) throw std::invalid_argument("control_dt_us must be positive");
}

protocol::TimeUs SimulationClock::NowUs() const noexcept {
  return now_us_.load(std::memory_order_acquire);
}

bool SimulationClock::Commit(const StepIdentity &identity) noexcept {
  if (committing_.test_and_set(std::memory_order_acquire)) return false;
  bool accepted = false;
  // Check multiplication and the nonzero-origin addition before arithmetic.
  if (identity.run_id == run_id_ &&
      identity.control_step <=
          (std::numeric_limits<protocol::TimeUs>::max() - 1) / control_dt_us_ &&
      identity.logical_us == 1 + identity.control_step * control_dt_us_) {
    const bool ordered = !committed_ ||
                         identity.control_step > last_.control_step ||
                         (identity.control_step == last_.control_step &&
                          identity.phase >= last_.phase);
    if (ordered) {
      last_ = identity;
      committed_ = true;
      now_us_.store(identity.logical_us, std::memory_order_release);
      accepted = true;
    }
  }
  committing_.clear(std::memory_order_release);
  return accepted;
}

}  // namespace irmv::simulation
