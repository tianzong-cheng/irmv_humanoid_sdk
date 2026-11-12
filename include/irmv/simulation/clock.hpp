#pragma once

#include <atomic>

#include "irmv/protocol/types.hpp"

namespace irmv::simulation {

struct StepIdentity {
  protocol::Id run_id{};
  std::uint64_t control_step{0};
  std::uint32_t phase{0};
  protocol::TimeUs logical_us{1};
};

class LogicalClock {
 public:
  virtual ~LogicalClock() = default;
  virtual protocol::TimeUs NowUs() const noexcept = 0;
};

// A separate domain, supplied independently for cancellation/host deadlines.
class HostClock {
 public:
  virtual ~HostClock() = default;
  virtual protocol::TimeUs NowUs() const noexcept = 0;
};

// One coordinator writer, arbitrarily many readers. NowUs publishes only the
// immutable time of a committed phase, independent of host passage of time.
class SimulationClock final : public LogicalClock {
 public:
  SimulationClock(protocol::Id run_id, protocol::TimeUs control_dt_us);
  protocol::TimeUs NowUs() const noexcept override;
  // Contending commits are refused rather than blocking. No allocation or I/O.
  bool Commit(const StepIdentity &identity) noexcept;

 private:
  const protocol::Id run_id_;
  const protocol::TimeUs control_dt_us_;
  std::atomic<protocol::TimeUs> now_us_{1};
  std::atomic_flag committing_ = ATOMIC_FLAG_INIT;
  StepIdentity last_{};  // accessed only while holding committing_
  bool committed_{false};
};

}  // namespace irmv::simulation
