#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "irmv/protocol/types.hpp"

namespace irmv::telemetry {

// The clock every record timestamp is taken from. `CLOCK_MONOTONIC` on Linux
// does not jump with wall-clock adjustments, so two processes on one host can
// be lined up only when they also agree on the boot those readings belong to.
inline constexpr std::string_view kRecordClock = "linux_clock_monotonic";

// The kernel's identity of the current boot. It is read once at startup: a
// reboot changes it, so a saved run can be distinguished from one taken in an
// earlier boot even though both use the same monotonic clock.
inline constexpr std::string_view kBootIdPath =
    "/proc/sys/kernel/random/boot_id";

// What a run records about its clock. `available` is false when the boot id
// could not be read; the run may still be saved, but automatic cross-process
// merging must refuse it because an identity is what proves both processes
// share a boot.
struct RecordClockIdentity {
  std::string clock;
  std::string boot_id;
  bool available{false};
};

// Reads the boot id once at startup. On success the identity carries the clock
// name and the trimmed boot id, `available` is true and `error` is cleared. On
// failure it returns false, sets clock to kRecordClock already, clears boot_id,
// leaves `available` false and writes a startup diagnostic to `error`;
// recording may continue, merging may not.
bool LoadRecordClockIdentity(RecordClockIdentity* identity, std::string* error);

// The injectable form of the read above: the same contract, but from an
// explicit file. Tests use it so they never depend on the host's /proc.
bool LoadRecordClockIdentityFromFile(std::string_view boot_id_path,
                                     RecordClockIdentity* identity,
                                     std::string* error);

// A pure sampling plan for one process at an independent rate.
//
// The plan is an integer grid, never an accumulated floating period. The first
// Due() call at t is the phase origin: it reports one sample and the next
// deadline is t + period. Later deadlines are t + k*period. A call after a
// stall reports at most one current sample - the deadline is advanced
// arithmetically past now - so missed deadlines are never replayed and a
// stalled loop cannot produce a burst of fabricated history.
//
// Due() reads no clock, allocates nothing and performs no I/O: the caller
// supplies the monotonic timestamp. A zero rate is a disabled plan that is
// never due; a rate above 1 MHz cannot be represented in microseconds and is
// disabled for the same reason.
class SampleSchedule {
 public:
  explicit SampleSchedule(std::uint32_t sample_hz) noexcept;

  // True when `sample_hz` produced a usable period.
  bool Enabled() const noexcept { return period_us_ != 0; }

  // The integer period of the plan in microseconds, or zero when disabled.
  protocol::TimeUs PeriodUs() const noexcept { return period_us_; }

  // Reports whether `now_us` has reached the next deadline and, if so, advances
  // the deadline to the first grid point strictly after now.
  bool Due(protocol::TimeUs now_us) noexcept;

 private:
  protocol::TimeUs period_us_{0};
  protocol::TimeUs next_due_us_{0};
  bool started_{false};
};

}  // namespace irmv::telemetry
