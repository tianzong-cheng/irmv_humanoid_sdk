// The decision instant of one supervision iteration.
//
// The loop takes the decision time of an iteration after the transport events
// of that iteration were drained, raised to the newest host receive stamp among
// them. A decision therefore never precedes the data it is based on: a state
// the transport thread received while the iteration was starting carries a
// receive stamp at or before that instant, and a same-host state is published
// at or before it was received, so `stamp_us <= rx_us <= decision_now`. That
// chain is what stops a legal state from being judged fresh by the client's
// clamped receive age and stale by the initializer's publication-stamp rule -
// the microsecond race the repaired loop closes.
//
// The race itself is not reproduced here: a probabilistic test would either
// pass on a lucky interleaving or be flaky. What is pinned is the helper that
// makes the ordering structural - a decision instant is never earlier than the
// newest event it acts on - plus the concrete admission boundary in
// `application_initialization_test`. That a decision is taken after the event
// drain is a property of `RunApplication`'s body, not of this pure function.

#include <cstdint>
#include <iostream>
#include <limits>

#include "irmv/application/application_loop.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::CanServeSampleRate;
using irmv::application::DecisionInstant;
using irmv::application::WaitBudgetUs;
using irmv::application::WaitTimeoutMs;
using irmv::protocol::TimeUs;

// Both ends of the unsigned range, as literals: the raise must be a comparison
// and never an arithmetic step that could wrap.
constexpr TimeUs kZero = 0;
constexpr TimeUs kMax = std::numeric_limits<TimeUs>::max();

// An event received after the clock read raises the instant to the receive
// stamp: the one-microsecond case the review reproduced.
int NewerReceiveStampWins() {
  const TimeUs clock = 1'000'000;
  const TimeUs newer = clock + 1;
  CHECK(DecisionInstant(clock, newer) == newer);
  // An event the transport thread has held for a while cannot lower it either.
  const TimeUs much_newer = clock + 5'000'000;
  CHECK(DecisionInstant(clock, much_newer) == much_newer);
  return 0;
}

// An event that arrived before the clock read leaves the clock as the decision
// instant: the lane may hold a sample up to a whole wait period old.
int OlderReceiveStampKeepsTheClock() {
  const TimeUs clock = 1'000'000;
  CHECK(DecisionInstant(clock, clock - 1) == clock);
  CHECK(DecisionInstant(clock, clock - 10'000) == clock);
  CHECK(DecisionInstant(clock, kZero) == clock);
  return 0;
}

// No event at all: the decision instant is the clock itself.
int EqualInstantsAreTheSameInstant() {
  const TimeUs instant = 4'242;
  CHECK(DecisionInstant(instant, instant) == instant);
  return 0;
}

// Neither end of the range can be pushed past the data or wrapped behind it.
int BothRangeEnds() {
  CHECK(DecisionInstant(kZero, kMax) == kMax);
  CHECK(DecisionInstant(kMax, kZero) == kMax);
  CHECK(DecisionInstant(kZero, kZero) == kZero);
  CHECK(DecisionInstant(kMax, kMax) == kMax);
  return 0;
}

// The pacing wait is the loop period, shortened to the next velocity sample
// deadline and never lengthened - the heartbeat and FSM deadlines remain the
// loop's own.
int WaitIsCappedAtSampleDeadline() {
  constexpr TimeUs kPeriod = 10'000;  // 10 ms
  constexpr TimeUs kNow = 5'000'000;
  CHECK(WaitBudgetUs(kPeriod, 0, kNow) == kPeriod);
  CHECK(WaitTimeoutMs(kPeriod, 0, kNow) == 10);
  // A sample is due in 3 ms: the iteration must not sleep the whole period.
  CHECK(WaitBudgetUs(kPeriod, kNow + 3'000, kNow) == 3'000);
  CHECK(WaitTimeoutMs(kPeriod, kNow + 3'000, kNow) == 3);
  // The deadline already passed: one millisecond, the poll's floor.
  CHECK(WaitBudgetUs(kPeriod, kNow - 1, kNow) == 0);
  CHECK(WaitTimeoutMs(kPeriod, kNow - 1, kNow) == 1);
  CHECK(WaitBudgetUs(kPeriod, kNow, kNow) == 0);
  CHECK(WaitTimeoutMs(kPeriod, kNow, kNow) == 1);
  // A deadline beyond the loop's own pacing does not stretch the wait.
  CHECK(WaitBudgetUs(kPeriod, kNow + 25'000, kNow) == kPeriod);
  CHECK(WaitTimeoutMs(kPeriod, kNow + 25'000, kNow) == 10);
  return 0;
}

// The budget is reported in whole poll milliseconds, rounded up and clamped the
// same way the loop's own period was always clamped.
int WaitRoundsUpAndClamps() {
  constexpr TimeUs kNow = 1'000;
  CHECK(WaitTimeoutMs(1, 0, kNow) == 1);
  CHECK(WaitTimeoutMs(999, 0, kNow) == 1);
  CHECK(WaitTimeoutMs(1'000, 0, kNow) == 1);
  CHECK(WaitTimeoutMs(1'001, 0, kNow) == 2);
  CHECK(WaitTimeoutMs(1'000'000, 0, kNow) == 1000);
  CHECK(WaitTimeoutMs(5'000'000, 0, kNow) == 1000);
  // Both ends of the unsigned range are accepted without wrapping.
  CHECK(WaitBudgetUs(kMax, 0, 0) == kMax);
  CHECK(WaitTimeoutMs(kMax, 0, 0) == 1000);
  CHECK(WaitBudgetUs(kMax, kMax, kMax) == 0);
  CHECK(WaitTimeoutMs(kMax, kMax, kMax) == 1);
  CHECK(WaitBudgetUs(7'000, kMax, kZero) == 7'000);
  return 0;
}

// A slower loop is servable because the sample deadline caps its wait, but the
// poll floor bounds how fast this loop can wake; a disabled loop or schedule is
// never servable and is reported instead of silently recording another rate.
int SampleRateFitIsExplicit() {
  CHECK(CanServeSampleRate(2'000, 10'000));
  CHECK(CanServeSampleRate(10'000, 10'000));
  // The shipped 20 ms supervision loop serves the 100 Hz plan through the cap.
  CHECK(CanServeSampleRate(20'000, 10'000));
  CHECK(CanServeSampleRate(2'000, 1'000));
  CHECK(!CanServeSampleRate(2'000, 999));
  CHECK(!CanServeSampleRate(2'000, 0));
  CHECK(!CanServeSampleRate(0, 10'000));
  return 0;
}

}  // namespace

int main() {
  int failures = 0;
  failures += NewerReceiveStampWins();
  failures += OlderReceiveStampKeepsTheClock();
  failures += EqualInstantsAreTheSameInstant();
  failures += BothRangeEnds();
  failures += WaitIsCappedAtSampleDeadline();
  failures += WaitRoundsUpAndClamps();
  failures += SampleRateFitIsExplicit();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
