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

using irmv::application::DecisionInstant;
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

}  // namespace

int main() {
  int failures = 0;
  failures += NewerReceiveStampWins();
  failures += OlderReceiveStampKeepsTheClock();
  failures += EqualInstantsAreTheSameInstant();
  failures += BothRangeEnds();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
