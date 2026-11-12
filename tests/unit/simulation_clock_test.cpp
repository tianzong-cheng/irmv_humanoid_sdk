#include <atomic>
#include <limits>
#include <stdexcept>
#include <thread>

#include "irmv/simulation/clock.hpp"
#include "support/check.hpp"

int main() {
  using namespace irmv;
  protocol::Id run{};
  run[0] = 7;
  bool rejected_zero_dt = false;
  try {
    simulation::SimulationClock invalid(run, 0);
  } catch (const std::invalid_argument&) {
    rejected_zero_dt = true;
  }
  CHECK(rejected_zero_dt);
  simulation::SimulationClock clock(run, 2000);
  CHECK(clock.NowUs() == 1);
  CHECK(clock.Commit({run, 0, 0, 1}));
  CHECK(clock.Commit({run, 0, 0, 1}));
  CHECK(!clock.Commit({run, 0, 0, 2}));
  CHECK(clock.Commit({run, 0, 1, 1}));
  CHECK(!clock.Commit({run, 0, 0, 1}));
  CHECK(clock.Commit({run, 1, 0, 2001}));
  CHECK(clock.NowUs() == 2001);
  CHECK(!clock.Commit({run, 0, 2, 1}));
  auto other = run;
  other[15] = 1;
  CHECK(!clock.Commit({other, 2, 0, 4001}));
  CHECK(!clock.Commit({run, 2, 0, 4000}));
  CHECK(!clock.Commit({run, std::numeric_limits<std::uint64_t>::max(), 0, 1}));
  CHECK(clock.NowUs() == 2001);
  simulation::SimulationClock overflow(
      run, std::numeric_limits<protocol::TimeUs>::max());
  CHECK(!overflow.Commit({run, 1, 0, 0}));
  std::atomic<bool> done{false};
  std::atomic<bool> coherent{true};
  std::thread reader([&] {
    protocol::TimeUs previous = 2001;
    while (!done.load()) {
      const auto now = clock.NowUs();
      if (now < previous || now % 2000 != 1) coherent.store(false);
      previous = now;
    }
  });
  bool accepted = true;
  for (std::uint64_t n = 2; n <= 10000; ++n) {
    accepted = clock.Commit({run, n, 0, 1 + n * 2000}) && accepted;
  }
  done.store(true);
  reader.join();
  CHECK(accepted);
  CHECK(coherent.load());
  CHECK(clock.NowUs() == 20000001);
  return 0;
}
