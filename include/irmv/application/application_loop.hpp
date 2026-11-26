#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <optional>

#include "irmv/application/locomotion_source.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

// The decision instant of one supervision iteration: the host clock read after
// the transport events of that iteration were drained, raised to the newest
// host receive stamp among those events.
//
// Two instants pace one iteration and they are not interchangeable. `input_now`
// is read before the terminal is drained and stamps only the keystrokes that
// read produced; the decision instant is the one every supervision judgement
// uses (the router tick, the client poll, the behaviour step, the sends and the
// status cadence). Taking it after the drain and raising it to the newest
// receive stamp gives the loop the chain
//
//   publication stamp <= host receive stamp <= decision instant,
//
// which is what makes the ordering structural instead of lucky. A state the
// transport thread received while the iteration was still starting carries a
// receive stamp at or after the clock read of that iteration; without the raise
// the client's clamped receive age would call such a sample fresh (age 0)
// while the initializer, handed the earlier instant, would refuse it for
// `stamp_us > now_us` and latch a failed initialization. No iteration decision
// may therefore predate the data it acts on, and a legal same-host state can
// never be judged fresh by one layer and stale by the other.
//
// The raise is a comparison, not an addition: hostile inputs at either end of
// the unsigned range can only leave the clock where it was or move the instant
// to the receive stamp, never wrap it behind the data.
inline protocol::TimeUs DecisionInstant(
    protocol::TimeUs clock_us, protocol::TimeUs newest_rx_us) noexcept {
  return newest_rx_us > clock_us ? newest_rx_us : clock_us;
}

// True when a loop of `period_us` can serve a schedule of `sample_period_us`.
//
// A loop that is slower than the plan is still servable: the wait at the end of
// an iteration is capped at the next sample deadline, so the loop is woken at
// least once per sample period even when its nominal pacing period is longer.
// The poll's one-millisecond floor is the real bound on how fast this loop can
// wake, so a plan more frequent than 1 kHz cannot be honoured and is refused
// instead of being recorded at another rate than the metadata declares. A
// disabled loop period or schedule is never servable.
inline bool CanServeSampleRate(protocol::TimeUs period_us,
                               protocol::TimeUs sample_period_us) noexcept {
  return period_us != 0 && sample_period_us >= 1000;
}

// The longest wait one iteration may take before it must run again. It is the
// loop's own pacing period, shortened - never lengthened - to the next velocity
// sample deadline so a due sample is taken on time. Heartbeat and FSM deadlines
// are unaffected: they are judged from the decision instant of an iteration,
// and this helper only decides how long that iteration idles between them.
// `next_deadline_us` is zero when no sample plan is active.
inline protocol::TimeUs WaitBudgetUs(protocol::TimeUs period_us,
                                     protocol::TimeUs next_deadline_us,
                                     protocol::TimeUs now_us) noexcept {
  protocol::TimeUs budget = period_us == 0 ? 1 : period_us;
  if (next_deadline_us != 0) {
    const protocol::TimeUs until =
        next_deadline_us > now_us ? next_deadline_us - now_us : 0;
    if (until < budget) {
      budget = until;
    }
  }
  return budget;
}

// WaitBudgetUs rounded up to whole milliseconds and clamped to the poll
// timeout's own 1..1000 ms bounds, so a sample deadline at any resolution is
// never rounded away.
inline int WaitTimeoutMs(protocol::TimeUs period_us,
                         protocol::TimeUs next_deadline_us,
                         protocol::TimeUs now_us) noexcept {
  const protocol::TimeUs budget =
      WaitBudgetUs(period_us, next_deadline_us, now_us);
  // Division rather than `budget + 999`: both ends of the unsigned range are
  // legal inputs and the sum could wrap.
  const protocol::TimeUs milliseconds =
      budget / 1000 + (budget % 1000 == 0 ? 0 : 1);
  if (milliseconds < 1) {
    return 1;
  }
  if (milliseconds > 1000) {
    return 1000;
  }
  return static_cast<int>(milliseconds);
}

// Exit codes of the standalone application.
inline constexpr int kExitOk = 0;
// Invalid configuration or a failure of the startup sequence, including the
// DDS endpoints of the transport.
inline constexpr int kExitStartupError = 1;
// Unknown or malformed command line argument.
inline constexpr int kExitUsageError = 2;
// The ordered exit finished without proving a stop: the lower layer never
// confirmed one, so the process reports what it could not prove instead of
// claiming the hardware stopped.
inline constexpr int kExitStopUnknown = 3;

// The whole invocation of the application process: the configuration to load
// and the optional domain override of the command line. The override reaches
// the configuration, exactly like the runtime's own `--domain`, so the
// configuration digest the application compares against a published state is
// the digest of the configuration actually in effect.
// Who produces the joint targets of the locomotion behaviour is
// `LocomotionSource`, in locomotion_source.hpp: the behaviour has to
// distinguish a source that publishes its own frames from one this process
// relays, so the enumerator is shared instead of defined here.
struct ApplicationOptions {
  std::filesystem::path config_path;
  std::optional<std::uint32_t> dds_domain;
  // Unset means "follow the configuration": the local source when
  // `application.policy.model_path` is configured, and none otherwise. An
  // explicit `local` on a configuration without a model is refused at startup
  // rather than silently ignored.
  std::optional<LocomotionSource> locomotion_source;
  // The inherited datagram descriptor of the simulation viewer's input
  // channel. Only the simulation backend reads it; another backend ignores it
  // so no deployment gains a viewer it did not launch. Unset leaves the
  // terminal as the only input source.
  std::optional<int> viewer_input_fd;
};

// Runs the supervision loop until the ordered exit completes and returns the
// process exit code.
//
// The loop owns everything a behaviour needs and nothing a behaviour must not
// have: it loads the configuration (before the transport, which is the first
// thing of this process that touches DDS), starts the transport, feeds the
// client and the behaviour FSM, performs the terminal read that also paces the
// iteration, and prints the one status line. It never starts, stops, signals or
// holds the process id of a runtime process: the lower layer is an independent
// process that is entered through DDS alone and outlives this one.
//
// Precondition: stdin is already non-blocking (O_NONBLOCK), which is what
// main's terminal guard sets before calling this function and restores
// afterwards. The loop does not set it itself, and a second caller that handed
// in a blocking tty would stall inside the raw-mode read with the heartbeat
// stopped - the non-blocking flag is what keeps the pacing, the status line and
// the supervision heartbeat alive while no key arrives.
//
// A simulation started by the launcher may instead inherit a viewer input
// descriptor (`viewer_input_fd`). The loop reads it as a second, leased input
// source; its EOF is not the terminal's EOF, so a run without a tty keeps
// supervising as long as that channel exists. A terminal-only run keeps the
// documented terminal EOF exit unchanged.
//
// `stop_requested` is the only thing a signal handler may set; the loop turns
// it into the same RequestExit the exit key raises, so every exit is the one
// bounded, honest path.
int RunApplication(const ApplicationOptions &options,
                   const std::atomic<bool> &stop_requested);

}  // namespace irmv::application
