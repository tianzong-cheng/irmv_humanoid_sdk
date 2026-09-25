#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <optional>

namespace irmv::application {

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
// the configuration layer, exactly like the runtime's own `--domain`, so the
// configuration digest the application compares against a published state is
// the digest of the configuration actually in effect.
struct ApplicationOptions {
  std::filesystem::path config_path;
  std::optional<std::uint32_t> dds_domain;
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
// `stop_requested` is the only thing a signal handler may set; the loop turns
// it into the same RequestExit the exit key raises, so every exit is the one
// bounded, honest path.
int RunApplication(const ApplicationOptions &options,
                   const std::atomic<bool> &stop_requested);

}  // namespace irmv::application
