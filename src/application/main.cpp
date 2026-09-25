#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <string>
#include <string_view>

#include "irmv/application/application_loop.hpp"
#include "irmv/configuration/config.hpp"

namespace {

// Set by the SIGINT/SIGTERM handler and read by the supervision loop. The
// handler does nothing else: no logging, no allocation, no terminal call, no
// transport call. std::atomic is required to be lock free for that to hold,
// which the assertion below pins.
std::atomic<bool> g_stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "the stop flag must be usable from a signal handler");

extern "C" void HandleStopSignal(int signal_number) {
  static_cast<void>(signal_number);
  g_stop_requested.store(true, std::memory_order_relaxed);
}

bool InstallStopHandlers() {
  return std::signal(SIGINT, HandleStopSignal) != SIG_ERR &&
         std::signal(SIGTERM, HandleStopSignal) != SIG_ERR;
}

// The terminal of the supervision loop, held for the whole life of the process
// and given back on every exit path that this program can observe - the normal
// one included, because the exit a signal asks for is the same ordered exit as
// the exit key.
//
// Two things are saved and restored, and only these two: the line discipline
// attributes (ICANON and ECHO cleared, so one byte reaches the loop without a
// line to finish first) and the O_NONBLOCK file status flag, which is what
// makes a read that finds nothing return instead of waiting. A stdin that is
// not a terminal keeps its attributes untouched and is still read without
// blocking.
class TerminalGuard {
 public:
  TerminalGuard() { Acquire(); }
  ~TerminalGuard() { Restore(); }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard &operator=(const TerminalGuard &) = delete;

 private:
  void Acquire() {
    original_flags_ = ::fcntl(STDIN_FILENO, F_GETFL);
    if (original_flags_ >= 0) {
      static_cast<void>(
          ::fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK));
      have_flags_ = true;
    }
    if (::isatty(STDIN_FILENO) != 1) {
      return;
    }
    if (::tcgetattr(STDIN_FILENO, &original_attributes_) != 0) {
      return;
    }
    have_attributes_ = true;
    struct termios raw = original_attributes_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    // VMIN stays 1 on purpose: a terminal with VMIN=0 answers a data-less read
    // with a zero-length read, which is byte for byte what a real EOF looks
    // like. With VMIN=1 the O_NONBLOCK flag above is what keeps the read from
    // waiting, and an empty read then reports EAGAIN.
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    static_cast<void>(::tcsetattr(STDIN_FILENO, TCSANOW, &raw));
  }

  void Restore() {
    if (have_attributes_) {
      static_cast<void>(
          ::tcsetattr(STDIN_FILENO, TCSANOW, &original_attributes_));
      have_attributes_ = false;
    }
    if (have_flags_) {
      static_cast<void>(::fcntl(STDIN_FILENO, F_SETFL, original_flags_));
      have_flags_ = false;
    }
  }

  struct termios original_attributes_ {};
  int original_flags_{-1};
  bool have_attributes_{false};
  bool have_flags_{false};
};

// ---------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------

constexpr std::string_view kUsageLine =
    "usage: application --config PATH [--domain N]";

struct CommandLine {
  bool ok{false};
  int exit_code{irmv::application::kExitUsageError};
  std::string error;
  irmv::application::ApplicationOptions options;
};

CommandLine ParseCommandLine(int argc, char **argv) {
  CommandLine result;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    const bool has_value = index + 1 < argc;
    if (argument == "--config") {
      if (!has_value) {
        result.error = "--config requires a path";
        return result;
      }
      result.options.config_path = argv[++index];
      continue;
    }
    if (argument == "--domain") {
      if (!has_value) {
        result.error = "--domain requires a value";
        return result;
      }
      const std::string value(argv[++index]);
      std::size_t consumed = 0;
      std::uint64_t domain = 0;
      try {
        domain = std::stoull(value, &consumed);
      } catch (const std::exception &) {
        result.error = "--domain must be an integer, got \"" + value + "\"";
        return result;
      }
      if (consumed != value.size() ||
          domain > irmv::configuration::kMaxDdsDomain) {
        result.error = "--domain must be between 0 and " +
                       std::to_string(irmv::configuration::kMaxDdsDomain) +
                       ", got \"" + value + "\"";
        return result;
      }
      result.options.dds_domain = static_cast<std::uint32_t>(domain);
      continue;
    }
    result.error = "unknown argument \"" + std::string(argument) + "\"";
    return result;
  }

  if (result.options.config_path.empty()) {
    result.error = "--config is required";
    return result;
  }
  result.ok = true;
  result.exit_code = irmv::application::kExitOk;
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  // The whole command line is validated before anything else: a rejected
  // invocation touches no terminal, no DDS entity and no device.
  const CommandLine command_line = ParseCommandLine(argc, argv);
  if (!command_line.ok) {
    std::fprintf(stderr, "application: %s\n%.*s\n", command_line.error.c_str(),
                 static_cast<int>(kUsageLine.size()), kUsageLine.data());
    return command_line.exit_code;
  }
  if (!InstallStopHandlers()) {
    std::fprintf(stderr, "application: cannot install the stop handlers\n");
    return irmv::application::kExitStartupError;
  }

  // The terminal belongs to this file, and it is acquired after the command
  // line was accepted and before the loop reads anything. The guard restores
  // the attributes on every return of `main`, which is why a handled signal
  // leaves the terminal the way it was found.
  const TerminalGuard terminal;
  return irmv::application::RunApplication(command_line.options,
                                           g_stop_requested);
}
