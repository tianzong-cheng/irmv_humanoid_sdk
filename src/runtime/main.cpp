#include <unistd.h>

#include <array>
#include <cstdio>
#include <string_view>

#include "irmv/runtime/runtime_loop.hpp"
#include "irmv/telemetry/diagnostic_write.hpp"

// Standalone fake deployment. The whole command line is validated before the
// runtime is constructed: an unusable invocation never reaches a device, DDS or
// the log directory.
int main(int argc, char **argv) {
  const irmv::runtime::CommandLine command_line =
      irmv::runtime::ParseCommandLine(argc, argv);
  if (!command_line.ok) {
    // The argument error goes through the same bounded writer as every other
    // runtime diagnostic: this process may be started with an stderr pipe
    // nobody drains, and an unusable invocation must exit instead of waiting
    // for a reader.
    std::array<char, 512> line{};
    const int length =
        std::snprintf(line.data(), line.size(), "robot_runtime: %s\n%s\n",
                      command_line.error.c_str(), irmv::runtime::UsageLine());
    if (length > 0) {
      const std::size_t size = static_cast<std::size_t>(length) < line.size()
                                   ? static_cast<std::size_t>(length)
                                   : line.size() - 1;
      static_cast<void>(irmv::telemetry::WriteDiagnosticBounded(
          ::fileno(stderr), std::string_view(line.data(), size)));
    }
    return command_line.exit_code;
  }
  return irmv::runtime::RunStandaloneRuntime(command_line.options);
}
