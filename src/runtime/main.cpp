#include <cstdio>

#include "irmv/runtime/runtime_loop.hpp"

// Standalone fake deployment. The whole command line is validated before the
// runtime is constructed: an unusable invocation never reaches a device, DDS or
// the log directory.
int main(int argc, char **argv) {
  const irmv::runtime::CommandLine command_line =
      irmv::runtime::ParseCommandLine(argc, argv);
  if (!command_line.ok) {
    std::fprintf(stderr, "robot_runtime: %s\n%s\n", command_line.error.c_str(),
                 irmv::runtime::UsageLine());
    return command_line.exit_code;
  }
  return irmv::runtime::RunStandaloneRuntime(command_line.options);
}
