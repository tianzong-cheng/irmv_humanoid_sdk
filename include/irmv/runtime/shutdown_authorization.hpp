#pragma once
#include "irmv/protocol/types.hpp"
namespace irmv::runtime {
// Executable composition policy, not an execution-FSM exception. Only a
// validated independent simulation ShutdownPermit may set the last argument.
inline bool MayPlanLocalStop(protocol::ExecutionState state, bool has_session,
                             bool coordinator_shutdown = false) noexcept {
  return (state == protocol::ExecutionState::Damping ||
          state == protocol::ExecutionState::Tracking) &&
         (has_session || coordinator_shutdown);
}
}  // namespace irmv::runtime
