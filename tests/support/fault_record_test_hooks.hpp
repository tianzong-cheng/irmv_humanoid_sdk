#pragma once

#include <string_view>

// Test-only seam into the recorder's faults.jsonl writer. It is declared here,
// not in an installed header: production code never sets it. The hook runs on
// the recorder's worker thread before one serialized line is written, so a test
// can park the worker inside a write (a blocked device) or choose a failure:
//   0 - write the whole line;
//   1 - refuse it (no bytes are written);
//   2 - simulate a torn write (a prefix is written, then the sink fails).
// The function and context are read as one pair under a mutex; keep the context
// alive until the worker is quiescent, because an in-flight call may already
// hold the pair after removal. The `line` view is valid only for the duration
// of the call; copy it if the hook needs to keep it.
namespace irmv::telemetry {

void SetFaultLineWriteHookForTest(int (*hook)(void *context,
                                              std::string_view line),
                                  void *context) noexcept;

}  // namespace irmv::telemetry
