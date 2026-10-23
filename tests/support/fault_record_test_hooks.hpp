#pragma once

// Test-only seam into the recorder's faults.jsonl writer. It is declared here,
// not in an installed header: production code never sets it. The hook runs on
// the recorder's worker thread immediately before one line is written, so a
// test can park the worker inside a write (a blocked device) or make the write
// fail. Returning true lets the line be written; false reports a failed write
// without touching the file.
namespace irmv::telemetry {

void SetFaultLineWriteHookForTest(bool (*hook)(void *context),
                                  void *context) noexcept;

}  // namespace irmv::telemetry
