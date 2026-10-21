#pragma once

// Test-only seam into the recorder's producer exit path. It is declared here,
// not in an installed header: production code never sets it, and the hook is
// called after the producer count is decremented without touching recorder
// state.
namespace irmv::telemetry {

void SetProducerAfterDecrementHookForTest(void (*hook)()) noexcept;

}  // namespace irmv::telemetry
