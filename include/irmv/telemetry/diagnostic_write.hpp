#pragma once

#include <cstddef>
#include <string_view>

namespace irmv::telemetry {

// Longest text one bounded call attempts. A pipe reports POLLOUT only when it
// can still take PIPE_BUF (4096 on Linux) bytes, so a write of at most this
// many bytes cannot block once the descriptor was reported writable - provided
// no other writer of this process interleaves, which is exactly what the shared
// writer lock of these functions guarantees.
inline constexpr std::size_t kMaxBoundedDiagnosticBytes = 1024;

// Bounded, best-effort diagnostic on `fd`, for the control thread and for the
// exit path.
//
// These lines are written after the logger has been stopped - and possibly left
// detached inside a sink parked on a full pipe - so the call must not use stdio
// (the parked worker may hold the FILE lock) and must not wait for a consumer
// to make room. It takes the process-wide diagnostic lock with a bounded wait,
// probes the descriptor and drops whatever does not fit. A dropped diagnostic
// is an acceptable cost; a safe exit parked in a jammed pipe is not.
//
// Returns true only when the whole `text` was accepted, so a truncated or
// dropped line is never reported as written. Descriptors are assumed to be a
// pipe, a terminal or a regular file, which is what the runtime's stderr is.
bool WriteDiagnosticBounded(int fd, std::string_view text);

// Blocking counterpart for the log worker. The worker is allowed to park on a
// descriptor nobody drains - that is how a slow log consumer costs log records
// - so it waits for the whole line while holding the same process-wide lock.
// Holding the lock while parked is what keeps a bounded writer from being the
// second writer of a pipe it just probed, and it makes the control thread
// refuse its diagnostic instead of waiting behind the parked worker. Only a
// worker that cannot obtain the lock within its own budget gives up, which
// happens when a peer is already parked.
bool WriteDiagnosticBlocking(int fd, std::string_view text);

}  // namespace irmv::telemetry
