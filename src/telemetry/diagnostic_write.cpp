#include "irmv/telemetry/diagnostic_write.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <mutex>

namespace irmv::telemetry {
namespace {

// How long the bounded writer waits for the diagnostic lock before it gives up
// on the line. Long enough to interleave with a log worker writing its own line
// (a plain write, microseconds), and capped at one control period of the
// shipped deployment so a worker parked in a full pipe costs the control thread
// one counted tick overrun at most - never an unbounded delay, and never a
// stall several periods long. The line is dropped when the budget runs out.
constexpr auto kBoundedLockWait = std::chrono::milliseconds(2);

// How long the log worker waits for the same lock. Only a second worker whose
// peer is already parked in a jammed descriptor reaches it; dropping that line
// is then the same outcome as waiting for the park, without keeping a second
// worker inside the sink for ever. It never throws, which the LogSink contract
// requires.
constexpr auto kBlockingLockWait = std::chrono::milliseconds(500);

// One lock per process for every diagnostic written to the same descriptor.
// Two writers of one pipe cannot both be between "there is room" and the write
// without making the second one block, so they are serialized here instead of
// by the descriptor's own (stdio) lock.
std::timed_mutex &DiagnosticLock() {
  static std::timed_mutex lock;
  return lock;
}

bool WriteAll(int fd, std::string_view text) {
  std::size_t written = 0;
  while (written < text.size()) {
    const ssize_t count =
        ::write(fd, text.data() + written, text.size() - written);
    if (count > 0) {
      written += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace

bool WriteDiagnosticBounded(int fd, std::string_view text) {
  if (fd < 0 || text.empty()) {
    return false;
  }
  const std::size_t length = std::min(text.size(), kMaxBoundedDiagnosticBytes);
  if (!DiagnosticLock().try_lock_for(kBoundedLockWait)) {
    return false;
  }
  struct pollfd descriptor {
    fd, POLLOUT, 0
  };
  const int ready = ::poll(&descriptor, 1, 0);
  bool complete = false;
  if (ready > 0 && (descriptor.revents & POLLOUT) != 0) {
    complete = WriteAll(fd, text.substr(0, length));
  }
  DiagnosticLock().unlock();
  // A truncated line is not a written line: the caller must be able to tell
  // that the diagnostic it asked for did not arrive whole.
  return complete && length == text.size();
}

bool WriteDiagnosticBlocking(int fd, std::string_view text) {
  if (fd < 0 || text.empty()) {
    return false;
  }
  if (!DiagnosticLock().try_lock_for(kBlockingLockWait)) {
    return false;
  }
  const bool complete = WriteAll(fd, text);
  DiagnosticLock().unlock();
  return complete;
}

}  // namespace irmv::telemetry
