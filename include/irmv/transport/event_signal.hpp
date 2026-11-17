#pragma once

#include <sys/eventfd.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>

namespace irmv::transport {

// Level-triggered progress notification. The counter is the only payload: a
// producer makes the descriptor readable after it has published data, and the
// waiting thread polls fd() and then Drain()s before it rechecks the queue that
// actually carries the data. A notification is therefore never evidence of
// delivery - the queue remains the sole authority - and a spurious one costs
// one condition check.
//
// The eventfd is non-blocking and close-on-exec. Open() is explicit so an owner
// can create it only when its worker starts; a failed Open leaves the signal
// invalid and the owner decides whether that is fatal.
class EventSignal {
 public:
  EventSignal() noexcept = default;
  ~EventSignal() { Close(); }

  EventSignal(const EventSignal &) = delete;
  EventSignal &operator=(const EventSignal &) = delete;

  bool Open() noexcept {
    if (fd_ < 0) fd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    return fd_ >= 0;
  }

  bool valid() const noexcept { return fd_ >= 0; }
  int fd() const noexcept { return fd_; }

  // Wakes a waiter. Returns false only on a real write error; EAGAIN means the
  // counter is already saturated, so a wakeup is pending and no loss occurs.
  bool Notify() noexcept {
    if (fd_ < 0) return false;
    const std::uint64_t value = 1;
    ssize_t result;
    do {
      result = ::write(fd_, &value, sizeof(value));
    } while (result < 0 && errno == EINTR);
    return result == static_cast<ssize_t>(sizeof(value)) ||
           (result < 0 && errno == EAGAIN);
  }

  // Clears every pending notification. Non-blocking: EAGAIN is the normal
  // "nothing pending" outcome.
  void Drain() noexcept {
    if (fd_ < 0) return;
    std::uint64_t value;
    ssize_t result;
    do {
      result = ::read(fd_, &value, sizeof(value));
    } while (result < 0 && errno == EINTR);
  }

  // The owner must only close after every thread that may notify has stopped.
  void Close() noexcept {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  int fd_{-1};
};

}  // namespace irmv::transport
