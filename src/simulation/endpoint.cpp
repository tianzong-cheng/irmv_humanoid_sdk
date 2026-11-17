#include "irmv/simulation/endpoint.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstring>
#include <stdexcept>

#include "irmv/simulation/codec.hpp"
namespace irmv::simulation {
namespace {
using Clock = std::chrono::steady_clock;

// Milliseconds until `deadline`, rounded up so a positive remainder never turns
// into a zero-timeout spin. The value is only ever handed to poll(2).
int RemainingMs(Clock::time_point deadline) {
  const auto now = Clock::now();
  if (now >= deadline) return 0;
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(deadline - now)
          .count();
  const auto ms = (us + 999) / 1000;
  return ms > INT_MAX ? INT_MAX : static_cast<int>(ms);
}

bool PrivateDirectory(int fd) {
  struct stat s {};
  return fd >= 0 && fstat(fd, &s) == 0 && S_ISDIR(s.st_mode) &&
         s.st_uid == geteuid() && (s.st_mode & 0777) == 0700;
}
}  // namespace
Endpoint::~Endpoint() { Stop(); }
void Endpoint::Start(const EndpointConfig& c) {
  if (started_) throw std::runtime_error("endpoint is single use");
  if (!c.socket_path.is_absolute() || !c.startup_timeout_ms ||
      !c.io_timeout_ms || c.local_hello.role == Role::Coordinator ||
      c.peer_hello.role != Role::Coordinator ||
      !MatchesHello(c.local_hello, c.peer_hello))
    throw std::runtime_error("endpoint configuration");
  Frame hello;
  hello.identity.run_id = c.run_id;
  hello.physics_dt_us = c.local_hello.physics_dt_us;
  hello.control_dt_us = c.local_hello.control_dt_us;
  hello.payload = c.local_hello;
  ByteBuffer encoded;
  if (!Encode(hello, &encoded))
    throw std::runtime_error("invalid endpoint hello");
  // Walk every directory component without following symlinks; pin final inode.
  int dir = open("/", O_DIRECTORY | O_CLOEXEC | O_RDONLY);
  for (const auto& component : c.socket_path.parent_path().relative_path()) {
    if (component == "..") {
      close(dir);
      throw std::runtime_error("parent traversal");
    }
    int next = openat(dir, component.c_str(),
                      O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW | O_RDONLY);
    close(dir);
    dir = next;
    if (dir < 0) throw std::runtime_error("unsafe endpoint parent");
  }
  if (!PrivateDirectory(dir)) {
    if (dir >= 0) close(dir);
    throw std::runtime_error("endpoint requires private 0700 parent");
  }
  struct stat s {};
  if (fstatat(dir, c.socket_path.filename().c_str(), &s, AT_SYMLINK_NOFOLLOW) ==
      0) {
    if (!S_ISSOCK(s.st_mode) || s.st_uid != geteuid()) {
      close(dir);
      throw std::runtime_error("unsafe endpoint path");
    }
  } else if (errno != ENOENT) {
    close(dir);
    throw std::runtime_error("endpoint stat");
  }
  config_ = c;
  if (!wake_.Open() || !notice_.Open()) {
    wake_.Close();
    notice_.Close();
    close(dir);
    throw std::runtime_error("endpoint eventfd");
  }
  directory_ = dir;
  started_ = true;
  try {
    worker_ = std::thread(&Endpoint::Run, this);
  } catch (...) {
    wake_.Close();
    notice_.Close();
    close(directory_);
    directory_ = -1;
    throw;
  }
}
void Endpoint::Wake() noexcept {
  if (!wake_.valid()) return;
  if (!wake_.Notify()) failed_ = true;
}
void Endpoint::Notify() noexcept {
  if (!notice_.valid()) return;
  if (!notice_.Notify()) failed_ = true;
}
bool Endpoint::TrySend(const Frame& f) noexcept {
  if (!ready_ || failed_ || stop_) return false;
  if (!outbound_.TryPush(f)) {
    ++rejected_;
    failed_ = true;
    return false;
  }
  Wake();
  return true;
}
bool Endpoint::TryTake(Frame* f) noexcept { return f && inbound_.TryPop(*f); }
void Endpoint::Stop() {
  stop_ = true;
  Wake();
  if (worker_.joinable()) worker_.join();
  ready_ = false;
  wake_.Close();
  notice_.Close();
  if (directory_ >= 0) {
    close(directory_);
    directory_ = -1;
  }
}
void Endpoint::Run() noexcept {
  int fd = -1;
  try {
    auto deadline =
        Clock::now() + std::chrono::milliseconds(config_.startup_timeout_ms);
    // /proc resolves the already verified directory inode, avoiding parent
    // replacement.
    std::string path = "/proc/self/fd/" + std::to_string(directory_) + "/" +
                       config_.socket_path.filename().string();
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.size() >= sizeof(address.sun_path))
      throw std::runtime_error("socket path length");
    std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
    bool connected = false;
    while (!stop_ && Clock::now() < deadline) {
      struct stat s {};
      if (fstatat(directory_, config_.socket_path.filename().c_str(), &s,
                  AT_SYMLINK_NOFOLLOW) == 0 &&
          (!S_ISSOCK(s.st_mode) || s.st_uid != geteuid()))
        throw std::runtime_error("changed endpoint path");
      fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (fd < 0) throw std::runtime_error("socket");
      if (connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) ==
          0) {
        connected = true;
        break;
      }
      close(fd);
      fd = -1;
      // Bounded connect retry, but Stop wakes the wait immediately.
      pollfd wait{wake_.fd(), POLLIN, 0};
      if (poll(&wait, 1, 5) > 0) wake_.Drain();
    }
    if (!connected) {
      if (!stop_) {
        failed_ = true;
        Notify();
      }
      return;
    }
    ucred cred{};
    socklen_t len = sizeof(cred);
    if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &len) != 0 ||
        len != sizeof(cred) || cred.uid != geteuid() || cred.pid <= 0)
      throw std::runtime_error("peer credentials");
    Frame hello;
    hello.identity.run_id = config_.run_id;
    hello.payload = config_.local_hello;
    hello.physics_dt_us = config_.local_hello.physics_dt_us;
    hello.control_dt_us = config_.local_hello.control_dt_us;
    ByteBuffer tx, rx;
    Encode(hello, &tx);
    bool pending = true, handshake = false;
    auto send_deadline = deadline;
    while (!stop_ && !failed_) {
      if (Clock::now() >= deadline ||
          (pending && Clock::now() >= send_deadline))
        throw std::runtime_error("IPC deadline");
      pollfd descriptors[2]{
          {fd, static_cast<short>(POLLIN | (pending ? POLLOUT : 0)), 0},
          {wake_.fd(), POLLIN, 0}};
      // Wake on socket traffic, outbound/Stop notifications, or the exact
      // deadline. No periodic poll: an idle peer is detected at io_timeout and
      // a blocked outgoing packet at send_deadline, not on a 5 ms retry.
      int timeout_ms = RemainingMs(deadline);
      if (pending)
        timeout_ms = std::min(timeout_ms, RemainingMs(send_deadline));
      int n = poll(descriptors, 2, timeout_ms);
      if (n < 0 && errno != EINTR) throw std::runtime_error("poll");
      if (n > 0 && (descriptors[1].revents & POLLIN)) wake_.Drain();
      if (n <= 0) continue;
      const auto& p = descriptors[0];
      if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
        throw std::runtime_error("peer closed");
      // Receive the whole available burst and notify the control thread once.
      // Frames keep their SEQPACKET boundaries and channel order.
      if (p.revents & POLLIN) {
        bool received = false;
        for (std::size_t i = 0; i < kFrameQueueCapacity; ++i) {
          auto got = recv(fd, rx.bytes.data(), rx.bytes.size(),
                          MSG_DONTWAIT | MSG_TRUNC);
          if (got < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
              break;
            throw std::runtime_error("packet length");
          }
          if (got == 0 || got > static_cast<ssize_t>(kMaxPacketBytes))
            throw std::runtime_error("packet length");
          Frame f;
          if (!Decode({rx.bytes.data(), static_cast<std::size_t>(got)}, &f) ||
              f.identity.run_id != config_.run_id ||
              f.physics_dt_us != config_.peer_hello.physics_dt_us ||
              f.control_dt_us != config_.peer_hello.control_dt_us)
            throw std::runtime_error("frame identity");
          if (!handshake) {
            auto h = std::get_if<Hello>(&f.payload);
            if (!h || h->role != config_.peer_hello.role ||
                !MatchesHello(*h, config_.peer_hello))
              throw std::runtime_error("peer hello");
            handshake = true;
            ready_ = true;
          } else {
            if (std::holds_alternative<Hello>(f.payload))
              throw std::runtime_error("duplicate hello");
            if (!inbound_.TryPush(f)) {
              ++rejected_;
              throw std::runtime_error("inbound full");
            }
          }
          received = true;
        }
        if (received) {
          deadline =
              Clock::now() + std::chrono::milliseconds(config_.io_timeout_ms);
          Notify();
        }
      }
      // Send the whole ready burst: the pending packet first, then queued
      // frames while the socket accepts them. Notify capacity once.
      bool freed = false;
      std::size_t sent_frames = 0;
      while (sent_frames < kFrameQueueCapacity) {
        if (pending) {
          auto sent =
              send(fd, tx.bytes.data(), tx.size, MSG_NOSIGNAL | MSG_DONTWAIT);
          if (sent == static_cast<ssize_t>(tx.size))
            pending = false;
          else if (sent >= 0 ||
                   (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
            throw std::runtime_error("send");
          if (pending) break;  // poll will wait for POLLOUT
        }
        if (!handshake) break;
        Frame f;
        if (!outbound_.TryPop(f)) break;
        freed = true;
        if (f.identity.run_id != config_.run_id ||
            f.physics_dt_us != config_.local_hello.physics_dt_us ||
            f.control_dt_us != config_.local_hello.control_dt_us ||
            std::holds_alternative<Hello>(f.payload) || !Encode(f, &tx))
          throw std::runtime_error("outbound frame");
        pending = true;
        // Incoming traffic cannot extend a blocked outgoing packet forever.
        send_deadline =
            Clock::now() + std::chrono::milliseconds(config_.io_timeout_ms);
        ++sent_frames;
      }
      if (freed) Notify();
    }
  } catch (...) {
    failed_ = true;
    Notify();
  }
  ready_ = false;
  Notify();
  if (fd >= 0) close(fd);
}
}  // namespace irmv::simulation
