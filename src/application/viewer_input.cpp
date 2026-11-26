#include "irmv/application/viewer_input.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>

namespace irmv::application {

namespace {

using configuration::InputAction;

// Bound on the datagrams one iteration drains. A viewer cannot starve the
// supervision work by writing faster than the loop reads, and every frame is
// exactly one datagram, so this is also a bound on keystrokes handled here.
constexpr int kReadAttempts = 64;

bool IsVelocityAction(InputAction action) {
  switch (action) {
    case InputAction::VelocityForward:
    case InputAction::VelocityBackward:
    case InputAction::VelocityLeft:
    case InputAction::VelocityRight:
    case InputAction::VelocityYawLeft:
    case InputAction::VelocityYawRight:
      return true;
    default:
      return false;
  }
}

}  // namespace

ViewerInput::ViewerInput(int fd, const configuration::ApplicationConfig &config,
                         protocol::TimeUs host_now_us)
    : fd_(fd),
      config_(config),
      velocity_router_(config, /*key_up_aware=*/true),
      last_heartbeat_us_(host_now_us),
      source_active_(fd >= 0) {
  if (fd_ < 0) {
    return;
  }
  const int flags = ::fcntl(fd_, F_GETFL);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    // An invalid channel is not a startup error: the application keeps
    // supervising, and the viewer just owns no movement.
    ::close(fd_);
    fd_ = -1;
    source_active_ = false;
    return;
  }
}

ViewerInput::~ViewerInput() { Close(); }

void ViewerInput::Poll(InputRouter *mode_router,
                       protocol::TimeUs business_now_us,
                       protocol::TimeUs host_now_us) {
  Tick(host_now_us);
  if (closed_ || fd_ < 0) {
    return;
  }
  if (!ready_sent_) {
    constexpr char ready = '\x01';
    if (::send(fd_, &ready, sizeof(ready), MSG_DONTWAIT | MSG_NOSIGNAL) != 1) {
      Close();
      return;
    }
    ready_sent_ = true;
  }
  std::array<char, kFrameBytes> frame{};
  for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
    // MSG_TRUNC reports the whole datagram even when it is longer than the two
    // bytes this reader accepts, which is how a malformed frame is detected
    // instead of silently read as a valid prefix.
    const ssize_t count =
        ::recv(fd_, frame.data(), frame.size(), MSG_DONTWAIT | MSG_TRUNC);
    if (count > 0) {
      if (count == static_cast<ssize_t>(kFrameBytes)) {
        protocol::TimeUs sent_us = 0;
        for (std::size_t index = frame.size(); index > 2; --index) {
          sent_us =
              (sent_us << 8U) | static_cast<unsigned char>(frame[index - 1]);
        }
        // A paused consumer must not present old queued inputs as fresh.
        // The writer may send after this poll sampled the host clock.
        if (sent_us > host_now_us ? sent_us - host_now_us <= kLeaseUs
                                  : host_now_us - sent_us <= kLeaseUs) {
          HandleFrame(mode_router, frame.data(), business_now_us,
                      sent_us > host_now_us ? host_now_us : sent_us);
        }
      } else {
        // A datagram that is not exactly one command and one key is invalid:
        // it gives the movement back and nothing else.
        ClearMovement();
      }
      continue;
    }
    if (count == 0) {
      Close();
      return;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return;
    }
    // A channel error ends the source; it never ends the application.
    Close();
    return;
  }
}

void ViewerInput::Tick(protocol::TimeUs host_now_us) {
  if (closed_ || fd_ < 0 || !source_active_) {
    return;
  }
  if (host_now_us - last_heartbeat_us_ > kLeaseUs) {
    // The heartbeat stopped: everything the viewer holds is revoked, and a
    // later heartbeat can revive the source without restarting the process.
    source_active_ = false;
    ClearMovement();
  }
}

void ViewerInput::HandleFrame(InputRouter *mode_router, const char *frame,
                              protocol::TimeUs business_now_us,
                              protocol::TimeUs sent_us) {
  const char command = frame[0];
  const char key = frame[1];
  if (command == 'H') {
    if (key != '\0') {
      ClearMovement();
      return;
    }
    last_heartbeat_us_ = sent_us;
    source_active_ = true;
    return;
  }
  if (command == 'B') {
    if (key != '\0') {
      ClearMovement();
      return;
    }
    // Focus loss is not source loss: the viewer is still there, it just holds
    // no keys any more.
    ClearMovement();
    return;
  }
  if (command != 'P' && command != 'R') {
    ClearMovement();
    return;
  }
  if (static_cast<unsigned char>(key) > 0x7fU) {
    // The channel carries ASCII keys only; a high byte is a malformed frame,
    // not an unbound key.
    ClearMovement();
    return;
  }
  if (!source_active_) {
    // An expired lease ignores key state until the next heartbeat proves the
    // viewer is alive; accepting it here would let movement survive the loss
    // of the lease.
    return;
  }
  const std::size_t byte =
      static_cast<std::size_t>(static_cast<unsigned char>(key));
  const InputAction action = config_.input.actions[byte];
  if (action == InputAction::None) {
    return;
  }
  const bool pressed = command == 'P';
  if (IsVelocityAction(action)) {
    static_cast<void>(velocity_router_.OnKey(key, pressed, business_now_us));
    return;
  }
  if (mode_router != nullptr) {
    static_cast<void>(mode_router->OnKey(key, pressed, business_now_us));
  }
}

void ViewerInput::ClearMovement() { velocity_router_.ReleaseAllVelocity(); }

void ViewerInput::Close() {
  if (closed_) {
    return;
  }
  closed_ = true;
  source_active_ = false;
  ClearMovement();
  if (fd_ >= 0) {
    static_cast<void>(::close(fd_));
    fd_ = -1;
  }
}

}  // namespace irmv::application
