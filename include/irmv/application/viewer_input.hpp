#pragma once

#include <cstddef>

#include "irmv/application/input_router.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

// The optional input channel of the simulation viewer. The launcher hands one
// end of a SOCK_SEQPACKET socketpair to this process; the viewer writes exactly
// ten bytes per datagram: two ASCII bytes followed by the sender's uint64
// host monotonic timestamp in microseconds (little endian).
// The application sends a single 0x01 datagram when it first starts reading.
//
//   b'P' + key   a key press
//   b'R' + key   a key release
//   b'B' + 0     the window lost focus: revoke every held key
//   b'H' + 0     heartbeat, at most 50 ms apart on the host clock
//
// The channel is a source of its own, not another terminal. It has a 250 ms
// host-monotonic lease refreshed only by heartbeats: when the viewer closes,
// stops heartbeating, sends a malformed frame or loses focus, the movement it
// holds is given back and the application keeps supervising. Nothing here is
// ever an exit or a reset.
//
// Mode keys are routed into the application's one InputRouter, so the viewer
// and the terminal share the same batch and event priority. Velocity keys are
// routed into a separate key-up aware InputRouter, so a viewer key and a
// terminal key never combine into one intent. A live, held viewer intent
// takes precedence over terminal movement.
class ViewerInput {
 public:
  // A viewer that has not heartbeated for longer than this no longer owns the
  // movement keys. Host monotonic time; the simulation's logical clock is not
  // involved.
  static constexpr protocol::TimeUs kLeaseUs = 250'000;
  static constexpr std::size_t kFrameBytes = 10;

  // `fd` is inherited from the launcher and becomes non-blocking here; an
  // unusable descriptor leaves the object inert instead of failing the
  // application. `host_now_us` anchors the startup lease until the first
  // heartbeat arrives.
  ViewerInput(int fd, const configuration::ApplicationConfig &config,
              protocol::TimeUs host_now_us);
  ~ViewerInput();

  ViewerInput(const ViewerInput &) = delete;
  ViewerInput &operator=(const ViewerInput &) = delete;

  // True while the descriptor is a channel this object can read.
  bool usable() const { return fd_ >= 0; }
  // The descriptor to poll for readability, or -1 when there is no channel
  // left to read.
  int fd() const { return fd_; }
  // The peer closed or the channel failed; no further frame will be read.
  bool closed() const { return closed_; }
  // The lease is live: the viewer is alive and this source owns movement.
  bool active() const { return fd_ >= 0 && source_active_ && !closed_; }
  const VelocityIntent &velocity_intent() const {
    return velocity_router_.velocity_intent();
  }

  // Ages the lease and drains every pending datagram, bounded like the
  // terminal read. `business_now_us` stamps the events of the application's
  // clock; `host_now_us` stamps the lease and is never the simulation clock.
  void Poll(InputRouter *mode_router, protocol::TimeUs business_now_us,
            protocol::TimeUs host_now_us);
  // Ages the lease without reading. The loop calls this when the descriptor is
  // not part of a wait, so movement cannot outlive its heartbeat.
  void Tick(protocol::TimeUs host_now_us);

 private:
  void HandleFrame(InputRouter *mode_router, const char *frame,
                   protocol::TimeUs business_now_us, protocol::TimeUs sent_us);
  void ClearMovement();
  void Close();

  int fd_;
  const configuration::ApplicationConfig &config_;
  InputRouter velocity_router_;
  protocol::TimeUs last_heartbeat_us_{0};
  bool ready_sent_{false};
  bool source_active_{false};
  bool closed_{false};
};

}  // namespace irmv::application
