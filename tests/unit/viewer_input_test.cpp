// The optional simulation-viewer input channel: timestamped datagrams, a
// host-monotonic heartbeat lease, key-up aware velocity, blur/close cleanup and
// mode keys that still enter the application's one router.
//
// Every case drives the descriptor and the clock explicitly. No thread, no
// sleep and no simulation process takes part; the socketpair is the real
// SOCK_SEQPACKET contract the launcher hands over.

#include "irmv/application/viewer_input.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstddef>
#include <iostream>

#include "irmv/application/input_router.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::InputEvent;
using irmv::application::InputRouter;
using irmv::application::ViewerInput;
using irmv::configuration::ApplicationConfig;
using irmv::configuration::LoadApplicationConfig;
using irmv::protocol::TimeUs;

const ApplicationConfig &Config() {
  static const ApplicationConfig config =
      LoadApplicationConfig("config/sdk.json");
  return config;
}

constexpr TimeUs kStartUs = 5'000'000;

// One real SOCK_SEQPACKET pair, closed on both ends at scope exit.
class Pair {
 public:
  Pair() {
    int descriptors[2] = {-1, -1};
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, descriptors) == 0) {
      child = descriptors[0];
      peer = descriptors[1];
    }
  }
  ~Pair() {
    if (child >= 0) ::close(child);
    if (peer >= 0) ::close(peer);
  }
  Pair(const Pair &) = delete;
  Pair &operator=(const Pair &) = delete;

  bool Send(const char *bytes, std::size_t size) const {
    return ::send(peer, bytes, size, 0) == static_cast<ssize_t>(size);
  }
  bool Frame(char tag, char key, TimeUs stamp) const {
    std::array<char, ViewerInput::kFrameBytes> frame{tag, key};
    for (std::size_t i = 2; i < frame.size(); ++i) {
      frame[i] = static_cast<char>(stamp & 0xffU);
      stamp >>= 8U;
    }
    return Send(frame.data(), frame.size());
  }
  bool Press(char key) const { return Frame('P', key, kStartUs); }
  bool Release(char key) const { return Frame('R', key, kStartUs); }
  bool Heartbeat(TimeUs stamp = kStartUs) const {
    return Frame('H', '\0', stamp);
  }
  bool Blur() const { return Frame('B', '\0', kStartUs); }

  int child{-1};
  int peer{-1};
};

// The key-up aware router behind the viewer: a still-held key survives the
// release of another key on its axis, an unheld release is a no-op, and a real
// release of the held key gives the axis back.
int KeyUpAwareVelocityKeepsTheHeldKey() {
  InputRouter router(Config(), /*key_up_aware=*/true);
  CHECK(router.OnKey('w', true, kStartUs) == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().forward == 1.0);

  // An opposite key claims the axis while both are held...
  CHECK(router.OnKey('s', true, kStartUs + 10) == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().forward == -1.0);
  // ...and releasing it gives the axis back to the still-held key instead of
  // zeroing it.
  CHECK(router.OnKey('s', false, kStartUs + 20) == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().forward == 1.0);
  CHECK(router.velocity_intent().active);

  // Releasing a key this router never held says nothing about the axis.
  CHECK(router.OnKey('s', false, kStartUs + 30) == InputEvent::None);
  CHECK(router.velocity_intent().forward == 1.0);

  // The held key still lets go normally.
  CHECK(router.OnKey('w', false, kStartUs + 40) == InputEvent::VelocityIntent);
  CHECK(!router.velocity_intent().active);
  CHECK(router.velocity_intent().forward == 0.0);
  return 0;
}

// Blur/close/lease cleanup is one operation: every held key and the intent go.
int ReleaseAllVelocityClearsEverything() {
  InputRouter router(Config(), /*key_up_aware=*/true);
  CHECK(router.OnKey('w', true, kStartUs) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('a', true, kStartUs) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('q', true, kStartUs) == InputEvent::VelocityIntent);
  router.ReleaseAllVelocity();
  CHECK(!router.velocity_intent().active);
  CHECK(router.velocity_intent().forward == 0.0);
  CHECK(router.velocity_intent().lateral == 0.0);
  CHECK(router.velocity_intent().yaw == 0.0);

  // A release after the cleanup is a no-op, not a revival.
  CHECK(router.OnKey('w', false, kStartUs + 1) == InputEvent::None);
  CHECK(!router.velocity_intent().active);
  return 0;
}

// One heartbeat leases the source for 250 ms of host time, strictly.
int HeartbeatLeaseIsIndependentAndStrict() {
  Pair pair;
  CHECK(pair.child >= 0);
  ViewerInput viewer(pair.child, Config(), kStartUs);
  CHECK(viewer.usable());
  CHECK(viewer.active());
  CHECK(pair.Heartbeat(kStartUs + 1));
  viewer.Poll(nullptr, kStartUs, kStartUs + 1);
  CHECK(viewer.active());

  // A key held without any auto-repeat survives inside the lease.
  CHECK(pair.Press('w'));
  viewer.Poll(nullptr, kStartUs + 2, kStartUs + 2);
  CHECK(viewer.velocity_intent().forward == 1.0);
  viewer.Tick(kStartUs + 1 + ViewerInput::kLeaseUs);
  CHECK(viewer.active());
  CHECK(viewer.velocity_intent().forward == 1.0);

  // One microsecond past the heartbeat lease revokes the movement, without
  // closing the channel or reviving the terminal.
  viewer.Tick(kStartUs + 1 + ViewerInput::kLeaseUs + 1);
  CHECK(!viewer.active());
  CHECK(!viewer.closed());
  CHECK(!viewer.velocity_intent().active);
  CHECK(viewer.velocity_intent().forward == 0.0);

  // Old buffered keys and heartbeats cannot reopen the source after expiry.
  CHECK(pair.Heartbeat());
  CHECK(pair.Press('w'));
  CHECK(pair.Press('d'));
  InputRouter router(Config());
  const TimeUs revived = kStartUs + ViewerInput::kLeaseUs + 10;
  viewer.Poll(&router, revived, revived);
  CHECK(!viewer.active());
  CHECK(router.TakeEvent() == InputEvent::None);
  // A fresh heartbeat revives the source but not the keys blurred by expiry.
  CHECK(pair.Heartbeat(revived));
  viewer.Poll(nullptr, kStartUs + ViewerInput::kLeaseUs + 10,
              kStartUs + ViewerInput::kLeaseUs + 10);
  CHECK(viewer.active());
  CHECK(!viewer.velocity_intent().active);
  return 0;
}

// Blur revokes the held keys but leaves the live source alone; focus returning
// needs no restart and starts from no movement.
int BlurRevokesKeysButNotTheSource() {
  Pair pair;
  CHECK(pair.child >= 0);
  ViewerInput viewer(pair.child, Config(), kStartUs);
  CHECK(pair.Heartbeat());
  CHECK(pair.Press('w'));
  viewer.Poll(nullptr, kStartUs, kStartUs);
  CHECK(viewer.velocity_intent().forward == 1.0);
  CHECK(pair.Blur());
  viewer.Poll(nullptr, kStartUs + 1, kStartUs + 1);
  CHECK(viewer.active());
  CHECK(!viewer.velocity_intent().active);

  // The key that was held before the blur is gone; a new press is needed.
  CHECK(pair.Release('w'));
  viewer.Poll(nullptr, kStartUs + 2, kStartUs + 2);
  CHECK(!viewer.velocity_intent().active);
  CHECK(pair.Press('w'));
  viewer.Poll(nullptr, kStartUs + 3, kStartUs + 3);
  CHECK(viewer.velocity_intent().forward == 1.0);
  return 0;
}

// A malformed datagram clears movement and nothing else; the channel stays
// readable and a later valid frame works.
int InvalidFrameClearsMovementWithoutExiting() {
  Pair pair;
  CHECK(pair.child >= 0);
  ViewerInput viewer(pair.child, Config(), kStartUs);
  CHECK(pair.Heartbeat());
  CHECK(pair.Press('w'));
  viewer.Poll(nullptr, kStartUs, kStartUs);
  CHECK(viewer.velocity_intent().forward == 1.0);

  const char oversized[ViewerInput::kFrameBytes + 1] = {'P', 'a'};
  CHECK(pair.Send(oversized, sizeof(oversized)));
  viewer.Poll(nullptr, kStartUs + 1, kStartUs + 1);
  CHECK(!viewer.closed());
  CHECK(!viewer.velocity_intent().active);

  const char truncated[1] = {'H'};
  CHECK(pair.Send(truncated, sizeof(truncated)));
  viewer.Poll(nullptr, kStartUs + 2, kStartUs + 2);
  CHECK(!viewer.closed());

  CHECK(pair.Frame('H', 'x', kStartUs + 3));
  viewer.Poll(nullptr, kStartUs + 3, kStartUs + 3);
  CHECK(!viewer.closed());

  CHECK(pair.Press('w'));
  viewer.Poll(nullptr, kStartUs + 4, kStartUs + 4);
  CHECK(viewer.velocity_intent().forward == 1.0);
  return 0;
}

// EOF ends the source, zeroes the movement and never reaches the application
// as an exit request.
int PeerCloseRetiresTheSource() {
  Pair pair;
  CHECK(pair.child >= 0);
  ViewerInput viewer(pair.child, Config(), kStartUs);
  CHECK(pair.Heartbeat());
  CHECK(pair.Press('w'));
  viewer.Poll(nullptr, kStartUs, kStartUs);
  CHECK(viewer.velocity_intent().forward == 1.0);

  ::close(pair.peer);
  pair.peer = -1;
  viewer.Poll(nullptr, kStartUs + 1, kStartUs + 1);
  CHECK(viewer.closed());
  CHECK(!viewer.active());
  CHECK(viewer.fd() == -1);
  CHECK(!viewer.velocity_intent().active);

  // It stays retired and inert.
  viewer.Poll(nullptr, kStartUs + 2, kStartUs + 2);
  CHECK(viewer.closed());
  return 0;
}

// Mode keys of the viewer go through the caller's one router and keep its
// priority; velocity keys stay in the viewer's own router.
int ModeKeysShareTheMainRouter() {
  Pair pair;
  CHECK(pair.child >= 0);
  ViewerInput viewer(pair.child, Config(), kStartUs);
  InputRouter router(Config());
  CHECK(pair.Heartbeat());
  CHECK(pair.Press('w'));
  CHECK(pair.Press('d'));
  CHECK(pair.Press(' '));
  viewer.Poll(&router, kStartUs, kStartUs);
  CHECK(viewer.velocity_intent().forward == 1.0);
  // The velocity key never entered the main router.
  CHECK(router.velocity_intent().forward == 0.0);
  CHECK(router.TakeEvent() == InputEvent::EmergencyStop);
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// A channel that cannot be made non-blocking is inert, and the application
// keeps running with the terminal source alone.
int InvalidDescriptorIsInert() {
  ViewerInput missing(-1, Config(), kStartUs);
  CHECK(!missing.usable());
  CHECK(!missing.active());
  CHECK(missing.fd() == -1);
  missing.Poll(nullptr, kStartUs, kStartUs);
  CHECK(!missing.active());

  InputRouter router(Config());
  ViewerInput not_a_socket(::open("/dev/null", O_RDONLY), Config(), kStartUs);
  // A descriptor that is not a datagram channel is not a startup failure
  // either: the first read retires it, and no event or exit was raised.
  CHECK(not_a_socket.usable());
  not_a_socket.Poll(&router, kStartUs, kStartUs);
  CHECK(not_a_socket.closed());
  CHECK(!not_a_socket.active());
  CHECK(not_a_socket.fd() == -1);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

}  // namespace

int main() {
  int failures = 0;
  failures += KeyUpAwareVelocityKeepsTheHeldKey();
  failures += ReleaseAllVelocityClearsEverything();
  failures += HeartbeatLeaseIsIndependentAndStrict();
  failures += BlurRevokesKeysButNotTheSource();
  failures += InvalidFrameClearsMovementWithoutExiting();
  failures += PeerCloseRetiresTheSource();
  failures += ModeKeysShareTheMainRouter();
  failures += InvalidDescriptorIsInert();
  if (failures != 0) {
    std::cerr << failures << " viewer input case(s) failed\n";
    return 1;
  }
  std::cout << "viewer input: all cases passed\n";
  return 0;
}
