// The keyboard input router: one press is one semantic event, the behavioural
// repeat guard a terminal without key-up needs, the batch priority the
// emergency stop needs, and the one body-velocity intent with its deadline.
//
// Every case drives the router with explicit simulated instants. No clock is
// read, nothing sleeps, no thread, no terminal and no DDS takes part, and the
// router never sees the RobotClient, a joint command or a transport.
//
// The assertions are as explicit about what is *not* raised (a release, a
// suppressed auto-repeat, an unbound byte, an expired intent) as about the
// positive events, because the terminal's missing key-up is exactly the gap
// the repeat guard has to cover.

#include "irmv/application/input_router.hpp"

#include <iostream>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::InputEvent;
using irmv::application::InputRouter;
using irmv::application::VelocityIntent;
using irmv::configuration::ApplicationConfig;
using irmv::configuration::LoadApplicationConfig;
using irmv::protocol::TimeUs;

// The shipped bindings are the subject of these cases, so the configuration is
// the repository's own file, loaded once (the test's working directory is the
// project source directory).
const ApplicationConfig &Config() {
  static const ApplicationConfig config =
      LoadApplicationConfig("config/sdk.json");
  return config;
}

// Simulated instants. Everything below is derived from these and from the
// configured guard and intent deadline; nothing reads a clock.
constexpr TimeUs kNowUs = 1'000'000;

TimeUs GuardUs() { return Config().input.repeat_guard_us; }
TimeUs IntentUs() { return Config().input.intent_timeout_us; }

// The terminal's auto-repeat spacing: strictly inside the guard, so every byte
// after the first of a held key is a repeat of the same press.
TimeUs RepeatUs() { return (GuardUs() * 3u) / 5u; }

// An unbound byte is inert, a bound press raises exactly one event, and a
// release of a one-shot key is never a new request.
int OnePressRaisesExactlyOneEvent() {
  InputRouter router(Config());

  // Bytes the file does not bind, including the shifted form of a bound key.
  CHECK(router.OnKey('9', true, kNowUs) == InputEvent::None);
  CHECK(router.OnKey('D', true, kNowUs) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);

  // One press, one event: the batch holds it exactly once.
  CHECK(router.OnKey('d', true, kNowUs) == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::None);

  // Releasing a mode key is not a request of any kind.
  CHECK(router.OnKey('d', false, kNowUs + 1) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);

  // A real re-press after the guard raises again, and only that one event.
  CHECK(router.OnKey('d', true, kNowUs + 1 + GuardUs()) == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// A held key that the terminal keeps reporting (auto-repeat, long press)
// raises the start of the move once, and its guard slides with each repeat so
// it stays silent until the key is really re-pressed.
int RepeatGuardSuppressesAHeldKey() {
  InputRouter router(Config());

  CHECK(router.OnKey('i', true, kNowUs) == InputEvent::Initialize);
  CHECK(router.TakeEvent() == InputEvent::Initialize);

  // Three auto-repeat bytes of the same held key, each repeat well inside the
  // guard of the previous byte.
  TimeUs at_us = kNowUs;
  for (int repeat = 0; repeat < 3; ++repeat) {
    at_us += RepeatUs();
    CHECK(router.OnKey('i', true, at_us) == InputEvent::None);
  }
  CHECK(router.TakeEvent() == InputEvent::None);

  // The guard follows the last repeat, so a byte one microsecond short of a
  // full guard after it is still the same press.
  CHECK(router.OnKey('i', true, at_us + GuardUs() - 1) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);

  // A byte a full guard after the last repeat is a real re-press.
  at_us += GuardUs() - 1;  // the suppressed byte above
  CHECK(router.OnKey('i', true, at_us + GuardUs()) == InputEvent::Initialize);
  CHECK(router.TakeEvent() == InputEvent::Initialize);

  // Reset is guarded the same way, and its guard is its own: a different key
  // pressed while it is suppressed still raises its own event immediately.
  CHECK(router.OnKey('r', true, kNowUs) == InputEvent::Reset);
  CHECK(router.OnKey('r', true, kNowUs + GuardUs() - 1) == InputEvent::None);
  CHECK(router.OnKey('d', true, kNowUs + GuardUs() - 1) == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::Reset);
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// The loop reads a whole burst and drains it afterwards, so a reserved space
// bar read in the same batch must be taken before the ordinary keys it
// preempts - without discarding them.
int SpacePreemptsOrdinaryKeysOfTheSameBatch() {
  InputRouter router(Config());

  CHECK(router.OnKey('d', true, kNowUs) == InputEvent::Damping);
  CHECK(router.OnKey('i', true, kNowUs) == InputEvent::Initialize);
  CHECK(router.OnKey(' ', true, kNowUs) == InputEvent::EmergencyStop);

  CHECK(router.TakeEvent() == InputEvent::EmergencyStop);
  // The preempted keys stay queued, in the documented order.
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::Initialize);
  CHECK(router.TakeEvent() == InputEvent::None);

  // The rest of the order: an exit yields to the emergency stop and outranks
  // the ordinary requests.
  InputRouter priority(Config());
  CHECK(priority.OnKey('x', true, kNowUs) == InputEvent::Exit);
  CHECK(priority.OnKey('z', true, kNowUs) == InputEvent::ZeroTorque);
  CHECK(priority.OnKey(' ', true, kNowUs) == InputEvent::EmergencyStop);
  CHECK(priority.TakeEvent() == InputEvent::EmergencyStop);
  CHECK(priority.TakeEvent() == InputEvent::Exit);
  CHECK(priority.TakeEvent() == InputEvent::ZeroTorque);
  CHECK(priority.TakeEvent() == InputEvent::None);
  return 0;
}

// The whole documented order, all seven bound actions pressed at one instant
// and drained one by one: emergency stop, exit, reset, explicit zero-torque
// stop, ordinary damping stop, initialize, velocity intent last. Seven distinct
// events also fill the batch, so the eight slots never truncate one of them.
int TheBatchDrainsInTheDocumentedOrder() {
  InputRouter router(Config());
  CHECK(router.OnKey('w', true, kNowUs) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('i', true, kNowUs) == InputEvent::Initialize);
  CHECK(router.OnKey('d', true, kNowUs) == InputEvent::Damping);
  CHECK(router.OnKey('z', true, kNowUs) == InputEvent::ZeroTorque);
  CHECK(router.OnKey('r', true, kNowUs) == InputEvent::Reset);
  CHECK(router.OnKey('x', true, kNowUs) == InputEvent::Exit);
  CHECK(router.OnKey(' ', true, kNowUs) == InputEvent::EmergencyStop);

  CHECK(router.TakeEvent() == InputEvent::EmergencyStop);
  CHECK(router.TakeEvent() == InputEvent::Exit);
  CHECK(router.TakeEvent() == InputEvent::Reset);
  CHECK(router.TakeEvent() == InputEvent::ZeroTorque);
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::Initialize);
  CHECK(router.TakeEvent() == InputEvent::VelocityIntent);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// The guard arithmetic, on the two ends a terminal can actually produce: the
// very first press of a byte is unguarded even below the guard (nothing was
// seen yet), and a guard of one microsecond still suppresses a repeat in the
// same microsecond while a byte one microsecond later is a real re-press.
int TheRepeatGuardCountsFromTheLastSeenByte() {
  InputRouter fresh(Config());
  CHECK(GuardUs() > 1);
  CHECK(fresh.OnKey('d', true, GuardUs() - 1) == InputEvent::Damping);
  CHECK(fresh.TakeEvent() == InputEvent::Damping);
  CHECK(fresh.TakeEvent() == InputEvent::None);

  ApplicationConfig tight_config = Config();
  tight_config.input.repeat_guard_us = 1;
  InputRouter tight(tight_config);
  CHECK(tight.OnKey('d', true, 0) == InputEvent::Damping);
  CHECK(tight.OnKey('d', true, 0) == InputEvent::None);  // the same microsecond
  CHECK(tight.OnKey('d', true, 1) == InputEvent::Damping);
  CHECK(tight.TakeEvent() == InputEvent::Damping);
  CHECK(tight.TakeEvent() == InputEvent::None);
  return 0;
}

// A device that can report a release must not turn the release of the reserved
// emergency-stop key into anything at all, least of all a Reset.
int ReleasingSpaceIsNotAReset() {
  InputRouter router(Config());

  CHECK(router.OnKey(' ', true, kNowUs) == InputEvent::EmergencyStop);
  CHECK(router.TakeEvent() == InputEvent::EmergencyStop);

  CHECK(router.OnKey(' ', false, kNowUs + 1) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);

  // The Reset path is untouched by that release and still raises its own event.
  CHECK(router.OnKey('r', true, kNowUs + 2) == InputEvent::Reset);
  CHECK(router.TakeEvent() == InputEvent::Reset);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// An empty read, an unbound byte, a release and a suppressed auto-repeat are
// all "no new key"; none of them is a disconnection.
int NoNewKeyIsNotAnInputLoss() {
  InputRouter router(Config());

  // An empty read is a loop that calls nothing but Tick.
  router.Tick(kNowUs);
  CHECK(router.TakeEvent() == InputEvent::None);
  CHECK(!router.input_lost());

  CHECK(router.OnKey('\t', true, kNowUs) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);
  CHECK(!router.input_lost());

  CHECK(router.OnKey('z', false, kNowUs) == InputEvent::None);
  CHECK(!router.input_lost());

  CHECK(router.OnKey('z', true, kNowUs) == InputEvent::ZeroTorque);
  CHECK(router.TakeEvent() == InputEvent::ZeroTorque);
  CHECK(router.OnKey('z', true, kNowUs + GuardUs() - 1) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);
  CHECK(!router.input_lost());

  // Only the adapter's own lost-device report raises the latch.
  router.OnInputLost();
  CHECK(router.input_lost());
  return 0;
}

// EOF or a terminal error asks for the orderly exit, never for the emergency
// stop. The request is coalesced per batch: draining it and reporting the loss
// again raises it again.
int InputLostAsksForTheExit() {
  InputRouter router(Config());

  router.OnInputLost();
  CHECK(router.input_lost());
  CHECK(router.TakeEvent() == InputEvent::Exit);
  CHECK(router.TakeEvent() == InputEvent::None);

  // Reporting it again keeps the latch and asks again: the request is coalesced
  // per batch only, so once the batch above was drained a second lost-device
  // report raises a second Exit.
  router.OnInputLost();
  CHECK(router.input_lost());
  CHECK(router.TakeEvent() == InputEvent::Exit);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// The velocity intent is zeroed strictly after its deadline and nothing else
// ages with it.
int VelocityIntentExpiresAtItsDeadline() {
  InputRouter router(Config());

  CHECK(router.OnKey('w', true, kNowUs) == InputEvent::VelocityIntent);
  CHECK(router.TakeEvent() == InputEvent::VelocityIntent);
  const VelocityIntent &held = router.velocity_intent();
  CHECK(held.active);
  CHECK(held.forward == 1.0);
  CHECK(held.lateral == 0.0);
  CHECK(held.yaw == 0.0);
  CHECK(held.stamp_us == kNowUs);

  // At exactly the deadline the intent is still live: the expiry is strict.
  router.Tick(kNowUs + IntentUs());
  CHECK(router.velocity_intent().active);
  CHECK(router.velocity_intent().forward == 1.0);

  // One microsecond later it is zeroed and inactive, and it stays that way.
  router.Tick(kNowUs + IntentUs() + 1);
  CHECK(!router.velocity_intent().active);
  CHECK(router.velocity_intent().forward == 0.0);
  CHECK(router.velocity_intent().lateral == 0.0);
  CHECK(router.velocity_intent().yaw == 0.0);
  router.Tick(kNowUs + IntentUs() + 2);
  CHECK(!router.velocity_intent().active);

  // A one-shot event of the same age is untouched: only the intent ages.
  InputRouter stop(Config());
  CHECK(stop.OnKey('z', true, kNowUs) == InputEvent::ZeroTorque);
  stop.Tick(kNowUs + IntentUs() + 1);
  CHECK(stop.TakeEvent() == InputEvent::ZeroTorque);

  // A new press revives the intent from its own stamp, never from the old one.
  const TimeUs again_us = kNowUs + IntentUs() + 1;
  CHECK(router.OnKey('w', true, again_us) == InputEvent::VelocityIntent);
  CHECK(router.TakeEvent() == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().active);
  CHECK(router.velocity_intent().stamp_us == again_us);
  router.Tick(again_us + IntentUs());
  CHECK(router.velocity_intent().active);
  return 0;
}

// An intent event that was raised but not drained before the deadline is not
// delivered afterwards: the expired intent cannot be reused.
int ExpiredVelocityEventIsNotDelivered() {
  InputRouter router(Config());

  CHECK(router.OnKey('w', true, kNowUs) == InputEvent::VelocityIntent);
  router.Tick(kNowUs + IntentUs() + 1);
  CHECK(!router.velocity_intent().active);
  CHECK(router.TakeEvent() == InputEvent::None);

  // A new byte is the only thing that raises a new velocity event.
  CHECK(router.OnKey('w', true, kNowUs + IntentUs() + 1) ==
        InputEvent::VelocityIntent);
  CHECK(router.TakeEvent() == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().active);
  return 0;
}

// A velocity byte has no repeat guard: every byte refreshes the deadline, each
// axis carries its own documented unit value, and a release gives back only
// that axis.
int VelocityIntentTracksTheHeldAxes() {
  InputRouter router(Config());

  // A held velocity key is kept alive by its auto-repeat bytes.
  CHECK(router.OnKey('w', true, kNowUs) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('w', true, kNowUs + RepeatUs()) ==
        InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().stamp_us == kNowUs + RepeatUs());
  router.Tick(kNowUs + IntentUs());
  CHECK(router.velocity_intent().active);
  CHECK(router.velocity_intent().forward == 1.0);

  // Each axis key claims its own component with the documented unit value.
  const TimeUs axes_us = kNowUs + IntentUs();
  CHECK(router.OnKey('a', true, axes_us) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('q', true, axes_us) == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().forward == 1.0);
  CHECK(router.velocity_intent().lateral == 1.0);
  CHECK(router.velocity_intent().yaw == 1.0);

  // The opposing keys flip a component; the intent is a unit vector, never an
  // accumulation.
  CHECK(router.OnKey('s', true, axes_us) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('f', true, axes_us) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('e', true, axes_us) == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().forward == -1.0);
  CHECK(router.velocity_intent().lateral == -1.0);
  CHECK(router.velocity_intent().yaw == -1.0);

  // A release removes only its own component...
  CHECK(router.OnKey('f', false, axes_us + 1) == InputEvent::VelocityIntent);
  CHECK(router.velocity_intent().lateral == 0.0);
  CHECK(router.velocity_intent().forward == -1.0);
  CHECK(router.velocity_intent().yaw == -1.0);
  // ...and the intent is inactive once no component is left.
  CHECK(router.OnKey('s', false, axes_us + 1) == InputEvent::VelocityIntent);
  CHECK(router.OnKey('e', false, axes_us + 1) == InputEvent::VelocityIntent);
  CHECK(!router.velocity_intent().active);
  CHECK(router.velocity_intent().forward == 0.0);
  CHECK(router.velocity_intent().yaw == 0.0);

  // Every byte of the burst raised the one event, not one per byte.
  CHECK(router.TakeEvent() == InputEvent::VelocityIntent);
  CHECK(router.TakeEvent() == InputEvent::None);
  return 0;
}

// An explicit zero-torque stop and an ordinary damping stop are two different
// events, and neither of them touches the intent or the loss latch.
int ZeroTorqueAndDampingStopsAreDistinct() {
  InputRouter router(Config());
  CHECK(router.OnKey('z', true, kNowUs) == InputEvent::ZeroTorque);
  CHECK(router.OnKey('d', true, kNowUs) == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::ZeroTorque);
  CHECK(router.TakeEvent() == InputEvent::Damping);
  CHECK(router.TakeEvent() == InputEvent::None);

  // Neither stop path touched anything else the router keeps.
  CHECK(!router.velocity_intent().active);
  CHECK(router.velocity_intent().stamp_us == 0);
  CHECK(!router.input_lost());

  // Each byte keeps its own guard: a repeated 'd' stays silent while a first
  // 'z' in the same instant raises immediately.
  InputRouter guarded(Config());
  CHECK(guarded.OnKey('d', true, kNowUs) == InputEvent::Damping);
  CHECK(guarded.OnKey('d', true, kNowUs + GuardUs() - 1) == InputEvent::None);
  CHECK(guarded.OnKey('z', true, kNowUs + GuardUs() - 1) ==
        InputEvent::ZeroTorque);
  CHECK(guarded.TakeEvent() == InputEvent::ZeroTorque);
  CHECK(guarded.TakeEvent() == InputEvent::Damping);
  CHECK(guarded.TakeEvent() == InputEvent::None);
  return 0;
}

}  // namespace

// The locomotion key is a mode request like the others: one event per press,
// suppressed while the key is held, and it never claims a velocity axis.
int StartLocomotionIsAModeRequest() {
  InputRouter router(Config());

  CHECK(router.OnKey('l', true, kNowUs) == InputEvent::StartLocomotion);
  CHECK(router.TakeEvent() == InputEvent::StartLocomotion);

  // A held key raises nothing more until it is really pressed again.
  CHECK(router.OnKey('l', true, kNowUs + RepeatUs()) == InputEvent::None);
  CHECK(router.TakeEvent() == InputEvent::None);
  CHECK(router.OnKey('l', true, kNowUs + RepeatUs() + GuardUs()) ==
        InputEvent::StartLocomotion);
  CHECK(router.TakeEvent() == InputEvent::StartLocomotion);

  // Its release is not a new request, and it leaves the velocity intent alone.
  CHECK(router.OnKey('l', false, kNowUs) == InputEvent::None);
  CHECK(!router.velocity_intent().active);
  return 0;
}

int main() {
  int failures = 0;
  failures += OnePressRaisesExactlyOneEvent();
  failures += RepeatGuardSuppressesAHeldKey();
  failures += SpacePreemptsOrdinaryKeysOfTheSameBatch();
  failures += TheBatchDrainsInTheDocumentedOrder();
  failures += TheRepeatGuardCountsFromTheLastSeenByte();
  failures += ReleasingSpaceIsNotAReset();
  failures += NoNewKeyIsNotAnInputLoss();
  failures += InputLostAsksForTheExit();
  failures += VelocityIntentExpiresAtItsDeadline();
  failures += ExpiredVelocityEventIsNotDelivered();
  failures += VelocityIntentTracksTheHeldAxes();
  failures += ZeroTorqueAndDampingStopsAreDistinct();
  failures += StartLocomotionIsAModeRequest();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
