#include "irmv/application/input_router.hpp"

namespace irmv::application {

namespace {

using configuration::InputAction;

// The one value of a velocity key: each axis key claims its component at unit
// magnitude with the sign of the direction it asks for, so the intent is a
// direction and never an accumulating command.
constexpr double kUnitVelocity = 1.0;

}  // namespace

InputRouter::InputRouter(const configuration::ApplicationConfig &config,
                         bool key_up_aware)
    : config_(config), key_up_aware_(key_up_aware) {}

InputEvent InputRouter::OnKey(char key, bool pressed, protocol::TimeUs now_us) {
  // The table is indexed by the byte, not by its signed value, so every one of
  // the 256 entries is reachable and an unbound byte stays inert.
  const std::size_t byte =
      static_cast<std::size_t>(static_cast<unsigned char>(key));
  const InputAction action = config_.input.actions[byte];
  switch (action) {
    case InputAction::None:
      return InputEvent::None;
    case InputAction::Damping:
      return OneShot(byte, pressed, now_us, InputEvent::Damping);
    case InputAction::Initialize:
      return OneShot(byte, pressed, now_us, InputEvent::Initialize);
    case InputAction::ZeroTorque:
      return OneShot(byte, pressed, now_us, InputEvent::ZeroTorque);
    case InputAction::EmergencyStop:
      return OneShot(byte, pressed, now_us, InputEvent::EmergencyStop);
    case InputAction::Reset:
      return OneShot(byte, pressed, now_us, InputEvent::Reset);
    case InputAction::Exit:
      return OneShot(byte, pressed, now_us, InputEvent::Exit);
    case InputAction::StartLocomotion:
      // A mode request: one event per press, like Initialize. Whether it can be
      // honoured is the behaviour's decision, not the router's.
      return OneShot(byte, pressed, now_us, InputEvent::StartLocomotion);
    case InputAction::VelocityForward:
    case InputAction::VelocityBackward:
    case InputAction::VelocityLeft:
    case InputAction::VelocityRight:
    case InputAction::VelocityYawLeft:
    case InputAction::VelocityYawRight:
      // Velocity keys are held, not latched: every byte refreshes the intent.
      return pressed ? PressVelocity(byte, action, now_us)
                     : ReleaseVelocity(byte, action, now_us);
  }
  return InputEvent::None;
}

InputEvent InputRouter::TakeEvent() {
  if (batch_size_ == 0) {
    return InputEvent::None;
  }
  // The highest-priority event is taken and removed; everything the batch still
  // holds stays queued for the following calls.
  std::size_t best = 0;
  for (std::size_t index = 1; index < batch_size_; ++index) {
    if (Priority(batch_[index]) < Priority(batch_[best])) {
      best = index;
    }
  }
  const InputEvent event = batch_[best];
  for (std::size_t index = best + 1; index < batch_size_; ++index) {
    batch_[index - 1] = batch_[index];
  }
  --batch_size_;
  return event;
}

void InputRouter::Tick(protocol::TimeUs now_us) {
  // Only the velocity intent ages, and only strictly past its deadline. A
  // one-shot event is delivered until it is taken; the loss latch is never
  // inferred from the passage of time.
  if (!intent_.active) {
    return;
  }
  if (now_us - intent_.stamp_us > config_.input.intent_timeout_us) {
    ExpireIntent();
  }
}

void InputRouter::OnInputLost() {
  // EOF or a terminal error asks for the orderly exit: the loop's exit path
  // starts with the ordinary damping stop, and this is never an emergency stop
  // and never a claim that the hardware stopped.
  input_lost_ = true;
  Push(InputEvent::Exit);
}

InputEvent InputRouter::OneShot(std::size_t byte, bool pressed,
                                protocol::TimeUs now_us, InputEvent event) {
  if (!pressed) {
    // A terminal byte stream has no reliable key-up, and a device that does
    // report one must not turn a release into a new request - releasing the
    // reserved emergency-stop key is not a reset.
    return InputEvent::None;
  }
  if (seen_[byte] &&
      now_us - last_seen_us_[byte] < config_.input.repeat_guard_us) {
    // The guard follows the last byte, not the last delivered event, so a key
    // the terminal keeps repeating stays suppressed until it is really
    // re-pressed after a full guard.
    last_seen_us_[byte] = now_us;
    return InputEvent::None;
  }
  seen_[byte] = true;
  last_seen_us_[byte] = now_us;
  Push(event);
  return event;
}

bool InputRouter::VelocityAxis(InputAction action, std::size_t *axis,
                               double *value) noexcept {
  switch (action) {
    case InputAction::VelocityForward:
      *axis = 0;
      *value = kUnitVelocity;
      return true;
    case InputAction::VelocityBackward:
      *axis = 0;
      *value = -kUnitVelocity;
      return true;
    case InputAction::VelocityLeft:
      *axis = 1;
      *value = kUnitVelocity;
      return true;
    case InputAction::VelocityRight:
      *axis = 1;
      *value = -kUnitVelocity;
      return true;
    case InputAction::VelocityYawLeft:
      *axis = 2;
      *value = kUnitVelocity;
      return true;
    case InputAction::VelocityYawRight:
      *axis = 2;
      *value = -kUnitVelocity;
      return true;
    default:
      return false;
  }
}

void InputRouter::SetAxis(std::size_t axis, double value) noexcept {
  switch (axis) {
    case 0:
      intent_.forward = value;
      break;
    case 1:
      intent_.lateral = value;
      break;
    case 2:
      intent_.yaw = value;
      break;
    default:
      break;
  }
}

InputEvent InputRouter::PressVelocity(std::size_t byte, InputAction action,
                                      protocol::TimeUs now_us) {
  std::size_t axis = 0;
  double value = 0.0;
  if (!VelocityAxis(action, &axis, &value)) {
    return InputEvent::None;
  }
  if (key_up_aware_) {
    // A real key-up stream: the newest held key owns the axis until it is
    // released, and a repeated press is the same key held, not a new claim.
    velocity_held_[byte] = true;
    velocity_pressed_us_[byte] = now_us;
    velocity_owner_[axis] = byte;
  }
  SetAxis(axis, value);
  intent_.active = true;
  intent_.stamp_us = now_us;
  Push(InputEvent::VelocityIntent);
  return InputEvent::VelocityIntent;
}

InputEvent InputRouter::ReleaseVelocity(std::size_t byte, InputAction action,
                                        protocol::TimeUs now_us) {
  std::size_t axis = 0;
  double value = 0.0;
  if (!VelocityAxis(action, &axis, &value)) {
    return InputEvent::None;
  }
  if (key_up_aware_) {
    if (!velocity_held_[byte]) {
      // A release of a key this router does not hold says nothing about the
      // keys it does: it must not cancel the axis.
      return InputEvent::None;
    }
    velocity_held_[byte] = false;
    if (velocity_owner_[axis] == byte) {
      // The owner let go: the newest still-held key of this axis takes over,
      // which is what keeps an opposite or duplicate key from cancelling it.
      ReclaimAxis(axis);
    }
  } else {
    // A terminal has no key-up, so a release gives back the whole component of
    // its axis; the other axes and the deadline are untouched.
    SetAxis(axis, 0.0);
  }
  intent_.active =
      intent_.forward != 0.0 || intent_.lateral != 0.0 || intent_.yaw != 0.0;
  intent_.stamp_us = now_us;
  Push(InputEvent::VelocityIntent);
  return InputEvent::VelocityIntent;
}

void InputRouter::ReclaimAxis(std::size_t axis) {
  std::size_t owner = kKeyCount;
  protocol::TimeUs newest = 0;
  for (std::size_t byte = 0; byte < kKeyCount; ++byte) {
    if (!velocity_held_[byte]) {
      continue;
    }
    std::size_t candidate_axis = 0;
    double candidate = 0.0;
    if (!VelocityAxis(config_.input.actions[byte], &candidate_axis,
                      &candidate) ||
        candidate_axis != axis) {
      continue;
    }
    // `>=` makes the newest press win, and a repeated press of the same key
    // moves its own stamp without changing which key wins.
    if (owner == kKeyCount || velocity_pressed_us_[byte] >= newest) {
      owner = byte;
      newest = velocity_pressed_us_[byte];
    }
  }
  velocity_owner_[axis] = owner;
  if (owner == kKeyCount) {
    SetAxis(axis, 0.0);
    return;
  }
  std::size_t held_axis = 0;
  double held_value = 0.0;
  static_cast<void>(
      VelocityAxis(config_.input.actions[owner], &held_axis, &held_value));
  SetAxis(axis, held_value);
}

void InputRouter::ReleaseAllVelocity() {
  velocity_held_.fill(false);
  velocity_pressed_us_.fill(0);
  velocity_owner_.fill(kKeyCount);
  ExpireIntent();
}

void InputRouter::Push(InputEvent event) {
  if (event == InputEvent::None) {
    return;
  }
  // A repeated event carries no more meaning than the one already queued, so
  // the batch never holds the same event twice and eight slots cover the seven
  // semantic events.
  for (std::size_t index = 0; index < batch_size_; ++index) {
    if (batch_[index] == event) {
      return;
    }
  }
  // Dedupe caps the batch at the seven distinct events, so this capacity guard
  // is unreachable by construction: it is the boundary the fixed array needs,
  // not a case the router can reach.
  if (batch_size_ < kBatchCapacity) {
    batch_[batch_size_] = event;
    ++batch_size_;
  }
}

std::size_t InputRouter::Priority(InputEvent event) {
  switch (event) {
    case InputEvent::EmergencyStop:
      return 0;
    case InputEvent::Exit:
      return 1;
    case InputEvent::Reset:
      return 2;
    case InputEvent::ZeroTorque:
      return 3;
    case InputEvent::Damping:
      return 4;
    case InputEvent::Initialize:
      return 5;
    // The locomotion request ranks with the ordinary mode requests and above
    // the velocity intent: a key that asks for a behaviour must not be held
    // behind the axis keys it would then be driven by.
    case InputEvent::StartLocomotion:
      return 6;
    case InputEvent::VelocityIntent:
      return 7;
    case InputEvent::None:
      break;
  }
  return 7;
}

void InputRouter::ExpireIntent() {
  intent_ = VelocityIntent{};
  // The event an expired intent raised is not delivered afterwards: the intent
  // it describes is gone, and only a new key byte raises a new one.
  std::size_t write = 0;
  for (std::size_t read = 0; read < batch_size_; ++read) {
    if (batch_[read] != InputEvent::VelocityIntent) {
      batch_[write] = batch_[read];
      ++write;
    }
  }
  batch_size_ = write;
}

}  // namespace irmv::application
