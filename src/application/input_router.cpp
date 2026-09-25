#include "irmv/application/input_router.hpp"

namespace irmv::application {

namespace {

using configuration::InputAction;

// The one value of a velocity key: each axis key claims its component at unit
// magnitude with the sign of the direction it asks for, so the intent is a
// direction and never an accumulating command.
constexpr double kUnitVelocity = 1.0;

}  // namespace

InputRouter::InputRouter(const configuration::ApplicationConfig &config)
    : config_(config) {}

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
    case InputAction::VelocityForward:
    case InputAction::VelocityBackward:
    case InputAction::VelocityLeft:
    case InputAction::VelocityRight:
    case InputAction::VelocityYawLeft:
    case InputAction::VelocityYawRight:
      // Velocity keys are held, not latched: every byte refreshes the intent.
      return pressed ? PressVelocity(action, now_us)
                     : ReleaseVelocity(action, now_us);
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

InputEvent InputRouter::PressVelocity(InputAction action,
                                      protocol::TimeUs now_us) {
  switch (action) {
    case InputAction::VelocityForward:
      intent_.forward = kUnitVelocity;
      break;
    case InputAction::VelocityBackward:
      intent_.forward = -kUnitVelocity;
      break;
    case InputAction::VelocityLeft:
      intent_.lateral = kUnitVelocity;
      break;
    case InputAction::VelocityRight:
      intent_.lateral = -kUnitVelocity;
      break;
    case InputAction::VelocityYawLeft:
      intent_.yaw = kUnitVelocity;
      break;
    case InputAction::VelocityYawRight:
      intent_.yaw = -kUnitVelocity;
      break;
    default:
      break;
  }
  intent_.active = true;
  intent_.stamp_us = now_us;
  Push(InputEvent::VelocityIntent);
  return InputEvent::VelocityIntent;
}

InputEvent InputRouter::ReleaseVelocity(InputAction action,
                                        protocol::TimeUs now_us) {
  // A release gives back only the component of its own key; the other axes and
  // the deadline the held keys refreshed are untouched.
  switch (action) {
    case InputAction::VelocityForward:
    case InputAction::VelocityBackward:
      intent_.forward = 0.0;
      break;
    case InputAction::VelocityLeft:
    case InputAction::VelocityRight:
      intent_.lateral = 0.0;
      break;
    case InputAction::VelocityYawLeft:
    case InputAction::VelocityYawRight:
      intent_.yaw = 0.0;
      break;
    default:
      break;
  }
  intent_.active =
      intent_.forward != 0.0 || intent_.lateral != 0.0 || intent_.yaw != 0.0;
  intent_.stamp_us = now_us;
  Push(InputEvent::VelocityIntent);
  return InputEvent::VelocityIntent;
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
    case InputEvent::VelocityIntent:
      return 6;
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
