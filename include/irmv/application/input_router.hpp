#pragma once

#include <array>
#include <cstddef>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

// One semantic event of the supervision loop. The input router raises them
// (a bound byte of the keyboard table, a lost input device); this phase has no
// locomotion target generator, so `VelocityIntent` is delivered but the
// behaviour FSM has nothing to do for it beyond keeping and expiring the
// intent.
//
// The enumerator order is frozen: the behaviour FSM switches on these values,
// so inserting or reordering one changes the meaning of a value already
// written down. The order is *not* the router's batch priority: that one is
// explicit in TakeEvent() through Priority() (emergency stop first, velocity
// intent last).
enum class InputEvent {
  None = 0,
  Damping,
  Initialize,
  ZeroTorque,
  EmergencyStop,
  Reset,
  Exit,
  VelocityIntent,
};

// The body-velocity intent the keys currently hold. Each axis is zero or the
// unit value of the bound key (+1 or -1); the intent is never a ramp and never
// accumulates, so a held key cannot drive an unbounded command.
//
// Releasing either direction key of one axis clears that axis (the two keys of
// an axis are one component, so releasing the one that is not claimed is a
// no-op anyway) and refreshes `stamp_us`; the other axes and the deadline the
// held keys keep refreshing are untouched.
//
// Units: `forward` and `lateral` are m/s in the body frame (positive forward,
// positive left), `yaw` is rad/s (positive counter-clockwise). `stamp_us` is
// the host monotonic time of the last mapped velocity byte, and the intent is
// expired `input.intent_timeout_us` after it. Phase 2 only tracks, refreshes
// and expires the intent: no locomotion target generator consumes it yet, so
// nothing here is a commanded velocity.
struct VelocityIntent {
  double forward{0.0};
  double lateral{0.0};
  double yaw{0.0};
  bool active{false};
  protocol::TimeUs stamp_us{0};
};

// The input half of the application: it turns bytes of a bound key table into
// the semantic events above and keeps the one body-velocity intent the keys are
// holding.
//
// It is a pure function of the bytes and times it is given. It performs no
// terminal I/O (the non-blocking read and the raw-mode lifetime belong to the
// terminal adapter), never touches the RobotClient or a joint command, reads no
// clock, allocates nothing, never throws and has no thread. The keyboard and a
// future gamepad therefore share this entry point.
//
// The caller's clock (`now_us`) must be non-decreasing. A backwards jump is
// harmless and nothing here defends against it: the repeat guard fails open
// (the press is raised again) and the intent expires on the spot, because its
// age underflows past the deadline. Neither is a claim about the machine, and
// the next key byte restores both.
//
// A terminal byte stream has no reliable key-up event, so the router does not
// claim one: the repeat guard is behavioural idempotence for the one-shot mode
// keys (one raised event per press of a key that the terminal reports as
// repeated), while `pressed`/`released` are honoured for the devices that can
// really report them. A release of a mode key is never a new request, and in
// particular releasing the reserved emergency-stop key is not a Reset.
class InputRouter {
 public:
  explicit InputRouter(const configuration::ApplicationConfig &config);

  // Routes one key byte. `pressed` is the transition the device reported: a
  // bound press raises (and returns) its event, an unbound byte and the release
  // of a one-shot key return `None` and change nothing. A one-shot press inside
  // `input.repeat_guard_us` of the last press of the same byte is suppressed
  // behaviourally, so a held key raises exactly one event; a velocity byte has
  // no guard and refreshes the intent, which is how a held velocity key keeps
  // it alive.
  InputEvent OnKey(char key, bool pressed, protocol::TimeUs now_us);

  // Removes and returns the highest-priority event raised since the last call,
  // `None` when none is left. The order is fixed: emergency stop, exit, reset,
  // explicit zero-torque stop, ordinary damping stop, initialize, velocity
  // intent. A space read in the same batch therefore preempts an ordinary key,
  // and the events it did not preempt stay queued for the following calls.
  InputEvent TakeEvent();

  // Ages one thing only: an active velocity intent is zeroed and deactivated
  // strictly more than `input.intent_timeout_us` after its last byte, and a
  // pending velocity event of that expired intent is dropped rather than
  // delivered. Nothing else ages here: a one-shot event neither expires nor is
  // revived, and the loss latch is never inferred.
  void Tick(protocol::TimeUs now_us);

  // The live body-velocity intent; zero and inactive while no velocity key has
  // been pressed or after its deadline.
  const VelocityIntent &velocity_intent() const { return intent_; }

  // The input device reported EOF or a terminal error. That asks for the
  // orderly exit - whose first step is the ordinary damping stop - so it raises
  // `Exit`; it is never an emergency stop and never claims the hardware
  // stopped. Nothing else changes.
  void OnInputLost();

  // True once OnInputLost() was called. An empty read, an unbound byte and a
  // suppressed repeat are not an input loss and never set this.
  bool input_lost() const { return input_lost_; }

 private:
  // At most the seven semantic events above can be queued at once, so eight
  // slots are enough; duplicates are coalesced because a repeated event carries
  // no more meaning than the one already queued.
  static constexpr std::size_t kBatchCapacity = 8;
  static constexpr std::size_t kKeyCount = 256;

  // One-shot routing shared by every mode key: the release and the guarded
  // repeat are suppressed, the guard follows the last press of the byte.
  InputEvent OneShot(std::size_t byte, bool pressed, protocol::TimeUs now_us,
                     InputEvent event);

  // Velocity routing: a press claims the axis, a release gives it back, both
  // raise `VelocityIntent`.
  InputEvent PressVelocity(configuration::InputAction action,
                           protocol::TimeUs now_us);
  InputEvent ReleaseVelocity(configuration::InputAction action,
                             protocol::TimeUs now_us);

  // Queues `event` unless an identical one is already queued.
  void Push(InputEvent event);
  // Ranks the batch; a smaller value is taken first.
  static std::size_t Priority(InputEvent event);
  // Zeroes the intent and drops a queued event of the expired intent.
  void ExpireIntent();

  // The loop owns the configuration for the process life, like RobotClient's.
  const configuration::ApplicationConfig &config_;
  // Last press of each byte and whether it was ever seen, so the very first
  // press of a byte raises even at a time below the guard.
  std::array<protocol::TimeUs, kKeyCount> last_seen_us_{};
  std::array<bool, kKeyCount> seen_{};
  std::array<InputEvent, kBatchCapacity> batch_{};
  std::size_t batch_size_{0};
  VelocityIntent intent_{};
  bool input_lost_{false};
};

}  // namespace irmv::application
