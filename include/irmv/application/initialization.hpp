#pragma once

#include <array>
#include <cstddef>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

// What one Step or Hold concluded.
//
// Running: the emitted frame is the target for this instant of the move.
// Ready: the move is finished and the emitted frame is the final hold command.
// Failed: no frame was emitted and none may be sent. That covers a latched
// failure, and also "no move is in flight at all": a call before Start, after a
// completed move (Ready), and a null output. The caller tells the cases apart
// with ready() and failed().
enum class InitializationStatus { Running, Ready, Failed };

// The initialization trajectory, its completion judgement and the Ready hold.
//
// The generator is a pure function of the states it is handed and the times it
// is given: it reads no clock, allocates nothing, never throws, and never
// fabricates a fresh sample. A state that is missing, stale, unobserved,
// unusable, or stamped in the future fails the move instead of becoming the
// source of a target the runtime would answer `STALE_SOURCE` for.
//
// The trajectory is a raised-cosine ramp from the position measured at Start to
// `initialization.target_q` over `trajectory.duration_us`:
//
//   u     = t / duration_us                                 (0 <= t < duration)
//   q(t)  = start_q + (target_q - start_q) * (1 - cos(pi * u)) / 2
//   dq(t) = (target_q - start_q) * (pi / (2 * D)) * sin(pi * u)
//   D     = trajectory.duration_us / 1e6                    (seconds)
//
// so the velocity command is the analytic derivative of the position profile,
// both ends have zero velocity (the first frame carries the measured position
// and `dq == 0`), the position is monotonic towards the target and inside the
// physical limits, and the peak rate is `|target_q - start_q| * pi / (2 * D)`.
// At and past the duration the frame is exactly `q = target_q`, `dq = 0`,
// `tau_ff = 0` with the configured gains.
//
// Start admits a plan only when the runtime can accept it: every measured
// origin inside `[position_min, position_max]`, every planned peak rate inside
// `runtime.safety.target_velocity_limits`, and the first frame (the measured
// position, zero velocity, zero feed-forward and the configured gains) inside
// the runtime's own continuity bounds against the damped predecessor
// `-damping_kd * dq_fb`. A plan that fails is refused cleanly - no frame, no
// latch - instead of stalling on a `DISCONTINUITY` rejection or latching a
// mid-move fault; the operator then has to raise `trajectory.duration_us` (and
// `deadline_us` with it) for a move that starts far from its target.
//
// The move is finished only after the ramp, when every joint has been inside
// its position and velocity tolerances for the whole
// `trajectory.stable_window_us`; the final frame, and every later Ready hold,
// sits exactly on the target with no feed-forward torque.
//
// The frames carry what a target generator owns: the joints, the layout, the
// protocol version, the producer, the epoch, `command_seq` and
// `source_state_seq`. RobotClient completes the routing identity (runtime id,
// session id, send stamp) on its way to the wire, and only the caller's
// transport may send a frame.
class Initialization {
 public:
  // `config` must outlive the generator.
  explicit Initialization(const configuration::ApplicationConfig &config);

  Initialization(const Initialization &) = delete;
  Initialization &operator=(const Initialization &) = delete;

  // Fixes the trajectory origin at the measured position of `fresh` and starts
  // the timing window; no frame is emitted. Refused when `fresh` cannot be a
  // target source, with the all-zero producer, while a move is already active
  // (a repeated Start is the FSM's to intercept, and refusing here keeps a
  // restart from silently moving the trajectory origin without resetting the
  // timing), and when the plan itself is not one the runtime would accept (see
  // the class comment: origin outside the position limits, peak rate above the
  // target velocity limit, or a first frame breaking a continuity bound). A
  // refusal changes nothing and latches nothing, so the operator can raise
  // `trajectory.duration_us`/`deadline_us` and start again.
  bool Start(const protocol::RobotState &fresh, protocol::Sequence epoch,
             protocol::Id producer_id, protocol::TimeUs now_us);

  // Emits the frame of this instant of the move, or reports why it cannot:
  // `Running` with the frame the caller sends this period, `Ready` with the
  // final hold frame, `Failed` with `command` untouched. A mid-move failure
  // (unusable state, a null output, the deadline) latches `failed()`; a call
  // with no move in flight does not.
  InitializationStatus Step(const protocol::RobotState &fresh,
                            protocol::TimeUs now_us,
                            protocol::JointCommand *command);

  // Emits one Ready hold frame: the target exactly, the same gains, producer
  // and epoch as the finished initialization, but a new command sequence and
  // the state sequence of the state it is built from. Failed without emitting
  // when the initialization is not ready, when the feedback is not usable, and
  // before the trajectory ever ran; an unusable state does not clear Ready and
  // does not latch `failed()`, so a fresh state holds again.
  InitializationStatus Hold(const protocol::RobotState &fresh,
                            protocol::TimeUs now_us,
                            protocol::JointCommand *command);

  // A move is in flight.
  bool active() const { return active_; }
  // The move hit an unusable sample or its deadline; latched until Start.
  bool failed() const { return failed_; }
  // The move finished and its target holds.
  bool ready() const { return ready_; }
  // The epoch and producer the frames carry; Ready keeps them.
  protocol::Sequence epoch() const { return epoch_; }
  const protocol::Id &producer_id() const { return producer_id_; }

 private:
  // The feedback usability rule of the binding contract: the frozen version, a
  // complete and observed joint set with finite position and velocity, a state
  // sequence the runtime can look up, a stamp no older than the runtime's own
  // source-state deadline, no stamp from the future, and a host time that never
  // runs backwards between calls.
  bool Usable(const protocol::RobotState &fresh, protocol::TimeUs now_us) const;

  // Whether the plan Start is about to fix is one the runtime would accept:
  // origin inside the position limits, raised-cosine peak rate inside the
  // target velocity limit, and the first frame (measured position, zero
  // velocity, zero feed-forward, configured gains) inside the continuity bounds
  // against the damped predecessor of this feedback.
  bool PlanRunnable(
      const protocol::RobotState &fresh,
      const std::array<double, protocol::kMaxJoints> &origin) const;

  // Every joint at the target within the position and velocity tolerances.
  bool WithinCompletionTolerance(const protocol::RobotState &fresh) const;

  // Completes the frame identity the generator owns; false when the layout
  // cannot be written, in which case it changes nothing.
  bool BeginFrame(protocol::JointCommand *command,
                  const protocol::RobotState &fresh);

  // Writes the target hold values: q = target, dq = 0, tau_ff = 0, the
  // configured gains, all in canonical joint order.
  void FillTarget(protocol::JointCommand *command) const;
  // Writes the raised-cosine ramp values for `elapsed` inside the duration: the
  // position of the profile and the velocity that is its derivative.
  void FillRamp(protocol::JointCommand *command,
                protocol::TimeUs elapsed) const;

  void Fail();

  const configuration::ApplicationConfig &config_;
  std::array<double, protocol::kMaxJoints> start_q_{};
  protocol::Id producer_id_{};
  protocol::Sequence epoch_{0};
  // The sequence of the last emitted frame; only ever increases while a move
  // runs, so a frame can never repeat or go back.
  protocol::Sequence command_seq_{0};
  // The host instant the trajectory origin was fixed at.
  protocol::TimeUs start_us_{0};
  // The stability window: opened by the first in-tolerance sample at or after
  // the ramp end, tested in the same call, so the window may complete on the
  // very call that opens it when it is zero.
  bool stable_open_{false};
  protocol::TimeUs stable_since_{0};
  // The host instant of the last accepted call. It is the only protection
  // against a time that runs backwards through the unsigned arithmetic, and it
  // deliberately survives Fail() and a new Start so that neither can be moved
  // back onto an older instant.
  protocol::TimeUs last_us_{0};
  bool have_last_{false};
  bool active_{false};
  bool failed_{false};
  bool ready_{false};
};

}  // namespace irmv::application
