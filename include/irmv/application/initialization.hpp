#pragma once

#include <array>
#include <cstddef>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

// What one Step or Hold concluded. Running: the emitted frame is the target for
// this instant of the move. Ready: the move is finished (the emitted frame is
// the final hold command). Failed: the run cannot continue and no frame was
// emitted, so the caller can never send a target after a failure.
enum class InitializationStatus { Running, Ready, Failed };

// The initialization trajectory, its completion judgement and the Ready hold.
//
// The generator is a pure function of the states it is handed and the times it
// is given: it reads no clock, allocates nothing, never throws, and never
// fabricates a fresh sample. A state that is missing, stale, unobserved or
// unusable fails the move instead of becoming the source of a target the
// runtime would answer `STALE_SOURCE` for.
//
// The frames it writes carry only what a target generator owns (the joints, the
// layout, the producer, the epoch, `command_seq` and `source_state_seq`);
// `RobotClient::SendCommand` completes the routing identity (protocol version,
// runtime id, session id, send stamp) on its way to the wire, and only the
// caller's transport may send a frame.
//
// The trajectory is a linear ramp from the position measured at Start to
// `initialization.target_q`: for `t` inside `trajectory.duration_us` the target
// is `start_q + (target_q - start_q) * t / duration_us` and the velocity
// command is the ramp rate `(target_q - start_q) / duration_us` recomputed in
// seconds, so the two agree; at and past the duration the target is
// `target_q` with no velocity command. The move is finished only after the ramp
// when every joint has been inside its position and velocity tolerances for the
// whole `trajectory.stable_window_us`; the final frame, and every later Ready
// hold, sits exactly on the target with no feed-forward torque.
class Initialization {
 public:
  // `config` must outlive the generator.
  explicit Initialization(const configuration::ApplicationConfig &config);

  Initialization(const Initialization &) = delete;
  Initialization &operator=(const Initialization &) = delete;

  // Fixes the trajectory origin at the measured position of `fresh` and starts
  // the timing window; no frame is emitted. Refused when `fresh` cannot be a
  // target source, with the all-zero producer, and while a move is already
  // active: a repeated Start is the FSM's to intercept, and refusing here keeps
  // a restart from silently moving the trajectory origin without resetting the
  // timing.
  bool Start(const protocol::RobotState &fresh, protocol::Sequence epoch,
             protocol::Id producer_id, protocol::TimeUs now_us);

  // Emits the frame of this instant of the move, or reports why it cannot:
  // `Running` with the frame the caller sends this period, `Ready` with the
  // final hold frame, `Failed` with `command` untouched.
  InitializationStatus Step(const protocol::RobotState &fresh,
                            protocol::TimeUs now_us,
                            protocol::JointCommand *command);

  // Emits one Ready hold frame: the target exactly, the same gains, producer
  // and epoch as the finished initialization, but a new command sequence and
  // the state sequence of the state it is built from. Failed when the
  // initialization is not ready, when the feedback is not usable (in which case
  // no command is emitted), or before the trajectory ever ran.
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
  // complete and observed joint set with finite position and velocity, a stamp
  // no older than the runtime's own source-state deadline, no stamp from the
  // future, and a host time that never runs backwards between calls.
  bool Usable(const protocol::RobotState &fresh, protocol::TimeUs now_us) const;
  // Every joint at the target within the position and velocity tolerances.
  bool WithinCompletionTolerance(const protocol::RobotState &fresh) const;
  // Completes the frame identity the generator owns; false when the layout
  // cannot be written, in which case it changes nothing.
  bool BeginFrame(protocol::JointCommand *command,
                  const protocol::RobotState &fresh);
  // Writes the target hold values: q = target, dq = 0, tau_ff = 0, the
  // configured gains, all in canonical joint order.
  void FillTarget(protocol::JointCommand *command) const;
  // Writes the ramp values for `elapsed` inside the duration: the position of
  // the linear ramp towards the target and the velocity that matches it.
  void FillRamp(protocol::JointCommand *command,
                protocol::TimeUs elapsed) const;
  void Fail();

  const configuration::ApplicationConfig *config_;
  std::array<double, protocol::kMaxJoints> start_q_{};
  protocol::Id producer_id_{};
  protocol::Sequence epoch_{0};
  protocol::Sequence command_seq_{0};
  protocol::TimeUs start_us_{0};
  protocol::TimeUs stable_since_{0};
  protocol::TimeUs last_us_{0};
  bool have_last_{false};
  bool stable_open_{false};
  bool active_{false};
  bool failed_{false};
  bool ready_{false};
};

}  // namespace irmv::application
