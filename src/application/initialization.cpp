#include "irmv/application/initialization.hpp"

#include <cmath>
#include <cstdint>

#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::application {

namespace {

constexpr double kPi = 3.14159265358979323846;

// The peak rate of the raised-cosine profile: the supremum of
// |delta * ((pi / (2 * D)) * sin(pi * u))| over the ramp, which is
// |delta| * (pi / (2 * D)) because |sin| <= 1.
//
// This is the bound of the profile, not a bit-exact replay of a sample:
// `FillRamp` emits `delta * ((pi / (2 * D)) * sin(phase))`, whose association
// differs from this expression, so the two can differ by a rounding step. The
// bound still holds (the supremum is what admission compares against the
// runtime's target velocity limit, and the runtime checks every frame itself).
double PeakRate(double delta, double duration_seconds) {
  return std::fabs(delta) * (kPi / (2.0 * duration_seconds));
}

}  // namespace

Initialization::Initialization(const configuration::ApplicationConfig &config)
    : config_(config) {}

bool Initialization::Usable(const protocol::RobotState &fresh,
                            protocol::TimeUs now_us) const {
  if (fresh.protocol_version != protocol::kProtocolVersion) {
    return false;
  }
  const std::size_t joint_count = config_.joint_count();
  if (joint_count == 0 || joint_count > protocol::kMaxJoints) {
    return false;
  }
  if (fresh.feedback.count != joint_count) {
    return false;
  }
  // `state_seq` is the only state identity the runtime looks up when it judges
  // the freshness of a target, and a zero sequence can never have been
  // recorded, so such a state cannot carry a target.
  if (fresh.state_seq == 0) {
    return false;
  }
  // The stamp is the runtime's publication instant and `now_us` is the host's
  // monotonic clock; both are same-host monotonic times, so their difference is
  // the age of the snapshot.
  if (fresh.stamp_us == 0 || now_us < fresh.stamp_us) {
    return false;
  }
  if (now_us - fresh.stamp_us > config_.runtime.timeouts.source_state_us) {
    return false;
  }
  if (have_last_ && now_us < last_us_) {
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const protocol::JointFeedback &joint = fresh.feedback.joints[index];
    if (!joint.valid || !joint.connected) {
      return false;
    }
    if (!std::isfinite(joint.q) || !std::isfinite(joint.dq)) {
      return false;
    }
  }
  return true;
}

bool Initialization::PlanRunnable(
    const protocol::RobotState &fresh,
    const std::array<double, protocol::kMaxJoints> &origin) const {
  const configuration::RuntimeConfig &runtime = config_.runtime;
  const configuration::TrajectoryConfig &trajectory =
      config_.initialization.trajectory;
  const double duration_seconds =
      static_cast<double>(trajectory.duration_us) / 1'000'000.0;
  for (std::size_t index = 0; index < config_.joint_count(); ++index) {
    const double measured = origin[index];
    const double target = config_.initialization.target_q[index];
    // The measured origin is the first target of the move and the ramp ends on
    // the configured target, so both must be positions the runtime accepts.
    if (!std::isfinite(measured) || measured < runtime.position_min[index] ||
        measured > runtime.position_max[index]) {
      return false;
    }
    if (!std::isfinite(target) || target < runtime.position_min[index] ||
        target > runtime.position_max[index]) {
      return false;
    }
    // Every frame of the ramp must stay inside the runtime's target velocity
    // limit; the raised-cosine profile is bounded by its peak rate.
    const double delta = target - measured;
    const double peak_rate = PeakRate(delta, duration_seconds);
    if (!std::isfinite(peak_rate) ||
        peak_rate > runtime.target_velocity_limits[index]) {
      return false;
    }
    // The first frame the runtime judges is the measured position, a zero
    // velocity command and no feed-forward torque, against the damped
    // predecessor of that same feedback.
    const protocol::JointFeedback &feedback = fresh.feedback.joints[index];
    constexpr double kCommandVelocity = 0.0;
    constexpr double kFeedForward = 0.0;
    if (!(std::fabs(measured - feedback.q) <=
          runtime.continuity.position_delta[index])) {
      return false;
    }
    if (!(std::fabs(kCommandVelocity - feedback.dq) <=
          runtime.continuity.velocity_delta[index])) {
      return false;
    }
    const double tau_new =
        config_.initialization.gains.kp[index] * (measured - feedback.q) +
        config_.initialization.gains.kd[index] *
            (kCommandVelocity - feedback.dq) +
        kFeedForward;
    const double tau_old = -runtime.damping_kd[index] * feedback.dq;
    if (!(std::fabs(tau_new - tau_old) <=
          runtime.continuity.torque_delta[index])) {
      return false;
    }
  }
  return true;
}

bool Initialization::WithinCompletionTolerance(
    const protocol::RobotState &fresh) const {
  const configuration::TrajectoryConfig &trajectory =
      config_.initialization.trajectory;
  for (std::size_t index = 0; index < config_.joint_count(); ++index) {
    if (std::fabs(fresh.feedback.joints[index].q -
                  config_.initialization.target_q[index]) >
        trajectory.position_tolerance) {
      return false;
    }
    if (std::fabs(fresh.feedback.joints[index].dq) >
        trajectory.velocity_tolerance) {
      return false;
    }
  }
  return true;
}

bool Initialization::BeginFrame(protocol::JointCommand *command,
                                const protocol::RobotState &fresh) {
  if (!command->layout_id.Assign(config_.layout_id())) {
    return false;
  }
  // The frame is stamped with the frozen version here rather than left to
  // RobotClient, because the first frame of a move travels inside an
  // ExecutionRequest as well, where the generator owns the whole payload; the
  // client sets the same value again when it completes the routing identity.
  command->protocol_version = protocol::kProtocolVersion;
  command->joints = protocol::JointVector{};
  command->joints.count = static_cast<std::uint32_t>(config_.joint_count());
  command->producer_id = producer_id_;
  command->epoch = epoch_;
  command->command_seq = ++command_seq_;
  command->source_state_seq = fresh.state_seq;
  return true;
}

void Initialization::FillTarget(protocol::JointCommand *command) const {
  for (std::size_t index = 0; index < config_.joint_count(); ++index) {
    command->joints.q[index] = config_.initialization.target_q[index];
    command->joints.dq[index] = 0.0;
    command->joints.tau_ff[index] = 0.0;
    command->joints.kp[index] = config_.initialization.gains.kp[index];
    command->joints.kd[index] = config_.initialization.gains.kd[index];
  }
}

void Initialization::FillRamp(protocol::JointCommand *command,
                              protocol::TimeUs elapsed) const {
  const configuration::TrajectoryConfig &trajectory =
      config_.initialization.trajectory;
  const double fraction = static_cast<double>(elapsed) /
                          static_cast<double>(trajectory.duration_us);
  const double duration_seconds =
      static_cast<double>(trajectory.duration_us) / 1'000'000.0;
  const double phase = kPi * fraction;
  const double eased = (1.0 - std::cos(phase)) / 2.0;
  const double rate_scale = (kPi / (2.0 * duration_seconds)) * std::sin(phase);
  for (std::size_t index = 0; index < config_.joint_count(); ++index) {
    const double delta =
        config_.initialization.target_q[index] - start_q_[index];
    command->joints.q[index] = start_q_[index] + delta * eased;
    command->joints.dq[index] = delta * rate_scale;
    command->joints.tau_ff[index] = 0.0;
    command->joints.kp[index] = config_.initialization.gains.kp[index];
    command->joints.kd[index] = config_.initialization.gains.kd[index];
  }
}

void Initialization::Fail() {
  active_ = false;
  ready_ = false;
  failed_ = true;
  stable_open_ = false;
}

bool Initialization::Start(const protocol::RobotState &fresh,
                           protocol::Sequence epoch, protocol::Id producer_id,
                           protocol::TimeUs now_us,
                           protocol::JointCommand *first_command) {
  if (active_) {
    return false;
  }
  if (first_command == nullptr) {
    // A caller bug, refused like every other Start refusal: nothing changed,
    // nothing filled.
    return false;
  }
  if (producer_id == protocol::kNoId) {
    return false;
  }
  if (!Usable(fresh, now_us)) {
    return false;
  }
  std::array<double, protocol::kMaxJoints> origin{};
  for (std::size_t index = 0; index < config_.joint_count(); ++index) {
    origin[index] = fresh.feedback.joints[index].q;
  }
  // Everything the runtime would refuse is refused here, before a single member
  // is touched, so a refused plan leaves the generator exactly as it was.
  if (!PlanRunnable(fresh, origin)) {
    return false;
  }
  start_q_ = origin;
  producer_id_ = producer_id;
  epoch_ = epoch;
  // Safe only because an accepted Start follows a fresh authorization: the
  // request that carries the first frame proposes a new epoch, and the runtime
  // clears the gate's sequence floor with it. A Start inside an epoch the gate
  // already authorized would emit frames the gate drops as stale.
  command_seq_ = 0;
  start_us_ = now_us;
  stable_open_ = false;
  stable_since_ = 0;
  have_last_ = true;
  last_us_ = now_us;
  active_ = true;
  failed_ = false;
  ready_ = false;
  // Emit the frame the guard just validated, so the first frame a caller can
  // send is exactly the one admission approved. BeginFrame can only fail when
  // the frozen layout text does not fit, which no loaded configuration can do;
  // if it ever did, it leaves the command untouched and the move failed rather
  // than half-started.
  if (!BeginFrame(first_command, fresh)) {
    Fail();
    return false;
  }
  FillRamp(first_command, 0);
  return true;
}

InitializationStatus Initialization::Step(const protocol::RobotState &fresh,
                                          protocol::TimeUs now_us,
                                          protocol::JointCommand *command) {
  if (!active_) {
    return InitializationStatus::Failed;
  }
  if (command == nullptr) {
    Fail();
    return InitializationStatus::Failed;
  }
  if (!Usable(fresh, now_us)) {
    Fail();
    return InitializationStatus::Failed;
  }
  last_us_ = now_us;
  const configuration::TrajectoryConfig &trajectory =
      config_.initialization.trajectory;
  const protocol::TimeUs elapsed = now_us - start_us_;
  // One decision for the whole call, so the completion judgement and the value
  // selection can never disagree about where the ramp is.
  const bool ramp_done = elapsed >= trajectory.duration_us;

  // The deadline is the final bound of the move and is judged before the
  // completion block: a run that arrives at this instant without having already
  // completed its stable judgement fails here, whatever the tolerance and
  // window state say, so a move that only becomes stable after its deadline is
  // never Ready. The instant itself is still inside the bound - a window that
  // completed exactly on `deadline_us` is ready below.
  if (elapsed > trajectory.deadline_us) {
    // The command stays untouched: a call the caller may not send must never
    // hand it a frame.
    Fail();
    return InitializationStatus::Failed;
  }

  // Completion: a move that reached its target and held it for the whole stable
  // window is ready. Before the ramp finished the window is never opened.
  if (ramp_done && WithinCompletionTolerance(fresh)) {
    if (!stable_open_) {
      stable_open_ = true;
      stable_since_ = now_us;
    }
    if (now_us - stable_since_ >= trajectory.stable_window_us) {
      if (!BeginFrame(command, fresh)) {
        Fail();
        return InitializationStatus::Failed;
      }
      FillTarget(command);
      active_ = false;
      ready_ = true;
      stable_open_ = false;
      stable_since_ = 0;
      return InitializationStatus::Ready;
    }
  } else if (stable_open_) {
    // One out-of-tolerance sample after the ramp finished throws the
    // accumulated stability away; the window restarts at the next in-tolerance
    // sample.
    stable_open_ = false;
    stable_since_ = 0;
  }

  if (!BeginFrame(command, fresh)) {
    Fail();
    return InitializationStatus::Failed;
  }
  if (ramp_done) {
    FillTarget(command);
  } else {
    FillRamp(command, elapsed);
  }
  return InitializationStatus::Running;
}

InitializationStatus Initialization::Hold(const protocol::RobotState &fresh,
                                          protocol::TimeUs now_us,
                                          protocol::JointCommand *command) {
  if (!ready_) {
    return InitializationStatus::Failed;
  }
  if (command == nullptr) {
    return InitializationStatus::Failed;
  }
  if (!Usable(fresh, now_us)) {
    // The move stays ready: an unusable sample is the caller's to wait out, and
    // a fresh state holds again.
    return InitializationStatus::Failed;
  }
  last_us_ = now_us;
  if (!BeginFrame(command, fresh)) {
    Fail();
    return InitializationStatus::Failed;
  }
  FillTarget(command);
  return InitializationStatus::Ready;
}

}  // namespace irmv::application
