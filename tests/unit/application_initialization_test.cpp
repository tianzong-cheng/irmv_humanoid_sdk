// The initialization trajectory, completion judgement and the Ready hold.
//
// The runtime accepts neither a first frame that jumps past its continuity
// bounds nor a target computed from a state older than its source-state
// deadline, so the generator is judged here against the runtime's own formula
// and against states that are missing, stale or unusable.

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "irmv/application/initialization.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::application::Initialization;
using irmv::application::InitializationStatus;
using irmv::configuration::ApplicationConfig;
using irmv::configuration::LoadApplicationConfig;
using irmv::protocol::JointCommand;
using irmv::protocol::kMaxJoints;
using irmv::protocol::kNoId;
using irmv::protocol::kProtocolVersion;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::TimeUs;

using JointPair = std::array<double, 2>;

// The fixture values the literals below rely on.
constexpr Sequence kEpoch = 7;
constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

irmv::protocol::Id Producer() {
  irmv::protocol::Id id{};
  id[0] = 0x33;
  id[15] = 0xcc;
  return id;
}

ApplicationConfig FakeConfig() {
  return LoadApplicationConfig("config/sdk.json");
}

// One fresh state whose feedback reports the two joints at `position`/
// `velocity`, published with `state_seq` at `stamp_us`. The driver snapshot
// sequence is deliberately a different counter: a target must carry the state
// sequence the runtime records, never `feedback.snapshot_seq`.
RobotState FreshState(const ApplicationConfig &config,
                      const JointPair &position, const JointPair &velocity,
                      TimeUs stamp_us, Sequence state_seq) {
  RobotState state{};
  state.protocol_version = kProtocolVersion;
  state.state_seq = state_seq;
  state.stamp_us = stamp_us;
  state.feedback.snapshot_seq = state_seq + 5'000;
  state.feedback.stamp_us = stamp_us;
  state.feedback.count = static_cast<std::uint32_t>(config.joint_count());
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    state.feedback.joints[index].valid = true;
    state.feedback.joints[index].connected = true;
    state.feedback.joints[index].q = position[index];
    state.feedback.joints[index].dq = velocity[index];
    state.feedback.joints[index].rx_stamp_us = stamp_us;
  }
  return state;
}

// A command that a failed call must leave exactly as it found it. The last
// joint slot is beyond the configured count and deliberately dirty, so a
// re-emitted frame has to be rebuilt, not patched in place.
JointCommand Sentinel() {
  JointCommand command{};
  command.command_seq = 424242;
  command.joints.count = 99;
  command.joints.q[kMaxJoints - 1] = 123.5;
  command.joints.kd[kMaxJoints - 1] = 456.5;
  return command;
}

int CheckUntouched(const JointCommand &command) {
  CHECK(command.command_seq == 424242);
  CHECK(command.joints.count == 99);
  CHECK(command.joints.q[kMaxJoints - 1] == 123.5);
  CHECK(command.joints.kd[kMaxJoints - 1] == 456.5);
  return 0;
}

// The ramp length of the fake fixture within the runtime's velocity continuity
// bound: |target - start| / duration stays far below velocity_delta.
constexpr TimeUs kSlowDurationUs = 4'000'000;

// Sets the move length together with a deadline that keeps the loader's rule
// (deadline > duration) satisfied, so every mutated fixture is one the real
// configuration would accept.
void SetDuration(ApplicationConfig &config, TimeUs duration) {
  config.initialization.trajectory.duration_us = duration;
  config.initialization.trajectory.deadline_us = duration * 4;
}

// The first frame of a move is judged by the runtime against the damped
// predecessor of the same feedback, so the generator must start from the
// measured position instead of jumping to the target.
int FirstFrameContinuity() {
  ApplicationConfig config = FakeConfig();
  SetDuration(config, kSlowDurationUs);
  const JointPair measured{-0.5, 0.5};
  const JointPair still{0.0, 0.0};
  const TimeUs now = 1'000'000;
  const RobotState fresh = FreshState(config, measured, still, now - 1'000, 11);

  Initialization initialization(config);
  CHECK(initialization.Start(fresh, kEpoch, Producer(), now));
  CHECK(initialization.active());
  CHECK(!initialization.ready());
  CHECK(!initialization.failed());

  JointCommand frame{};
  CHECK(initialization.Step(fresh, now, &frame) ==
        InitializationStatus::Running);
  CHECK(frame.joints.count == config.joint_count());
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    const double position = frame.joints.q[index];
    const double measured_position = fresh.feedback.joints[index].q;
    const double measured_velocity = fresh.feedback.joints[index].dq;
    // Position continuity, exactly the runtime's own bound.
    CHECK(std::fabs(position - measured_position) <=
          config.runtime.continuity.position_delta[index]);
    CHECK(std::fabs(frame.joints.dq[index] - measured_velocity) <=
          config.runtime.continuity.velocity_delta[index]);
    // The first target is the measured position: a jump of zero, and the
    // smooth profile starts with zero velocity, so the first frame is inside
    // the velocity bound for every start position.
    CHECK(position == measured_position);
    CHECK(frame.joints.dq[index] == 0.0);
    // Torque continuity against both possible predecessors, computed with the
    // runtime's formula tau = kp*(q - q_fb) + kd*(dq - dq_fb) + tau_ff.
    const double tau_new =
        frame.joints.kp[index] * (position - measured_position) +
        frame.joints.kd[index] * (frame.joints.dq[index] - measured_velocity) +
        frame.joints.tau_ff[index];
    const double tau_damping =
        -config.runtime.damping_kd[index] * measured_velocity;
    constexpr double tau_zero_torque = 0.0;
    CHECK(std::fabs(tau_new - tau_damping) <=
          config.runtime.continuity.torque_delta[index]);
    CHECK(std::fabs(tau_new - tau_zero_torque) <=
          config.runtime.continuity.torque_delta[index]);
  }
  // A repeated Start is the FSM's to intercept; here it must not silently move
  // the trajectory origin or restart the timing window.
  const TimeUs later = now + 500'000;
  const RobotState moved = FreshState(config, JointPair{0.0, 0.0},
                                      JointPair{0.0, 0.0}, later - 1'000, 12);
  CHECK(!initialization.Start(moved, kEpoch + 1, Producer(), later));
  CHECK(initialization.active());
  CHECK(initialization.epoch() == kEpoch);
  return 0;
}

// The interpolation is the documented raised-cosine ramp: monotonic towards
// the target, held inside the physical limits, with the analytic derivative as
// its velocity command. The feedback below lags the command by one control
// period, like a real tracker, and the position and velocity continuity bounds
// are asserted frame by frame.
int MonotonicInterpolation() {
  ApplicationConfig config = FakeConfig();
  // Slow enough that even the profile's peak rate (2.25*pi/(2*8) = 0.44 rad/s)
  // stays inside the runtime's velocity continuity bound 0.5.
  const TimeUs duration = 8'000'000;
  SetDuration(config, duration);
  config.initialization.target_q[0] = 0.25;
  config.initialization.target_q[1] = -0.75;
  const JointPair start_q{-1.0, 1.5};
  const double duration_seconds = static_cast<double>(duration) / 1e6;
  const TimeUs period = config.supervision.target_period_us;
  const TimeUs start_us = 2'000'000;

  Initialization initialization(config);
  const RobotState origin =
      FreshState(config, start_q, JointPair{0.0, 0.0}, start_us - 1'000, 20);
  CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));

  const auto profile_q = [&](std::size_t index, TimeUs elapsed) {
    const double delta = config.initialization.target_q[index] - start_q[index];
    const double fraction =
        static_cast<double>(elapsed) / static_cast<double>(duration);
    return start_q[index] + delta * (1.0 - std::cos(kPi * fraction)) / 2.0;
  };
  const auto profile_dq = [&](std::size_t index, TimeUs elapsed) {
    const double delta = config.initialization.target_q[index] - start_q[index];
    const double fraction =
        static_cast<double>(elapsed) / static_cast<double>(duration);
    return delta * (kPi / (2.0 * duration_seconds)) * std::sin(kPi * fraction);
  };

  JointPair previous = start_q;
  for (std::size_t step = 0; step <= 8; ++step) {
    const TimeUs elapsed = static_cast<TimeUs>(step) * duration / 8;
    const TimeUs now = start_us + elapsed;
    // The measured state is the command of one control period ago, and
    // therefore the start position at the very first call.
    const TimeUs lagged = elapsed > period ? elapsed - period : 0;
    JointPair measured{};
    JointPair measured_velocity{};
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      measured[index] = profile_q(index, lagged);
      measured_velocity[index] = profile_dq(index, lagged);
    }
    const RobotState fresh =
        FreshState(config, measured, measured_velocity, now - 1'000, 20 + step);
    JointCommand frame{};
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      // Inside the ramp the frame is exactly the profile and its derivative...
      const double expected_q = elapsed >= duration
                                    ? config.initialization.target_q[index]
                                    : profile_q(index, elapsed);
      const double expected_dq =
          elapsed >= duration ? 0.0 : profile_dq(index, elapsed);
      CHECK(std::fabs(frame.joints.q[index] - expected_q) <= kTolerance);
      CHECK(std::fabs(frame.joints.dq[index] - expected_dq) <= kTolerance);
      CHECK(frame.joints.tau_ff[index] == 0.0);
      // ... inside the segment towards the target and the physical limits,
      // and inside the runtime's target velocity limit.
      const double low =
          std::fmin(start_q[index], config.initialization.target_q[index]);
      const double high =
          std::fmax(start_q[index], config.initialization.target_q[index]);
      CHECK(frame.joints.q[index] >= low - kTolerance);
      CHECK(frame.joints.q[index] <= high + kTolerance);
      CHECK(frame.joints.q[index] >= config.runtime.position_min[index]);
      CHECK(frame.joints.q[index] <= config.runtime.position_max[index]);
      CHECK(std::fabs(frame.joints.dq[index]) <=
            config.runtime.target_velocity_limits[index]);
      // Monotonic: never further from the target than the previous frame.
      CHECK(std::fabs(frame.joints.q[index] -
                      config.initialization.target_q[index]) <=
            std::fabs(previous[index] - config.initialization.target_q[index]) +
                kTolerance);
      previous[index] = frame.joints.q[index];
      // The frame is continuous with the state it was built from, which is what
      // the runtime measures a first frame against.
      CHECK(std::fabs(frame.joints.q[index] - fresh.feedback.joints[index].q) <=
            config.runtime.continuity.position_delta[index]);
      CHECK(
          std::fabs(frame.joints.dq[index] - fresh.feedback.joints[index].dq) <=
          config.runtime.continuity.velocity_delta[index]);
    }
  }
  // The ramp is finished: the target exactly, with no velocity command.
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    CHECK(previous[index] == config.initialization.target_q[index]);
  }
  return 0;
}

// Every frame is a complete target in the canonical joint order, carrying the
// generator's identity and a fresh sequence.
int FrameCompleteness() {
  ApplicationConfig config = FakeConfig();
  SetDuration(config, 1'000'000);
  // Distinct per joint, so an index mix-up cannot pass.
  config.initialization.gains.kp[0] = 3.0;
  config.initialization.gains.kp[1] = 7.5;
  config.initialization.gains.kd[0] = 0.25;
  config.initialization.gains.kd[1] = 0.75;
  const JointPair start_q{-0.25, 0.25};
  const TimeUs start_us = 5'000'000;

  Initialization initialization(config);
  const RobotState origin =
      FreshState(config, start_q, JointPair{0.0, 0.0}, start_us - 1'000, 100);
  CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));

  Sequence previous_seq = 0;
  for (std::size_t step = 0; step < 4; ++step) {
    const TimeUs now = start_us + step * 20'000;
    const RobotState fresh =
        FreshState(config, start_q, JointPair{0.0, 0.0}, now - 500, 100 + step);
    JointCommand frame = Sentinel();
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Running);
    CHECK(frame.joints.count == config.joint_count());
    // The frame is rebuilt, not patched: a dirty slot beyond the configured
    // count cannot survive into an emitted frame.
    CHECK(frame.joints.q[kMaxJoints - 1] == 0.0);
    CHECK(frame.joints.kd[kMaxJoints - 1] == 0.0);
    CHECK(frame.layout_id.View() == config.layout_id());
    CHECK(frame.producer_id == Producer());
    CHECK(frame.epoch == kEpoch);
    CHECK(frame.command_seq > previous_seq);
    CHECK(frame.source_state_seq == fresh.state_seq);
    // The routing identity is RobotClient::SendCommand's to complete.
    CHECK(frame.runtime_id == kNoId);
    CHECK(frame.session_id == kNoId);
    CHECK(frame.stamp_us == 0);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(frame.joints.kp[index] == config.initialization.gains.kp[index]);
      CHECK(frame.joints.kd[index] == config.initialization.gains.kd[index]);
      CHECK(frame.joints.tau_ff[index] == 0.0);
      CHECK(std::isfinite(frame.joints.q[index]));
      CHECK(std::isfinite(frame.joints.dq[index]));
      CHECK(frame.joints.q[index] >= config.runtime.position_min[index]);
      CHECK(frame.joints.q[index] <= config.runtime.position_max[index]);
    }
    previous_seq = frame.command_seq;
  }
  return 0;
}

// A state that is missing, stale, unobserved, non-finite or from the future is
// never a target source: the move fails with no frame emitted, and Start
// refuses to open on it.
int UnusableFeedbackFails() {
  ApplicationConfig config = FakeConfig();
  SetDuration(config, kSlowDurationUs);
  const JointPair measured{-0.5, 0.5};
  const JointPair still{0.0, 0.0};
  const TimeUs start_us = 3'000'000;
  const RobotState good =
      FreshState(config, measured, still, start_us - 1'000, 30);

  // Start refuses every unusable origin.
  {
    RobotState state = good;
    state.feedback.count = 1;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
    CHECK(!initialization.active());
  }
  {
    RobotState state = good;
    state.feedback.joints[1].valid = false;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.feedback.joints[0].connected = false;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.stamp_us = 0;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.stamp_us = start_us - config.runtime.timeouts.source_state_us - 1;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.stamp_us = start_us + 1;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    // A zero state sequence names no published state, so the runtime could
    // never look it up: it is not a target source.
    RobotState state = good;
    state.state_seq = 0;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.protocol_version = kProtocolVersion + 1;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.feedback.joints[0].q = kNaN;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    RobotState state = good;
    state.feedback.joints[1].dq = kInf;
    Initialization initialization(config);
    CHECK(!initialization.Start(state, kEpoch, Producer(), start_us));
  }
  {
    Initialization initialization(config);
    CHECK(!initialization.Start(good, kEpoch, kNoId, start_us));
  }

  // A running move fails on the same rules, latches, and emits nothing.
  struct Case {
    const char *name;
    RobotState state;
    TimeUs now;
  };
  RobotState missing = good;
  missing.feedback.count = 0;
  RobotState unobserved = good;
  unobserved.feedback.joints[0].valid = false;
  RobotState disconnected = good;
  disconnected.feedback.joints[1].connected = false;
  RobotState old = good;
  old.stamp_us = start_us - config.runtime.timeouts.source_state_us - 1;
  RobotState no_stamp = good;
  no_stamp.stamp_us = 0;
  RobotState no_state_seq = good;
  no_state_seq.state_seq = 0;
  RobotState future = good;
  future.stamp_us = start_us + 10'000;
  RobotState nan_position = good;
  nan_position.feedback.joints[1].q = kNaN;
  RobotState infinite_velocity = good;
  infinite_velocity.feedback.joints[0].dq = -kInf;
  RobotState wrong_version = good;
  wrong_version.protocol_version = 0;
  const Case cases[] = {
      {"missing feedback", missing, start_us + 10'000},
      {"unobserved joint", unobserved, start_us + 10'000},
      {"disconnected joint", disconnected, start_us + 10'000},
      {"old feedback", old, start_us + 10'000},
      {"no feedback stamp", no_stamp, start_us + 10'000},
      {"zero state sequence", no_state_seq, start_us + 10'000},
      // Now is before the state stamp: a state from the future is unusable.
      {"future feedback", future, start_us + 5'000},
      {"non-finite position", nan_position, start_us + 10'000},
      {"non-finite velocity", infinite_velocity, start_us + 10'000},
      {"wrong version", wrong_version, start_us + 10'000},
  };
  for (const Case &item : cases) {
    Initialization initialization(config);
    CHECK(initialization.Start(good, kEpoch, Producer(), start_us));
    JointCommand frame = Sentinel();
    CHECK(initialization.Step(item.state, item.now, &frame) ==
          InitializationStatus::Failed);
    // A failed call never emits: the caller cannot send a frame after it.
    CHECK(CheckUntouched(frame) == 0);
    CHECK(initialization.failed());
    CHECK(!initialization.active());
    CHECK(!initialization.ready());
    // A failure latches until Start, which is possible again.
    JointCommand again = Sentinel();
    CHECK(initialization.Step(item.state, item.now + 1'000, &again) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(again) == 0);
    CHECK(initialization.Start(good, kEpoch, Producer(), start_us + 20'000));
    CHECK(initialization.active());
    CHECK(!initialization.failed());
  }

  // Time going backwards compared with the previous call is a failure too,
  // even when it stays inside the timing window (a time below Start alone would
  // already wrap the elapsed time past the deadline).
  {
    Initialization initialization(config);
    CHECK(initialization.Start(good, kEpoch, Producer(), start_us));
    JointCommand frame{};
    const RobotState forward =
        FreshState(config, measured, still, start_us + 29'500, 31);
    CHECK(initialization.Step(forward, start_us + 30'000, &frame) ==
          InitializationStatus::Running);
    const RobotState reversed =
        FreshState(config, measured, still, start_us + 9'500, 32);
    JointCommand untouched = Sentinel();
    CHECK(initialization.Step(reversed, start_us + 10'000, &untouched) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(untouched) == 0);
    CHECK(initialization.failed());
    // Start accepts a later origin again after the latch.
    CHECK(initialization.Start(good, kEpoch, Producer(), start_us + 31'000));
    CHECK(initialization.active());
  }
  // A null output is a mid-move failure like any other: it latches and emits
  // nothing.
  {
    Initialization initialization(config);
    CHECK(initialization.Start(good, kEpoch, Producer(), start_us));
    CHECK(initialization.Step(good, start_us + 1'000, nullptr) ==
          InitializationStatus::Failed);
    CHECK(initialization.failed());
    CHECK(!initialization.active());
  }
  return 0;
}

// The plan Start is about to fix must be one the runtime can accept. The
// shipped 500 ms duration with a start far from the target used to break the
// first-frame velocity bound and the target velocity limit; the raised-cosine
// profile is admitted there, and a plan that is genuinely un-runnable is
// refused before any frame and without latching anything.
int AdmissionRefusals() {
  ApplicationConfig config = FakeConfig();
  const TimeUs start_us = 40'000'000;
  const JointPair still{0.0, 0.0};

  // 1.5 rad from the target in the shipped 500 ms: the linear ramp's 3 rad/s
  // first frame would have been a discontinuity against velocity_delta 0.5,
  // while this profile starts at zero velocity and its peak rate
  // 1.5*pi/(2*0.5) = 4.71 rad/s stays inside the 10 rad/s target limit.
  {
    const JointPair far{1.5, -1.5};
    const TimeUs duration = config.initialization.trajectory.duration_us;
    const double duration_seconds = static_cast<double>(duration) / 1e6;
    const RobotState fresh =
        FreshState(config, far, still, start_us - 1'000, 80);
    Initialization initialization(config);
    CHECK(initialization.Start(fresh, kEpoch, Producer(), start_us));
    JointCommand frame{};
    CHECK(initialization.Step(fresh, start_us, &frame) ==
          InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      const double measured = fresh.feedback.joints[index].q;
      const double measured_velocity = fresh.feedback.joints[index].dq;
      CHECK(frame.joints.q[index] == measured);
      CHECK(frame.joints.dq[index] == 0.0);
      CHECK(std::fabs(frame.joints.q[index] - measured) <=
            config.runtime.continuity.position_delta[index]);
      CHECK(std::fabs(frame.joints.dq[index] - measured_velocity) <=
            config.runtime.continuity.velocity_delta[index]);
      const double tau_new =
          frame.joints.kp[index] * (frame.joints.q[index] - measured) +
          frame.joints.kd[index] *
              (frame.joints.dq[index] - measured_velocity) +
          frame.joints.tau_ff[index];
      const double tau_old =
          -config.runtime.damping_kd[index] * measured_velocity;
      CHECK(std::fabs(tau_new - tau_old) <=
            config.runtime.continuity.torque_delta[index]);
      const double peak =
          std::fabs(config.initialization.target_q[index] - measured) * kPi /
          (2.0 * duration_seconds);
      CHECK(peak <= config.runtime.target_velocity_limits[index]);
    }
    // Halfway through, the profile is at the midpoint and at its peak rate.
    const TimeUs half = start_us + duration / 2;
    JointPair midpoint{};
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      midpoint[index] =
          far[index] +
          (config.initialization.target_q[index] - far[index]) / 2.0;
    }
    const RobotState mid =
        FreshState(config, midpoint, still, half - 1'000, 81);
    JointCommand half_frame{};
    CHECK(initialization.Step(mid, half, &half_frame) ==
          InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      const double peak =
          std::fabs(config.initialization.target_q[index] - far[index]) * kPi /
          (2.0 * duration_seconds);
      CHECK(std::fabs(half_frame.joints.q[index] - midpoint[index]) <=
            kTolerance);
      CHECK(std::fabs(std::fabs(half_frame.joints.dq[index]) - peak) <=
            kTolerance);
    }
  }

  // A duration too short for the move: 1.5 rad in 200 ms would peak at
  // 11.8 rad/s against a 10 rad/s limit. No frame, nothing latched.
  {
    ApplicationConfig short_config = FakeConfig();
    SetDuration(short_config, 200'000);
    const JointPair far{1.5, -1.5};
    const double peak =
        std::fabs(short_config.initialization.target_q[0] - far[0]) * kPi /
        (2.0 * (static_cast<double>(
                    short_config.initialization.trajectory.duration_us) /
                1e6));
    CHECK(peak > short_config.runtime.target_velocity_limits[0]);
    const RobotState fresh =
        FreshState(short_config, far, still, start_us - 1'000, 82);
    Initialization initialization(short_config);
    CHECK(!initialization.Start(fresh, kEpoch, Producer(), start_us));
    CHECK(!initialization.active());
    CHECK(!initialization.failed());
    CHECK(!initialization.ready());
    JointCommand untouched = Sentinel();
    CHECK(initialization.Step(fresh, start_us, &untouched) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(untouched) == 0);
    // The same plan is admitted once the duration can follow it.
    ApplicationConfig long_config = FakeConfig();
    SetDuration(long_config, 2'000'000);
    const RobotState later =
        FreshState(long_config, far, still, start_us + 1'000, 83);
    Initialization admitted(long_config);
    CHECK(admitted.Start(later, kEpoch, Producer(), start_us + 2'000));
    CHECK(admitted.active());
  }

  // An origin outside the physical limits is never a target.
  {
    ApplicationConfig limited = FakeConfig();
    SetDuration(limited, 4'000'000);
    const JointPair outside{2.5, 0.0};
    CHECK(outside[0] > limited.runtime.position_max[0]);
    Initialization initialization(limited);
    const RobotState fresh =
        FreshState(limited, outside, still, start_us - 1'000, 84);
    CHECK(!initialization.Start(fresh, kEpoch, Producer(), start_us));
    CHECK(!initialization.active());
    CHECK(!initialization.failed());
    // The same move from inside the limits is admitted, so the refusal is the
    // limit and not the duration.
    const JointPair inside{1.9, 0.0};
    const RobotState ok =
        FreshState(limited, inside, still, start_us - 1'000, 85);
    CHECK(initialization.Start(ok, kEpoch, Producer(), start_us));
    CHECK(initialization.active());
  }

  // A first frame that would break the velocity continuity bound because the
  // measured joint is already moving faster than the bound can absorb.
  {
    ApplicationConfig moving = FakeConfig();
    SetDuration(moving, 4'000'000);
    const JointPair origin{0.5, -0.5};
    const JointPair fast{1.0, 0.0};
    CHECK(std::fabs(fast[0]) > moving.runtime.continuity.velocity_delta[0]);
    Initialization initialization(moving);
    const RobotState fresh =
        FreshState(moving, origin, fast, start_us - 1'000, 86);
    CHECK(!initialization.Start(fresh, kEpoch, Producer(), start_us));
    CHECK(!initialization.active());
    CHECK(!initialization.failed());
    const RobotState ok =
        FreshState(moving, origin, JointPair{0.4, 0.0}, start_us - 1'000, 87);
    CHECK(initialization.Start(ok, kEpoch, Producer(), start_us));
    CHECK(initialization.active());
  }

  // A first frame that would break the torque continuity bound: the velocity
  // bound is widened so only the torque clause can refuse, because
  // kd*(0 - dq) against -damping_kd*dq gives |damping_kd - kd| * |dq| = 6.
  {
    ApplicationConfig torque = FakeConfig();
    SetDuration(torque, 4'000'000);
    for (std::size_t index = 0; index < torque.joint_count(); ++index) {
      torque.runtime.continuity.velocity_delta[index] = 100.0;
      torque.runtime.continuity.torque_delta[index] = 0.1;
    }
    const JointPair origin{0.5, -0.5};
    const JointPair fast{6.0, 0.0};
    Initialization initialization(torque);
    const RobotState fresh =
        FreshState(torque, origin, fast, start_us - 1'000, 88);
    CHECK(!initialization.Start(fresh, kEpoch, Producer(), start_us));
    CHECK(!initialization.active());
    CHECK(!initialization.failed());
    // Inside the torque bound the same plan is admitted.
    for (std::size_t index = 0; index < torque.joint_count(); ++index) {
      torque.runtime.continuity.torque_delta[index] = 10.0;
    }
    const RobotState ok =
        FreshState(torque, origin, fast, start_us - 1'000, 89);
    CHECK(initialization.Start(ok, kEpoch, Producer(), start_us));
    CHECK(initialization.active());
  }
  return 0;
}

// Completion is decided before the deadline: the ramp must be finished, every
// joint inside its tolerances, and the tolerance must hold for the whole stable
// window. A window broken by one out-of-tolerance sample restarts.
int StableWindowAndCompletion() {
  ApplicationConfig config = FakeConfig();
  // The ramp rate (0.5 rad over 2 s) stays inside the runtime's velocity
  // continuity bound, so these are frames the runtime would accept.
  const TimeUs duration = 2'000'000;
  const TimeUs window = 100'000;
  SetDuration(config, duration);
  config.initialization.trajectory.deadline_us = 10'000'000;
  config.initialization.trajectory.stable_window_us = window;
  config.initialization.trajectory.position_tolerance = 0.02;
  config.initialization.trajectory.velocity_tolerance = 0.05;
  const JointPair start_q{-0.5, 0.25};
  const JointPair tolerance_position{0.02, 0.02};
  const JointPair still{0.0, 0.0};
  const TimeUs start_us = 10'000'000;

  Initialization initialization(config);
  const RobotState origin =
      FreshState(config, start_q, still, start_us - 1'000, 40);
  CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));

  // Mid ramp the move keeps running even though the feedback already sits on
  // the target: the window never starts before the ramp is finished.
  {
    const TimeUs now = start_us + 50'000;
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 41);
    JointCommand frame{};
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Running);
    CHECK(!initialization.ready());
  }
  // The ramp is finished and the feedback is inside the tolerance: the window
  // starts now, so the move is not ready yet.
  JointCommand finish_frame{};
  {
    const TimeUs now = start_us + duration;
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 42);
    CHECK(initialization.Step(fresh, now, &finish_frame) ==
          InitializationStatus::Running);
    CHECK(!initialization.ready());
    // The ramp is done, so the frame is the target with no velocity.
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(finish_frame.joints.q[index] ==
            config.initialization.target_q[index]);
      CHECK(finish_frame.joints.dq[index] == 0.0);
    }
  }
  // Inside the window, still running.
  {
    const TimeUs now = start_us + duration + window - 1;
    const RobotState fresh = FreshState(
        config,
        JointPair{tolerance_position[0] / 4, -tolerance_position[1] / 4}, still,
        now - 1'000, 43);
    JointCommand frame{};
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Running);
    CHECK(!initialization.ready());
  }
  // The window has been held for its whole length: ready, with the final hold
  // frame exactly on the target.
  {
    const TimeUs now = start_us + duration + window;
    const RobotState fresh = FreshState(
        config,
        JointPair{tolerance_position[0] / 2, -tolerance_position[1] / 2}, still,
        now - 1'000, 44);
    JointCommand frame{};
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Ready);
    CHECK(initialization.ready());
    CHECK(!initialization.failed());
    CHECK(!initialization.active());
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(frame.joints.q[index] == config.initialization.target_q[index]);
      CHECK(frame.joints.dq[index] == 0.0);
      CHECK(frame.joints.tau_ff[index] == 0.0);
    }
  }

  // A broken window restarts: one out-of-tolerance sample after the ramp
  // finished throws the accumulated stability away.
  {
    Initialization restarted(config);
    const RobotState origin2 =
        FreshState(config, start_q, still, start_us - 1'000, 50);
    CHECK(restarted.Start(origin2, kEpoch, Producer(), start_us));
    JointCommand frame{};

    const TimeUs at_target = start_us + duration;
    const RobotState good =
        FreshState(config, JointPair{0.0, 0.0}, still, at_target - 1'000, 51);
    CHECK(restarted.Step(good, at_target, &frame) ==
          InitializationStatus::Running);

    // Out of position tolerance.
    const TimeUs off = at_target + 50'000;
    const RobotState drifted =
        FreshState(config, JointPair{0.05, 0.0}, still, off - 1'000, 52);
    CHECK(restarted.Step(drifted, off, &frame) ==
          InitializationStatus::Running);
    CHECK(!restarted.ready());

    // Out of velocity tolerance.
    const TimeUs fast = off + 12'500;
    const RobotState moving = FreshState(config, JointPair{0.0, 0.0},
                                         JointPair{0.0, 0.5}, fast - 1'000, 53);
    CHECK(restarted.Step(moving, fast, &frame) ==
          InitializationStatus::Running);

    // The window restarts at this call, not at the first good sample.
    const TimeUs reopened = at_target + 100'000;
    const RobotState again =
        FreshState(config, JointPair{0.0, 0.0}, still, reopened - 1'000, 54);
    CHECK(restarted.Step(again, reopened, &frame) ==
          InitializationStatus::Running);
    // If the window had not been restarted, this call would already be ready.
    const RobotState mid = FreshState(config, JointPair{0.0, 0.0}, still,
                                      reopened + 50'000 - 1'000, 55);
    CHECK(restarted.Step(mid, reopened + 50'000, &frame) ==
          InitializationStatus::Running);
    CHECK(!restarted.ready());
    const RobotState last = FreshState(config, JointPair{0.0, 0.0}, still,
                                       reopened + window - 1'000, 56);
    CHECK(restarted.Step(last, reopened + window, &frame) ==
          InitializationStatus::Ready);
    CHECK(restarted.ready());
  }
  return 0;
}

// Completion is judged before the deadline: a move whose stable window opens at
// the ramp end and completes after the deadline is Ready, not Failed.
int CompletionBeforeDeadline() {
  ApplicationConfig config = FakeConfig();
  const TimeUs duration = 1'000'000;
  const TimeUs window = 100'000;
  const TimeUs deadline = 1'050'000;
  // The deadline is deliberately shorter than duration + stable_window, so the
  // completion block can only answer Ready if it runs before the deadline
  // check.
  CHECK(deadline < duration + window);
  config.initialization.trajectory.duration_us = duration;
  config.initialization.trajectory.deadline_us = deadline;
  config.initialization.trajectory.stable_window_us = window;
  const JointPair start_q{-0.5, 0.5};
  const JointPair at_target{0.0, 0.0};
  const JointPair still{0.0, 0.0};
  const TimeUs start_us = 50'000'000;

  Initialization initialization(config);
  const RobotState origin =
      FreshState(config, start_q, still, start_us - 1'000, 90);
  CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));

  // The ramp ends here and the window opens; the deadline has not passed yet.
  {
    const TimeUs now = start_us + duration;
    CHECK(now - start_us <= deadline);
    JointCommand frame{};
    CHECK(initialization.Step(
              FreshState(config, at_target, still, now - 1'000, 91), now,
              &frame) == InitializationStatus::Running);
    CHECK(!initialization.ready());
  }
  // The window has been held for its whole length, so the move is Ready even
  // though the run is past its deadline.
  {
    const TimeUs now = start_us + duration + window;
    CHECK(now - start_us > deadline);
    JointCommand frame{};
    CHECK(initialization.Step(
              FreshState(config, at_target, still, now - 1'000, 92), now,
              &frame) == InitializationStatus::Ready);
    CHECK(initialization.ready());
    CHECK(!initialization.failed());
    CHECK(!initialization.active());
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(frame.joints.q[index] == config.initialization.target_q[index]);
      CHECK(frame.joints.dq[index] == 0.0);
      CHECK(frame.joints.tau_ff[index] == 0.0);
    }
  }
  return 0;
}

// The arithmetic boundaries: the tolerances are inclusive, a zero-length move
// is exact, and the extremes of `duration_us` neither divide by zero nor
// overflow.
int BoundaryArithmetic() {
  // Exactly on the tolerance counts as inside it.
  {
    ApplicationConfig config = FakeConfig();
    SetDuration(config, 1'000'000);
    config.initialization.trajectory.stable_window_us = 100'000;
    config.initialization.trajectory.position_tolerance = 0.02;
    config.initialization.trajectory.velocity_tolerance = 0.05;
    const JointPair start_q{0.0, 0.0};
    const TimeUs start_us = 60'000'000;
    Initialization initialization(config);
    const RobotState origin =
        FreshState(config, start_q, JointPair{0.0, 0.0}, start_us - 1'000, 100);
    CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));
    JointCommand frame{};
    const TimeUs ramp_end = start_us + 1'000'000;
    CHECK(initialization.Step(FreshState(config, start_q, JointPair{0.0, 0.0},
                                         ramp_end - 1'000, 101),
                              ramp_end,
                              &frame) == InitializationStatus::Running);
    CHECK(!initialization.ready());
    // Exactly `position_tolerance` away and exactly `velocity_tolerance` fast.
    const JointPair boundary{0.02, -0.02};
    const JointPair boundary_velocity{0.05, -0.05};
    CHECK(std::fabs(boundary[0] - config.initialization.target_q[0]) ==
          config.initialization.trajectory.position_tolerance);
    CHECK(std::fabs(boundary_velocity[0]) ==
          config.initialization.trajectory.velocity_tolerance);
    const TimeUs held = ramp_end + 100'000;
    CHECK(initialization.Step(FreshState(config, boundary, boundary_velocity,
                                         held - 1'000, 102),
                              held, &frame) == InitializationStatus::Ready);
    CHECK(initialization.ready());
  }

  // target_q == start_q: the move is exactly the origin, at every instant.
  {
    ApplicationConfig config = FakeConfig();
    SetDuration(config, 1'000'000);
    config.initialization.target_q[0] = 0.5;
    config.initialization.target_q[1] = -0.5;
    const JointPair start_q{0.5, -0.5};
    const JointPair still{0.0, 0.0};
    const TimeUs start_us = 70'000'000;
    Initialization initialization(config);
    const RobotState origin =
        FreshState(config, start_q, still, start_us - 1'000, 110);
    CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));
    JointCommand frame{};
    for (const TimeUs elapsed :
         {TimeUs{0}, TimeUs{500'000}, TimeUs{1'000'000}}) {
      const TimeUs now = start_us + elapsed;
      CHECK(initialization.Step(
                FreshState(config, start_q, still, now - 1'000, 111), now,
                &frame) == InitializationStatus::Running);
      for (std::size_t index = 0; index < config.joint_count(); ++index) {
        CHECK(frame.joints.q[index] == start_q[index]);
        CHECK(frame.joints.dq[index] == 0.0);
      }
    }
    CHECK(!initialization.ready());
  }

  // duration_us == 1: a one-microsecond ramp of a stationary target is exact,
  // and a move that fast is refused instead of emitted.
  {
    ApplicationConfig config = FakeConfig();
    SetDuration(config, 1);
    config.initialization.target_q[0] = 0.5;
    config.initialization.target_q[1] = -0.5;
    const JointPair start_q{0.5, -0.5};
    const JointPair still{0.0, 0.0};
    const TimeUs start_us = 80'000'000;
    Initialization initialization(config);
    const RobotState origin =
        FreshState(config, start_q, still, start_us - 1'000, 120);
    CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));
    JointCommand frame{};
    // Inside the ramp: the origin exactly, with no velocity.
    CHECK(initialization.Step(
              FreshState(config, start_q, still, start_us - 1'000, 121),
              start_us, &frame) == InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(std::isfinite(frame.joints.q[index]));
      CHECK(std::isfinite(frame.joints.dq[index]));
      CHECK(frame.joints.q[index] == start_q[index]);
      CHECK(frame.joints.dq[index] == 0.0);
    }
    // At the ramp end: the target exactly.
    const TimeUs ended = start_us + 1;
    const JointPair target{config.initialization.target_q[0],
                           config.initialization.target_q[1]};
    CHECK(initialization.Step(
              FreshState(config, target, still, ended - 1'000, 122), ended,
              &frame) == InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(frame.joints.q[index] == config.initialization.target_q[index]);
      CHECK(frame.joints.dq[index] == 0.0);
    }
    // The same one-microsecond ramp of a real move needs an enormous rate, so
    // it is refused rather than emitted.
    ApplicationConfig moving = FakeConfig();
    SetDuration(moving, 1);
    const JointPair far{0.5, -0.5};
    Initialization refused(moving);
    const RobotState plan =
        FreshState(moving, far, still, start_us - 1'000, 123);
    CHECK(!refused.Start(plan, kEpoch, Producer(), start_us));
    CHECK(!refused.active());
    CHECK(!refused.failed());
  }

  // A huge duration stays finite and still ends exactly on the target.
  {
    ApplicationConfig config = FakeConfig();
    const TimeUs duration = 1ULL << 62;
    config.initialization.trajectory.duration_us = duration;
    config.initialization.trajectory.deadline_us =
        std::numeric_limits<TimeUs>::max();
    config.initialization.trajectory.stable_window_us = 100'000;
    const JointPair start_q{0.5, -0.5};
    const JointPair still{0.0, 0.0};
    const TimeUs start_us = 90'000'000;
    Initialization initialization(config);
    const RobotState origin =
        FreshState(config, start_q, still, start_us - 1'000, 130);
    CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));
    JointCommand frame{};
    // Deep inside a practically infinite ramp the frame is finite and inside
    // the segment.
    const TimeUs early = start_us + 1'000'000;
    CHECK(initialization.Step(
              FreshState(config, start_q, still, early - 1'000, 131), early,
              &frame) == InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(std::isfinite(frame.joints.q[index]));
      CHECK(std::isfinite(frame.joints.dq[index]));
      CHECK(frame.joints.q[index] >=
            std::fmin(start_q[index], config.initialization.target_q[index]));
      CHECK(frame.joints.q[index] <=
            std::fmax(start_q[index], config.initialization.target_q[index]));
    }
    // The ramp end lands exactly on the target, with no velocity.
    const TimeUs ended = start_us + duration;
    const JointPair target{config.initialization.target_q[0],
                           config.initialization.target_q[1]};
    CHECK(initialization.Step(
              FreshState(config, target, still, ended - 1'000, 132), ended,
              &frame) == InitializationStatus::Running);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(frame.joints.q[index] == config.initialization.target_q[index]);
      CHECK(frame.joints.dq[index] == 0.0);
    }
  }
  return 0;
}

// The run deadline is the final bound: past it the move fails, latches, and
// emits nothing while it stays failed.
int DeadlineFails() {
  ApplicationConfig config = FakeConfig();
  // A ramp rate inside the runtime's velocity continuity bound, with a deadline
  // that leaves the tolerance window plenty of room to stay unmet.
  const TimeUs duration = 4'000'000;
  const TimeUs deadline = 6'000'000;
  config.initialization.trajectory.duration_us = duration;
  config.initialization.trajectory.deadline_us = deadline;
  config.initialization.trajectory.stable_window_us = 50'000;
  config.initialization.trajectory.position_tolerance = 0.001;
  config.initialization.trajectory.velocity_tolerance = 0.001;
  const JointPair start_q{1.0, -1.0};
  const JointPair still{0.0, 0.0};
  const TimeUs start_us = 20'000'000;

  Initialization initialization(config);
  const RobotState origin =
      FreshState(config, start_q, still, start_us - 1'000, 60);
  CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));

  // The feedback never moves off the origin, so the tolerance is never met.
  JointCommand frame{};
  {
    const TimeUs now = start_us + 5'000'000;
    const RobotState fresh =
        FreshState(config, start_q, still, now - 1'000, 61);
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Running);
    CHECK(!initialization.failed());
  }
  // Exactly on the deadline is not yet overdue.
  {
    const TimeUs now = start_us + deadline;
    const RobotState fresh =
        FreshState(config, start_q, still, now - 1'000, 62);
    CHECK(initialization.Step(fresh, now, &frame) ==
          InitializationStatus::Running);
    CHECK(!initialization.failed());
  }
  // One microsecond past it the move is failed, and no frame is emitted.
  {
    const TimeUs now = start_us + deadline + 1;
    const RobotState fresh =
        FreshState(config, start_q, still, now - 1'000, 63);
    JointCommand untouched = Sentinel();
    CHECK(initialization.Step(fresh, now, &untouched) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(untouched) == 0);
    CHECK(initialization.failed());
    CHECK(!initialization.active());
    CHECK(!initialization.ready());
  }
  // Failed is latched: further calls fail without emitting, and Hold refuses.
  {
    const TimeUs now = start_us + deadline + 200'000;
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 64);
    JointCommand untouched = Sentinel();
    CHECK(initialization.Step(fresh, now, &untouched) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(untouched) == 0);
    JointCommand held = Sentinel();
    CHECK(initialization.Hold(fresh, now, &held) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(held) == 0);
    // Only a new Start clears the latch.
    const TimeUs restart = now + 1'000;
    const RobotState restart_state =
        FreshState(config, JointPair{0.0, 0.0}, still, restart - 1'000, 65);
    CHECK(initialization.Start(restart_state, kEpoch + 1, Producer(), restart));
    CHECK(initialization.active());
    CHECK(!initialization.failed());
    CHECK(!initialization.ready());
    CHECK(initialization.epoch() == kEpoch + 1);
  }
  return 0;
}

// Ready keeps the epoch, the producer and the gains, and every hold is a new
// frame from a new state, with the target unmoved.
int ReadyHold() {
  ApplicationConfig config = FakeConfig();
  // A ramp rate inside the runtime's velocity continuity bound.
  const TimeUs duration = 2'000'000;
  const TimeUs window = 100'000;
  SetDuration(config, duration);
  config.initialization.trajectory.deadline_us = 10'000'000;
  config.initialization.trajectory.stable_window_us = window;
  // Distinct per joint, so a hold that mixes the two gains is visible.
  config.initialization.gains.kp[0] = 3.0;
  config.initialization.gains.kp[1] = 7.5;
  config.initialization.gains.kd[0] = 0.25;
  config.initialization.gains.kd[1] = 0.75;
  const JointPair start_q{-0.5, 0.5};
  const JointPair still{0.0, 0.0};
  const TimeUs start_us = 30'000'000;

  Initialization initialization(config);
  const RobotState origin =
      FreshState(config, start_q, still, start_us - 1'000, 70);
  CHECK(initialization.Start(origin, kEpoch, Producer(), start_us));
  CHECK(initialization.epoch() == kEpoch);
  CHECK(initialization.producer_id() == Producer());

  JointCommand ready_frame{};
  {
    // The ramp finishes here and opens the stable window.
    const TimeUs now = start_us + duration;
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 71);
    CHECK(initialization.Step(fresh, now, &ready_frame) ==
          InitializationStatus::Running);
    CHECK(!initialization.ready());
  }
  {
    const TimeUs now = start_us + duration + window;
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 71);
    CHECK(initialization.Step(fresh, now, &ready_frame) ==
          InitializationStatus::Ready);
  }
  CHECK(initialization.ready());

  // A Step with no move in flight reports Failed, emits nothing and leaves the
  // finished move unlatched: the Ready state holds with Hold.
  {
    const TimeUs now = start_us + duration + window;
    JointCommand command = Sentinel();
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 72);
    CHECK(initialization.Step(fresh, now, &command) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(command) == 0);
    CHECK(initialization.ready());
    CHECK(!initialization.failed());
    CHECK(!initialization.active());
  }

  // Hold before Start is refused.
  {
    Initialization untouched(config);
    JointCommand command = Sentinel();
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, start_us - 1'000, 72);
    CHECK(untouched.Hold(fresh, start_us, &command) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(command) == 0);
  }

  // Ready keeps sending: same target, same gains, same epoch and producer, a
  // new command sequence and the state sequence it was built from.
  Sequence previous_seq = ready_frame.command_seq;
  const TimeUs first_hold_us = start_us + duration + window + 20'000;
  JointCommand first_hold{};
  {
    const RobotState fresh = FreshState(config, JointPair{0.01, -0.01}, still,
                                        first_hold_us - 1'000, 73);
    CHECK(initialization.Hold(fresh, first_hold_us, &first_hold) ==
          InitializationStatus::Ready);
    CHECK(initialization.ready());
    CHECK(!initialization.failed());
    CHECK(!initialization.active());
    CHECK(initialization.epoch() == kEpoch);
    CHECK(initialization.producer_id() == Producer());
    CHECK(first_hold.epoch == ready_frame.epoch);
    CHECK(first_hold.producer_id == ready_frame.producer_id);
    CHECK(first_hold.command_seq > previous_seq);
    CHECK(first_hold.source_state_seq == 73);
    CHECK(first_hold.layout_id.View() == config.layout_id());
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(first_hold.joints.q[index] ==
            config.initialization.target_q[index]);
      CHECK(first_hold.joints.dq[index] == 0.0);
      CHECK(first_hold.joints.tau_ff[index] == 0.0);
      CHECK(first_hold.joints.kp[index] ==
            config.initialization.gains.kp[index]);
      CHECK(first_hold.joints.kd[index] ==
            config.initialization.gains.kd[index]);
      // No jump of the target or the gains against the ready frame.
      CHECK(first_hold.joints.q[index] == ready_frame.joints.q[index]);
      CHECK(first_hold.joints.dq[index] == ready_frame.joints.dq[index]);
      CHECK(first_hold.joints.kp[index] == ready_frame.joints.kp[index]);
      CHECK(first_hold.joints.kd[index] == ready_frame.joints.kd[index]);
    }
    previous_seq = first_hold.command_seq;
  }
  // A second hold is another new frame from another state, still nothing moves.
  {
    const TimeUs now = first_hold_us + 20'000;
    const RobotState fresh =
        FreshState(config, JointPair{0.005, -0.005}, still, now - 1'000, 74);
    JointCommand second_hold{};
    CHECK(initialization.Hold(fresh, now, &second_hold) ==
          InitializationStatus::Ready);
    CHECK(second_hold.command_seq > previous_seq);
    CHECK(second_hold.source_state_seq == 74);
    CHECK(second_hold.epoch == first_hold.epoch);
    CHECK(second_hold.producer_id == first_hold.producer_id);
    for (std::size_t index = 0; index < config.joint_count(); ++index) {
      CHECK(second_hold.joints.q[index] == first_hold.joints.q[index]);
      CHECK(second_hold.joints.kp[index] == first_hold.joints.kp[index]);
      CHECK(second_hold.joints.kd[index] == first_hold.joints.kd[index]);
    }
  }
  // An unusable state fails the hold without emitting and without moving the
  // initialization out of Ready.
  {
    const TimeUs now = first_hold_us + 40'000;
    JointCommand command = Sentinel();
    RobotState stale =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 75);
    stale.stamp_us = now - config.runtime.timeouts.source_state_us - 1;
    CHECK(initialization.Hold(stale, now, &command) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(command) == 0);
    JointCommand command_stale_seq = Sentinel();
    RobotState unobserved =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 76);
    unobserved.feedback.joints[1].valid = false;
    CHECK(initialization.Hold(unobserved, now, &command_stale_seq) ==
          InitializationStatus::Failed);
    CHECK(CheckUntouched(command_stale_seq) == 0);
    // Ready is not lost: a usable state holds again.
    JointCommand command_again{};
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 77);
    CHECK(initialization.Hold(fresh, now, &command_again) ==
          InitializationStatus::Ready);
    // A null output fails the call without losing Ready.
    CHECK(initialization.Hold(fresh, now, nullptr) ==
          InitializationStatus::Failed);
    CHECK(initialization.ready());
    CHECK(!initialization.failed());
  }
  // A finished initialization may be started again; the new move clears Ready
  // and carries its own epoch.
  {
    const TimeUs now = first_hold_us + 60'000;
    const RobotState fresh =
        FreshState(config, JointPair{0.0, 0.0}, still, now - 1'000, 78);
    CHECK(initialization.Start(fresh, kEpoch + 1, Producer(), now));
    CHECK(initialization.active());
    CHECK(!initialization.ready());
    CHECK(!initialization.failed());
    CHECK(initialization.epoch() == kEpoch + 1);
  }
  return 0;
}

}  // namespace

int main() {
  int failures = 0;
  failures += FirstFrameContinuity();
  failures += MonotonicInterpolation();
  failures += FrameCompleteness();
  failures += UnusableFeedbackFails();
  failures += AdmissionRefusals();
  failures += StableWindowAndCompletion();
  failures += CompletionBeforeDeadline();
  failures += BoundaryArithmetic();
  failures += DeadlineFails();
  failures += ReadyHold();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
