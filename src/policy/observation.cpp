#include "irmv/policy/observation.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace irmv::policy {
namespace {

// One component of the assembled vector: clipped before it is scaled, which is
// the order the training observation manager applies them in
// (ObservationManager.compute_group: noise, then clip, then scale).
double TransformComponent(double value, const ObservationTerm &term,
                          std::size_t component) {
  if (term.clipped) {
    const ClipRange &range = term.clip[component];
    if (value < range.min) {
      value = range.min;
    } else if (value > range.max) {
      value = range.max;
    }
  }
  if (term.scaled) {
    value *= term.scale[component];
  }
  return value;
}

const ObservationTerm *FindTerm(const DeploymentContract &contract,
                                ObservationTermKind kind) {
  for (const ObservationTerm &term : contract.observation.terms) {
    if (term.kind == kind) {
      return &term;
    }
  }
  return nullptr;
}

// The gravity vector projected onto the base frame, in the convention the
// training used (`projected_gravity_b`): the world gravity direction expressed
// in the base frame, as a unit vector. With roll and pitch of the base and
// `g_b = R^T * (0, 0, -1)` this is
//
//   ( sin(pitch), -sin(roll) cos(pitch), -cos(roll) cos(pitch) ).
//
// The yaw of the base does not enter, exactly like the training term, which
// reads the rotation matrix rather than the full pose.
std::array<double, 3> ProjectedGravity(double roll, double pitch) {
  const double sin_roll = std::sin(roll);
  const double cos_roll = std::cos(roll);
  const double sin_pitch = std::sin(pitch);
  const double cos_pitch = std::cos(pitch);
  return {sin_pitch, -sin_roll * cos_pitch, -cos_roll * cos_pitch};
}

class Writer {
 public:
  explicit Writer(Observation *out) : out_(out) {}

  bool Push(double value) {
    if (!std::isfinite(value)) {
      return false;
    }
    if (count_ >= kMaxObservationDimension) {
      return false;
    }
    out_->values[count_] = static_cast<float>(value);
    ++count_;
    return true;
  }

  std::uint32_t count() const { return count_; }

 private:
  Observation *out_;
  std::uint32_t count_{0};
};

}  // namespace

bool BuildObservation(
    const DeploymentContract &contract, const JointMapping &mapping,
    const protocol::RobotState &state, const VelocityCommand &command,
    const std::array<float, protocol::kMaxJoints> &last_action,
    Observation *out) {
  if (out == nullptr) {
    return false;
  }
  const std::size_t joint_count = contract.action.joints.size();
  if (state.feedback.count != joint_count ||
      mapping.action.size() != joint_count ||
      mapping.joint_pos.size() != joint_count ||
      mapping.joint_vel.size() != joint_count) {
    return false;
  }
  // The gravity and the angular velocity both come from the IMU snapshot; an
  // invalid one would turn a sensor loss into two plausible-looking numbers.
  if (!state.feedback.imu.valid) {
    return false;
  }
  if (contract.observation.dimension > kMaxObservationDimension) {
    return false;
  }
  const ObservationTerm *command_term =
      FindTerm(contract, ObservationTermKind::Command);
  const ObservationTerm *gravity_term =
      FindTerm(contract, ObservationTermKind::ProjectedGravity);
  const ObservationTerm *angular_term =
      FindTerm(contract, ObservationTermKind::BaseAngularVelocity);
  const ObservationTerm *position_term =
      FindTerm(contract, ObservationTermKind::JointPosition);
  const ObservationTerm *velocity_term =
      FindTerm(contract, ObservationTermKind::JointVelocity);
  const ObservationTerm *last_action_term =
      FindTerm(contract, ObservationTermKind::LastAction);
  if (command_term == nullptr || gravity_term == nullptr ||
      angular_term == nullptr || position_term == nullptr ||
      velocity_term == nullptr || last_action_term == nullptr) {
    return false;
  }

  Observation assembled;
  Writer writer(&assembled);

  // 1. The velocity command the policy is asked to track, in the base frame.
  const std::array<double, 3> command_values = {command.forward,
                                                command.lateral, command.yaw};
  for (std::size_t component = 0; component < command_values.size();
       ++component) {
    if (!writer.Push(TransformComponent(command_values[component],
                                        *command_term, component))) {
      return false;
    }
  }

  // 2. The gravity direction in the base frame.
  const auto gravity = ProjectedGravity(state.feedback.imu.orientation_rpy[0],
                                        state.feedback.imu.orientation_rpy[1]);
  for (std::size_t component = 0; component < gravity.size(); ++component) {
    if (!writer.Push(
            TransformComponent(gravity[component], *gravity_term, component))) {
      return false;
    }
  }

  // 3. The base angular velocity, already in the base frame.
  for (std::size_t component = 0; component < 3; ++component) {
    if (!writer.Push(
            TransformComponent(state.feedback.imu.angular_velocity[component],
                               *angular_term, component))) {
      return false;
    }
  }

  // 4. and 5. The joint position and velocity relative to the model's own
  // reference, gathered into the model's order.
  for (std::size_t index = 0; index < joint_count; ++index) {
    const std::uint32_t robot_index = mapping.joint_pos[index];
    if (robot_index >= protocol::kMaxJoints ||
        !state.feedback.joints[robot_index].valid) {
      return false;
    }
    const double relative = state.feedback.joints[robot_index].q -
                            contract.observation.joint_pos_rel.reference[index];
    if (!writer.Push(TransformComponent(relative, *position_term, index))) {
      return false;
    }
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    const std::uint32_t robot_index = mapping.joint_vel[index];
    if (robot_index >= protocol::kMaxJoints ||
        !state.feedback.joints[robot_index].valid) {
      return false;
    }
    const double relative = state.feedback.joints[robot_index].dq -
                            contract.observation.joint_vel_rel.reference[index];
    if (!writer.Push(TransformComponent(relative, *velocity_term, index))) {
      return false;
    }
  }

  // 6. The action of the previous step, in the stage the contract names.
  for (std::size_t index = 0; index < joint_count; ++index) {
    if (!writer.Push(
            TransformComponent(last_action[index], *last_action_term, index))) {
      return false;
    }
  }

  if (writer.count() != contract.observation.dimension) {
    return false;
  }

  // The normalizer the SDK has to apply, if the contract has one that is not
  // already part of the graph. An embedded normalizer is the model's business
  // and is deliberately not applied twice.
  if (contract.normalizer.location == NormalizerLocation::Application) {
    for (std::uint32_t index = 0; index < writer.count(); ++index) {
      const double value = (static_cast<double>(assembled.values[index]) -
                            contract.normalizer.mean[index]) /
                           contract.normalizer.stddev[index];
      if (!std::isfinite(value)) {
        return false;
      }
      assembled.values[index] = static_cast<float>(value);
    }
  }

  assembled.count = writer.count();
  *out = assembled;
  return true;
}

}  // namespace irmv::policy
