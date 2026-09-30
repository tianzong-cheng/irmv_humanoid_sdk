#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "irmv/policy/deployment.hpp"
#include "irmv/protocol/types.hpp"

// The observation the local policy consumes, assembled from one coherent
// snapshot of the lower layer and the intent the operator holds. This is a pure
// transformation: it reads no clock, touches no transport, allocates nothing
// and never throws.
//
// The concatenation order is fixed by the deployment contract and is the order
// the training environment declared: command, projected_gravity, base_ang_vel,
// joint_pos_rel, joint_vel_rel, last_action. A term the contract does not carry
// is not assembled, and every segment is checked for the shape and the
// finiteness the contract promises, so a malformed snapshot is refused instead
// of being fed to the model.
namespace irmv::policy {

// The body-frame velocity the policy is asked to track. Units: m/s, m/s, rad/s,
// positive forward, positive left and positive counter-clockwise around the
// base z axis. It is the application's own type and not the input router's, so
// this library stays free of the application layer.
struct VelocityCommand {
  double forward{0.0};
  double lateral{0.0};
  double yaw{0.0};
};

// Largest observation this build can assemble: the three fixed three-component
// terms plus the three joint-sized ones at the protocol's joint limit.
inline constexpr std::size_t kMaxObservationDimension =
    9 + 3 * protocol::kMaxJoints;

// One assembled observation. Fixed capacity, so submitting it to the inference
// worker copies and never allocates.
struct Observation {
  std::uint32_t count{0};
  std::array<float, kMaxObservationDimension> values{};
};

// Builds the observation of one policy step.
//
// `state` is the coherent snapshot the runtime published for this instant and
// `last_action` the raw action of the previous step (the contract names which
// stage the training observed and the values it starts from). It returns false
// - leaving `*out` untouched - when the snapshot cannot be used: an invalid or
// absent IMU snapshot, a joint count that disagrees with the model, a joint of
// the position or velocity term whose feedback is not valid, or a value that is
// not finite. A policy that silently read a stale or invalid sample would turn
// a feedback loss into a motion command.
bool BuildObservation(
    const DeploymentContract &contract, const JointMapping &mapping,
    const protocol::RobotState &state, const VelocityCommand &command,
    const std::array<float, protocol::kMaxJoints> &last_action,
    Observation *out);

}  // namespace irmv::policy
