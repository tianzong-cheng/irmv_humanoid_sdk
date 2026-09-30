#include "irmv/policy/action.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace irmv::policy {
namespace {

double Clamp(double value, const ClipRange &range) {
  return std::min(std::max(value, range.min), range.max);
}

// One stage of the frozen pipeline. `order` is validated by the contract
// parser, so this only has to implement the four stages it accepted.
double ApplyStage(const ActionContract &action, ActionStage stage,
                  std::size_t index, double value) {
  switch (stage) {
    case ActionStage::WrapperClip:
      return action.wrapper_clip
                 ? Clamp(value, action.wrapper_clip_ranges[index])
                 : value;
    case ActionStage::Scale:
      return value * action.scale[index];
    case ActionStage::Offset:
      return value + action.offset[index];
    case ActionStage::TermClip:
      return action.term_clip ? Clamp(value, action.term_clip_ranges[index])
                              : value;
  }
  return value;
}

}  // namespace

bool ResolveGains(const DeploymentContract &contract,
                  const JointMapping &mapping,
                  const std::vector<std::string> &robot_joint_names,
                  const configuration::PolicyConfig &config,
                  std::string_view model_sha256, ResolvedGains *out) {
  if (out == nullptr) {
    return false;
  }
  const std::size_t joint_count = robot_joint_names.size();
  if (!CoversEveryJoint(contract, joint_count)) {
    return false;
  }
  if (mapping.action.size() != joint_count ||
      contract.action.kp.size() != joint_count ||
      contract.action.kd.size() != joint_count) {
    return false;
  }

  ResolvedGains resolved;
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double kp = contract.action.kp[index];
    const double kd = contract.action.kd[index];
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0 || kd < 0.0) {
      return false;
    }
    resolved.kp[index] = kp;
    resolved.kd[index] = kd;
  }

  // The override set that names this exact model. A digest of another model is
  // a different experiment and is never applied.
  const configuration::PolicyOverrides *matching = nullptr;
  for (const configuration::PolicyOverrides &candidate : config.overrides) {
    if (candidate.model_sha256 == model_sha256) {
      matching = &candidate;
      break;
    }
  }
  if (matching != nullptr) {
    for (const configuration::PolicyJointOverride &entry : matching->joints) {
      // The position of the joint in the model's action list: the gain table
      // is indexed by it, and the mapping scatters it into the frame.
      std::size_t action_index = joint_count;
      for (std::size_t candidate = 0; candidate < joint_count; ++candidate) {
        if (contract.action.joints[candidate] == entry.joint) {
          action_index = candidate;
          break;
        }
      }
      if (action_index == joint_count) {
        // The override names a joint the model does not command. The loader
        // already requires every override to name a configured joint; naming
        // one this model does not control is just as unusable, because no
        // frame would ever carry it.
        return false;
      }
      if (resolved.override_count >= protocol::kMaxJoints) {
        return false;
      }
      GainOverride &record = resolved.overrides[resolved.override_count];
      if (!record.joint.Assign(entry.joint)) {
        return false;
      }
      record.model_kp = resolved.kp[action_index];
      record.model_kd = resolved.kd[action_index];
      if (entry.overrides_kp) {
        if (!std::isfinite(entry.kp) || entry.kp < 0.0) {
          return false;
        }
        resolved.kp[action_index] = entry.kp;
        record.kp_overridden = true;
        record.applied_kp = entry.kp;
      }
      if (entry.overrides_kd) {
        if (!std::isfinite(entry.kd) || entry.kd < 0.0) {
          return false;
        }
        resolved.kd[action_index] = entry.kd;
        record.kd_overridden = true;
        record.applied_kd = entry.kd;
      }
      ++resolved.override_count;
    }
  }

  resolved.joint_count = static_cast<std::uint32_t>(joint_count);
  *out = resolved;
  return true;
}

bool BuildCommand(const DeploymentContract &contract,
                  const JointMapping &mapping, const ResolvedGains &gains,
                  const float *action, std::size_t action_count,
                  const protocol::RobotState &state, std::string_view layout_id,
                  protocol::Id producer_id, protocol::Sequence epoch,
                  protocol::Sequence command_seq, protocol::JointCommand *out) {
  if (out == nullptr || action == nullptr) {
    return false;
  }
  const std::size_t joint_count = contract.action.joints.size();
  if (action_count != joint_count || mapping.action.size() != joint_count ||
      gains.joint_count != joint_count || contract.action.order.size() != 4 ||
      contract.action.default_dq.size() != joint_count) {
    return false;
  }
  // The frame has to carry every joint the runtime reports, or the joints the
  // model does not command would keep whatever target they had.
  if (state.feedback.count != joint_count) {
    return false;
  }

  protocol::JointCommand frame;
  if (!frame.layout_id.Assign(layout_id)) {
    return false;
  }
  frame.producer_id = producer_id;
  frame.epoch = epoch;
  frame.command_seq = command_seq;
  frame.source_state_seq = state.state_seq;
  frame.joints.count = static_cast<std::uint32_t>(joint_count);

  for (std::size_t index = 0; index < joint_count; ++index) {
    double value = static_cast<double>(action[index]);
    if (!std::isfinite(value)) {
      return false;
    }
    for (const ActionStage stage : contract.action.order) {
      value = ApplyStage(contract.action, stage, index, value);
      if (!std::isfinite(value)) {
        return false;
      }
    }
    const std::uint32_t robot_index = mapping.action[index];
    if (robot_index >= protocol::kMaxJoints) {
      return false;
    }
    // The gain table is indexed by the position in the model's action list;
    // the frame is indexed by the deployment's canonical joint order.
    frame.joints.q[robot_index] = value;
    frame.joints.dq[robot_index] = contract.action.default_dq[index];
    frame.joints.tau_ff[robot_index] = 0.0;
    frame.joints.kp[robot_index] = gains.kp[index];
    frame.joints.kd[robot_index] = gains.kd[index];
  }

  *out = frame;
  return true;
}

}  // namespace irmv::policy
