#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/configuration/application_config.hpp"
#include "irmv/policy/deployment.hpp"
#include "irmv/protocol/types.hpp"

// The action half of the local policy: the model's output transformed into one
// complete Tracking frame, and the gain table that frame is sent with.
//
// Two rules shape it. The pipeline is the one the contract freezes - the raw
// action is clipped, scaled and offset into a joint position and then clipped
// again - and it is applied in that order and no other, because a different
// order produces a different target from the same model output. And the gains
// are the model's own, with only the overrides that name this exact model
// applied on top: nothing here invents a fallback gain, and nothing here may
// widen a runtime boundary, which the lower layer enforces on its own.
namespace irmv::policy {

// What one experimental override replaced, kept for the run log: the model's
// own value and the value that is actually sent. A joint that no override
// matched never gets a row, so the log names exactly the differences that were
// applied.
struct GainOverride {
  protocol::Text64 joint;
  bool kp_overridden{false};
  double model_kp{0.0};
  double applied_kp{0.0};
  bool kd_overridden{false};
  double model_kd{0.0};
  double applied_kd{0.0};
};

// The gains one frame is built with, in the deployment's canonical joint order,
// after the model's values and the matching override set were merged.
//
// It is built once per model load, not once per step: the per-step path then
// stays a pure transformation, and the merge - the one place an experiment can
// change a trained value - is recorded in `overrides`.
struct ResolvedGains {
  std::uint32_t joint_count{0};
  std::array<double, protocol::kMaxJoints> kp{};
  std::array<double, protocol::kMaxJoints> kd{};
  std::uint32_t override_count{0};
  std::array<GainOverride, protocol::kMaxJoints> overrides{};
};

// Merges the contract's per-joint gains with the override set whose complete
// SHA-256 equals `model_sha256`. `robot_joint_names` is the deployment's
// canonical joint table, which the model has to cover exactly.
//
// The model's own values are the starting point and stay in place for every
// joint an override does not name; an override set of another model is never
// applied, which is why the digest is compared whole and not by prefix or by
// file name. It returns false - leaving `*out` untouched - when the model does
// not command every configured joint, when a gain of the model is not finite
// and non-negative, or when the model's joint lists disagree with each other.
bool ResolveGains(const DeploymentContract &contract,
                  const JointMapping &mapping,
                  const std::vector<std::string> &robot_joint_names,
                  const configuration::PolicyConfig &config,
                  std::string_view model_sha256, ResolvedGains *out);

// Builds one complete Tracking frame from one model action.
//
// `action` is the raw model output, `state` the coherent snapshot the
// observation was taken from - the frame's source state sequence is the
// identity the runtime looks the sample up by - and the remaining arguments are
// the identity of this generation, exactly like the initialization generator.
// The routing identity (instance, session, send stamp) is left to
// RobotClient::SendCommand, which fills it in on the caller's frame.
//
// It returns false, leaving `*out` untouched, when the action has the wrong
// count, carries a value that is not finite, or when the model does not command
// every joint of the frame: a partial frame would leave the remaining joints at
// a stale target.
bool BuildCommand(const DeploymentContract &contract,
                  const JointMapping &mapping, const ResolvedGains &gains,
                  const float *action, std::size_t action_count,
                  const protocol::RobotState &state, std::string_view layout_id,
                  protocol::Id producer_id, protocol::Sequence epoch,
                  protocol::Sequence command_seq, protocol::JointCommand *out);

// The `last_action` observation of the first step of an authorization: the
// contract's own `initial` values, in the order of the model's action list, and
// zero for the joints the protocol allows beyond them.
//
// The contract names which stage the training observed and the values it
// started from; starting from anything else - zeros, say - feeds the model an
// observation the training environment never produced, which is why the
// application asks the contract instead of assuming. `out` is left untouched
// when the contract does not describe this deployment's action list.
void ResetLastAction(const ActionContract &contract,
                     std::array<float, protocol::kMaxJoints> *out);

// The `last_action` observation of the next step: this step's action, stored in
// the stage the contract names - the raw policy output, or the processed joint
// position the frame was built from. Both are what the training environment
// observed at the same point of its own step, and the two differ by the whole
// clipping/scaling/offset pipeline, so the stage cannot be ignored.
//
// It returns false, leaving `*out` untouched, when the action does not have the
// contract's count or when the processed value is not finite.
bool LastActionFromAction(const ActionContract &contract, const float *action,
                          std::size_t action_count,
                          std::array<float, protocol::kMaxJoints> *out);

// The processed joint position of one element of one raw action: the contract's
// frozen pipeline applied to it in the order the contract froze. It is the
// value both the frame's `q` and the `processed` stage of the `last_action`
// observation are built from, so the two can never disagree. Returns false when
// the value is not finite at any stage.
bool ProcessActionElement(const ActionContract &contract, const float *action,
                          std::size_t action_count, std::size_t index,
                          double *out);

}  // namespace irmv::policy
