// The two pure transformations of the local policy: the observation it consumes
// and the joint frame it produces.
//
// The fixtures are the frozen deployment documents of the contract test, so the
// math is exercised against exactly the arrays a real package carries, and the
// three-joint document's three different joint orders prove that every value is
// placed by name and never by position.
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "irmv/policy/action.hpp"
#include "irmv/policy/deployment.hpp"
#include "irmv/policy/observation.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::PolicyConfig;
using irmv::configuration::PolicyJointOverride;
using irmv::configuration::PolicyOverrides;
using irmv::policy::BuildCommand;
using irmv::policy::BuildObservation;
using irmv::policy::CoversEveryJoint;
using irmv::policy::DeploymentContract;
using irmv::policy::JointMapping;
using irmv::policy::Observation;
using irmv::policy::ResolvedGains;
using irmv::policy::ResolveGains;
using irmv::policy::VelocityCommand;
using irmv::protocol::JointCommand;
using irmv::protocol::RobotState;
using irmv::protocol::TimeUs;

std::string ReadFile(const std::string &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    std::cerr << "cannot open fixture " << path << '\n';
    return {};
  }
  std::ostringstream contents;
  contents << file.rdbuf();
  return contents.str();
}

// The three-joint document: the action list is knee, hip_pitch, ankle_pitch
// while the deployment table is ankle_pitch, knee, hip_pitch, so every value is
// gathered and scattered by name.
const std::vector<std::string> kRobotThree = {"ankle_pitch", "knee",
                                              "hip_pitch"};
// The two-joint document, resolved against a table in the opposite order.
const std::vector<std::string> kRobotTwo = {"right", "left"};

RobotState MakeState(std::size_t joint_count, TimeUs stamp_us,
                     std::uint64_t state_seq) {
  RobotState state;
  state.stamp_us = stamp_us;
  state.state_seq = state_seq;
  state.feedback.count = static_cast<std::uint32_t>(joint_count);
  state.feedback.imu.valid = true;
  for (std::size_t index = 0; index < joint_count; ++index) {
    state.feedback.joints[index].valid = true;
  }
  return state;
}

PolicyConfig MakePolicyConfig() {
  PolicyConfig config;
  config.model_path = "policy.onnx";
  config.inference_timeout_us = 100000;
  return config;
}

// The observation is stored as float, because that is what the model reads;
// the tolerance is the precision of that storage, not of the computation.
bool NearlyEqual(double left, double right) {
  return std::fabs(left - right) <= 1e-6;
}

// One replacement of a fixture document, for the cases that need a contract the
// fixtures do not carry (another last_action stage, non-zero initial values)
// while everything else stays the frozen document a real package would hold.
std::string ReplaceOnce(std::string value, std::string_view from,
                        std::string_view to) {
  const std::size_t position = value.find(from);
  if (position == std::string::npos) {
    return {};
  }
  value.replace(position, from.size(), to);
  return value;
}

}  // namespace

int main() {
  const std::string two_axis_text =
      ReadFile("tests/support/deployment/valid_two_axis.json");
  const std::string three_orders_text =
      ReadFile("tests/support/deployment/valid_three_orders.json");
  CHECK(!two_axis_text.empty());
  CHECK(!three_orders_text.empty());

  const DeploymentContract two_axis =
      irmv::policy::ParseDeployment(two_axis_text);
  const JointMapping two_axis_mapping =
      irmv::policy::BuildJointMapping(two_axis, kRobotTwo);
  CHECK(two_axis_mapping.action == std::vector<std::uint32_t>({1, 0}));
  CHECK(CoversEveryJoint(two_axis, kRobotTwo.size()));
  CHECK(!CoversEveryJoint(two_axis, kRobotTwo.size() + 1));

  // ---------------------------------------------------------------------
  // Observation: the six segments, in the order the contract fixes.
  // ---------------------------------------------------------------------
  {
    RobotState state = MakeState(2, 1000, 7);
    // The feedback is indexed by the deployment's canonical joint order
    // (right, left) while the model's lists are (left, right) and (right,
    // left).
    state.feedback.joints[0].q = 0.30;  // right
    state.feedback.joints[0].dq = 1.50;
    state.feedback.joints[1].q = -0.10;  // left
    state.feedback.joints[1].dq = -0.50;
    state.feedback.imu.orientation_rpy = {0.3, 0.0, 1.2};
    state.feedback.imu.angular_velocity = {0.1, 0.2, 0.3};

    VelocityCommand command;
    command.forward = 0.8;
    command.lateral = -0.2;
    command.yaw = 0.4;
    std::array<float, irmv::protocol::kMaxJoints> last_action{};
    last_action[0] = 0.5f;    // left
    last_action[1] = -0.25f;  // right

    Observation observation;
    CHECK(BuildObservation(two_axis, two_axis_mapping, state, command,
                           last_action, &observation));
    CHECK(observation.count == 15);

    // 1. command
    CHECK(NearlyEqual(observation.values[0], 0.8));
    CHECK(NearlyEqual(observation.values[1], -0.2));
    CHECK(NearlyEqual(observation.values[2], 0.4));
    // 2. projected gravity for roll 0.3, pitch 0
    CHECK(NearlyEqual(observation.values[3], 0.0));
    CHECK(NearlyEqual(observation.values[4], -std::sin(0.3)));
    CHECK(NearlyEqual(observation.values[5], -std::cos(0.3)));
    // 3. base angular velocity
    CHECK(NearlyEqual(observation.values[6], 0.1));
    CHECK(NearlyEqual(observation.values[7], 0.2));
    CHECK(NearlyEqual(observation.values[8], 0.3));
    // 4. joint_pos_rel in the model's own order (right, left) with the
    //    model's own references (-0.2, 0.1).
    CHECK(NearlyEqual(observation.values[9], 0.30 - (-0.2)));
    CHECK(NearlyEqual(observation.values[10], -0.10 - 0.1));
    // 5. joint_vel_rel in the velocity list's order (left, right).
    CHECK(NearlyEqual(observation.values[11], -0.50 - 0.0));
    CHECK(NearlyEqual(observation.values[12], 1.50 - 0.0));
    // 6. last_action, in the action list's order (left, right).
    CHECK(NearlyEqual(observation.values[13], 0.5));
    CHECK(NearlyEqual(observation.values[14], -0.25));
  }

  // A snapshot the policy must not use is refused instead of being fed in.
  {
    Observation observation;
    VelocityCommand command;
    std::array<float, irmv::protocol::kMaxJoints> last_action{};
    RobotState state = MakeState(2, 1000, 7);

    RobotState no_imu = state;
    no_imu.feedback.imu.valid = false;
    CHECK(!BuildObservation(two_axis, two_axis_mapping, no_imu, command,
                            last_action, &observation));

    RobotState wrong_count = state;
    wrong_count.feedback.count = 3;
    CHECK(!BuildObservation(two_axis, two_axis_mapping, wrong_count, command,
                            last_action, &observation));

    RobotState invalid_joint = state;
    invalid_joint.feedback.joints[1].valid = false;
    CHECK(!BuildObservation(two_axis, two_axis_mapping, invalid_joint, command,
                            last_action, &observation));

    RobotState non_finite = state;
    non_finite.feedback.joints[0].q = std::numeric_limits<double>::quiet_NaN();
    CHECK(!BuildObservation(two_axis, two_axis_mapping, non_finite, command,
                            last_action, &observation));

    std::array<float, irmv::protocol::kMaxJoints> bad_action{};
    bad_action[0] = std::numeric_limits<float>::infinity();
    CHECK(!BuildObservation(two_axis, two_axis_mapping, state, command,
                            bad_action, &observation));
  }

  // The three-joint document scaled and clipped one of its terms.
  {
    RobotState state = MakeState(3, 2000, 3);
    state.feedback.joints[0].q = 0.1;     // ankle_pitch
    state.feedback.joints[1].q = 0.2;     // knee
    state.feedback.joints[2].q = 0.3;     // hip_pitch
    state.feedback.joints[0].dq = 100.0;  // above the term's own clip of 30
    Observation observation;
    const DeploymentContract three_orders =
        irmv::policy::ParseDeployment(three_orders_text);
    const JointMapping three_mapping =
        irmv::policy::BuildJointMapping(three_orders, kRobotThree);
    std::array<float, irmv::protocol::kMaxJoints> last_action{};
    VelocityCommand command;
    CHECK(BuildObservation(three_orders, three_mapping, state, command,
                           last_action, &observation));
    CHECK(observation.count == 18);
    // base_ang_vel carries a scale of one, so it is unchanged.
    CHECK(NearlyEqual(observation.values[6], 0.0));
    // joint_vel_rel is clipped at +-30 before it is scaled. Its list is
    // (hip_pitch, ankle_pitch, knee), so the clipped ankle_pitch velocity is
    // the second component of that segment.
    CHECK(NearlyEqual(observation.values[12], 0.0));
    CHECK(NearlyEqual(observation.values[13], 30.0));
    CHECK(NearlyEqual(observation.values[14], 0.0));
    // joint_pos_rel is in the order (ankle_pitch, knee, hip_pitch) with the
    // references (-0.3, 0.6, -0.3).
    CHECK(NearlyEqual(observation.values[9], 0.1 - (-0.3)));
    CHECK(NearlyEqual(observation.values[10], 0.2 - 0.6));
    CHECK(NearlyEqual(observation.values[11], 0.3 - (-0.3)));
  }

  // ---------------------------------------------------------------------
  // Action: the frozen pipeline, then the scatter into the deployment order.
  // ---------------------------------------------------------------------
  {
    const DeploymentContract three_orders =
        irmv::policy::ParseDeployment(three_orders_text);
    const JointMapping mapping =
        irmv::policy::BuildJointMapping(three_orders, kRobotThree);
    CHECK(mapping.action == std::vector<std::uint32_t>({1, 2, 0}));

    ResolvedGains gains;
    PolicyConfig config = MakePolicyConfig();
    CHECK(ResolveGains(three_orders, mapping, kRobotThree, config,
                       "no-such-digest", &gains));
    CHECK(gains.joint_count == 3);
    CHECK(gains.override_count == 0);
    CHECK(NearlyEqual(gains.kp[0], 100.0));  // knee
    CHECK(NearlyEqual(gains.kp[2], 60.0));   // ankle_pitch

    RobotState state = MakeState(3, 3000, 9);
    const std::array<float, 3> action = {10.0f, -10.0f, 0.0f};
    JointCommand frame;
    CHECK(BuildCommand(three_orders, mapping, gains, action.data(),
                       action.size(), state, "fake_layout",
                       irmv::protocol::Id{}, 5, 11, &frame));
    CHECK(frame.joints.count == 3);
    CHECK(frame.epoch == 5);
    CHECK(frame.command_seq == 11);
    CHECK(frame.source_state_seq == 9);
    CHECK(frame.layout_id.View() == "fake_layout");
    // The wrapper clip of the document is +-2 and it is applied first: both
    // saturated actions land on the same scaled and offset position.
    // knee is action index 0 and deployment index 1.
    CHECK(NearlyEqual(frame.joints.q[1], 2.0 * 0.5 + 0.6));
    // hip_pitch is action index 1 and deployment index 2.
    CHECK(NearlyEqual(frame.joints.q[2], -2.0 * 0.5 + -0.3));
    // ankle_pitch is action index 2 and deployment index 0.
    CHECK(NearlyEqual(frame.joints.q[0], 0.0 * 0.5 + -0.3));
    // The velocity target is the model's own default velocity, the feed-forward
    // torque is zero and the gains follow the action joint.
    CHECK(NearlyEqual(frame.joints.dq[0], 0.0));
    CHECK(NearlyEqual(frame.joints.tau_ff[0], 0.0));
    CHECK(NearlyEqual(frame.joints.kp[0], 60.0));
    CHECK(NearlyEqual(frame.joints.kd[0], 4.0));
    CHECK(NearlyEqual(frame.joints.kp[1], 100.0));
    CHECK(NearlyEqual(frame.joints.kd[2], 6.0));
  }

  {
    // A frame with a wrong action count, a non-finite action or a state of
    // another joint count is refused whole.
    const DeploymentContract two_axis =
        irmv::policy::ParseDeployment(two_axis_text);
    const JointMapping mapping =
        irmv::policy::BuildJointMapping(two_axis, kRobotTwo);
    PolicyConfig config = MakePolicyConfig();
    ResolvedGains gains;
    CHECK(ResolveGains(two_axis, mapping, kRobotTwo, config, "", &gains));
    RobotState state = MakeState(2, 1000, 1);
    JointCommand frame;
    const std::array<float, 3> too_many = {0.0f, 0.0f, 0.0f};
    CHECK(!BuildCommand(two_axis, mapping, gains, too_many.data(),
                        too_many.size(), state, "layout", irmv::protocol::Id{},
                        1, 1, &frame));
    std::array<float, 2> non_finite = {std::numeric_limits<float>::quiet_NaN(),
                                       0.0f};
    CHECK(!BuildCommand(two_axis, mapping, gains, non_finite.data(),
                        non_finite.size(), state, "layout",
                        irmv::protocol::Id{}, 1, 1, &frame));
    RobotState other_count = MakeState(3, 1000, 1);
    const std::array<float, 2> action = {0.0f, 0.0f};
    CHECK(!BuildCommand(two_axis, mapping, gains, action.data(), action.size(),
                        other_count, "layout", irmv::protocol::Id{}, 1, 1,
                        &frame));
    // The layout text is a fixed capacity field; a longer one is refused rather
    // than truncated onto the wire.
    CHECK(!BuildCommand(two_axis, mapping, gains, action.data(), action.size(),
                        state, std::string(65, 'l'), irmv::protocol::Id{}, 1, 1,
                        &frame));
  }

  // ---------------------------------------------------------------------
  // Gains: the model's own values, and only the overrides of this model.
  // ---------------------------------------------------------------------
  {
    const DeploymentContract three_orders =
        irmv::policy::ParseDeployment(three_orders_text);
    const JointMapping mapping =
        irmv::policy::BuildJointMapping(three_orders, kRobotThree);
    PolicyConfig config = MakePolicyConfig();
    config.overrides.resize(2);
    config.overrides[0].model_sha256 = std::string(64, 'a');
    config.overrides[1].model_sha256 = std::string(64, 'b');
    config.overrides[0].joints.resize(1);
    config.overrides[0].joints[0].joint = "ankle_pitch";
    config.overrides[0].joints[0].overrides_kp = true;
    config.overrides[0].joints[0].kp = 42.0;
    config.overrides[1].joints.resize(1);
    config.overrides[1].joints[0].joint = "knee";
    config.overrides[1].joints[0].overrides_kd = true;
    config.overrides[1].joints[0].kd = 1.0;

    ResolvedGains matching;
    CHECK(ResolveGains(three_orders, mapping, kRobotThree, config,
                       std::string(64, 'a'), &matching));
    // ankle_pitch is action index 2.
    CHECK(NearlyEqual(matching.kp[2], 42.0));
    CHECK(NearlyEqual(matching.kd[2], 4.0));
    CHECK(NearlyEqual(matching.kp[0], 100.0));
    CHECK(matching.override_count == 1);
    CHECK(matching.overrides[0].joint.View() == "ankle_pitch");
    CHECK(matching.overrides[0].kp_overridden);
    CHECK(!matching.overrides[0].kd_overridden);
    CHECK(NearlyEqual(matching.overrides[0].model_kp, 60.0));
    CHECK(NearlyEqual(matching.overrides[0].applied_kp, 42.0));

    // A digest of another model is a different experiment: nothing applies.
    ResolvedGains other;
    CHECK(ResolveGains(three_orders, mapping, kRobotThree, config,
                       "no-such-digest", &other));
    CHECK(other.override_count == 0);
    CHECK(NearlyEqual(other.kp[2], 60.0));
    CHECK(NearlyEqual(other.kd[0], 6.0));

    // The override set of another model still applies when its digest matches.
    ResolvedGains second;
    CHECK(ResolveGains(three_orders, mapping, kRobotThree, config,
                       std::string(64, 'b'), &second));
    CHECK(second.override_count == 1);
    CHECK(NearlyEqual(second.kd[0], 1.0));
    CHECK(NearlyEqual(second.kp[0], 100.0));

    // An override that names a joint the model does not command describes
    // nothing that could ever be sent.
    PolicyConfig unknown = config;
    unknown.overrides.resize(1);
    unknown.overrides[0] = config.overrides[0];
    unknown.overrides[0].joints[0].joint = "hip_roll";
    ResolvedGains rejected;
    CHECK(!ResolveGains(three_orders, mapping, kRobotThree, unknown,
                        std::string(64, 'a'), &rejected));

    // A model that does not command every configured joint is refused instead
    // of producing a frame that leaves the rest at a stale target.
    ResolvedGains incomplete;
    const std::vector<std::string> four_joints = {"a", "b", "c", "d"};
    CHECK(!ResolveGains(three_orders, mapping, four_joints, MakePolicyConfig(),
                        "", &incomplete));
  }

  // ---------------------------------------------------------------------
  // last_action: the stage the contract names, and the values it starts from.
  // ---------------------------------------------------------------------
  {
    // The two stages differ by the whole pipeline, so one action feeds two
    // different observations. `raw` is the model's own output; `processed` is
    // the position the frame was built from, which is the contract's offset and
    // scale away from it.
    const std::array<float, irmv::protocol::kMaxJoints> action = {0.8f, -0.4f};
    std::array<float, irmv::protocol::kMaxJoints> stored{};

    CHECK(irmv::policy::LastActionFromAction(two_axis.action, action.data(), 2,
                                             &stored));
    CHECK(NearlyEqual(stored[0], 0.8));
    CHECK(NearlyEqual(stored[1], -0.4));

    // The processed element is the pipeline in the frozen order - scale, then
    // offset, then the (absent) clips - and it must be exactly what the frame
    // carries for the same action, or the observation would describe a step the
    // machine never made.
    double processed = 0.0;
    CHECK(irmv::policy::ProcessActionElement(two_axis.action, action.data(), 2,
                                             0, &processed));
    CHECK(NearlyEqual(processed, 0.8 * 0.5 + 0.1));
    CHECK(irmv::policy::ProcessActionElement(two_axis.action, action.data(), 2,
                                             1, &processed));
    CHECK(NearlyEqual(processed, -0.4 * 0.5 - 0.2));

    ResolvedGains gains;
    CHECK(ResolveGains(two_axis, two_axis_mapping, kRobotTwo,
                       MakePolicyConfig(), "", &gains));
    const RobotState state = MakeState(2, 1000, 4);
    JointCommand frame;
    CHECK(BuildCommand(two_axis, two_axis_mapping, gains, action.data(), 2,
                       state, "layout", irmv::protocol::kNoId, 1, 1, &frame));
    // The mapping scatters left into the deployment's second joint and right
    // into its first, so the comparison is by name: left is index 0 of the
    // action list and index 1 of the frame.
    CHECK(irmv::policy::ProcessActionElement(two_axis.action, action.data(), 2,
                                             0, &processed));
    CHECK(NearlyEqual(frame.joints.q[two_axis_mapping.action[0]], processed));
    CHECK(irmv::policy::ProcessActionElement(two_axis.action, action.data(), 2,
                                             1, &processed));
    CHECK(NearlyEqual(frame.joints.q[two_axis_mapping.action[1]], processed));

    // A count the contract does not describe is refused rather than partially
    // copied: a truncated last_action would feed the model a step it never
    // took.
    CHECK(!irmv::policy::LastActionFromAction(two_axis.action, action.data(), 1,
                                              &stored));
    CHECK(!irmv::policy::ProcessActionElement(two_axis.action, action.data(), 1,
                                              0, &processed));
    CHECK(!irmv::policy::ProcessActionElement(two_axis.action, action.data(), 2,
                                              2, &processed));

    // The `initial` of a contract is where an authorization starts, and it is
    // read from the contract instead of assumed: this document starts at a
    // non-zero position, so a reset that wrote zeros would feed the model an
    // observation the training environment never produced.
    std::array<float, irmv::protocol::kMaxJoints> reset{};
    reset.fill(9.0f);
    irmv::policy::ResetLastAction(two_axis.action, &reset);
    CHECK(NearlyEqual(reset[0], 0.0));
    CHECK(NearlyEqual(reset[1], 0.0));
    for (std::size_t index = 2; index < irmv::protocol::kMaxJoints; ++index) {
      CHECK(NearlyEqual(reset[index], 0.0));
    }

    const DeploymentContract processed_contract = irmv::policy::ParseDeployment(
        ReplaceOnce(ReplaceOnce(two_axis_text, "\"stage\": \"raw\"",
                                "\"stage\": \"processed\""),
                    "\"initial\": [0.0, 0.0]", "\"initial\": [0.5, -0.25]"));
    CHECK(processed_contract.action.last_action_stage ==
          irmv::policy::LastActionStage::Processed);
    irmv::policy::ResetLastAction(processed_contract.action, &reset);
    CHECK(NearlyEqual(reset[0], 0.5));
    CHECK(NearlyEqual(reset[1], -0.25));
    CHECK(irmv::policy::LastActionFromAction(processed_contract.action,
                                             action.data(), 2, &stored));
    CHECK(NearlyEqual(stored[0], 0.8 * 0.5 + 0.1));
    CHECK(NearlyEqual(stored[1], -0.4 * 0.5 - 0.2));
  }

  std::cout << "policy_math_test: ok\n";
  return 0;
}
