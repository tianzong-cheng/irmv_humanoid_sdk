// The SDK half of the training-side reference sample: the same state and the
// same velocity command as the training environment saw, pushed through this
// deployment's own code, compared element by element with the numbers the
// training side recorded.
//
//   verify_policy_sample <reference_sample.json> <policy.onnx> [tolerance]
//
// The reference document is written by
// `scripts/rsl_rl/reference_sample.py` in `irmv_humanoid_locomotion`; its shape
// is fixed by that script and is described in docs/testing.md. What this tool
// proves, and what no contract test can, is that the SDK reproduces the
// *numbers*: the observation vector the model consumes, the raw action it
// returns, and the joint position target the action pipeline produces.
//
// The comparison is deliberately done on the contract's own joint order as the
// deployment's table - the mapping onto the machine's joint table is the
// contract's job and is covered by its own tests - so every value here is
// compared per named joint and per element, with the maximum absolute deviation
// printed for each of the three vectors.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/configuration/application_config.hpp"
#include "irmv/policy/action.hpp"
#include "irmv/policy/deployment.hpp"
#include "irmv/policy/observation.hpp"
#include "irmv/policy/onnx_model.hpp"

namespace {

using Json = nlohmann::json;
using irmv::policy::BuildCommand;
using irmv::policy::BuildObservation;
using irmv::policy::DeploymentContract;
using irmv::policy::JointMapping;
using irmv::policy::OnnxModel;
using irmv::policy::ResolvedGains;
using irmv::policy::ResolveGains;
using irmv::protocol::JointCommand;
using irmv::protocol::RobotState;
using irmv::protocol::TimeUs;

// One sample's three comparisons: the largest absolute difference of each
// vector, and whether every element of it stayed inside the tolerance.
struct Comparison {
  double observation{0.0};
  double action{0.0};
  double target{0.0};
  bool ok{true};
};

std::vector<double> Doubles(const Json &array) {
  std::vector<double> values;
  values.reserve(array.size());
  for (const Json &value : array) {
    values.push_back(value.get<double>());
  }
  return values;
}

// One reference sample's state as this SDK's coherent snapshot: the joints in
// the contract's order, every sample valid, and the IMU the observation's
// gravity and angular-velocity terms are computed from.
RobotState MakeState(const Json &sample, std::size_t joint_count) {
  RobotState state{};
  state.state_seq = 1;
  state.stamp_us = 1000;
  state.feedback.count = static_cast<std::uint32_t>(joint_count);
  state.feedback.stamp_us = state.stamp_us;
  state.feedback.snapshot_seq = state.state_seq;
  state.feedback.imu.valid = true;
  const Json &imu = sample["state"]["imu"];
  for (std::size_t index = 0; index < 3; ++index) {
    state.feedback.imu.orientation_rpy[index] =
        imu["orientation_rpy"][index].get<double>();
    state.feedback.imu.angular_velocity[index] =
        imu["angular_velocity"][index].get<double>();
  }
  const Json &joints = sample["state"]["joints"];
  if (joints.size() != joint_count) {
    state.feedback.count = 0;
    return state;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    state.feedback.joints[index].valid = true;
    state.feedback.joints[index].connected = true;
    state.feedback.joints[index].q = joints[index]["q"].get<double>();
    state.feedback.joints[index].dq = joints[index]["dq"].get<double>();
    state.feedback.joints[index].rx_stamp_us = state.stamp_us;
  }
  return state;
}

// One sample: every vector the training side recorded, recomputed here.
bool CompareSample(const Json &sample, const OnnxModel &model,
                   const ResolvedGains &gains, std::string_view layout_id,
                   double tolerance, Comparison *out, std::string *error) {
  const DeploymentContract &contract = model.contract();
  const std::size_t joint_count = contract.action.joints.size();
  const RobotState state = MakeState(sample, joint_count);
  if (state.feedback.count != joint_count) {
    *error = "the sample does not carry one joint per contract joint";
    return false;
  }

  irmv::policy::VelocityCommand command;
  command.forward = sample["command"]["forward"].get<double>();
  command.lateral = sample["command"]["lateral"].get<double>();
  command.yaw = sample["command"]["yaw"].get<double>();

  std::array<float, irmv::protocol::kMaxJoints> last_action{};
  const Json &recorded_last = sample["last_action"];
  if (recorded_last.size() != joint_count) {
    *error = "the sample's last_action is not one value per contract joint";
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    last_action[index] = recorded_last[index].get<float>();
  }

  irmv::policy::Observation observation;
  if (!BuildObservation(contract, model.mapping(), state, command, last_action,
                        &observation)) {
    *error = "the SDK refused to build the observation for this state";
    return false;
  }
  const std::vector<double> expected_observation =
      Doubles(sample["expected"]["observation"]);
  if (expected_observation.size() != observation.count) {
    *error = "the recorded observation has another dimension than the model's";
    return false;
  }
  for (std::size_t index = 0; index < observation.count; ++index) {
    const double difference =
        std::fabs(static_cast<double>(observation.values[index]) -
                  expected_observation[index]);
    out->observation = std::max(out->observation, difference);
    out->ok = out->ok && difference <= tolerance;
  }

  std::array<float, irmv::protocol::kMaxJoints> action{};
  std::uint32_t action_count = 0;
  if (!model.Run(observation, &action, &action_count)) {
    *error = "the model did not answer for this observation";
    return false;
  }
  const std::vector<double> expected_action =
      Doubles(sample["expected"]["action"]);
  if (expected_action.size() != action_count) {
    *error = "the recorded action has another dimension than the model's";
    return false;
  }
  for (std::uint32_t index = 0; index < action_count; ++index) {
    const double difference =
        std::fabs(static_cast<double>(action[index]) - expected_action[index]);
    out->action = std::max(out->action, difference);
    out->ok = out->ok && difference <= tolerance;
  }

  JointCommand frame;
  if (!BuildCommand(contract, model.mapping(), gains, action.data(),
                    action_count, state, layout_id, irmv::protocol::kNoId, 1, 1,
                    &frame)) {
    *error = "the SDK refused to build the frame for this action";
    return false;
  }
  const std::vector<double> expected_target =
      Doubles(sample["expected"]["target_q"]);
  if (expected_target.size() != joint_count) {
    *error = "the recorded target is not one value per contract joint";
    return false;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    // The frame is indexed by the deployment's own order, and this comparison
    // uses the contract's order as that table, so the index is the joint.
    const double difference =
        std::fabs(frame.joints.q[index] - expected_target[index]);
    out->target = std::max(out->target, difference);
    out->ok = out->ok && difference <= tolerance;
  }
  return true;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 3 || argc > 4) {
    std::fprintf(stderr,
                 "usage: verify_policy_sample <reference_sample.json> "
                 "<policy.onnx> [tolerance]\n");
    return 2;
  }
  const double tolerance = argc == 4 ? std::stod(argv[3]) : 1e-5;

  std::ifstream document_file(argv[1]);
  if (!document_file) {
    std::fprintf(stderr, "verify_policy_sample: cannot open %s\n", argv[1]);
    return 2;
  }
  Json document = Json::parse(document_file, nullptr, false);
  if (document.is_discarded() || !document.is_object()) {
    std::fprintf(stderr, "verify_policy_sample: %s is not a JSON object\n",
                 argv[1]);
    return 2;
  }

  // The deployment's joint table for this comparison is the contract's own
  // action order, which the document states: the mapping onto a machine's own
  // table is the contract's subject and is checked by its tests, while what
  // this tool compares is the numbers.
  std::vector<std::string> robot_joints;
  const Json &joints = document.value("joints", Json::array());
  if (!joints.is_array() || joints.empty()) {
    std::fprintf(stderr,
                 "verify_policy_sample: the document names no joints\n");
    return 2;
  }
  for (const Json &joint : joints) {
    robot_joints.push_back(joint.get<std::string>());
  }

  std::unique_ptr<OnnxModel> model;
  try {
    model = OnnxModel::Load(argv[2], robot_joints);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "verify_policy_sample: %s\n", error.what());
    return 1;
  }
  const JointMapping mapping =
      irmv::policy::BuildJointMapping(model->contract(), robot_joints);
  ResolvedGains gains;
  const irmv::configuration::PolicyConfig empty_config;
  if (!ResolveGains(model->contract(), mapping, robot_joints, empty_config, "",
                    &gains)) {
    std::fprintf(stderr,
                 "verify_policy_sample: the gains cannot be resolved against "
                 "the contract's own joint table\n");
    return 1;
  }

  const std::string model_sha256 = model->sha256();
  const std::string recorded_sha256 =
      document.value("model", Json::object()).value("sha256", std::string());
  int failures = 0;
  if (!recorded_sha256.empty() && recorded_sha256 != model_sha256) {
    std::fprintf(stderr,
                 "verify_policy_sample: the document describes model %s but "
                 "was given %s\n",
                 recorded_sha256.c_str(), model_sha256.c_str());
    ++failures;
  }

  const Json &samples = document.value("samples", Json::array());
  if (!samples.is_array() || samples.empty()) {
    std::fprintf(stderr,
                 "verify_policy_sample: the document holds no sample\n");
    return 1;
  }
  std::printf("verify_policy_sample: model=%s samples=%zu tolerance=%g\n",
              model_sha256.c_str(), samples.size(), tolerance);
  // The tree the sample came from, printed with the numbers it proves: a
  // document generated while that tree was being edited records the hash of its
  // own diff, and the commit alone would name the wrong revision.
  const Json &training_meta = document.value("training", Json::object());
  std::printf(
      "verify_policy_sample: training task=%s seed=%lld commit=%s dirty=%s "
      "diff=%s checkpoint=%s\n",
      training_meta.value("task", std::string("unknown")).c_str(),
      static_cast<long long>(training_meta.value("seed", 0)),
      training_meta.value("training_repo_commit", std::string("unknown"))
          .c_str(),
      training_meta.value("training_repo_dirty", false) ? "true" : "false",
      training_meta.value("training_repo_diff_sha256", std::string("-"))
          .c_str(),
      training_meta.value("checkpoint", std::string("unknown")).c_str());
  for (const Json &sample : samples) {
    const std::string name = sample.value("name", std::string("?"));
    Comparison comparison;
    std::string error;
    if (!CompareSample(sample, *model, gains, "reference-sample", tolerance,
                       &comparison, &error)) {
      std::fprintf(stderr, "verify_policy_sample: %s: %s\n", name.c_str(),
                   error.c_str());
      ++failures;
      continue;
    }
    std::printf(
        "verify_policy_sample: %-14s observation=%.3e action=%.3e target=%.3e "
        "%s\n",
        name.c_str(), comparison.observation, comparison.action,
        comparison.target, comparison.ok ? "ok" : "OUT OF TOLERANCE");
    if (!comparison.ok) {
      ++failures;
    }
  }
  if (failures != 0) {
    std::fprintf(stderr, "verify_policy_sample: %d comparison(s) failed\n",
                 failures);
    return 1;
  }
  std::printf("verify_policy_sample: every vector matches within %g\n",
              tolerance);
  return 0;
}
