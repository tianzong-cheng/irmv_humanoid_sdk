// The deployment contract of a policy ONNX: the two fixture documents that must
// parse, the joint mappings they must produce, and the mutations that must be
// refused with the JSON field path of the offending value.
//
// The fixtures are files, not strings, so the offline export test of the
// training side can validate the very same documents against the same schema.
#include "irmv/policy/deployment.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "support/check.hpp"

namespace {

using json = nlohmann::json;

using irmv::policy::ActionStage;
using irmv::policy::DeploymentContract;
using irmv::policy::DeploymentError;
using irmv::policy::JointMapping;
using irmv::policy::LastActionStage;
using irmv::policy::NormalizerLocation;
using irmv::policy::ObservationTermKind;

constexpr const char *kTwoAxisPath =
    "tests/support/deployment/valid_two_axis.json";
constexpr const char *kThreeOrdersPath =
    "tests/support/deployment/valid_three_orders.json";

// The deployment joint tables the fixtures are resolved against. They are not
// the fixtures' vectors: the names are what the mapping is checked on, and the
// table order is deliberately different from every model list.
const std::vector<std::string> kRobotTwo = {"right", "left", "spare"};
const std::vector<std::string> kRobotThree = {"ankle_pitch", "knee",
                                              "hip_pitch", "hip_roll"};
// Same table without one of the model's joints.
const std::vector<std::string> kRobotTwoIncomplete = {"right", "spare"};

// One inclusive clip range. It is built field by field on purpose: a JSON
// initializer list of two strings would be read as an object instead.
json ClipRange(double min, double max) {
  json range = json::object();
  range["min"] = min;
  range["max"] = max;
  return range;
}

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

// Replaces the first occurrence of `from`. It is how the non-finite and
// truncated cases are built: the document stays a real document, and only the
// one token under test changes.
std::string ReplaceFirst(std::string text, const std::string &from,
                         const std::string &to) {
  const std::size_t position = text.find(from);
  if (position == std::string::npos) {
    std::cerr << "fixture does not contain " << from << '\n';
    return {};
  }
  return text.replace(position, from.size(), to);
}

// True when the document is refused with exactly `path`. The parameter is the
// JSON field path of the first violation, not a substring of it.
bool RefusedAt(const std::string &text, const std::string &path,
               const std::string &note) {
  try {
    const DeploymentContract contract = irmv::policy::ParseDeployment(text);
    (void)contract;
  } catch (const DeploymentError &error) {
    if (error.field_path() != path) {
      std::cerr << note << ": expected field path \"" << path << "\", got \""
                << error.field_path() << "\" (" << error.what() << ")\n";
      return false;
    }
    return true;
  }
  std::cerr << note << ": expected a DeploymentError\n";
  return false;
}

bool RefusedToMap(const DeploymentContract &contract,
                  const std::vector<std::string> &robot_joints,
                  const std::string &path, const std::string &note) {
  try {
    const JointMapping mapping =
        irmv::policy::BuildJointMapping(contract, robot_joints);
    (void)mapping;
  } catch (const DeploymentError &error) {
    if (error.field_path() != path) {
      std::cerr << note << ": expected field path \"" << path << "\", got \""
                << error.field_path() << "\" (" << error.what() << ")\n";
      return false;
    }
    return true;
  }
  std::cerr << note << ": expected a DeploymentError\n";
  return false;
}

}  // namespace

int main() {
  const std::string two_axis_text = ReadFile(kTwoAxisPath);
  const std::string three_orders_text = ReadFile(kThreeOrdersPath);
  CHECK(!two_axis_text.empty());
  CHECK(!three_orders_text.empty());

  // -------------------------------------------------------------------------
  // The minimal two-joint document: every array aligned with its joint list,
  // both clip layers absent and the pure feed-forward gains of one joint.
  // -------------------------------------------------------------------------
  const DeploymentContract two_axis =
      irmv::policy::ParseDeployment(two_axis_text);
  CHECK(two_axis.schema_version == 1);
  CHECK(two_axis.policy_type == "joint_position");
  CHECK(two_axis.input.name == "obs");
  CHECK(two_axis.input.dimension == 15);
  CHECK(two_axis.output.name == "actions");
  CHECK(two_axis.output.dimension == 2);
  CHECK(two_axis.policy_period_us == 20000);

  CHECK(two_axis.action.joints == std::vector<std::string>({"left", "right"}));
  CHECK(two_axis.action.source_joints ==
        std::vector<std::string>({"left_joint", "right_joint"}));
  CHECK(two_axis.action.default_q == std::vector<double>({0.1, -0.2}));
  CHECK(two_axis.action.default_dq == std::vector<double>({0.0, 0.0}));
  // A zero kp/kd pair is legal: a feed-forward command carries no PD term.
  CHECK(two_axis.action.kp == std::vector<double>({0.0, 100.0}));
  CHECK(two_axis.action.kd == std::vector<double>({0.0, 6.0}));
  CHECK(two_axis.action.scale == std::vector<double>({0.5, 0.5}));
  CHECK(two_axis.action.offset == std::vector<double>({0.1, -0.2}));
  CHECK(!two_axis.action.wrapper_clip);
  CHECK(!two_axis.action.term_clip);
  CHECK(two_axis.action.wrapper_clip_ranges.empty());
  CHECK(two_axis.action.order.size() == 4);
  CHECK(two_axis.action.order[0] == ActionStage::WrapperClip);
  CHECK(two_axis.action.order[1] == ActionStage::Scale);
  CHECK(two_axis.action.order[2] == ActionStage::Offset);
  CHECK(two_axis.action.order[3] == ActionStage::TermClip);
  CHECK(two_axis.action.last_action_stage == LastActionStage::Raw);
  CHECK(two_axis.action.last_action_initial == std::vector<double>({0.0, 0.0}));

  CHECK(two_axis.observation.dimension == 15);
  CHECK(two_axis.observation.terms.size() == 6);
  CHECK(two_axis.observation.terms[0].kind == ObservationTermKind::Command);
  CHECK(two_axis.observation.terms[0].dimension == 3);
  CHECK(two_axis.observation.terms[0].unit == "m/s,m/s,rad/s");
  CHECK(two_axis.observation.terms[0].frame == "body");
  CHECK(!two_axis.observation.terms[0].scaled);
  CHECK(!two_axis.observation.terms[0].clipped);
  CHECK(two_axis.observation.terms[0].history_length == 0);
  CHECK(two_axis.observation.terms[1].kind ==
        ObservationTermKind::ProjectedGravity);
  CHECK(two_axis.observation.terms[1].unit == "1");
  CHECK(two_axis.observation.terms[1].frame == "body");
  CHECK(two_axis.observation.terms[2].kind ==
        ObservationTermKind::BaseAngularVelocity);
  CHECK(two_axis.observation.terms[2].unit == "rad/s");
  CHECK(two_axis.observation.terms[3].kind ==
        ObservationTermKind::JointPosition);
  CHECK(two_axis.observation.terms[3].dimension == 2);
  CHECK(two_axis.observation.terms[4].kind ==
        ObservationTermKind::JointVelocity);
  CHECK(two_axis.observation.terms[4].frame == "joint");
  CHECK(two_axis.observation.terms[5].kind == ObservationTermKind::LastAction);
  CHECK(two_axis.observation.terms[5].unit == "1");
  CHECK(two_axis.observation.terms[5].frame == "none");

  // The position and the velocity list are their own order, and neither is the
  // action's: the mapping proves the three are resolved independently.
  CHECK(two_axis.observation.joint_pos_rel.term == "joint_pos_rel");
  CHECK(two_axis.observation.joint_pos_rel.unit == "rad");
  CHECK(two_axis.observation.joint_pos_rel.joints ==
        std::vector<std::string>({"right", "left"}));
  CHECK(two_axis.observation.joint_pos_rel.reference ==
        std::vector<double>({-0.2, 0.1}));
  CHECK(two_axis.observation.joint_vel_rel.unit == "rad/s");
  CHECK(two_axis.observation.joint_vel_rel.joints ==
        std::vector<std::string>({"left", "right"}));
  CHECK(two_axis.observation.joint_vel_rel.reference ==
        std::vector<double>({0.0, 0.0}));

  CHECK(!two_axis.normalizer.enabled);
  CHECK(!two_axis.normalizer.embedded);
  CHECK(two_axis.normalizer.location == NormalizerLocation::None);
  CHECK(two_axis.normalizer.algorithm.empty());
  CHECK(two_axis.normalizer.mean.empty());

  CHECK(two_axis.source.checkpoint_file.has_value());
  CHECK(*two_axis.source.checkpoint_file == "model_2999.pt");
  CHECK(two_axis.source.checkpoint_sha256.has_value());
  CHECK(two_axis.source.checkpoint_sha256->size() == 64);
  CHECK(two_axis.source.missing.empty());
  CHECK(two_axis.source.software.size() == 5);

  const JointMapping two_axis_mapping =
      irmv::policy::BuildJointMapping(two_axis, kRobotTwo);
  CHECK(two_axis_mapping.action == std::vector<std::uint32_t>({1, 0}));
  CHECK(two_axis_mapping.joint_pos == std::vector<std::uint32_t>({0, 1}));
  CHECK(two_axis_mapping.joint_vel == std::vector<std::uint32_t>({1, 0}));

  // -------------------------------------------------------------------------
  // The three-joint document: three joint lists in three different orders, two
  // non-null clip layers and a scaled and a clipped observation term.
  // -------------------------------------------------------------------------
  const DeploymentContract three_orders =
      irmv::policy::ParseDeployment(three_orders_text);
  CHECK(three_orders.action.joints ==
        std::vector<std::string>({"knee", "hip_pitch", "ankle_pitch"}));
  CHECK(three_orders.action.kp == std::vector<double>({100.0, 100.0, 60.0}));
  CHECK(three_orders.action.kd == std::vector<double>({6.0, 6.0, 4.0}));
  CHECK(three_orders.action.wrapper_clip);
  CHECK(three_orders.action.wrapper_clip_ranges.size() == 3);
  CHECK(three_orders.action.wrapper_clip_ranges[0].min == -2.0);
  CHECK(three_orders.action.wrapper_clip_ranges[0].max == 2.0);
  CHECK(!three_orders.action.term_clip);
  CHECK(three_orders.observation.terms[2].scaled);
  CHECK(three_orders.observation.terms[2].scale ==
        std::vector<double>({1.0, 1.0, 1.0}));
  CHECK(three_orders.observation.terms[4].clipped);
  CHECK(three_orders.observation.terms[4].clip[0].min == -30.0);
  CHECK(three_orders.observation.terms[4].clip[0].max == 30.0);
  CHECK(!three_orders.source.training_commit.has_value());
  CHECK(three_orders.source.missing ==
        std::vector<std::string>({"training_commit"}));

  const JointMapping three_orders_mapping =
      irmv::policy::BuildJointMapping(three_orders, kRobotThree);
  CHECK(three_orders_mapping.action == std::vector<std::uint32_t>({1, 2, 0}));
  CHECK(three_orders_mapping.joint_pos ==
        std::vector<std::uint32_t>({0, 1, 2}));
  CHECK(three_orders_mapping.joint_vel ==
        std::vector<std::uint32_t>({2, 0, 1}));

  // A model joint the deployment does not have is refused by name: a partial
  // mapping would leave that joint at a stale target.
  const std::string missing_note = "missing deployment joint";
  CHECK(RefusedToMap(two_axis, kRobotTwoIncomplete, "action.joints[0]",
                     missing_note));
  CHECK(RefusedToMap(three_orders,
                     std::vector<std::string>({"knee", "hip_pitch"}),
                     "action.joints[2]", missing_note));

  // -------------------------------------------------------------------------
  // The normalizer: the disabled form is the only one the fixtures carry, and
  // the three enabled forms are built from the minimal document.
  // -------------------------------------------------------------------------
  {
    json document = json::parse(two_axis_text);
    document["normalizer"] = json::object();
    document["normalizer"]["enabled"] = true;
    document["normalizer"]["embedded"] = true;
    document["normalizer"]["location"] = "model";
    document["normalizer"]["algorithm"] = "mean_std";
    const DeploymentContract contract =
        irmv::policy::ParseDeployment(document.dump());
    CHECK(contract.normalizer.enabled);
    CHECK(contract.normalizer.embedded);
    CHECK(contract.normalizer.location == NormalizerLocation::Model);
    CHECK(contract.normalizer.algorithm == "mean_std");
  }
  {
    json document = json::parse(two_axis_text);
    document["normalizer"] = json::object();
    document["normalizer"]["enabled"] = true;
    document["normalizer"]["embedded"] = false;
    document["normalizer"]["location"] = "application";
    document["normalizer"]["algorithm"] = "mean_std";
    document["normalizer"]["statistics"]["count"] = 4096;
    document["normalizer"]["statistics"]["mean"] = std::vector<double>(15, 0.0);
    document["normalizer"]["statistics"]["std"] = std::vector<double>(15, 1.0);
    const DeploymentContract contract =
        irmv::policy::ParseDeployment(document.dump());
    CHECK(contract.normalizer.location == NormalizerLocation::Application);
    CHECK(contract.normalizer.count == 4096);
    CHECK(contract.normalizer.mean.size() == 15);
    CHECK(contract.normalizer.stddev.size() == 15);

    const std::string note = "non-positive standard deviation";
    json zero_std = document;
    zero_std["normalizer"]["statistics"]["std"][0] = 0.0;
    CHECK(RefusedAt(zero_std.dump(), "normalizer.statistics.std[0]", note));
    zero_std["normalizer"]["statistics"]["std"][0] = -1.0;
    CHECK(RefusedAt(zero_std.dump(), "normalizer.statistics.std[0]", note));

    json wrong_mean = document;
    wrong_mean["normalizer"]["statistics"]["mean"] =
        std::vector<double>(14, 0.0);
    CHECK(RefusedAt(wrong_mean.dump(), "normalizer.statistics.mean",
                    "statistics length"));

    json no_statistics = document;
    no_statistics["normalizer"].erase("statistics");
    CHECK(RefusedAt(no_statistics.dump(), "normalizer.statistics",
                    "enabled normalizer without statistics"));

    json embedded_statistics = document;
    embedded_statistics["normalizer"]["embedded"] = true;
    embedded_statistics["normalizer"]["location"] = "model";
    CHECK(RefusedAt(embedded_statistics.dump(), "normalizer.statistics",
                    "embedded normalizer repeating statistics"));

    json unknown_algorithm = document;
    unknown_algorithm["normalizer"]["algorithm"] = "min_max";
    CHECK(RefusedAt(unknown_algorithm.dump(), "normalizer.algorithm",
                    "unsupported normalizer algorithm"));

    json no_algorithm = document;
    no_algorithm["normalizer"].erase("algorithm");
    CHECK(RefusedAt(no_algorithm.dump(), "normalizer.algorithm",
                    "enabled normalizer without algorithm"));

    json wrong_location = document;
    wrong_location["normalizer"]["location"] = "none";
    CHECK(RefusedAt(wrong_location.dump(), "normalizer.location",
                    "enabled normalizer at location none"));

    json disabled_with_statistics = json::parse(two_axis_text);
    disabled_with_statistics["normalizer"]["statistics"]["count"] = 1;
    CHECK(RefusedAt(disabled_with_statistics.dump(), "normalizer",
                    "disabled normalizer carrying statistics"));
  }

  // -------------------------------------------------------------------------
  // Rejected documents. Every case names the field path of its first
  // violation, so a rejection that moved to another field is a failure too.
  // -------------------------------------------------------------------------
  const json base = json::parse(two_axis_text);
  const auto reject = [&base](const std::string &note, const std::string &path,
                              const std::function<void(json &)> &mutate) {
    json document = base;
    mutate(document);
    return RefusedAt(document.dump(), path, note);
  };

  CHECK(reject("missing root field", "policy_type",
               [](json &d) { d.erase("policy_type"); }));
  CHECK(reject("unknown root field", "extra", [](json &d) { d["extra"] = 1; }));
  CHECK(reject("schema version 2", "schema_version",
               [](json &d) { d["schema_version"] = 2; }));
  CHECK(reject("schema version missing", "schema_version",
               [](json &d) { d.erase("schema_version"); }));
  CHECK(reject("unsupported policy type", "policy_type",
               [](json &d) { d["policy_type"] = "joint_torque"; }));
  CHECK(reject("policy period zero", "policy_period_us",
               [](json &d) { d["policy_period_us"] = 0; }));
  CHECK(reject("policy period not an integer", "policy_period_us",
               [](json &d) { d["policy_period_us"] = 20000.5; }));
  CHECK(reject("unknown model field", "model.extra",
               [](json &d) { d["model"]["extra"] = 1; }));
  CHECK(reject("missing model input", "model.input",
               [](json &d) { d["model"].erase("input"); }));
  CHECK(reject("input dimension mismatch", "model.input.dimension",
               [](json &d) { d["model"]["input"]["dimension"] = 14; }));
  CHECK(reject("output dimension mismatch", "model.output.dimension",
               [](json &d) { d["model"]["output"]["dimension"] = 3; }));
  CHECK(reject("missing action field", "action.kp",
               [](json &d) { d["action"].erase("kp"); }));
  CHECK(reject("gain array too short", "action.kd",
               [](json &d) { d["action"]["kd"] = json::array({1.0}); }));
  CHECK(reject("negative gain", "action.kp[0]",
               [](json &d) { d["action"]["kp"][0] = -1.0; }));
  CHECK(reject("duplicate joint name", "action.joints[1]",
               [](json &d) { d["action"]["joints"][1] = "left"; }));
  CHECK(
      reject("source joint count differs", "action.source_joints", [](json &d) {
        d["action"]["source_joints"] = json::array({"left_joint"});
      }));
  CHECK(reject("joint name too long", "action.joints[0]", [](json &d) {
    d["action"]["joints"][0] = std::string(65, 'a');
  }));
  CHECK(reject("action clip length", "action.clip.term", [](json &d) {
    d["action"]["clip"]["term"] = json::array({ClipRange(-1.0, 1.0)});
  }));
  CHECK(reject("action clip min above max", "action.clip.wrapper[0]",
               [](json &d) {
                 d["action"]["clip"]["wrapper"] =
                     json::array({ClipRange(2.0, -2.0), ClipRange(-2.0, 2.0)});
               }));
  CHECK(reject("unknown clip field", "action.clip.wrapper[0].low", [](json &d) {
    json range = ClipRange(-1.0, 1.0);
    range["low"] = 0.0;
    d["action"]["clip"]["wrapper"] = json::array({range, ClipRange(-1.0, 1.0)});
  }));
  CHECK(reject("reordered action pipeline", "action.order[0]", [](json &d) {
    d["action"]["order"] =
        json::array({"scale", "wrapper_clip", "offset", "term_clip"});
  }));
  CHECK(reject("unknown action stage", "action.order[3]", [](json &d) {
    d["action"]["order"] =
        json::array({"wrapper_clip", "scale", "offset", "clamp"});
  }));
  CHECK(
      reject("unknown last_action stage", "action.last_action.stage",
             [](json &d) { d["action"]["last_action"]["stage"] = "clipped"; }));
  CHECK(reject(
      "last_action initial length", "action.last_action.initial", [](json &d) {
        d["action"]["last_action"]["initial"] = json::array({0.0, 0.0, 0.0});
      }));
  CHECK(reject("observation term count", "observation.terms", [](json &d) {
    d["observation"]["terms"].erase(d["observation"]["terms"].begin());
  }));
  CHECK(reject(
      "observation term order", "observation.terms[0].name", [](json &d) {
        std::swap(d["observation"]["terms"][0], d["observation"]["terms"][1]);
      }));
  CHECK(
      reject("unknown observation term", "observation.terms[0].name",
             [](json &d) { d["observation"]["terms"][0]["name"] = "height"; }));
  CHECK(
      reject("unsupported observation unit", "observation.terms[2].unit",
             [](json &d) { d["observation"]["terms"][2]["unit"] = "deg/s"; }));
  CHECK(
      reject("unsupported observation frame", "observation.terms[1].frame",
             [](json &d) { d["observation"]["terms"][1]["frame"] = "world"; }));
  CHECK(reject("term dimension mismatch", "observation.terms[3].dimension",
               [](json &d) { d["observation"]["terms"][3]["dimension"] = 3; }));
  CHECK(reject(
      "stacked term history", "observation.terms[0].history_length",
      [](json &d) { d["observation"]["terms"][0]["history_length"] = 5; }));
  CHECK(reject("term scale length", "observation.terms[2].scale", [](json &d) {
    d["observation"]["terms"][2]["scale"] = json::array({1.0});
  }));
  CHECK(reject("joint reference length", "observation.joint_pos_rel.reference",
               [](json &d) {
                 d["observation"]["joint_pos_rel"]["reference"] =
                     json::array({0.0});
               }));
  CHECK(reject("joint reference term name", "observation.joint_vel_rel.term",
               [](json &d) {
                 d["observation"]["joint_vel_rel"]["term"] = "joint_pos_rel";
               }));
  CHECK(reject("unknown observation field", "observation.extra",
               [](json &d) { d["observation"]["extra"] = 1; }));
  CHECK(reject("unexplained null provenance", "source.training_commit",
               [](json &d) { d["source"]["training_commit"] = nullptr; }));
  CHECK(reject("missing entry for a present field", "source.missing",
               [](json &d) {
                 d["source"]["missing"] = json::array({"training_repo"});
               }));
  CHECK(reject("unknown missing entry", "source.missing",
               [](json &d) { d["source"]["missing"] = json::array({"foo"}); }));
  CHECK(reject("duplicate missing entry", "source.missing[1]", [](json &d) {
    d["source"]["training_commit"] = nullptr;
    d["source"]["missing"] =
        json::array({"training_commit", "training_commit"});
  }));
  CHECK(reject("malformed digest", "source.checkpoint_sha256", [](json &d) {
    d["source"]["checkpoint_sha256"] = std::string(64, 'Z');
  }));
  CHECK(reject("short digest", "source.checkpoint_sha256",
               [](json &d) { d["source"]["checkpoint_sha256"] = "deadbeef"; }));
  CHECK(reject("unknown source field", "source.extra",
               [](json &d) { d["source"]["extra"] = 1; }));
  CHECK(reject("software version not a string", "source.software.torch",
               [](json &d) { d["source"]["software"]["torch"] = 2.7; }));
  CHECK(reject("missing software block", "source.software",
               [](json &d) { d["source"].erase("software"); }));

  // Non-finite and unreadable documents are refused whole before any field is
  // interpreted: the reference JSON grammar has the NaN/Infinity tokens and
  // numbers that overflow the double range, and neither can reach a contract
  // field.
  const std::string nan_note = "NaN token";
  CHECK(RefusedAt(ReplaceFirst(two_axis_text, "0.5", "NaN"), "", nan_note));
  CHECK(RefusedAt(ReplaceFirst(two_axis_text, "100.0", "Infinity"), "",
                  "Infinity token"));
  CHECK(RefusedAt(ReplaceFirst(two_axis_text, "0.1", "-Infinity"), "",
                  "negative Infinity token"));
  CHECK(RefusedAt(ReplaceFirst(two_axis_text, "100.0", "1e400"), "",
                  "overflowing number"));
  CHECK(RefusedAt("{ not json", "", "truncated document"));
  CHECK(RefusedAt("", "", "empty document"));
  CHECK(RefusedAt("[]", "", "document that is not an object"));

  std::cout << "deployment_test: ok\n";
  return 0;
}
