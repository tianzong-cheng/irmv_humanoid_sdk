#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/protocol/types.hpp"

// The deployment contract a policy ONNX carries in its single model-level
// metadata key `irmv.deployment`: what the model consumes, what it produces and
// the exact transformation the deployment layer performs on both ends.
//
// The document is descriptive and never changes the graph. The training side
// writes it from the instantiated environment (never from a configuration file
// or a regex), the SDK validates it here and implements the transformation from
// it, and neither side infers a value the other did not state. Every array
// names the joints it belongs to, so no mapping ever depends on the iteration
// order of a dictionary or on the accidental alignment of two arrays.
//
// The single source of the deployed values is the model; the single source of
// the deployment's own facts (hardware joint table, wiring, limits,
// initialization gains, fallback damping) is config/sdk.json. Nothing here
// overrides a runtime safety boundary.
namespace irmv::policy {

// The one metadata key the SDK reads. A model without it, with a duplicated one
// or with a value that is not this contract is refused before anything is
// loaded from it.
inline constexpr std::string_view kDeploymentMetadataKey = "irmv.deployment";

// The only schema version and policy type this build implements.
inline constexpr std::uint32_t kDeploymentSchemaVersion = 1;
inline constexpr std::string_view kJointPositionPolicyType = "joint_position";

// A contract violation. `field_path` locates the offending JSON, in the dotted
// form the configuration parser uses (`action.joints[3]`, `source.missing`).
class DeploymentError : public std::runtime_error {
 public:
  DeploymentError(std::string field_path, std::string message);

  const std::string &field_path() const noexcept { return field_path_; }

 private:
  std::string field_path_;
};

// One inclusive clip range of one component. `min <= max` holds; the two bounds
// are finite.
struct ClipRange {
  double min{0.0};
  double max{0.0};
};

// One stage of the frozen action pipeline. The pipeline is the training
// environment's own order - the runner's raw-action clip, then the action
// term's `raw * scale + offset`, then the term clip - and the contract states
// it explicitly instead of assuming it.
enum class ActionStage {
  WrapperClip,
  Scale,
  Offset,
  TermClip,
};

// Canonical token of a stage, spelled like the contract's `order` entries.
const char *ActionStageName(ActionStage stage) noexcept;

// Which action the `last_action` observation holds: the raw policy output the
// environment received, or the processed q target the action term produced.
// The recorded training observes the raw action.
enum class LastActionStage {
  Raw,
  Processed,
};

// The action half of the contract: the ordered joint vector and every constant
// the pipeline needs, each array aligned with `joints` element-wise.
struct ActionContract {
  // Deployment joint names, in the order of the model output. Resolved by exact
  // name against the deployment's joint table; never by position.
  std::vector<std::string> joints;
  // The names the training asset resolved for `joints`, element-wise aligned.
  // They are what makes the training-to-deployment mapping checkable.
  std::vector<std::string> source_joints;
  std::vector<double> default_q;   // rad
  std::vector<double> default_dq;  // rad/s
  std::vector<double> kp;          // N*m/rad, finite and non-negative
  std::vector<double> kd;          // N*m*s/rad, finite and non-negative
  std::vector<double> scale;       // finite
  std::vector<double> offset;      // rad, finite
  bool wrapper_clip{false};
  std::vector<ClipRange> wrapper_clip_ranges;  // size == joints, when present
  bool term_clip{false};
  std::vector<ClipRange> term_clip_ranges;  // size == joints, when present
  std::vector<ActionStage> order;           // the frozen four stages, in order
  LastActionStage last_action_stage{LastActionStage::Raw};
  std::vector<double> last_action_initial;  // size == joints

  std::size_t size() const noexcept { return joints.size(); }
};

// The six supported observation terms of the policy group. The parser accepts
// exactly these, in exactly this order; anything else is a contract violation,
// because this phase implements these six and nothing else.
enum class ObservationTermKind {
  Command,
  ProjectedGravity,
  BaseAngularVelocity,
  JointPosition,
  JointVelocity,
  LastAction,
};

// One term of the concatenated policy observation. `unit` and `frame` are
// pinned to the frozen values of the term's kind: they are carried in the
// document so the deployment never has to guess what a term means, and checked
// so a document cannot claim a meaning this build does not implement.
struct ObservationTerm {
  ObservationTermKind kind{ObservationTermKind::Command};
  std::string name;
  std::uint32_t dimension{0};
  std::string unit;
  std::string frame;
  bool scaled{false};
  std::vector<double> scale;  // size == dimension, when scaled
  bool clipped{false};
  std::vector<ClipRange> clip;  // size == dimension, when clipped
  std::uint32_t history_length{0};
};

// The ordered joint list of a joint observation term and the reference it
// subtracts.
struct JointReference {
  std::string term;  // "joint_pos_rel" or "joint_vel_rel"
  std::vector<std::string> joints;
  std::vector<std::string> source_joints;
  std::vector<double> reference;  // rad or rad/s, size == joints
  std::string unit;
};

struct ObservationContract {
  std::vector<ObservationTerm> terms;  // concatenation order
  JointReference joint_pos_rel;
  JointReference joint_vel_rel;
  std::uint32_t dimension{0};  // sum of the term dimensions
};

// Whether the trained actor normalizes its observation, and if so, where.
enum class NormalizerLocation {
  None,         // disabled: the SDK applies nothing
  Model,        // embedded in the graph: the SDK must not normalize again
  Application,  // the SDK applies the statistics below
};

// Component-wise `(x - mean) / std` over the concatenated observation.
struct NormalizerContract {
  bool enabled{false};
  bool embedded{false};
  NormalizerLocation location{NormalizerLocation::None};
  std::string algorithm;  // "mean_std" when enabled, empty otherwise
  std::uint32_t count{0};
  std::vector<double> mean;
  std::vector<double> stddev;
};

// One named tensor of the model interface.
struct ModelInterface {
  std::string name;
  std::uint32_t dimension{0};
};

// What produced the model. Every field is optional because a package may lack
// a fact, and a missing fact must be listed in `missing` instead of being
// filled with a plausible value.
struct SourceContract {
  std::optional<std::string> checkpoint_file;
  std::optional<std::string> checkpoint_sha256;
  std::optional<std::string> env_config_file;
  std::optional<std::string> env_config_sha256;
  std::optional<std::string> agent_config_file;
  std::optional<std::string> agent_config_sha256;
  std::optional<std::string> training_repo;
  std::optional<std::string> training_commit;
  std::optional<std::string> asset_revision;
  std::vector<std::pair<std::string, std::string>> software;
  std::vector<std::string> missing;
};

struct DeploymentContract {
  std::uint32_t schema_version{0};
  std::string policy_type;
  ModelInterface input;
  ModelInterface output;
  protocol::TimeUs policy_period_us{0};
  ActionContract action;
  ObservationContract observation;
  NormalizerContract normalizer;
  SourceContract source;
};

// Parses and fully validates one `irmv.deployment` document. Throws
// DeploymentError on the first violation, with the JSON field path of it: a
// missing or duplicated field, an unknown field, an unknown schema version or
// policy type, a null that the contract does not allow (including a `source`
// null that `source.missing` does not explain), a NaN or an infinity, an array
// whose length disagrees with the joint list or with the declared model I/O, a
// duplicated joint name, an unsupported transformation, unit, frame, policy
// order or normalizer.
DeploymentContract ParseDeployment(std::string_view json);

// Where each element of the model's three ordered joint lists sits in the
// deployment's canonical joint table. Every index is valid: a model joint the
// table does not have is an error, not a zero.
struct JointMapping {
  std::vector<std::uint32_t> action;     // size == action.joints
  std::vector<std::uint32_t> joint_pos;  // size == joint_pos_rel.joints
  std::vector<std::uint32_t> joint_vel;  // size == joint_vel_rel.joints
};

// Resolves the contract's three joint lists against `robot_joint_names` (the
// deployment's canonical order, robot.joints[].name). Throws DeploymentError
// naming the first model joint the deployment does not have; a partial mapping
// is never returned, because a policy that commands only some joints would
// leave the others at a stale target.
JointMapping BuildJointMapping(
    const DeploymentContract &contract,
    const std::vector<std::string> &robot_joint_names);

// True when the model's three joint lists cover exactly `robot_joint_count`
// joints. A policy that commands only part of the robot would leave the
// remaining joints at a stale target, so the application refuses such a model
// when it is loaded instead of discovering it on the first frame.
bool CoversEveryJoint(const DeploymentContract &contract,
                      std::size_t robot_joint_count);

}  // namespace irmv::policy
