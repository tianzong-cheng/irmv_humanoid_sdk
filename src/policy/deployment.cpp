#include "irmv/policy/deployment.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace irmv::policy {
namespace {

using json = nlohmann::json;

// The bound of the frozen joint-name format of config/sdk.schema.json: a name
// is 1..64 UTF-8 bytes.
constexpr std::size_t kMaxJointNameBytes = 64;
constexpr std::size_t kMaxTextBytes = 256;
constexpr std::size_t kMaxTensorNameBytes = 256;
constexpr std::uint64_t kMaxDimension =
    protocol::kMaxJoints * protocol::kMaxJoints;

[[noreturn]] void Fail(const std::string &path, const std::string &message) {
  throw DeploymentError(path, message);
}

std::string IndexPath(const std::string &path, std::size_t index) {
  return path + "[" + std::to_string(index) + "]";
}

// Dotted field path; the root has no leading dot.
std::string Join(const std::string &path, std::string_view name) {
  if (path.empty()) {
    return std::string(name);
  }
  return path + "." + std::string(name);
}

// Rejects every key the contract does not declare, so a document cannot carry a
// field this build would silently ignore.
void RejectUnknownKeys(const json &object, const std::string &path,
                       std::initializer_list<std::string_view> allowed) {
  for (auto it = object.begin(); it != object.end(); ++it) {
    bool known = false;
    for (const std::string_view name : allowed) {
      if (it.key() == name) {
        known = true;
        break;
      }
    }
    if (!known) {
      Fail(Join(path, it.key()), "unknown field");
    }
  }
}

const json &RequireObject(const json &value, const std::string &path) {
  if (!value.is_object()) {
    Fail(path, "expected an object");
  }
  return value;
}

const json &RequireField(const json &object, const std::string &path,
                         std::string_view name) {
  const auto it = object.find(name);
  if (it == object.end()) {
    Fail(Join(path, name), "missing field");
  }
  return *it;
}

std::string Text(const json &value, const std::string &path,
                 std::size_t max_bytes) {
  if (!value.is_string()) {
    Fail(path, "expected a string");
  }
  std::string text = value.get<std::string>();
  if (text.empty() || text.size() > max_bytes) {
    Fail(path, "expected 1.." + std::to_string(max_bytes) + " UTF-8 bytes");
  }
  return text;
}

double FiniteNumber(const json &value, const std::string &path) {
  if (!value.is_number()) {
    Fail(path, "expected a number");
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    Fail(path, "expected a finite number");
  }
  return number;
}

std::uint64_t PositiveInteger(const json &value, const std::string &path,
                              std::uint64_t minimum, std::uint64_t maximum) {
  if (!value.is_number_integer() && !value.is_number_unsigned()) {
    Fail(path, "expected an integer");
  }
  if (value.is_number_integer() && value.get<std::int64_t>() < 0) {
    Fail(path, "expected an integer of at least " + std::to_string(minimum));
  }
  const std::uint64_t number = value.get<std::uint64_t>();
  if (number < minimum || number > maximum) {
    Fail(path, "expected an integer in [" + std::to_string(minimum) + ", " +
                   std::to_string(maximum) + "]");
  }
  return number;
}

std::uint32_t Dimension(const json &value, const std::string &path) {
  return static_cast<std::uint32_t>(
      PositiveInteger(value, path, 1, kMaxDimension));
}

// One array of finite numbers, either exactly `expected_size` long or - with
// `expected_size` zero - of any non-zero length.
std::vector<double> FiniteArray(const json &value, const std::string &path,
                                std::size_t expected_size,
                                bool require_non_negative) {
  if (!value.is_array()) {
    Fail(path, "expected an array");
  }
  if (value.empty()) {
    Fail(path, "expected a non-empty array");
  }
  if (expected_size != 0 && value.size() != expected_size) {
    Fail(path, "expected " + std::to_string(expected_size) +
                   " entries, one"
                   " per element of the list it describes");
  }
  std::vector<double> numbers;
  numbers.reserve(value.size());
  for (std::size_t i = 0; i < value.size(); ++i) {
    const std::string entry_path = IndexPath(path, i);
    const double number = FiniteNumber(value[i], entry_path);
    if (require_non_negative && number < 0.0) {
      Fail(entry_path, "expected a non-negative value");
    }
    numbers.push_back(number);
  }
  return numbers;
}

std::vector<std::string> JointNames(const json &value,
                                    const std::string &path) {
  if (!value.is_array()) {
    Fail(path, "expected an array of joint names");
  }
  if (value.empty()) {
    Fail(path, "expected at least one joint name");
  }
  if (value.size() > protocol::kMaxJoints) {
    Fail(path, "expected at most " + std::to_string(protocol::kMaxJoints) +
                   " joint names");
  }
  std::vector<std::string> names;
  names.reserve(value.size());
  std::unordered_set<std::string> seen;
  for (std::size_t i = 0; i < value.size(); ++i) {
    const std::string entry_path = IndexPath(path, i);
    std::string name = Text(value[i], entry_path, kMaxJointNameBytes);
    if (!seen.insert(name).second) {
      Fail(entry_path, "duplicate joint name");
    }
    names.push_back(std::move(name));
  }
  return names;
}

// One clip layer: null, or exactly one range per element of the list it
// belongs to.
void ParseClipLayer(const json &value, const std::string &path,
                    std::size_t component_count, bool *present,
                    std::vector<ClipRange> *ranges) {
  if (value.is_null()) {
    *present = false;
    ranges->clear();
    return;
  }
  if (!value.is_array()) {
    Fail(path, "expected null or an array of clip ranges");
  }
  if (value.size() != component_count) {
    Fail(path, "expected one clip range per joint");
  }
  ranges->assign(component_count, ClipRange{});
  for (std::size_t i = 0; i < value.size(); ++i) {
    const std::string entry_path = IndexPath(path, i);
    const json &entry = RequireObject(value[i], entry_path);
    RejectUnknownKeys(entry, entry_path, {"min", "max"});
    const double min = FiniteNumber(RequireField(entry, entry_path, "min"),
                                    entry_path + ".min");
    const double max = FiniteNumber(RequireField(entry, entry_path, "max"),
                                    entry_path + ".max");
    if (min > max) {
      Fail(entry_path, "min must not exceed max");
    }
    (*ranges)[i] = ClipRange{min, max};
  }
  *present = true;
}

ModelInterface ParseModelInterface(const json &value, const std::string &path) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(object, path, {"name", "dimension"});
  ModelInterface result;
  result.name = Text(RequireField(object, path, "name"), path + ".name",
                     kMaxTensorNameBytes);
  result.dimension =
      Dimension(RequireField(object, path, "dimension"), path + ".dimension");
  return result;
}

ActionContract ParseAction(const json &value, const std::string &path) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(object, path,
                    {"joints", "source_joints", "default_q", "default_dq", "kp",
                     "kd", "scale", "offset", "clip", "order", "last_action"});

  ActionContract action;
  action.joints =
      JointNames(RequireField(object, path, "joints"), path + ".joints");
  const std::size_t count = action.joints.size();
  action.source_joints = JointNames(RequireField(object, path, "source_joints"),
                                    path + ".source_joints");
  if (action.source_joints.size() != count) {
    Fail(path + ".source_joints",
         "expected one training joint name per deployment joint name");
  }

  action.default_q = FiniteArray(RequireField(object, path, "default_q"),
                                 path + ".default_q", count, false);
  action.default_dq = FiniteArray(RequireField(object, path, "default_dq"),
                                  path + ".default_dq", count, false);
  action.kp =
      FiniteArray(RequireField(object, path, "kp"), path + ".kp", count, true);
  action.kd =
      FiniteArray(RequireField(object, path, "kd"), path + ".kd", count, true);
  action.scale = FiniteArray(RequireField(object, path, "scale"),
                             path + ".scale", count, false);
  action.offset = FiniteArray(RequireField(object, path, "offset"),
                              path + ".offset", count, false);

  const json &clip =
      RequireObject(RequireField(object, path, "clip"), path + ".clip");
  RejectUnknownKeys(clip, path + ".clip", {"wrapper", "term"});
  ParseClipLayer(RequireField(clip, path + ".clip", "wrapper"),
                 path + ".clip.wrapper", count, &action.wrapper_clip,
                 &action.wrapper_clip_ranges);
  ParseClipLayer(RequireField(clip, path + ".clip", "term"),
                 path + ".clip.term", count, &action.term_clip,
                 &action.term_clip_ranges);

  // The pipeline order is frozen. An unrecognised, missing or reordered stage
  // is refused instead of being re-derived, because a different order silently
  // produces a different joint target.
  static constexpr ActionStage kFrozenOrder[] = {
      ActionStage::WrapperClip, ActionStage::Scale, ActionStage::Offset,
      ActionStage::TermClip};
  const json &order = RequireField(object, path, "order");
  if (!order.is_array() || order.size() != 4) {
    Fail(path + ".order", "expected the frozen four-stage action order");
  }
  for (std::size_t i = 0; i < 4; ++i) {
    const std::string entry_path = IndexPath(path + ".order", i);
    const std::string token = Text(order[i], entry_path, 32);
    if (token != ActionStageName(kFrozenOrder[i])) {
      Fail(entry_path, "expected the frozen action pipeline order");
    }
    action.order.push_back(kFrozenOrder[i]);
  }

  const json &last_action = RequireObject(
      RequireField(object, path, "last_action"), path + ".last_action");
  RejectUnknownKeys(last_action, path + ".last_action", {"stage", "initial"});
  const std::string stage =
      Text(RequireField(last_action, path + ".last_action", "stage"),
           path + ".last_action.stage", 32);
  if (stage == "raw") {
    action.last_action_stage = LastActionStage::Raw;
  } else if (stage == "processed") {
    action.last_action_stage = LastActionStage::Processed;
  } else {
    Fail(path + ".last_action.stage", "expected \"raw\" or \"processed\"");
  }
  action.last_action_initial =
      FiniteArray(RequireField(last_action, path + ".last_action", "initial"),
                  path + ".last_action.initial", count, false);
  return action;
}

JointReference ParseJointReference(const json &value, const std::string &path,
                                   std::string_view term,
                                   std::string_view unit) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(object, path,
                    {"term", "joints", "source_joints", "reference", "unit"});
  JointReference reference;
  reference.term = Text(RequireField(object, path, "term"), path + ".term", 64);
  if (reference.term != term) {
    Fail(path + ".term", "expected \"" + std::string(term) + "\"");
  }
  reference.joints =
      JointNames(RequireField(object, path, "joints"), path + ".joints");
  reference.source_joints = JointNames(
      RequireField(object, path, "source_joints"), path + ".source_joints");
  if (reference.source_joints.size() != reference.joints.size()) {
    Fail(path + ".source_joints",
         "expected one training joint name per deployment joint name");
  }
  reference.reference =
      FiniteArray(RequireField(object, path, "reference"), path + ".reference",
                  reference.joints.size(), false);
  reference.unit = Text(RequireField(object, path, "unit"), path + ".unit", 32);
  if (reference.unit != unit) {
    Fail(path + ".unit", "expected \"" + std::string(unit) + "\"");
  }
  return reference;
}

// The frozen description of one supported observation term.
struct TermSpec {
  ObservationTermKind kind;
  const char *name;
  const char *unit;
  const char *frame;
  std::size_t fixed_dimension;  // 0 when the term is as long as a joint list
};

constexpr TermSpec kTermSpecs[] = {
    {ObservationTermKind::Command, "command", "m/s,m/s,rad/s", "body", 3},
    {ObservationTermKind::ProjectedGravity, "projected_gravity", "1", "body",
     3},
    {ObservationTermKind::BaseAngularVelocity, "base_ang_vel", "rad/s", "body",
     3},
    {ObservationTermKind::JointPosition, "joint_pos_rel", "rad", "joint", 0},
    {ObservationTermKind::JointVelocity, "joint_vel_rel", "rad/s", "joint", 0},
    {ObservationTermKind::LastAction, "last_action", "1", "none", 0},
};
constexpr std::size_t kTermCount = sizeof(kTermSpecs) / sizeof(kTermSpecs[0]);

ObservationTerm ParseObservationTerm(const json &value, const std::string &path,
                                     std::size_t expected_dimension,
                                     const TermSpec &spec) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(object, path,
                    {"name", "dimension", "unit", "frame", "scale", "clip",
                     "history_length"});
  ObservationTerm term;
  term.kind = spec.kind;
  term.name = Text(RequireField(object, path, "name"), path + ".name", 64);
  if (term.name != spec.name) {
    Fail(path + ".name", "expected \"" + std::string(spec.name) + "\"");
  }
  term.dimension =
      Dimension(RequireField(object, path, "dimension"), path + ".dimension");
  if (term.dimension != expected_dimension) {
    Fail(path + ".dimension",
         "expected " + std::to_string(expected_dimension) + " for this term");
  }
  term.unit = Text(RequireField(object, path, "unit"), path + ".unit", 32);
  if (term.unit != spec.unit) {
    Fail(path + ".unit", "expected \"" + std::string(spec.unit) + "\"");
  }
  term.frame = Text(RequireField(object, path, "frame"), path + ".frame", 32);
  if (term.frame != spec.frame) {
    Fail(path + ".frame", "expected \"" + std::string(spec.frame) + "\"");
  }

  const json &scale = RequireField(object, path, "scale");
  if (!scale.is_null()) {
    term.scaled = true;
    term.scale = FiniteArray(scale, path + ".scale", term.dimension, false);
  }
  const json &clip = RequireField(object, path, "clip");
  if (!clip.is_null()) {
    if (!clip.is_array() || clip.size() != term.dimension) {
      Fail(path + ".clip", "expected one clip range per component");
    }
    term.clipped = true;
    term.clip.assign(term.dimension, ClipRange{});
    for (std::size_t i = 0; i < clip.size(); ++i) {
      const std::string entry_path = IndexPath(path + ".clip", i);
      const json &entry = RequireObject(clip[i], entry_path);
      RejectUnknownKeys(entry, entry_path, {"min", "max"});
      const double min = FiniteNumber(RequireField(entry, entry_path, "min"),
                                      entry_path + ".min");
      const double max = FiniteNumber(RequireField(entry, entry_path, "max"),
                                      entry_path + ".max");
      if (min > max) {
        Fail(entry_path, "min must not exceed max");
      }
      term.clip[i] = ClipRange{min, max};
    }
  }

  const std::uint64_t history =
      PositiveInteger(RequireField(object, path, "history_length"),
                      path + ".history_length", 0, 1);
  if (history > 1) {
    Fail(path + ".history_length",
         "unsupported: this build does not stack observation history");
  }
  term.history_length = static_cast<std::uint32_t>(history);
  return term;
}

ObservationContract ParseObservation(const json &value, const std::string &path,
                                     const ActionContract &action) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(object, path, {"terms", "joint_pos_rel", "joint_vel_rel"});

  ObservationContract observation;
  observation.joint_pos_rel =
      ParseJointReference(RequireField(object, path, "joint_pos_rel"),
                          path + ".joint_pos_rel", "joint_pos_rel", "rad");
  observation.joint_vel_rel =
      ParseJointReference(RequireField(object, path, "joint_vel_rel"),
                          path + ".joint_vel_rel", "joint_vel_rel", "rad/s");

  const json &terms = RequireField(object, path, "terms");
  if (!terms.is_array() || terms.size() != kTermCount) {
    Fail(path + ".terms",
         "expected exactly the six supported observation terms");
  }
  observation.terms.reserve(kTermCount);
  std::uint64_t total = 0;
  for (std::size_t i = 0; i < kTermCount; ++i) {
    const TermSpec &spec = kTermSpecs[i];
    const std::string term_path = IndexPath(path + ".terms", i);
    // The frozen order is part of the contract: the SDK concatenates exactly
    // this sequence, so a term at another index describes a different vector.
    std::size_t expected = spec.fixed_dimension;
    if (expected == 0) {
      if (spec.kind == ObservationTermKind::JointPosition) {
        expected = observation.joint_pos_rel.joints.size();
      } else if (spec.kind == ObservationTermKind::JointVelocity) {
        expected = observation.joint_vel_rel.joints.size();
      } else {
        expected = action.joints.size();
      }
    }
    observation.terms.push_back(
        ParseObservationTerm(terms[i], term_path, expected, spec));
    total += expected;
  }
  observation.dimension = static_cast<std::uint32_t>(total);
  return observation;
}

NormalizerContract ParseNormalizer(const json &value, const std::string &path,
                                   std::size_t observation_dimension) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(
      object, path,
      {"enabled", "embedded", "location", "algorithm", "statistics"});
  NormalizerContract normalizer;
  const json &enabled = RequireField(object, path, "enabled");
  if (!enabled.is_boolean()) {
    Fail(path + ".enabled", "expected a boolean");
  }
  normalizer.enabled = enabled.get<bool>();
  const json &embedded = RequireField(object, path, "embedded");
  if (!embedded.is_boolean()) {
    Fail(path + ".embedded", "expected a boolean");
  }
  normalizer.embedded = embedded.get<bool>();
  const std::string location =
      Text(RequireField(object, path, "location"), path + ".location", 32);
  const bool has_algorithm = object.find("algorithm") != object.end();
  const bool has_statistics = object.find("statistics") != object.end();

  if (!normalizer.enabled) {
    if (normalizer.embedded || location != "none") {
      Fail(path,
           "a disabled normalizer must be location \"none\" and not"
           " embedded");
    }
    if (has_algorithm || has_statistics) {
      Fail(path,
           "a disabled normalizer must not carry an algorithm or"
           " statistics");
    }
    return normalizer;
  }

  if (!has_algorithm) {
    Fail(path + ".algorithm", "missing field");
  }
  normalizer.algorithm =
      Text(RequireField(object, path, "algorithm"), path + ".algorithm", 32);
  if (normalizer.algorithm != "mean_std") {
    Fail(path + ".algorithm",
         "unsupported normalizer algorithm: this build implements only"
         " per-component (x - mean) / std");
  }

  if (normalizer.embedded) {
    if (location != "model") {
      Fail(path + ".location", "expected \"model\" for an embedded normalizer");
    }
    if (has_statistics) {
      Fail(path + ".statistics",
           "an embedded normalizer must not repeat its statistics");
    }
    normalizer.location = NormalizerLocation::Model;
    return normalizer;
  }

  if (location != "application") {
    Fail(path + ".location",
         "expected \"application\" for a normalizer the SDK applies");
  }
  normalizer.location = NormalizerLocation::Application;
  const json &statistics = RequireField(object, path, "statistics");
  const json &statistics_object =
      RequireObject(statistics, path + ".statistics");
  RejectUnknownKeys(statistics_object, path + ".statistics",
                    {"count", "mean", "std"});
  normalizer.count = static_cast<std::uint32_t>(PositiveInteger(
      RequireField(statistics_object, path + ".statistics", "count"),
      path + ".statistics.count", 1, UINT32_MAX));
  normalizer.mean =
      FiniteArray(RequireField(statistics_object, path + ".statistics", "mean"),
                  path + ".statistics.mean", observation_dimension, false);
  normalizer.stddev =
      FiniteArray(RequireField(statistics_object, path + ".statistics", "std"),
                  path + ".statistics.std", observation_dimension, false);
  for (std::size_t i = 0; i < normalizer.stddev.size(); ++i) {
    if (normalizer.stddev[i] <= 0.0) {
      Fail(IndexPath(path + ".statistics.std", i),
           "expected a positive standard deviation");
    }
  }
  return normalizer;
}

// Optional text of the provenance block: present as a string, explicitly null,
// or - never - absent.
std::optional<std::string> ParseOptionalText(const json &object,
                                             const std::string &path,
                                             std::string_view name,
                                             bool digest) {
  const json &value = RequireField(object, path, name);
  if (value.is_null()) {
    return std::nullopt;
  }
  const std::string field_path = Join(path, name);
  std::string text = digest ? Text(value, field_path, 64)
                            : Text(value, field_path, kMaxTextBytes);
  if (digest) {
    if (text.size() != 64) {
      Fail(field_path,
           "expected a 64 character lower-case hexadecimal"
           " SHA-256 digest");
    }
    for (const char byte : text) {
      const bool hex =
          (byte >= '0' && byte <= '9') || (byte >= 'a' && byte <= 'f');
      if (!hex) {
        Fail(field_path, "expected a lower-case hexadecimal SHA-256 digest");
      }
    }
  }
  return text;
}

SourceContract ParseSource(const json &value, const std::string &path) {
  const json &object = RequireObject(value, path);
  RejectUnknownKeys(object, path,
                    {"checkpoint_file", "checkpoint_sha256", "env_config_file",
                     "env_config_sha256", "agent_config_file",
                     "agent_config_sha256", "training_repo", "training_commit",
                     "asset_revision", "software", "missing"});

  SourceContract source;
  source.checkpoint_file =
      ParseOptionalText(object, path, "checkpoint_file", false);
  source.checkpoint_sha256 =
      ParseOptionalText(object, path, "checkpoint_sha256", true);
  source.env_config_file =
      ParseOptionalText(object, path, "env_config_file", false);
  source.env_config_sha256 =
      ParseOptionalText(object, path, "env_config_sha256", true);
  source.agent_config_file =
      ParseOptionalText(object, path, "agent_config_file", false);
  source.agent_config_sha256 =
      ParseOptionalText(object, path, "agent_config_sha256", true);
  source.training_repo =
      ParseOptionalText(object, path, "training_repo", false);
  source.training_commit =
      ParseOptionalText(object, path, "training_commit", false);
  source.asset_revision =
      ParseOptionalText(object, path, "asset_revision", false);

  const json &software =
      RequireObject(RequireField(object, path, "software"), path + ".software");
  for (auto it = software.begin(); it != software.end(); ++it) {
    const std::string entry_path = Join(Join(path, "software"), it.key());
    if (it.key().empty()) {
      Fail(path + ".software", "expected non-empty software names");
    }
    source.software.emplace_back(it.key(),
                                 Text(it.value(), entry_path, kMaxTextBytes));
  }

  const json &missing = RequireField(object, path, "missing");
  if (!missing.is_array()) {
    Fail(path + ".missing", "expected an array");
  }
  std::unordered_set<std::string> declared;
  for (std::size_t i = 0; i < missing.size(); ++i) {
    const std::string entry_path = IndexPath(path + ".missing", i);
    const std::string name = Text(missing[i], entry_path, kMaxTextBytes);
    if (!declared.insert(name).second) {
      Fail(entry_path, "duplicate missing entry");
    }
    source.missing.push_back(name);
  }
  // Every null is explained and no absence is invented: a field the package
  // could not determine must be null and named here, and no other name may
  // appear.
  const std::pair<std::string_view, const std::optional<std::string> *>
      fields[] = {{"checkpoint_file", &source.checkpoint_file},
                  {"checkpoint_sha256", &source.checkpoint_sha256},
                  {"env_config_file", &source.env_config_file},
                  {"env_config_sha256", &source.env_config_sha256},
                  {"agent_config_file", &source.agent_config_file},
                  {"agent_config_sha256", &source.agent_config_sha256},
                  {"training_repo", &source.training_repo},
                  {"training_commit", &source.training_commit},
                  {"asset_revision", &source.asset_revision}};
  for (const auto &field : fields) {
    const bool is_null = !*field.second;
    const bool is_declared = declared.count(std::string(field.first)) != 0;
    if (is_null && !is_declared) {
      Fail(Join(path, field.first),
           "a null provenance field must be listed in source.missing");
    }
    if (!is_null && is_declared) {
      Fail(path + ".missing",
           std::string(field.first) + " is present but listed as missing");
    }
  }
  // Only the nullable provenance fields can be declared missing: a name the
  // contract does not know would otherwise hide the fact that nothing is
  // absent.
  for (const std::string &name : declared) {
    bool known = false;
    for (const auto &field : fields) {
      if (name == field.first) {
        known = true;
        break;
      }
    }
    if (!known) {
      Fail(path + ".missing", "unknown provenance field \"" + name + "\"");
    }
  }
  return source;
}

}  // namespace

DeploymentError::DeploymentError(std::string field_path, std::string message)
    : std::runtime_error(field_path.empty()
                             ? "deployment contract error: " + message
                             : "deployment contract error at " + field_path +
                                   ": " + message),
      field_path_(std::move(field_path)) {}

const char *ActionStageName(ActionStage stage) noexcept {
  switch (stage) {
    case ActionStage::WrapperClip:
      return "wrapper_clip";
    case ActionStage::Scale:
      return "scale";
    case ActionStage::Offset:
      return "offset";
    case ActionStage::TermClip:
      return "term_clip";
  }
  return "unknown";
}

DeploymentContract ParseDeployment(std::string_view json_text) {
  json document;
  try {
    document = json::parse(json_text.begin(), json_text.end());
  } catch (const json::exception &error) {
    // The metadata value is not JSON this build can read: not JSON at all,
    // truncated, carrying the NaN/Infinity tokens of the reference
    // implementation, or holding a number that overflows the double range.
    // Nothing is inferred from a document that could not be read whole.
    Fail("",
         std::string("the metadata value is not valid JSON: ") + error.what());
  }
  const json &root = RequireObject(document, "");
  RejectUnknownKeys(
      root, "",
      {"schema_version", "policy_type", "model", "policy_period_us", "action",
       "observation", "normalizer", "source"});

  DeploymentContract contract;
  const std::uint64_t version =
      PositiveInteger(RequireField(root, "", "schema_version"),
                      "schema_version", 1, UINT32_MAX);
  if (version != kDeploymentSchemaVersion) {
    Fail("schema_version", "unsupported deployment schema version");
  }
  contract.schema_version = static_cast<std::uint32_t>(version);
  contract.policy_type =
      Text(RequireField(root, "", "policy_type"), "policy_type", 64);
  if (contract.policy_type != kJointPositionPolicyType) {
    Fail("policy_type", "unsupported policy type");
  }

  const json &model = RequireObject(RequireField(root, "", "model"), "model");
  RejectUnknownKeys(model, "model", {"input", "output"});
  contract.input =
      ParseModelInterface(RequireField(model, "model", "input"), "model.input");
  contract.output = ParseModelInterface(RequireField(model, "model", "output"),
                                        "model.output");
  contract.policy_period_us =
      PositiveInteger(RequireField(root, "", "policy_period_us"),
                      "policy_period_us", 1, UINT64_MAX);

  contract.action = ParseAction(RequireField(root, "", "action"), "action");
  contract.observation = ParseObservation(RequireField(root, "", "observation"),
                                          "observation", contract.action);
  contract.normalizer =
      ParseNormalizer(RequireField(root, "", "normalizer"), "normalizer",
                      contract.observation.dimension);
  contract.source = ParseSource(RequireField(root, "", "source"), "source");

  // Cross-checks against the declared model interface: the arrays must be able
  // to fill the tensor and the tensor must consume all of them.
  if (contract.observation.dimension != contract.input.dimension) {
    Fail("model.input.dimension",
         "the declared input dimension does not match the " +
             std::to_string(contract.observation.dimension) +
             " components of the observation contract");
  }
  if (contract.action.joints.size() != contract.output.dimension) {
    Fail("model.output.dimension",
         "the declared output dimension does not match the " +
             std::to_string(contract.action.joints.size()) +
             " action joints of the contract");
  }
  return contract;
}

bool CoversEveryJoint(const DeploymentContract &contract,
                      std::size_t robot_joint_count) {
  return contract.action.joints.size() == robot_joint_count &&
         contract.observation.joint_pos_rel.joints.size() ==
             robot_joint_count &&
         contract.observation.joint_vel_rel.joints.size() == robot_joint_count;
}

JointMapping BuildJointMapping(const DeploymentContract &contract,
                               const std::vector<std::string> &robot_joints) {
  std::unordered_map<std::string, std::uint32_t> index_of;
  index_of.reserve(robot_joints.size());
  for (std::size_t i = 0; i < robot_joints.size(); ++i) {
    index_of.emplace(robot_joints[i], static_cast<std::uint32_t>(i));
  }

  const auto resolve = [&index_of](const std::vector<std::string> &names,
                                   const std::string &path) {
    std::vector<std::uint32_t> indices;
    indices.reserve(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto it = index_of.find(names[i]);
      if (it == index_of.end()) {
        Fail(IndexPath(path, i),
             "the deployment has no joint named \"" + names[i] + "\"");
      }
      indices.push_back(it->second);
    }
    return indices;
  };

  JointMapping mapping;
  mapping.action = resolve(contract.action.joints, "action.joints");
  mapping.joint_pos = resolve(contract.observation.joint_pos_rel.joints,
                              "observation.joint_pos_rel.joints");
  mapping.joint_vel = resolve(contract.observation.joint_vel_rel.joints,
                              "observation.joint_vel_rel.joints");
  return mapping;
}

}  // namespace irmv::policy
