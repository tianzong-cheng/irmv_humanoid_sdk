#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "irmv/policy/deployment.hpp"
#include "irmv/policy/observation.hpp"
#include "irmv/protocol/types.hpp"

// The local policy model: the ONNX Runtime session of one `policy.onnx`, the
// `irmv.deployment` contract it carries, the mapping of its joints onto the
// deployment's joint table and the digest of the file it was loaded from.
//
// Loading is the one place a model is accepted or refused, and it refuses
// every model the contract cannot describe: a missing, duplicated or
// unsupported `irmv.deployment`, a document of another schema version, a joint
// the deployment does not have, and - the check that needs the runtime - a
// declared tensor name or shape that disagrees with the graph the session
// actually exposes. Nothing is inferred from a model that fails any of them.
//
// ONNX Runtime is linked into this library alone, which is what keeps it out of
// the lower process: `ldd build/robot_runtime` must stay free of it.
namespace irmv::policy {

// The model could not be read, checked or executed. Distinct from
// DeploymentError, which reports a document this build cannot honour: this one
// reports the file, the ONNX Runtime session or the tensors of it.
class ModelError : public std::runtime_error {
 public:
  explicit ModelError(const std::string &message)
      : std::runtime_error(message) {}
};

// One loaded policy. Constructed only by Load, never copied or moved: the
// session and the tensors it hands out live in the object.
class OnnxModel {
 public:
  // Loads and validates the model at `path`, resolving its joints against
  // `robot_joint_names` (the deployment's canonical joint table, in order).
  // Throws ModelError on the first failure, with a message naming what was
  // refused. The digest that is compared against the configured overrides is
  // the SHA-256 of exactly the bytes the session was created from.
  static std::unique_ptr<OnnxModel> Load(
      const std::filesystem::path &path,
      const std::vector<std::string> &robot_joint_names);

  ~OnnxModel();
  OnnxModel(const OnnxModel &) = delete;
  OnnxModel &operator=(const OnnxModel &) = delete;
  OnnxModel(OnnxModel &&) = delete;
  OnnxModel &operator=(OnnxModel &&) = delete;

  const DeploymentContract &contract() const;
  const JointMapping &mapping() const;
  const std::string &sha256() const;
  // The digest of the `irmv.deployment` document the model carries, as the
  // bytes of that document. The model's file digest names the artifact, which
  // is what an override set is keyed to; this names the contract the artifact
  // declares, which is what a recorded run is joined to when the artifact
  // itself is rebuilt without changing what it says.
  const std::string &contract_sha256() const;

  // Runs one inference on an assembled observation and hands back the raw
  // model output. It returns false - leaving `action` and `action_count`
  // untouched - when the observation does not have the dimension the model
  // reads, when ONNX Runtime reports an error, or when the output has another
  // shape or a value that is not finite. The worker turns that into a failed
  // policy step; it is never a silent zero action.
  bool Run(const Observation &observation,
           std::array<float, protocol::kMaxJoints> *action,
           std::uint32_t *action_count) const;

 private:
  struct Impl;
  explicit OnnxModel(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace irmv::policy
