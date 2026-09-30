#include "irmv/policy/onnx_model.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "irmv/configuration/sha256.hpp"

namespace irmv::policy {
namespace {

// The ORT environment is one per process and outlives every session of it. It
// is created on first use and never destroyed before the last model of the
// process is: an application owns its model for the life of its supervision
// loop, which ends before static destruction.
Ort::Env &Environment() {
  static Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "irmv_policy");
  return environment;
}

std::string ReadFile(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw ModelError("cannot open the policy model " + path.string());
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  if (!input.good() && !input.eof()) {
    throw ModelError("cannot read the policy model " + path.string());
  }
  std::string bytes = contents.str();
  if (bytes.empty()) {
    throw ModelError("the policy model " + path.string() + " is empty");
  }
  return bytes;
}

std::string InputName(const Ort::Session &session, OrtAllocator *allocator) {
  const Ort::AllocatedStringPtr name =
      session.GetInputNameAllocated(0, allocator);
  if (!name) {
    throw ModelError("the model exposes no input name");
  }
  return std::string(name.get());
}

std::string OutputName(const Ort::Session &session, OrtAllocator *allocator) {
  const Ort::AllocatedStringPtr name =
      session.GetOutputNameAllocated(0, allocator);
  if (!name) {
    throw ModelError("the model exposes no output name");
  }
  return std::string(name.get());
}

// The [1, N] shape of one declared tensor. Every other rank, a batch dimension
// other than one and a dynamic width are refused: the worker feeds exactly one
// observation and reads exactly one action per step.
std::int64_t WidthOf(const Ort::ConstTensorTypeAndShapeInfo &info,
                     const std::string &name) {
  const std::vector<std::int64_t> shape = info.GetShape();
  if (shape.size() != 2) {
    throw ModelError("the model tensor " + name + " must be rank 2");
  }
  if (shape[0] != 1) {
    throw ModelError("the model tensor " + name +
                     " must have a fixed batch dimension of 1");
  }
  if (shape[1] < 1) {
    throw ModelError("the model tensor " + name + " has a dynamic width");
  }
  return shape[1];
}

}  // namespace

struct OnnxModel::Impl {
  Impl(std::string session_bytes, const std::filesystem::path &path,
       const std::vector<std::string> &robot_joint_names)
      : sha256(configuration::Sha256(session_bytes)) {
    Ort::SessionOptions options;
    // One policy step is a small network; the defaults of the pinned runtime
    // are used unchanged, so the session behaves the way the exported model was
    // verified with tools/verify_policy.py.
    session = Ort::Session(Environment(), session_bytes.data(),
                           session_bytes.size(), options);

    Ort::AllocatorWithDefaultOptions allocator;
    if (session.GetInputCount() != 1) {
      throw ModelError("the policy model must have exactly one input");
    }
    if (session.GetOutputCount() != 1) {
      throw ModelError("the policy model must have exactly one output");
    }
    input_name = InputName(session, allocator);
    output_name = OutputName(session, allocator);
    input_width = WidthOf(
        session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo(), input_name);
    output_width = WidthOf(
        session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo(), output_name);

    // The single `irmv.deployment` entry, read through the same interface the
    // application would use at runtime. A missing key is refused here, before a
    // single byte of the document is interpreted.
    //
    // The occurrence count below is a backstop, not the duplicate check: the
    // pinned ONNX Runtime collapses the metadata map when it loads the model,
    // so two entries with this key reach this layer as one. A duplicated
    // contract is therefore refused by the offline gate - onnx.checker rejects
    // such a model outright and tools/verify_policy.py runs that checker - and
    // the counting here only catches a runtime that would ever report both.
    const Ort::ModelMetadata metadata = session.GetModelMetadata();
    const std::vector<Ort::AllocatedStringPtr> keys =
        metadata.GetCustomMetadataMapKeysAllocated(allocator);
    std::size_t occurrences = 0;
    for (const Ort::AllocatedStringPtr &key : keys) {
      if (key && std::string_view(key.get()) == kDeploymentMetadataKey) {
        ++occurrences;
      }
    }
    if (occurrences == 0) {
      throw ModelError("the policy model " + path.string() + " carries no " +
                       std::string(kDeploymentMetadataKey) + " document");
    }
    if (occurrences > 1) {
      throw ModelError("the policy model " + path.string() + " carries " +
                       std::to_string(occurrences) + " " +
                       std::string(kDeploymentMetadataKey) + " documents");
    }
    const Ort::AllocatedStringPtr document =
        metadata.LookupCustomMetadataMapAllocated(
            std::string(kDeploymentMetadataKey).c_str(), allocator);
    if (!document) {
      throw ModelError("the policy model " + path.string() + " carries no " +
                       std::string(kDeploymentMetadataKey) + " document");
    }

    contract = ParseDeployment(std::string_view(document.get()));
    if (contract.input.name != input_name ||
        contract.input.dimension != static_cast<std::uint32_t>(input_width)) {
      throw ModelError("the deployment document declares the input " +
                       contract.input.name + "[" +
                       std::to_string(contract.input.dimension) +
                       "] but the session exposes " + input_name + "[" +
                       std::to_string(input_width) + "]");
    }
    if (contract.output.name != output_name ||
        contract.output.dimension != static_cast<std::uint32_t>(output_width)) {
      throw ModelError("the deployment document declares the output " +
                       contract.output.name + "[" +
                       std::to_string(contract.output.dimension) +
                       "] but the session exposes " + output_name + "[" +
                       std::to_string(output_width) + "]");
    }
    if (contract.observation.dimension > kMaxObservationDimension) {
      throw ModelError("the deployment document describes " +
                       std::to_string(contract.observation.dimension) +
                       " observation components, above this build's limit of " +
                       std::to_string(kMaxObservationDimension));
    }
    mapping = BuildJointMapping(contract, robot_joint_names);
  }

  std::string sha256;
  Ort::Session session{nullptr};
  DeploymentContract contract;
  JointMapping mapping;
  std::string input_name;
  std::string output_name;
  std::int64_t input_width{0};
  std::int64_t output_width{0};
};

OnnxModel::OnnxModel(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

OnnxModel::~OnnxModel() = default;

std::unique_ptr<OnnxModel> OnnxModel::Load(
    const std::filesystem::path &path,
    const std::vector<std::string> &robot_joint_names) {
  // The file is read once and the session is created from those very bytes, so
  // the digest the configured overrides are matched against is the digest of
  // the model that is actually executed.
  const std::string bytes = ReadFile(path);
  try {
    return std::unique_ptr<OnnxModel>(
        new OnnxModel(std::make_unique<Impl>(bytes, path, robot_joint_names)));
  } catch (const Ort::Exception &error) {
    throw ModelError("cannot load the policy model " + path.string() + ": " +
                     error.what());
  }
}

const DeploymentContract &OnnxModel::contract() const {
  return impl_->contract;
}

const JointMapping &OnnxModel::mapping() const { return impl_->mapping; }

const std::string &OnnxModel::sha256() const { return impl_->sha256; }

bool OnnxModel::Run(const Observation &observation,
                    std::array<float, protocol::kMaxJoints> *action,
                    std::uint32_t *action_count) const {
  if (action == nullptr || action_count == nullptr) {
    return false;
  }
  const std::size_t expected = impl_->contract.observation.dimension;
  const std::size_t produced =
      static_cast<std::size_t>(impl_->contract.output.dimension);
  if (observation.count != expected || expected > kMaxObservationDimension ||
      produced > protocol::kMaxJoints) {
    return false;
  }

  try {
    const std::array<std::int64_t, 2> shape = {
        1, static_cast<std::int64_t>(expected)};
    Ort::MemoryInfo memory =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    // ONNX Runtime reads the input and never writes it; the buffer is the
    // worker's own assembled observation, which nobody else touches while this
    // call runs.
    Ort::Value input = Ort::Value::CreateTensor<float>(
        memory, const_cast<float *>(observation.values.data()), expected,
        shape.data(), shape.size());
    const char *input_names[] = {impl_->input_name.c_str()};
    const char *output_names[] = {impl_->output_name.c_str()};
    std::vector<Ort::Value> outputs = impl_->session.Run(
        Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    if (outputs.size() != 1 || !outputs[0].IsTensor()) {
      return false;
    }
    const std::vector<std::int64_t> output_shape =
        outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (output_shape.size() != 2 || output_shape[0] != 1 ||
        output_shape[1] != impl_->output_width) {
      return false;
    }
    const float *data = outputs[0].GetTensorData<float>();
    std::array<float, protocol::kMaxJoints> result{};
    for (std::size_t index = 0; index < produced; ++index) {
      if (!std::isfinite(data[index])) {
        return false;
      }
      result[index] = data[index];
    }
    *action = result;
    *action_count = static_cast<std::uint32_t>(produced);
    return true;
  } catch (const Ort::Exception &) {
    // A failing session is a failed policy step; the caller latches the upper
    // fault and whether the runtime keeps moving is its own decision.
    return false;
  }
}

}  // namespace irmv::policy
