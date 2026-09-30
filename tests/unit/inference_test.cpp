// The inference worker: what it hands over, what it hands back, and that it
// never lets one side wait for the other.
#include "irmv/policy/inference.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "irmv/policy/onnx_model.hpp"
#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

using irmv::policy::InferenceWorker;
using irmv::policy::OnnxModel;
using irmv::policy::PolicyRequest;
using irmv::policy::PolicyResponse;

const std::vector<std::string> kRobotJoints = {"ankle_pitch", "knee",
                                               "hip_pitch"};

// Bounded wait for one response. The worker is a thread of its own, so the test
// waits for it the way the supervision loop would wait for the next tick.
bool TakeWithin(InferenceWorker &worker, PolicyResponse *response,
                int milliseconds = 5000) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(milliseconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (worker.TryTake(response)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

PolicyRequest MakeRequest(std::uint64_t generation,
                          irmv::protocol::Sequence epoch,
                          irmv::protocol::Sequence state_seq,
                          irmv::protocol::TimeUs submitted_us,
                          std::size_t dimension) {
  PolicyRequest request;
  request.generation = generation;
  request.epoch = epoch;
  request.state_seq = state_seq;
  request.state_stamp_us = submitted_us + 17;
  request.submitted_us = submitted_us;
  request.observation.count = static_cast<std::uint32_t>(dimension);
  return request;
}

}  // namespace

int main(int argc, char **argv) {
  CHECK(argc == 2);
  const std::string directory = argv[1];

  const std::unique_ptr<OnnxModel> model =
      OnnxModel::Load(directory + "/valid.onnx", kRobotJoints);
  CHECK(model != nullptr);
  const std::size_t dimension = model->contract().observation.dimension;

  // A new worker runs nothing until something is submitted.
  {
    InferenceWorker worker(*model);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(worker.submitted() == 0);
    CHECK(worker.completed() == 0);
    CHECK(worker.failed() == 0);
    PolicyResponse response;
    CHECK(!worker.TryTake(&response));
    worker.Stop();
    worker.Stop();
  }

  // One request produces one response that carries the identity of its request,
  // so the caller can tell which step it belongs to.
  {
    InferenceWorker worker(*model);
    const PolicyRequest request = MakeRequest(3, 11, 42, 1000, dimension);
    worker.Submit(request);
    PolicyResponse response;
    CHECK(TakeWithin(worker, &response));
    CHECK(response.generation == 3);
    CHECK(response.epoch == 11);
    CHECK(response.state_seq == 42);
    CHECK(response.state_stamp_us == 1017);
    CHECK(response.submitted_us == 1000);
    CHECK(response.ok);
    CHECK(response.action_count == 3);
    for (std::uint32_t index = 0; index < response.action_count; ++index) {
      CHECK(std::isfinite(response.action[index]));
    }
    CHECK(worker.submitted() == 1);
    CHECK(worker.completed() == 1);
    CHECK(worker.replaced() == 0);
    // The result was collected, so nothing was superseded.
    CHECK(worker.uncollected() == 0);
    CHECK(!worker.TryTake(&response));
    worker.Stop();
  }

  // A burst never blocks the submitter and never queues: the worker runs the
  // steps it can and the superseded ones are counted, not accumulated.
  {
    InferenceWorker worker(*model);
    constexpr int kBurst = 64;
    for (int index = 0; index < kBurst; ++index) {
      worker.Submit(MakeRequest(
          1, 2, static_cast<irmv::protocol::Sequence>(index),
          static_cast<irmv::protocol::TimeUs>(2000 + index), dimension));
    }
    PolicyResponse response;
    CHECK(TakeWithin(worker, &response));
    worker.Stop();
    CHECK(worker.submitted() == kBurst);
    CHECK(worker.completed() + worker.failed() ==
          worker.submitted() - worker.replaced());
    CHECK(worker.replaced() > 0);
    CHECK(worker.completed() <= kBurst);
  }

  // A model that cannot answer produces a failed step, never a silent zero
  // action: the caller has to see the difference.
  {
    const std::unique_ptr<OnnxModel> broken =
        OnnxModel::Load(directory + "/non_finite.onnx", kRobotJoints);
    InferenceWorker worker(*broken);
    worker.Submit(
        MakeRequest(1, 1, 1, 3000, broken->contract().observation.dimension));
    PolicyResponse response;
    CHECK(TakeWithin(worker, &response));
    CHECK(!response.ok);
    CHECK(worker.failed() == 1);
    CHECK(worker.completed() == 0);
    worker.Stop();
  }

  // An observation of the wrong width is refused by the model and reported as a
  // failed step.
  {
    InferenceWorker worker(*model);
    worker.Submit(MakeRequest(1, 1, 1, 4000, dimension - 1));
    PolicyResponse response;
    CHECK(TakeWithin(worker, &response));
    CHECK(!response.ok);
    CHECK(worker.failed() == 1);
    worker.Stop();
  }

  std::cout << "inference_test: ok\n";
  return 0;
}
