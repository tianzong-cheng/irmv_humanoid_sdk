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

  const std::shared_ptr<OnnxModel> model =
      OnnxModel::Load(directory + "/valid.onnx", kRobotJoints);
  CHECK(model != nullptr);
  const std::size_t dimension = model->contract().observation.dimension;

  // A new worker runs nothing until something is submitted.
  {
    InferenceWorker worker(model);
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
    InferenceWorker worker(model);
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
    InferenceWorker worker(model);
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

  // A valid but long-running Loop model must be stopped by the cancellation
  // token, not by waiting for its trip count. The worker owns the Ort
  // RunOptions that Stop() terminates.
  {
    const std::shared_ptr<OnnxModel> long_running =
        OnnxModel::Load(directory + "/long_running.onnx", kRobotJoints);
    InferenceWorker worker(long_running);
    worker.Submit(MakeRequest(1, 1, 1, 2500,
                              long_running->contract().observation.dimension));
    // Give the worker time to take the request and enter the Loop.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    CHECK(worker.completed() == 0);
    const auto started = std::chrono::steady_clock::now();
    CHECK(worker.Stop());
    const auto elapsed = std::chrono::steady_clock::now() - started;
    CHECK(elapsed < std::chrono::milliseconds(1500));
    // The cancelled step is reported as failed, so the request really entered
    // the model rather than being discarded before it started.
    CHECK(worker.failed() == 1);
  }

  // A model that cannot answer produces a failed step, never a silent zero
  // action: the caller has to see the difference.
  {
    const std::shared_ptr<OnnxModel> broken =
        OnnxModel::Load(directory + "/non_finite.onnx", kRobotJoints);
    InferenceWorker worker(broken);
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
    InferenceWorker worker(model);
    worker.Submit(MakeRequest(1, 1, 1, 4000, dimension - 1));
    PolicyResponse response;
    CHECK(TakeWithin(worker, &response));
    CHECK(!response.ok);
    CHECK(worker.failed() == 1);
    worker.Stop();
  }

  // The wakeup bookkeeping across a submission that lands while the previous
  // one is still queued or running: the newest request must still be answered.
  // Splitting the take of a request from the reset of the flag it was woken for
  // leaves a window in which that reset erases the flag of a submission that
  // arrived in between, and the newest request then sits in the channel with
  // nothing left to wake the worker for it - which is what this churn drives,
  // one round at a time, with a bounded wait so a lost wakeup fails the round
  // instead of hanging.
  {
    InferenceWorker worker(model);
    constexpr int kChurn = 2000;
    for (int round = 1; round <= kChurn; ++round) {
      const auto older = static_cast<irmv::protocol::Sequence>(round * 2 - 1);
      const auto newest = static_cast<irmv::protocol::Sequence>(round * 2);
      const auto submitted =
          static_cast<irmv::protocol::TimeUs>(10000 + round * 2);
      worker.Submit(MakeRequest(1, 2, older, submitted, dimension));
      worker.Submit(MakeRequest(1, 2, newest, submitted + 1, dimension));
      PolicyResponse response;
      CHECK(TakeWithin(worker, &response));
      // Both were submitted; whichever the worker ran first may be the one
      // collected here, so the newest is taken too.
      if (response.state_seq != newest) {
        CHECK(TakeWithin(worker, &response));
      }
      CHECK(response.state_seq == newest);
    }
    worker.Stop();
    CHECK(worker.replaced() > 0);
  }

  // A result is judged against the step the loop is waiting for, and every
  // part of that identity matters: the local generation, the authorization
  // epoch and the submission instant that names the step. A result of a
  // withdrawn authorization can never drive a frame of the current one, and a
  // result that outlived its own deadline is a failure whether the loop's
  // deadline fired first or not.
  {
    PolicyResponse response;
    response.generation = 7;
    response.epoch = 3;
    response.state_seq = 42;
    response.state_stamp_us = 1000;
    response.submitted_us = 2000;
    response.ok = true;
    response.action_count = 3;

    constexpr irmv::protocol::TimeUs kTimeout = 100000;
    CHECK(irmv::policy::JudgePolicyResult(response, 7, 3, 2000 + kTimeout,
                                          kTimeout, 2000) ==
          irmv::policy::PolicyVerdict::Usable);
    // The boundary is inclusive: a result that arrived exactly at the deadline
    // is still a step of this authorization, one microsecond later is not.
    CHECK(
        irmv::policy::JudgePolicyResult(response, 7, 3, 2000, kTimeout, 2000) ==
        irmv::policy::PolicyVerdict::Usable);
    CHECK(irmv::policy::JudgePolicyResult(response, 7, 3, 2000 + kTimeout + 1,
                                          kTimeout, 2000) ==
          irmv::policy::PolicyVerdict::Failed);

    // Another authorization, another epoch, another step: ignored, never a
    // fault and never a frame.
    CHECK(irmv::policy::JudgePolicyResult(response, 8, 3, 2000 + kTimeout,
                                          kTimeout, 2000) ==
          irmv::policy::PolicyVerdict::Ignored);
    CHECK(irmv::policy::JudgePolicyResult(response, 7, 4, 2000 + kTimeout,
                                          kTimeout, 2000) ==
          irmv::policy::PolicyVerdict::Ignored);
    CHECK(irmv::policy::JudgePolicyResult(response, 7, 3, 2000 + kTimeout,
                                          kTimeout, 1500) ==
          irmv::policy::PolicyVerdict::Ignored);

    // A failed step latches the fault at the same point a late one does.
    PolicyResponse failed = response;
    failed.ok = false;
    CHECK(irmv::policy::JudgePolicyResult(failed, 7, 3, 2000 + kTimeout,
                                          kTimeout, 2000) ==
          irmv::policy::PolicyVerdict::Failed);

    // The source state: one this loop has actually observed, and not older
    // than the runtime's own window. A result computed from a state ahead of
    // the newest one, from no state at all, or from one the runtime would
    // already refuse is not a frame to send.
    //
    // The window is inclusive, exactly like the runtime's own
    // `now_us - generation_us > source_state_us`: a state exactly at its edge
    // is still lookable up, one microsecond past it is not.
    constexpr irmv::protocol::TimeUs kSourceWindow = 150000;
    CHECK(irmv::policy::PolicySourceStateUsable(response, 100, 2000,
                                                kSourceWindow));
    CHECK(irmv::policy::PolicySourceStateUsable(
        response, 100, 1000 + kSourceWindow, kSourceWindow));
    CHECK(!irmv::policy::PolicySourceStateUsable(
        response, 100, 1000 + kSourceWindow + 1, kSourceWindow));
    CHECK(!irmv::policy::PolicySourceStateUsable(response, 41, 2000,
                                                 kSourceWindow));
    PolicyResponse no_state = response;
    no_state.state_seq = 0;
    CHECK(!irmv::policy::PolicySourceStateUsable(no_state, 100, 2000,
                                                 kSourceWindow));
    PolicyResponse no_stamp = response;
    no_stamp.state_stamp_us = 0;
    CHECK(!irmv::policy::PolicySourceStateUsable(no_stamp, 100, 2000,
                                                 kSourceWindow));
    PolicyResponse ahead_stamp = response;
    ahead_stamp.state_stamp_us = 2001;
    CHECK(!irmv::policy::PolicySourceStateUsable(ahead_stamp, 100, 2000,
                                                 kSourceWindow));
  }

  std::cout << "inference_test: ok\n";
  return 0;
}
