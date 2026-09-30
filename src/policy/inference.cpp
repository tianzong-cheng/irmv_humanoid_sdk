#include "irmv/policy/inference.hpp"

#include <utility>

namespace irmv::policy {

InferenceWorker::InferenceWorker(const OnnxModel &model) : model_(model) {
  worker_ = std::thread([this] { Run(); });
}

InferenceWorker::~InferenceWorker() { Stop(); }

void InferenceWorker::Submit(const PolicyRequest &request) {
  bool evicted = false;
  if (!requests_.Publish(request, &evicted)) {
    // No slot could be claimed, which the one-producer/one-consumer contract
    // makes unreachable; the value is counted by the channel and this call
    // reports it by not counting a submission.
    return;
  }
  submitted_.fetch_add(1);
  if (evicted) {
    replaced_.fetch_add(1);
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pending_ = true;
  }
  wakeup_.notify_one();
}

bool InferenceWorker::TryTake(PolicyResponse *response) {
  if (response == nullptr) {
    return false;
  }
  return responses_.TryTake(*response);
}

void InferenceWorker::Stop() {
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wakeup_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void InferenceWorker::Run() {
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wakeup_.wait(lock, [this] { return pending_ || stopping_; });
      if (stopping_ && !pending_) {
        return;
      }
    }

    PolicyRequest request;
    const bool taken = requests_.TryTake(request);
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      // Cleared before the step runs, so a submission that arrives while this
      // one is being computed is not lost and is not mistaken for this one.
      pending_ = false;
    }
    if (!taken) {
      // The request was superseded between the wakeup and the take: nothing is
      // lost, the newer one is published and the loop will wake again.
      continue;
    }

    PolicyResponse response;
    response.generation = request.generation;
    response.epoch = request.epoch;
    response.state_seq = request.state_seq;
    response.state_stamp_us = request.state_stamp_us;
    response.submitted_us = request.submitted_us;
    response.ok = model_.Run(request.observation, &response.action,
                             &response.action_count);

    bool evicted = false;
    // A response the caller did not collect is superseded by this one, which is
    // the same loss the request side counts: the newer step is the one that
    // matters.
    if (responses_.Publish(response, &evicted)) {
      if (response.ok) {
        completed_.fetch_add(1);
      } else {
        failed_.fetch_add(1);
      }
      if (evicted) {
        uncollected_.fetch_add(1);
      }
    } else {
      failed_.fetch_add(1);
    }
  }
}

}  // namespace irmv::policy
