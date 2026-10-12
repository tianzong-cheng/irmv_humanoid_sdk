#include "irmv/policy/inference.hpp"

#include <utility>

namespace irmv::policy {

PolicyVerdict JudgePolicyResult(const PolicyResponse &response,
                                std::uint64_t generation,
                                protocol::Sequence epoch,
                                protocol::TimeUs now_us,
                                protocol::TimeUs timeout_us,
                                protocol::TimeUs last_submit_us) {
  if (response.generation != generation || response.epoch != epoch ||
      response.submitted_us != last_submit_us) {
    // Not the step this loop is waiting for: another authorization, another
    // epoch, or a step it already gave up on. Nothing of it may touch the
    // current one.
    return PolicyVerdict::Ignored;
  }
  if (now_us - response.submitted_us > timeout_us) {
    // Later than the deadline the configuration gave this source. The loop
    // latches the same fault here as the deadline does, so a result that
    // outsprinted the worker or a deadline that fired first agree on what the
    // outcome is.
    return PolicyVerdict::Failed;
  }
  return response.ok ? PolicyVerdict::Usable : PolicyVerdict::Failed;
}

bool PolicySourceStateUsable(const PolicyResponse &response,
                             protocol::Sequence newest_state_seq,
                             protocol::TimeUs newest_state_stamp_us,
                             protocol::TimeUs source_state_timeout_us) {
  if (response.state_seq == 0 || response.state_stamp_us == 0) {
    // A result that names no source state describes a step no state supports.
    return false;
  }
  if (response.state_seq > newest_state_seq ||
      response.state_stamp_us > newest_state_stamp_us) {
    // Ahead of the newest state this loop has: a sequence or a stamp the
    // runtime never published as the source of this step.
    return false;
  }
  return newest_state_stamp_us - response.state_stamp_us <=
         source_state_timeout_us;
}

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
    PolicyRequest request;
    {
      // The take and the reset of the flag are one critical section of the same
      // mutex Submit sets the flag under, and that is the whole point of the
      // ordering: a submission that arrives while this thread is waking up
      // either already published its request and finds the flag set here - so
      // it is the one taken - or it sets the flag after this section, and the
      // notification it sends finds the next wait either parked or about to
      // park. Splitting the two (taking with the flag still set, clearing it
      // afterwards) leaves a window in which that submission's flag is erased
      // by this thread: the newest request then stays in the channel with
      // nothing left to wake the worker for it.
      std::unique_lock<std::mutex> lock(mutex_);
      wakeup_.wait(lock, [this] { return pending_ || stopping_; });
      if (stopping_ && !pending_) {
        return;
      }
      if (!requests_.TryTake(request)) {
        // The flag was set by a submission whose request this take already
        // consumed: nothing is left for this wakeup. Clearing the flag parks
        // the thread on the next wait, and a submission that arrives after
        // this section sets it again.
        pending_ = false;
        continue;
      }
      // Cleared before the step runs, so a submission that arrives while this
      // one is being computed is not lost and is not mistaken for this one.
      pending_ = false;
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
