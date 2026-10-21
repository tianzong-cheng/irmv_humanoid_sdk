#include "irmv/policy/inference.hpp"

#include <chrono>
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

namespace {

// The one deadline Stop() waits for. ONNX cancellation is cooperative; an
// execution that never observes the terminate flag cannot be joined safely
// from here, and the worker then keeps only its shared state alive.
constexpr std::uint32_t kInferenceStopTimeoutMs = 2000;

}  // namespace

struct InferenceWorker::Shared {
  explicit Shared(std::shared_ptr<const OnnxModel> worker_model)
      : model(std::move(worker_model)) {}

  std::shared_ptr<const OnnxModel> model;
  // Owned by this worker and handed to every Run it starts, so Stop() can
  // terminate the call that is really executing.
  RunCancellation cancellation;
  transport::LatestValueChannel<PolicyRequest> requests;
  transport::LatestValueChannel<PolicyResponse> responses;
  // Wakes the worker only; the two channels carry the payloads.
  std::mutex mutex;
  std::condition_variable wakeup;
  std::condition_variable finished_cv;
  bool pending{false};
  bool stop_requested{false};
  std::atomic<bool> accepting{true};
  bool finished{false};
  std::thread worker;
  std::atomic<std::uint64_t> submitted{0};
  std::atomic<std::uint64_t> replaced{0};
  std::atomic<std::uint64_t> completed{0};
  std::atomic<std::uint64_t> failed{0};
  std::atomic<std::uint64_t> uncollected{0};
};

InferenceWorker::InferenceWorker(std::shared_ptr<const OnnxModel> model)
    : shared_(std::make_shared<Shared>(std::move(model))) {
  const std::shared_ptr<Shared> shared = shared_;
  shared_->worker = std::thread([shared] { InferenceWorker::Run(shared); });
}

InferenceWorker::~InferenceWorker() { static_cast<void>(Stop()); }

void InferenceWorker::Submit(const PolicyRequest &request) {
  Shared &shared = *shared_;
  if (!shared.accepting.load(std::memory_order_acquire)) {
    // Shutdown has already rejected new work.
    return;
  }
  bool evicted = false;
  if (!shared.requests.Publish(request, &evicted)) {
    // No slot could be claimed, which the one-producer/one-consumer contract
    // makes unreachable; the value is counted by the channel and this call
    // reports it by not counting a submission.
    return;
  }
  shared.submitted.fetch_add(1);
  if (evicted) {
    shared.replaced.fetch_add(1);
  }
  {
    const std::lock_guard<std::mutex> lock(shared.mutex);
    shared.pending = true;
  }
  shared.wakeup.notify_one();
}

bool InferenceWorker::TryTake(PolicyResponse *response) {
  if (response == nullptr) {
    return false;
  }
  return shared_->responses.TryTake(*response);
}

bool InferenceWorker::Stop() {
  const std::shared_ptr<Shared> shared = shared_;
  if (!shared) {
    return true;
  }
  {
    const std::lock_guard<std::mutex> lock(shared->mutex);
    // Reject first, then cancel: a submission that passed the accepting check
    // may still be published, but the worker returns on the stop request and
    // does not start a new step for it.
    shared->accepting.store(false, std::memory_order_release);
    shared->stop_requested = true;
    shared->pending = false;
  }
  shared->cancellation.Request();
  shared->wakeup.notify_all();

  bool finished = false;
  {
    std::unique_lock<std::mutex> lock(shared->mutex);
    finished = shared->finished_cv.wait_for(
        lock, std::chrono::milliseconds(kInferenceStopTimeoutMs),
        [&shared] { return shared->finished; });
  }
  if (finished) {
    if (shared->worker.joinable()) {
      shared->worker.join();
    }
    return true;
  }
  // The kernel did not observe the terminate flag inside the deadline. The
  // thread references only `shared`, so detaching is safe; returning false
  // tells the caller the inference was not proven stopped. A strict bound on
  // uninterruptible kernels belongs in process isolation, not in a join that
  // could block the control loop forever.
  if (shared->worker.joinable()) {
    shared->worker.detach();
  }
  return false;
}

std::uint64_t InferenceWorker::submitted() const {
  return shared_->submitted.load();
}

std::uint64_t InferenceWorker::replaced() const {
  return shared_->replaced.load();
}

std::uint64_t InferenceWorker::completed() const {
  return shared_->completed.load();
}

std::uint64_t InferenceWorker::failed() const { return shared_->failed.load(); }

std::uint64_t InferenceWorker::uncollected() const {
  return shared_->uncollected.load();
}

void InferenceWorker::Run(std::shared_ptr<Shared> shared) {
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
      std::unique_lock<std::mutex> lock(shared->mutex);
      shared->wakeup.wait(lock, [&shared] {
        return shared->pending || shared->stop_requested;
      });
      if (shared->stop_requested) {
        break;
      }
      if (!shared->requests.TryTake(request)) {
        // The flag was set by a submission whose request this take already
        // consumed: nothing is left for this wakeup. Clearing the flag parks
        // the thread on the next wait, and a submission that arrives after
        // this section sets it again.
        shared->pending = false;
        continue;
      }
      // Cleared before the step runs, so a submission that arrives while this
      // one is being computed is not lost and is not mistaken for this one.
      shared->pending = false;
    }

    PolicyResponse response;
    response.generation = request.generation;
    response.epoch = request.epoch;
    response.state_seq = request.state_seq;
    response.state_stamp_us = request.state_stamp_us;
    response.submitted_us = request.submitted_us;
    response.ok =
        shared->model->Run(request.observation, &response.action,
                           &response.action_count, &shared->cancellation);

    bool evicted = false;
    // A response the caller did not collect is superseded by this one, which is
    // the same loss the request side counts: the newer step is the one that
    // matters.
    if (shared->responses.Publish(response, &evicted)) {
      if (response.ok) {
        shared->completed.fetch_add(1);
      } else {
        shared->failed.fetch_add(1);
      }
      if (evicted) {
        shared->uncollected.fetch_add(1);
      }
    } else {
      shared->failed.fetch_add(1);
    }
  }
  {
    const std::lock_guard<std::mutex> lock(shared->mutex);
    shared->finished = true;
  }
  shared->finished_cv.notify_all();
}

}  // namespace irmv::policy
