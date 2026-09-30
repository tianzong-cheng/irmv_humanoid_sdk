#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include "irmv/policy/observation.hpp"
#include "irmv/policy/onnx_model.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/bounded_channel.hpp"

// The inference worker of the local policy: one dedicated thread that runs the
// model off the supervision loop.
//
// The supervision loop must keep its tick, its heartbeat and its reaction to
// the lower layer whatever the model does, so it never runs an inference
// itself: it submits the newest observation through a bounded keep-last-1
// channel and picks the newest finished result up on a later tick. Neither side
// blocks the other, a submission that is superseded before the worker reached
// it is counted rather than queued, and a result the loop did not collect is
// replaced the same way.
//
// A request carries the identity of the step it belongs to - the local
// authorization generation, the epoch, the state sequence and stamp the
// observation was taken from, and the host instant it was submitted - and the
// response carries exactly the same identity back. The loop is what decides
// whether a result is still usable; the worker only runs the model and reports
// whether it produced one.
//
// The channel header is included from the transport tree because it is a
// header-only, dependency-free handoff utility; linking the transport (and with
// it DDS) into the policy library would be a different statement.
namespace irmv::policy {

// One submitted policy step.
struct PolicyRequest {
  // Local authorization generation: increments whenever the upper layer enters
  // a new authorization, so a result whose generation is older than the current
  // one can never drive a frame.
  std::uint64_t generation{0};
  protocol::Sequence epoch{0};
  protocol::Sequence state_seq{0};
  protocol::TimeUs state_stamp_us{0};
  protocol::TimeUs submitted_us{0};
  Observation observation;
};

// One finished policy step, carrying the identity of its request.
struct PolicyResponse {
  std::uint64_t generation{0};
  protocol::Sequence epoch{0};
  protocol::Sequence state_seq{0};
  protocol::TimeUs state_stamp_us{0};
  protocol::TimeUs submitted_us{0};
  // True when the model produced a usable action; false when the session, the
  // tensors or the values failed, in which case `action` is not used.
  bool ok{false};
  std::uint32_t action_count{0};
  std::array<float, protocol::kMaxJoints> action{};
};

// Runs the model of one authorization on its own thread. The model must outlive
// the worker.
class InferenceWorker {
 public:
  // Starts the worker thread. Construction does not run an inference.
  explicit InferenceWorker(const OnnxModel &model);
  ~InferenceWorker();

  InferenceWorker(const InferenceWorker &) = delete;
  InferenceWorker &operator=(const InferenceWorker &) = delete;
  InferenceWorker(InferenceWorker &&) = delete;
  InferenceWorker &operator=(InferenceWorker &&) = delete;

  // Hands over the newest request. Never blocks and never allocates: an
  // unconsumed request is replaced and counted in replaced().
  void Submit(const PolicyRequest &request);

  // Takes the newest finished response, if one is waiting. Never blocks.
  bool TryTake(PolicyResponse *response);

  // Requests the worker to finish the step it is running and join it. Safe to
  // call more than once and from the destructor.
  void Stop();

  // Requests accepted by the channel since construction.
  std::uint64_t submitted() const { return submitted_.load(); }
  // Requests replaced before the worker picked them up.
  std::uint64_t replaced() const { return replaced_.load(); }
  // Steps the model answered, with a usable action.
  std::uint64_t completed() const { return completed_.load(); }
  // Steps the model could not answer, or answered with a value that is not
  // usable. Both are a failed policy step for the caller.
  std::uint64_t failed() const { return failed_.load(); }
  // Responses replaced by a newer one before the caller collected them.
  std::uint64_t uncollected() const { return uncollected_.load(); }

 private:
  void Run();

  const OnnxModel &model_;
  transport::LatestValueChannel<PolicyRequest> requests_;
  transport::LatestValueChannel<PolicyResponse> responses_;
  // Wakes the worker only; the two channels carry the payloads.
  std::mutex mutex_;
  std::condition_variable wakeup_;
  bool pending_{false};
  bool stopping_{false};
  std::thread worker_;
  std::atomic<std::uint64_t> submitted_{0};
  std::atomic<std::uint64_t> replaced_{0};
  std::atomic<std::uint64_t> completed_{0};
  std::atomic<std::uint64_t> failed_{0};
  std::atomic<std::uint64_t> uncollected_{0};
};

}  // namespace irmv::policy
