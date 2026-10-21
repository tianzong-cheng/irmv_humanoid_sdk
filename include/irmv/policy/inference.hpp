#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
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

// What one finished result means for the authorization the supervision loop
// holds. The worker reports what the model did; only the loop knows which step
// it is waiting for, so the judgement belongs here and not in the worker.
enum class PolicyVerdict {
  // A result this loop is no longer waiting for: of another authorization or
  // epoch, or of a step it already wrote off. It drives nothing, and it is not
  // a fault - the loop's own state has moved past it.
  Ignored,
  // A usable step of the authorization the loop holds: its frame may be built.
  Usable,
  // The step failed, or its result arrived after the deadline the configuration
  // gave this source. The upper layer latches the local policy fault.
  Failed,
};

// Judges one result against the step the loop is waiting for.
//
// `last_submit_us` is the instant the outstanding step was submitted, which
// names it: the deadline can write a step off while the worker is still running
// it, so a result that arrives later than `timeout_us` after its own submission
// is not a step of this authorization any more, however well its identity
// matches. The identity is checked whole - the local generation and the
// authorization epoch as well as the submission instant - because a result of a
// withdrawn authorization must never drive a frame of the current one.
PolicyVerdict JudgePolicyResult(const PolicyResponse &response,
                                std::uint64_t generation,
                                protocol::Sequence epoch,
                                protocol::TimeUs now_us,
                                protocol::TimeUs timeout_us,
                                protocol::TimeUs last_submit_us);

// Whether the source state a result was computed from is one the runtime can
// still accept. The frame the loop builds is stamped with that sequence and
// stamp, and the runtime looks the sample up by them, so a state this loop
// never observed (a sequence ahead of the newest one) or one older than the
// runtime's own source-state window is refused here rather than sent to be
// refused there.
bool PolicySourceStateUsable(const PolicyResponse &response,
                             protocol::Sequence newest_state_seq,
                             protocol::TimeUs newest_state_stamp_us,
                             protocol::TimeUs source_state_timeout_us);

// Runs the model of one authorization on its own thread. The worker shares
// ownership of the model with its thread, so a cancellation that is still
// finishing cannot outlive the model it executes.
class InferenceWorker {
 public:
  // Starts the worker thread. Construction does not run an inference.
  explicit InferenceWorker(std::shared_ptr<const OnnxModel> model);
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

  // Rejects later submissions, requests cooperative termination of the
  // running inference through its owned Ort::RunOptions, and waits for the
  // worker against one deadline. Returns true when the worker finished and was
  // joined. On false the cancellation was not observed by an uninterruptible
  // kernel; the thread keeps only shared state alive, never this object, so
  // returning at the deadline is safe. Safe to call more than once and from the
  // destructor.
  bool Stop();

  // Requests accepted by the channel since construction.
  std::uint64_t submitted() const;
  // Requests replaced before the worker picked them up.
  std::uint64_t replaced() const;
  // Steps the model answered, with a usable action.
  std::uint64_t completed() const;
  // Steps the model could not answer, or answered with a value that is not
  // usable. Both are a failed policy step for the caller.
  std::uint64_t failed() const;
  // Responses replaced by a newer one before the caller collected them.
  std::uint64_t uncollected() const;

 private:
  struct Shared;

  static void Run(std::shared_ptr<Shared> shared);

  std::shared_ptr<Shared> shared_;
};

}  // namespace irmv::policy
