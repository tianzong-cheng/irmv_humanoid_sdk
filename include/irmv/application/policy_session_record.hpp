#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "irmv/policy/action.hpp"
#include "irmv/protocol/types.hpp"

// The application-side half of the recorded session: what the run was joined
// to. The runtime's own session event cannot carry it - the runtime never
// reads the ONNX deployment contract and must not link the inference stack, so
// its `RecordSessionEvent` names the model `unknown`. The model that was
// actually armed, the deployment contract it declared and the gains the
// operator overrode are facts only the application holds, and the review
// finding was that a recorded run could not be joined to the model that
// produced it. This module records exactly those facts.
//
// The split is the same as the run recorder's: the control path enqueues one
// bounded copy and returns, and a worker thread owns the file. `TryRecord`
// never blocks and never throws; a full handoff or a stopped recorder costs
// the event and is counted. The file is one JSON object per line, flushed after
// every line, so a process that is killed still leaves whole lines and never a
// half-written record a reader would have to guess about.
namespace irmv::application {

// One gain the model declared and the value the deployment actually armed, for
// the run log. It is the policy layer's own record of an applied override - a
// joint no override matched never gets a row - copied whole so a reader can see
// both sides of every difference the experiment made.
using PolicyGainOverride = policy::GainOverride;

// Sessions the bounded handoff holds. A session is not a tick: a run adopts a
// handful of contexts, not one per step, so the queue is deliberately tiny and
// the control thread never pays for a slow disk. A full handoff costs the event
// and is counted, exactly like the run recorder's sample queue.
inline constexpr std::size_t kPolicySessionQueueCapacity = 16;

// The sidecar's name and schema identity. A reader refuses an object of another
// schema or version instead of reinterpreting its fields.
inline constexpr std::string_view kPolicySessionFile = "policy_sessions.jsonl";
inline constexpr std::string_view kPolicySessionSchema = "irmv.policy_session";
inline constexpr std::uint32_t kPolicySessionSchemaVersion = 1;

// Everything the record needs that lives on the control path at the moment a
// locomotion context is confirmed. Plain data: the caller copies into it.
//
// The identity is the join key the runtime's session event uses -
// `{runtime_id, session_id, producer_process_id, epoch}` - so a reader can
// match the two records of one context. `producer_process_id` and `epoch` are
// empty/zero for a local session, and `external` says which source ran.
struct PolicySessionEvent {
  protocol::Id runtime_id{};
  protocol::Id producer_process_id{};  // all-zero for a local session
  protocol::Id session_id{};
  protocol::Sequence epoch{0};  // 0 for a local session
  bool external{false};
  // The model that was armed, as the policy layer reported it when it loaded.
  // `contract_sha256` is the digest of the `irmv.deployment` metadata the model
  // carried; it is never derived here.
  std::string model_sha256;
  std::string model_path;
  std::string model_input;
  std::string model_output;
  std::string contract_sha256;
  // The overrides that were applied, in the order the policy layer resolved
  // them: the model's own value and the effective value, per joint.
  std::vector<PolicyGainOverride> gains;
};

// Fills `out` with the gain rows of a local session: exactly the overrides the
// policy layer applied to this model, copied whole and in the order it resolved
// them, so the record names every difference the experiment made and nothing
// else. `out` is cleared first; a null pointer is ignored. The only allocation
// is the vector's, which is the caller's.
void FillGainOverrides(const policy::ResolvedGains &gains,
                       std::vector<PolicyGainOverride> *out);

// Fills `out` with the gain rows of an external session: one kp row and one kd
// row per joint of the first frame the runtime adopted, in the frame's own
// order, with the value the source declared. Nothing was overridden on this
// side, so the declared and effective values are the same and the overridden
// flags stay false. The joint names come from the deployment's canonical table,
// which is the order the frame's joints are in; a frame longer than that table
// produces no rows at all rather than names that would not mean anything. `out`
// is cleared first, a null pointer is ignored, and a frame of zero joints
// leaves it empty.
void FillFrameGains(const protocol::JointCommand &frame,
                    const std::vector<std::string> &joint_names,
                    std::vector<PolicyGainOverride> *out);

// Appends one JSON line per accepted session to
// `<directory>/policy_sessions.jsonl`. The directory must exist; the file is
// created if it is missing and appended to otherwise, so a run that is split
// across recorder lifetimes still reads as one file.
class PolicySessionRecorder {
 public:
  // Opens the sidecar and starts one worker thread. A directory that does not
  // exist, or a file that cannot be opened, is reported through error() - the
  // recorder is left stopped and TryRecord refuses every event - and is never
  // an exception.
  explicit PolicySessionRecorder(std::string directory);
  ~PolicySessionRecorder();
  PolicySessionRecorder(const PolicySessionRecorder &) = delete;
  PolicySessionRecorder &operator=(const PolicySessionRecorder &) = delete;

  // The control path: enqueue a copy, never block, never throw. Returns false
  // when the bounded queue was full or the recorder is not running; the event
  // is counted as dropped either way. The copy is the one allocation this call
  // can make, and a failure to allocate is a dropped event, not an exception.
  bool TryRecord(const PolicySessionEvent &event) noexcept;

  // Events the worker wrote to the sidecar.
  std::uint64_t recorded() const noexcept;
  // Events the recorder refused: the queue was full, or it was not running.
  std::uint64_t dropped() const noexcept;
  // Lines the worker could not write: a full disk or a removed directory. The
  // run keeps accepting until it is stopped; the loss is named rather than
  // silent.
  std::uint64_t write_failures() const noexcept;
  // True while the sidecar is open and the worker is running.
  bool started() const noexcept;
  // Empty on success; otherwise why the sidecar is not running. Never thrown
  // from TryRecord.
  const std::string &error() const noexcept;
  // Stops the worker after draining what it accepted, then closes the file.
  // Idempotent, and called by the destructor.
  void Stop();

 private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace irmv::application
