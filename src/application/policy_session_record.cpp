#include "irmv/application/policy_session_record.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>

#include "irmv/transport/bounded_channel.hpp"

namespace irmv::application {
namespace {

// A 16-byte identity as its lower-case hex form, the same rendering the run
// recorder's session sidecar uses: an identity is opaque text, not a number.
std::string IdHex(const protocol::Id &id) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string text(2 * id.size(), '0');
  for (std::size_t index = 0; index < id.size(); ++index) {
    text[2 * index] = kHex[(id[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHex[id[index] & 0x0f];
  }
  return text;
}

// The producer identity of an external session, or the empty string when no
// producer was involved. The all-zero identity is the protocol's "no producer",
// which is exactly a local session; a real identity is never empty.
std::string ProducerIdHex(const protocol::Id &id) {
  if (id == protocol::kNoId) {
    return std::string();
  }
  return IdHex(id);
}

std::uint64_t HostMicros() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

std::uint64_t MonotonicMicros() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

// The hook and its context are replaced as one pair under a mutex, so the
// worker never observes a function from one installation and the context of
// another. The mutex is leaked on purpose: a timed-out Stop() detaches the
// worker, which may still be writing during static destruction.
std::mutex &PolicyLineHookMutex() {
  static std::mutex *const mutex = new std::mutex;
  return *mutex;
}

void (*g_policy_line_hook)(void *, std::string_view) = nullptr;
void *g_policy_line_hook_context = nullptr;

}  // namespace

void FillGainOverrides(const policy::ResolvedGains &gains,
                       std::vector<PolicyGainOverride> *out) {
  if (out == nullptr) {
    return;
  }
  out->clear();
  // The policy layer validates its own counts; the clamp is a defensive bound
  // on the fixed array, never a routine path.
  const std::size_t count =
      std::min<std::size_t>(gains.override_count, protocol::kMaxJoints);
  out->reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    out->push_back(gains.overrides[index]);
  }
}

void FillFrameGains(const protocol::JointCommand &frame,
                    const std::vector<std::string> &joint_names,
                    std::vector<PolicyGainOverride> *out) {
  if (out == nullptr) {
    return;
  }
  out->clear();
  const std::size_t count = frame.joints.count;
  // A frame the canonical table cannot name completely is refused whole: a
  // partial set of names would be worse than no rows, because a reader could
  // not tell which joints the rows meant.
  if (count > joint_names.size()) {
    return;
  }
  out->reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    PolicyGainOverride gain;
    if (!gain.joint.Assign(joint_names[index])) {
      // A canonical name longer than the identity field cannot be written
      // truthfully, so the whole set is refused rather than truncated.
      out->clear();
      return;
    }
    // A frame gain was declared by the source and applied untouched, so the
    // declared and effective values are the same and neither flag claims an
    // override.
    gain.model_kp = frame.joints.kp[index];
    gain.applied_kp = gain.model_kp;
    gain.model_kd = frame.joints.kd[index];
    gain.applied_kd = gain.model_kd;
    out->push_back(gain);
  }
}

struct PolicySessionRecorder::Impl {
  // One event as the control thread handed it over: the copy of the event and
  // the two clock reads taken where it happened, not where it was written.
  struct Queued {
    PolicySessionEvent event;
    std::uint64_t host_us{0};
    std::uint64_t monotonic_us{0};
    std::uint64_t logical_us{0};
  };

  void Run();
  // Takes what the channel holds and writes it. Never blocks: it stops when the
  // channel is empty.
  void Drain();
  // One JSON object per line, flushed after the line so a killed process leaves
  // complete records.
  void WriteLine(const Queued &queued);

  // Shared ownership: the worker captures this state, not the recorder
  // object, so a timed-out Stop() leaves the worker with everything it needs.
  std::string directory;
  std::ofstream file;
  std::thread worker;
  std::atomic<bool> started{false};
  std::atomic<bool> stop_requested{false};
  std::mutex mutex;
  std::condition_variable cv;
  bool stop_signalled{false};
  bool finished{false};
  // How many control-thread calls are inside TryRecord. The worker waits for
  // this to reach zero before its final drain, so no event can land behind the
  // tail it takes.
  std::atomic<std::uint32_t> offering{0};
  std::atomic<std::uint64_t> recorded{0};
  std::atomic<std::uint64_t> dropped{0};
  std::atomic<std::uint64_t> write_failures{0};
  std::string error;
  transport::BoundedChannel<Queued, kPolicySessionQueueCapacity> queue;
};

void PolicySessionRecorder::Impl::WriteLine(const Queued &queued) {
  const PolicySessionEvent &event = queued.event;
  // The gains name exactly the differences that were applied for a local
  // session: a joint no override matched has no row, and a gain that was not
  // overridden on a matched joint has none either. An external session has no
  // overrides - the application changed none of the source's gains - so both
  // kinds of every declared joint are written, with model and effective equal.
  // The order is the policy layer's - one joint at a time, kp before kd - so
  // two runs of the same gains write the same bytes.
  nlohmann::json gains = nlohmann::json::array();
  for (const PolicyGainOverride &gain : event.gains) {
    const std::string joint(gain.joint.View());
    if (event.external || gain.kp_overridden) {
      gains.push_back({{"joint", joint},
                       {"gain", "kp"},
                       {"model", gain.model_kp},
                       {"effective", gain.applied_kp}});
    }
    if (event.external || gain.kd_overridden) {
      gains.push_back({{"joint", joint},
                       {"gain", "kd"},
                       {"model", gain.model_kd},
                       {"effective", gain.applied_kd}});
    }
  }
  nlohmann::json model;
  model["name"] = event.model_name;
  model["sha256"] = event.model_sha256;
  model["path"] = event.model_path;
  model["input"] = event.model_input;
  model["output"] = event.model_output;
  model["contract_sha256"] = event.contract_sha256;

  nlohmann::json entry;
  entry["schema"] = std::string(kPolicySessionSchema);
  entry["version"] = kPolicySessionSchemaVersion;
  entry["host_us"] = queued.host_us;
  entry["monotonic_us"] = queued.monotonic_us;
  if (queued.logical_us) entry["logical_us"] = queued.logical_us;
  entry["runtime_id"] = IdHex(event.runtime_id);
  entry["session_id"] = IdHex(event.session_id);
  entry["producer_process_id"] = ProducerIdHex(event.producer_process_id);
  entry["epoch"] = event.epoch;
  entry["source"] = event.external ? "external" : "local";
  entry["model"] = std::move(model);
  entry["gains"] = std::move(gains);

  std::string line;
  try {
    line = entry.dump();
  } catch (...) {
    // A value the writer cannot render is a lost line, not a dead worker.
    write_failures.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  void (*hook)(void *, std::string_view) = nullptr;
  void *hook_context = nullptr;
  {
    const std::lock_guard<std::mutex> lock(PolicyLineHookMutex());
    hook = g_policy_line_hook;
    hook_context = g_policy_line_hook_context;
  }
  if (hook != nullptr) {
    hook(hook_context, line);
  }
  file << line << '\n';
  file.flush();
  if (!file.good()) {
    write_failures.fetch_add(1, std::memory_order_relaxed);
  }
}

void PolicySessionRecorder::Impl::Drain() {
  Queued queued;
  while (queue.TryPop(queued)) {
    WriteLine(queued);
  }
}

void PolicySessionRecorder::Impl::Run() {
  while (!stop_requested.load(std::memory_order_relaxed)) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait_for(lock, std::chrono::milliseconds(50), [this] {
        return stop_requested.load(std::memory_order_relaxed);
      });
    }
    Drain();
  }
  // Stop() has already refused new events. Wait for a control thread that was
  // inside TryRecord to leave, so what remains is exactly the accepted events
  // no pass has written yet: the tail is not a loss. The sequential-consistent
  // handshake makes this zero observation final; a caller that announced
  // after it must have seen the refusal and never touched the queue.
  for (;;) {
    if (offering.load(std::memory_order_seq_cst) == 0) {
      break;
    }
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait_for(lock, std::chrono::milliseconds(10));
  }
  Drain();
  file.flush();
  // Publish completion. A Stop() that timed out has detached this thread; the
  // shared state keeps the objects below alive until this point.
  {
    const std::lock_guard<std::mutex> lock(mutex);
    finished = true;
  }
  cv.notify_all();
}

PolicySessionRecorder::PolicySessionRecorder(std::string directory)
    : impl_(std::make_shared<Impl>()) {
  const std::shared_ptr<Impl> impl = impl_;
  impl->directory = std::move(directory);
  if (impl->directory.empty()) {
    impl->error = "the policy session record has no directory";
    return;
  }
  std::error_code code;
  if (!std::filesystem::is_directory(impl->directory, code) || code) {
    impl->error = "the policy session record directory " + impl->directory +
                  " does not exist";
    return;
  }
  // The file is opened for append so a directory shared by two lifetimes of the
  // recorder still reads as one stream of sessions.
  const std::filesystem::path path =
      std::filesystem::path(impl->directory) / std::string(kPolicySessionFile);
  impl->file.open(path, std::ios::app);
  if (!impl->file) {
    impl->error = "the policy session record could not open " + path.string();
    return;
  }
  impl->started.store(true, std::memory_order_seq_cst);
  impl->worker = std::thread([impl] { impl->Run(); });
}

PolicySessionRecorder::~PolicySessionRecorder() { Stop(); }

bool PolicySessionRecorder::TryRecord(const PolicySessionEvent &event,
                                      std::uint64_t logical_us) noexcept {
  Impl &impl = *impl_;
  // Announce before reading started_, so Stop() can tell a push that is already
  // under way from one that has not begun. The announcement is two lock-free
  // increments and never waits. The counts and the refusal store are
  // sequentially consistent, so the worker's final zero observation is final.
  impl.offering.fetch_add(1, std::memory_order_seq_cst);
  bool accepted = false;
  if (impl.started.load(std::memory_order_seq_cst)) {
    try {
      Impl::Queued queued;
      queued.event = event;
      queued.host_us = HostMicros();
      queued.monotonic_us = MonotonicMicros();
      queued.logical_us = logical_us;
      accepted = impl.queue.TryPush(queued);
    } catch (...) {
      // The copy is the only allocation on this path; a failure to make it
      // costs the event and is counted, and never reaches the caller.
      accepted = false;
    }
  }
  impl.offering.fetch_sub(1, std::memory_order_seq_cst);
  if (accepted) {
    impl.recorded.fetch_add(1, std::memory_order_relaxed);
  } else {
    impl.dropped.fetch_add(1, std::memory_order_relaxed);
  }
  return accepted;
}

std::uint64_t PolicySessionRecorder::recorded() const noexcept {
  return impl_->recorded.load(std::memory_order_relaxed);
}

std::uint64_t PolicySessionRecorder::dropped() const noexcept {
  return impl_->dropped.load(std::memory_order_relaxed);
}

std::uint64_t PolicySessionRecorder::write_failures() const noexcept {
  return impl_->write_failures.load(std::memory_order_relaxed);
}

bool PolicySessionRecorder::started() const noexcept {
  return impl_->started.load(std::memory_order_seq_cst);
}

const std::string &PolicySessionRecorder::error() const noexcept {
  return impl_->error;
}

bool PolicySessionRecorder::Stop() {
  if (!impl_) {
    return true;
  }
  const std::shared_ptr<Impl> impl = impl_;
  {
    const std::lock_guard<std::mutex> lock(impl->mutex);
    if (impl->finished) {
      if (impl->worker.joinable()) {
        impl->worker.join();
      }
      return true;
    }
    // A previous Stop already timed out and left the worker detached. Waiting
    // again would give one exit path two full deadlines.
    if (impl->stop_signalled && !impl->worker.joinable()) {
      return false;
    }
    // A recorder whose construction failed has no worker to wait for.
    if (!impl->worker.joinable() &&
        !impl->started.load(std::memory_order_seq_cst)) {
      impl->finished = true;
      return true;
    }
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kPolicySessionStopTimeoutMs);
  // Refuse every later event first. The worker waits for any control thread
  // still inside TryRecord and then drains the accepted tail; this caller only
  // waits for the worker against the deadline.
  impl->started.store(false, std::memory_order_seq_cst);
  {
    const std::lock_guard<std::mutex> lock(impl->mutex);
    if (!impl->stop_signalled) {
      impl->stop_signalled = true;
      impl->stop_requested.store(true, std::memory_order_relaxed);
    }
  }
  impl->cv.notify_all();

  bool finished = false;
  {
    std::unique_lock<std::mutex> lock(impl->mutex);
    finished =
        impl->cv.wait_until(lock, deadline, [&impl] { return impl->finished; });
  }
  if (finished) {
    if (impl->worker.joinable()) {
      impl->worker.join();
    }
    return true;
  }
  // A blocked writer must not hold the process exit. The worker only references
  // this shared state, which it keeps alive itself, and it is the only writer
  // from here on; Stop() returns without joining or touching the file.
  if (impl->worker.joinable()) {
    impl->worker.detach();
  }
  return false;
}

void SetPolicySessionLineWriteHookForTest(void (*hook)(void *context,
                                                       std::string_view line),
                                          void *context) noexcept {
  const std::lock_guard<std::mutex> lock(PolicyLineHookMutex());
  g_policy_line_hook = hook;
  g_policy_line_hook_context = context;
}

}  // namespace irmv::application
