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
  };

  void Run();
  // Takes what the channel holds and writes it. Never blocks: it stops when the
  // channel is empty.
  void Drain();
  // One JSON object per line, flushed after the line so a killed process leaves
  // complete records.
  void WriteLine(const Queued &queued);

  std::string directory;
  std::ofstream file;
  std::thread worker;
  std::atomic<bool> started{false};
  std::atomic<bool> stop_requested{false};
  // How many control-thread calls are inside TryRecord. Stop() waits for this
  // to reach zero before it stops the worker, so no event can land behind the
  // tail the worker takes.
  std::atomic<std::uint32_t> offering{0};
  std::mutex mutex;
  std::condition_variable cv;
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
  // Stop() rejects new events before it signals, so what is left is exactly the
  // accepted events no pass has written yet: the tail is not a loss.
  Drain();
}

PolicySessionRecorder::PolicySessionRecorder(std::string directory)
    : impl_(std::make_unique<Impl>()) {
  Impl &impl = *impl_;
  impl.directory = std::move(directory);
  if (impl.directory.empty()) {
    impl.error = "the policy session record has no directory";
    return;
  }
  std::error_code code;
  if (!std::filesystem::is_directory(impl.directory, code) || code) {
    impl.error = "the policy session record directory " + impl.directory +
                 " does not exist";
    return;
  }
  // The file is opened for append so a directory shared by two lifetimes of the
  // recorder still reads as one stream of sessions.
  const std::filesystem::path path =
      std::filesystem::path(impl.directory) / std::string(kPolicySessionFile);
  impl.file.open(path, std::ios::app);
  if (!impl.file) {
    impl.error = "the policy session record could not open " + path.string();
    return;
  }
  impl.started.store(true, std::memory_order_release);
  impl.worker = std::thread([&impl] { impl.Run(); });
}

PolicySessionRecorder::~PolicySessionRecorder() { Stop(); }

bool PolicySessionRecorder::TryRecord(
    const PolicySessionEvent &event) noexcept {
  Impl &impl = *impl_;
  // Announce before reading started_, so Stop() can tell a push that is already
  // under way from one that has not begun. The announcement is two lock-free
  // increments and never waits.
  impl.offering.fetch_add(1, std::memory_order_acquire);
  bool accepted = false;
  if (impl.started.load(std::memory_order_acquire)) {
    try {
      Impl::Queued queued;
      queued.event = event;
      queued.host_us = HostMicros();
      queued.monotonic_us = MonotonicMicros();
      accepted = impl.queue.TryPush(queued);
    } catch (...) {
      // The copy is the only allocation on this path; a failure to make it
      // costs the event and is counted, and never reaches the caller.
      accepted = false;
    }
  }
  impl.offering.fetch_sub(1, std::memory_order_release);
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
  return impl_->started.load(std::memory_order_acquire);
}

const std::string &PolicySessionRecorder::error() const noexcept {
  return impl_->error;
}

void PolicySessionRecorder::Stop() {
  Impl &impl = *impl_;
  // Nothing is accepted from here on, and a control thread already inside
  // TryRecord is waited out: that freezes the queue for the tail the worker
  // takes next.
  impl.started.store(false, std::memory_order_release);
  while (impl.offering.load(std::memory_order_acquire) != 0) {
    std::this_thread::yield();
  }
  {
    const std::lock_guard<std::mutex> lock(impl.mutex);
    impl.stop_requested.store(true, std::memory_order_relaxed);
  }
  impl.cv.notify_all();
  if (impl.worker.joinable()) {
    impl.worker.join();
  }
  impl.file.flush();
}

}  // namespace irmv::application
