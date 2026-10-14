#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/telemetry/record.hpp"
#include "irmv/transport/bounded_channel.hpp"

// The run recorder: a bounded, non-blocking handoff from the control loop to a
// worker that writes one columnar chunk per stretch of samples, plus the
// metadata and index a reader needs to walk a run by time.
//
// The control thread only ever calls TryRecord, which copies the sample into a
// fixed-capacity queue and returns. It never allocates, formats, touches a file
// or waits for the worker; a full queue costs the sample and is counted, which
// is what keeps a slow disk from delaying the control tick and therefore the
// machine.
//
// The worker owns everything else: the chunk buffers, the files and every loss
// it can see from its side. A chunk is written under a temporary name and
// renamed into place only once it is complete, so a run that ends abruptly has
// an identifiable tail rather than a half-written block a reader would have to
// guess about. A chunk also carries an explicit payload length and a CRC-32
// over its own header and over its payload, so a reader can tell a chunk that
// was corrupted in place from one that was cut short and refuse the former
// instead of reading its rows. Every column of a chunk is contiguous, and the
// joint columns are `rows * joint_count` wide rather than `rows * kMaxJoints`,
// so a twelve-joint deployment does not pay for the sixty-four the protocol
// allows.
//
// The run's loss table is the one thing the worker cannot finish alone: samples
// can still be sitting in the handoff when the worker is done, and only Stop()
// sees them. Stop() therefore takes that tail, folds in the refusals the
// control thread counted, and writes the index the same table gaps() returns -
// after the worker has been joined, on the path that is already shutting the
// run down.
//
// Nothing here decides what to record: the runtime builds the sample and this
// component stores it, reports what it could not store, and keeps the control
// loop out of it.
namespace irmv::telemetry {

// Rows one chunk holds before the worker closes it. A chunk is the unit a
// reader locates by time, so it is small enough to find a range quickly and
// large enough that a run is not thousands of files: 900 rows is nine seconds
// at the default 100 Hz.
inline constexpr std::size_t kRecordChunkRows = 900;

// The largest handoff this build can hold. The channel is fixed capacity, so a
// configuration may ask for less but never for more: Start() refuses a larger
// request with a message instead of silently imposing its own bound. At 100 Hz
// 256 samples is roughly two seconds of a slow disk falling behind.
inline constexpr std::size_t kRecordQueueCapacity = 256;

// The session channel's fixed capacity. Sessions are not samples: the runtime
// emits one event per authorization context it adopts - a handful per run, not
// one per tick - on a second bounded handoff of its own. A full channel costs
// the event and is counted, exactly like a full sample queue: the control
// thread never waits for it.
inline constexpr std::size_t kRecordSessionQueueCapacity = 64;

// The refusal stretches the control thread reports in one run. The control
// thread tracks the losses the full handoff costs it, and it may not allocate,
// so its table is fixed capacity; the worker's own losses (a chunk it could not
// write, the tail Stop() could not drain) are recorded beside it. Once this
// many separate stretches are recorded, a further one is folded into the last
// QueueFull stretch already there - that stretch is extended to cover it - so a
// pathological run reports one coarse loss rather than silence. The resolution
// degrades past this bound; the loss itself is still named.
inline constexpr std::size_t kRecordGapSegments = 64;

// One run's configuration.
struct RecorderConfig {
  // Whether the recorder writes anything at all. A disabled recorder accepts no
  // samples and opens no file: this is the switch the configuration exposes.
  bool enabled{false};
  // The sampling rate of the *recorder*, which is independent of the control
  // rate: the runtime offers the control tick to the recorder and the
  // recorder's caller decides which ticks those are.
  std::uint32_t sample_hz{100};
  // How many control ticks pass between two recorded samples. The recorder is
  // told this because the gap detector works in the sequence the runtime
  // published: consecutive rows of a run sampled every fifth tick differ by
  // five, and calling that a gap would make every run report a hole per row.
  std::uint32_t stride{1};
  // Samples the handoff holds. Refused by Start() when it is above the fixed
  // capacity of this build.
  std::size_t queue_capacity{kRecordQueueCapacity};
  // Where one run's directory is created. The directory itself must exist.
  std::filesystem::path directory;
};

// What the control thread gives the recorder once per tick it sampled.
class Recorder {
 public:
  Recorder();
  ~Recorder();

  Recorder(const Recorder &) = delete;
  Recorder &operator=(const Recorder &) = delete;

  // Prepares a run directory and starts the worker. Returns false - with
  // `error` filled in - when the run cannot be created; no recorder state is
  // left behind in that case. `meta` is written as the run's metadata file; it
  // is what makes a directory self-describing (schema version, configuration
  // and layout digests, the joint table, the units and the sampling rate).
  bool Start(const RecorderConfig &config, std::string meta_json,
             std::string *error);

  // Offers one sample. Never blocks, never allocates, never throws and never
  // touches a file. Returns false when the sample was not accepted: the
  // recorder is disabled or stopped, or the queue is full. Every refusal is
  // counted and visible through dropped(), and a run of refusals is reported as
  // the stretch of the sequence it covers.
  bool TryRecord(const RecordSample &sample) noexcept;

  // Offers one session event on the recorder's second bounded handoff. Like
  // TryRecord it never blocks, allocates or throws: a full session channel or a
  // stopped recorder costs the event, counted in session_dropped(). The worker
  // writes what it accepted to `sessions.json` in the run directory.
  bool TryRecordSession(const RecordSessionEvent &event) noexcept;

  // Stops the worker after draining what it can, closes the run and writes the
  // index: the same gap table gaps() returns. Idempotent, and called by the
  // destructor. A drain that does not finish inside the bound is reported by
  // returning false rather than waiting for the disk: the control loop that
  // owns this call has its own deadlines.
  bool Stop();

  bool started() const;
  // The directory of the run this recorder created. It keeps naming it after
  // Stop(), so a caller can still read the run it just closed.
  const std::filesystem::path &run_directory() const;

  // Samples the recorder refused because the queue was full or because it was
  // not running.
  std::uint64_t dropped() const;
  // Session events the recorder refused because the session channel was full or
  // because it was not running. A loss here costs only the event, never a tick.
  std::uint64_t session_dropped() const;
  // Session events the worker wrote to `sessions.json`.
  std::uint64_t sessions_written() const;
  // Samples the handoff accepted. A chunk that could not be written is a gap,
  // so this is what the recorder was given, not always what a reader finds.
  std::uint64_t recorded() const;
  // Chunks the worker closed and renamed into place.
  std::uint64_t chunks() const;
  // Chunk writes that failed. The run keeps going, and the stretch they cover
  // is a gap the index reports.
  std::uint64_t write_failures() const;
  // The run's losses, in the order of the sequence they cover, each with the
  // sequence and stamp range of the samples it stands for. Complete once Stop()
  // returns, and the same list the index's `gaps` array holds.
  std::vector<RecordGap> gaps() const;

 private:
  struct Impl;

  // One stretch of loss, in the sequence space the runtime published and the
  // stamps it published it with. The control thread records the refusals of the
  // handoff as these, because a refused sample never reaches the worker: only
  // the side that saw the loss can name the stretch it covers.
  struct RefusalSpan {
    protocol::Sequence first_seq{0};
    protocol::Sequence last_seq{0};
    protocol::TimeUs first_stamp_us{0};
    protocol::TimeUs last_stamp_us{0};
  };

  // The body of TryRecord, run with this thread announced in `offering_`.
  bool OfferSample(const RecordSample &sample) noexcept;
  // Records one refused sample in the stretch being tracked. The stretch grows
  // while the refusals are the same sample offered again or the next sample of
  // the run's stride; anything else, or a sample recorded since the stretch
  // opened, ends it and starts a new one.
  void NoteRefusal(const RecordSample &sample) noexcept;
  // Closes the open stretch into the table, or folds it into the last entry
  // when the table is full. See kRecordGapSegments.
  void CloseRefusal() noexcept;
  // Files one span in the table under the overflow rule. Shared with Stop(),
  // which files the stretch that was still open when the run was closed.
  static void PutRefusal(RefusalSpan *table, std::uint32_t *count,
                         const RefusalSpan &span) noexcept;
  // Appends the refusal stretches, and the stretch still open, to `gaps`.
  void AppendRefusalGaps(std::vector<RecordGap> *gaps) const;

  std::unique_ptr<Impl> impl_;
  std::filesystem::path run_directory_;
  std::atomic<bool> started_{false};
  transport::BoundedChannel<RecordSample, kRecordQueueCapacity> queue_;
  // The session events' own bounded handoff. The control thread pushes and the
  // worker drains; a full channel costs the event and is counted, exactly like
  // the sample queue, so a session can never delay a tick.
  transport::BoundedChannel<RecordSessionEvent, kRecordSessionQueueCapacity>
      session_queue_;
  // How many control-thread calls are inside TryRecord. The control thread
  // announces itself around every offer - two lock-free increments, never a
  // wait - so a Stop() on another thread can tell when the refusal table below
  // is quiescent and when no sample can still land behind the tail it takes.
  std::atomic<std::uint32_t> offering_{0};
  // The same announcement for the session channel, so Stop() can tell when no
  // session event can still land behind the drain it takes.
  std::atomic<std::uint32_t> session_offering_{0};
  // The refusal stretches already recorded, and the one still being extended,
  // written by the control thread in TryRecord and read by Stop() once
  // `offering_` is zero.
  RefusalSpan refused_[kRecordGapSegments]{};
  std::uint32_t refused_count_{0};
  RefusalSpan refusal_open_{};
  bool refusal_open_valid_{false};
  // The accepted-sample count `recorded_` held when the open stretch started: a
  // sample that made it into the handoff ends the stretch, which is what makes
  // two refusals with a healthy run between them two losses instead of one gap
  // spanning the samples that were recorded fine.
  std::uint64_t refusal_open_recorded_{0};
  // The run's sampling stride, fixed by Start() before the control thread can
  // call TryRecord.
  protocol::Sequence stride_{1};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> session_dropped_{0};
  std::atomic<std::uint64_t> sessions_written_{0};
  std::atomic<std::uint64_t> recorded_{0};
  std::atomic<std::uint64_t> chunks_{0};
  std::atomic<std::uint64_t> write_failures_{0};
  // The run's loss table, published by Stop(). Guarded because gaps() may be
  // asked from another thread while the run is closing.
  mutable std::mutex gaps_mutex_;
  std::vector<RecordGap> gaps_;
};

}  // namespace irmv::telemetry
