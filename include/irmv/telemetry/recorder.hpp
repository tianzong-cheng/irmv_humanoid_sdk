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
// Finalization belongs to the worker: Stop() first closes the handoff, waits
// for the producers already inside TryRecord to leave, and then asks the worker
// to drain the tail, fold in the refusals the control thread counted, and write
// the index the same table gaps() returns. The worker owns the queue and its
// own state through a shared allocation, so a sink or file system that blocks
// past Stop()'s deadline can finish safely on its own; Stop() returns false at
// the deadline instead of joining unconditionally or writing files from the
// stopping thread.
//
// The index is replaced atomically after every committed chunk and at the start
// of the run, and carries a `complete` flag. A reader can therefore open a run
// that is still recording, and recover the chunks a crash committed before the
// next index replacement.
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

// The bound Stop() gives its worker to drain and finalize. A blocked device is
// handed off to the worker, which owns its state, rather than holding the
// control loop past this deadline.
inline constexpr std::uint32_t kRecorderStopTimeoutMs = 2000;

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

  // Closes the handoff, asks the worker to drain and finalize, and waits for it
  // against one deadline. Returns true when the worker finished and was joined;
  // false when the deadline expired, in which case the worker still owns every
  // queue, buffer and file and finishes on its own. Idempotent, and called by
  // the destructor. No filesystem write happens on the caller's thread after
  // the deadline.
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

  std::shared_ptr<Impl> impl_;
  // Counters for offers made before Start() or after a failed Start(), when no
  // run state exists to hold them.
  std::atomic<std::uint64_t> unattached_dropped_{0};
  std::atomic<std::uint64_t> unattached_session_dropped_{0};
};

}  // namespace irmv::telemetry
