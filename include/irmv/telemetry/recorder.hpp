#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
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
// The worker owns everything else: the chunk buffers, the files and the index.
// A chunk is written under a temporary name and renamed into place only once it
// is complete, so a run that ends abruptly has an identifiable tail rather than
// a half-written block a reader would have to guess about. Every column of a
// chunk is contiguous, and the joint columns are `rows * joint_count` wide
// rather than `rows * kMaxJoints`, so a twelve-joint deployment does not pay
// for the sixty-four the protocol allows.
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

// Capacity of the handoff queue. At 100 Hz this is roughly two seconds of
// samples, which is far more than a healthy disk needs and small enough to stay
// a bounded amount of memory (a sample is a few kilobytes).
inline constexpr std::size_t kRecordQueueCapacity = 256;

// One run's configuration.
struct RecorderConfig {
  // Whether the recorder writes anything at all. A disabled recorder accepts no
  // samples and opens no file: this is the switch the configuration exposes.
  bool enabled{false};
  // The sampling rate of the *recorder*, which is independent of the control
  // rate: the runtime offers the control tick to the recorder and the
  // recorder's caller decides which ticks those are.
  std::uint32_t sample_hz{100};
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
  // counted and visible through dropped().
  bool TryRecord(const RecordSample &sample) noexcept;

  // Stops the worker after draining what it can, closes the run and writes the
  // index. Idempotent, and called by the destructor. A drain that does not
  // finish inside the bound is reported by returning false rather than waiting
  // for the disk: the control loop that owns this call has its own deadlines.
  bool Stop();

  bool started() const;
  // The directory of the run this recorder created. It keeps naming it after
  // Stop(), so a caller can still read the run it just closed.
  const std::filesystem::path &run_directory() const;

  // Samples the recorder refused because the queue was full or because it was
  // not running.
  std::uint64_t dropped() const;
  // Samples written into completed chunks.
  std::uint64_t recorded() const;
  // Chunks the worker closed and renamed into place.
  std::uint64_t chunks() const;
  // Chunk writes that failed. The run keeps going, and the stretch they cover
  // is a gap the index reports.
  std::uint64_t write_failures() const;
  // The gaps the run knows about, in the order they happened.
  std::vector<RecordGap> gaps() const;

 private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
  std::filesystem::path run_directory_;
  std::atomic<bool> started_{false};
  transport::BoundedChannel<RecordSample, kRecordQueueCapacity> queue_;
  // The sequence range of the samples the queue refused. The control thread
  // records it as it counts the refusal, because a refused sample is never seen
  // by the worker and the worker is what reports gaps: without this the loss at
  // the tail of a full queue would be invisible in the run.
  std::atomic<protocol::Sequence> dropped_first_seq_{0};
  std::atomic<protocol::Sequence> dropped_last_seq_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> recorded_{0};
  std::atomic<std::uint64_t> chunks_{0};
  std::atomic<std::uint64_t> write_failures_{0};
};

}  // namespace irmv::telemetry
