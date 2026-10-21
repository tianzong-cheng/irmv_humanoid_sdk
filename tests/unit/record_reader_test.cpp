// The read side of a recorded run: a run written by the recorder is read back
// row by row, by time, with its gaps and its truncations reported rather than
// smoothed over.
//
// The round trip is deliberately through the recorder itself: the reader is
// tested against bytes the writer really produced, not against a fixture that
// could agree with a format neither of them implements.
#include "irmv/telemetry/record_reader.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "irmv/telemetry/recorder.hpp"
#include "support/check.hpp"

namespace {

using irmv::protocol::ExecutionState;
using irmv::protocol::OutputAction;
using irmv::protocol::Sequence;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::telemetry::Recorder;
using irmv::telemetry::RecorderConfig;
using irmv::telemetry::RecordGapReason;
using irmv::telemetry::RecordRange;
using irmv::telemetry::RecordReader;
using irmv::telemetry::RecordSample;

constexpr std::size_t kJoints = 4;
constexpr std::uint32_t kStride = 5;
constexpr TimeUs kStampStepUs = 10000;

// The byte layout of a chunk, from the writer's column order, so a test can cut
// one in a chosen section rather than at an arbitrary offset. A chunk is the
// eight-byte magic, then the schema version, joint count, row count and payload
// length as u32, the first and last stamp and sequence as u64, the payload and
// header CRC-32s as u32 - a 64-byte header - then the fixed-width per-row
// fields, then the column-major joint columns, eight columns of
// `rows * joint_count` doubles.
constexpr std::size_t kChunkHeaderBytes =
    sizeof("IRMVCH01") - 1 + 4 * sizeof(std::uint32_t) +
    4 * sizeof(std::uint64_t) + 2 * sizeof(std::uint32_t);
// The payload-length field's offset in the header, the byte a corruption test
// edits to prove a wrong length is named as corruption rather than read as a
// short file.
constexpr std::uint64_t kChunkLengthOffset =
    sizeof("IRMVCH01") - 1 + 3 * sizeof(std::uint32_t);
constexpr std::size_t kFixedRowBytes =
    2 * sizeof(std::uint64_t) + 3 * sizeof(std::uint32_t) +
    sizeof(std::uint64_t) + 7 * sizeof(std::uint8_t) + irmv::protocol::kIdSize +
    6 * sizeof(std::uint64_t);

// The number of bytes the joint columns of a chunk of `rows` rows take: the
// applied target's five columns plus the measured position and velocity and the
// output stage's torque, one double per joint per row.
std::uint64_t JointColumnBytes(std::uint64_t rows) {
  return 8 * rows * kJoints * sizeof(double);
}

RecordSample MakeSample(Sequence state_seq) {
  RecordSample sample;
  sample.stamp_us = 1000 + state_seq * kStampStepUs;
  sample.state_seq = state_seq;
  sample.execution_state = ExecutionState::Tracking;
  sample.output_action = OutputAction::Tracking;
  sample.stop_reason = StopReason::None;
  sample.fault_id = 7;
  sample.healthy = true;
  sample.damping_eligible = true;
  sample.command_present = true;
  sample.producer_id[0] = 0x5A;
  sample.epoch = 9;
  sample.command_seq = state_seq;
  sample.source_state_seq = state_seq;
  sample.command_rx_us = sample.stamp_us + 3;
  sample.feedback_seq = state_seq;
  sample.feedback_stamp_us = sample.stamp_us - 100;
  sample.feedback_valid = true;
  sample.imu_valid = true;
  sample.send_accepted = true;
  sample.output_valid = true;
  sample.joint_count = static_cast<std::uint32_t>(kJoints);
  for (std::size_t index = 0; index < kJoints; ++index) {
    sample.q[index] = 0.5 + 0.001 * state_seq + 0.01 * index;
    sample.dq[index] = 0.25 + 0.002 * state_seq;
    sample.tau_ff[index] = 0.125 + 0.003 * state_seq;
    sample.kp[index] = 50.0 + index;
    sample.kd[index] = 2.0 + 0.1 * index;
    // The feedback the tick used and the torque the output stage produced, both
    // of this same tick: the two column groups a v1 run could not carry.
    sample.measured_q[index] = 0.75 + 0.004 * state_seq + 0.02 * index;
    sample.measured_dq[index] = 0.35 + 0.005 * state_seq;
    sample.output_torque[index] = 0.15 + 0.006 * state_seq;
  }
  return sample;
}

std::filesystem::path MakeDirectory() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path directory =
      std::filesystem::temp_directory_path() /
      ("irmv-reader-test-" + std::to_string(now));
  std::filesystem::create_directory(directory);
  return directory;
}

// Flips exactly one byte of a file in place: the smallest edit a torn write, a
// bad sector or a stray edit makes, and the edit a length and a checksum exist
// to catch. The byte is only inverted; no offset moves, so a reader that still
// returns a row from the file has trusted bytes the run never wrote.
bool FlipByte(const std::filesystem::path &path, std::uint64_t offset) {
  std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
  if (!file) {
    return false;
  }
  file.seekg(static_cast<std::streamoff>(offset));
  char value = 0;
  file.read(&value, 1);
  if (!file) {
    return false;
  }
  value = static_cast<char>(static_cast<unsigned char>(value) ^ 0xFFU);
  file.seekp(static_cast<std::streamoff>(offset));
  file.write(&value, 1);
  return file.good();
}

// The rows of every chunk but `broken`, which is what a reader must still
// return when one chunk is refused: the run is bigger than the damaged file.
std::size_t RowsOutsideChunk(const irmv::telemetry::RecordReader &reader,
                             std::size_t broken) {
  std::size_t rows = 0;
  for (std::size_t index = 0; index < reader.chunks().size(); ++index) {
    if (index != broken) {
      rows += reader.chunks()[index].rows;
    }
  }
  return rows;
}

// Whether any sample of `range` falls inside the damaged chunk's own time span,
// which is where a row built from its corrupted bytes would have to land.
bool TookRowsFromChunk(const irmv::telemetry::RecordRange &range,
                       const irmv::telemetry::ChunkInfo &chunk) {
  for (const RecordSample &sample : range.samples) {
    if (sample.stamp_us >= chunk.first_stamp_us &&
        sample.stamp_us <= chunk.last_stamp_us) {
      return true;
    }
  }
  return false;
}

// Starts a recorder whose metadata a reader can open and whose samples are the
// four-joint run this file reads.
bool StartReaderRecorder(Recorder *recorder,
                         const std::filesystem::path &directory,
                         std::uint32_t stride = kStride) {
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.stride = stride;
  config.directory = directory;
  std::string error;
  const std::string metadata =
      std::string(R"({"schema_version":)") +
      std::to_string(irmv::telemetry::kRecordSchemaVersion) +
      R"(,"config_id":"cfg","layout_id":"lay","backend":"fake","control_hz":500.0,"sample_hz":100,"joint_count":4,"joints":["j1","j2","j3","j4"],"units":{"position":"rad"}})";
  if (!recorder->Start(config, metadata, &error)) {
    std::cerr << "recorder start failed: " << error << '\n';
    return false;
  }
  return true;
}

// Offers one sample until the bounded handoff has room, like a control loop
// that wants every sample recorded.
bool OfferUntilTaken(Recorder *recorder, Sequence seq) {
  for (int attempt = 0; attempt < 500; ++attempt) {
    if (recorder->TryRecord(MakeSample(seq))) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

// Writes a run of `count` samples with the given stride. The samples whose
// state sequence is in [hole_from, hole_to] are offered into a handoff that the
// samples before them have already filled, so the run really loses them: the
// gap the reader reports is one the recorder recorded, not a stretch the test
// quietly left out. Every other offered sample is offered until the handoff
// takes it, the way a control loop that wants every sample waits for room.
bool WriteRun(const std::filesystem::path &directory, int count,
              Sequence hole_from, Sequence hole_to,
              std::vector<RecordSample> *written,
              std::filesystem::path *run_directory) {
  Recorder recorder;
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.stride = kStride;
  config.directory = directory;
  std::string error;
  const std::string metadata =
      std::string(R"({"schema_version":)") +
      std::to_string(irmv::telemetry::kRecordSchemaVersion) +
      R"(,"config_id":"cfg","layout_id":"lay","backend":"fake","control_hz":500.0,"sample_hz":100,"joint_count":4,"joints":["j1","j2","j3","j4"],"units":{"position":"rad"}})";
  if (!recorder.Start(config, metadata, &error)) {
    std::cerr << "recorder start failed: " << error << '\n';
    return false;
  }
  Sequence seq = 1;
  for (int index = 0; index < count; ++index) {
    const RecordSample sample = MakeSample(seq);
    if (seq >= hole_from && seq <= hole_to) {
      // The handoff holds one sample per stride tick, and the worker waits
      // before its first drain: the samples before this one have filled it, so
      // this offer is refused and the refusal is the hole.
      if (recorder.TryRecord(sample)) {
        std::cerr << "the handoff took a sample this run means to lose\n";
        return false;
      }
      if (seq == hole_to) {
        // Let the writer drain before recording again, so what follows is the
        // sample after the hole rather than another refusal.
        std::this_thread::sleep_for(std::chrono::milliseconds(70));
      }
    } else {
      // The handoff is bounded and the worker drains it on its own clock: a run
      // longer than the queue offered as a burst would be refused, which is the
      // recorder's design, not what this helper means to produce. Wait for room
      // the way the control loop would, so every offered sample is a recorded
      // one.
      int attempts = 0;
      while (!recorder.TryRecord(sample)) {
        if (++attempts > 200) {
          std::cerr << "the recorder refused a sample\n";
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (written != nullptr) {
        written->push_back(sample);
      }
    }
    seq += kStride;
  }
  const bool stopped = recorder.Stop();
  if (stopped && run_directory != nullptr) {
    *run_directory = recorder.run_directory();
  }
  return stopped;
}

}  // namespace

// A run read back whole: every field of every row, and the metadata the run
// declares about itself.
int FullRead() {
  const std::filesystem::path directory = MakeDirectory();
  std::vector<RecordSample> written;
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, &written, &run));
  CHECK(written.size() == 30);

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.metadata().schema_version ==
        irmv::telemetry::kRecordSchemaVersion);
  CHECK(reader.metadata().config_id == "cfg");
  CHECK(reader.metadata().layout_id == "lay");
  CHECK(reader.metadata().sample_hz == 100);
  CHECK(reader.metadata().joint_count == kJoints);
  CHECK(reader.metadata().joints.size() == kJoints);
  CHECK(reader.metadata().joints[0] == "j1");
  CHECK(reader.chunks().size() == 1);
  CHECK(reader.gaps().empty());

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(!range.truncated);
  CHECK(range.samples.size() == written.size());
  for (std::size_t index = 0; index < written.size(); ++index) {
    CHECK(range.samples[index] == written[index]);
  }
  // The two column groups a v1 chunk could not carry, read back with the values
  // that were written rather than with the zeros a decoder reading the wrong
  // layout would leave behind.
  const RecordSample &first = range.samples.front();
  CHECK(first.output_valid);
  for (std::uint32_t joint = 0; joint < kJoints; ++joint) {
    CHECK(first.measured_q[joint] == written.front().measured_q[joint]);
    CHECK(first.measured_dq[joint] == written.front().measured_dq[joint]);
    CHECK(first.output_torque[joint] == written.front().output_torque[joint]);
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A range read returns exactly the rows inside it, and the gap that overlaps it
// is reported with the range.
int RangeRead() {
  const std::filesystem::path directory = MakeDirectory();
  std::vector<RecordSample> written;
  // Two samples of the recorded grid, five ticks apart, at the point where the
  // handoff of a run of this length is exactly full: they are refused, and the
  // run says so.
  const Sequence hole_from = 1 + 256 * kStride;
  const Sequence hole_to = hole_from + kStride;
  std::filesystem::path run;
  CHECK(WriteRun(directory, 300, hole_from, hole_to, &written, &run));
  CHECK(written.size() == 298);

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));

  const RecordSample &first = written.front();
  const RecordSample &middle = written[written.size() / 3];
  RecordRange range;
  // The window covers the first row and a few after it.
  CHECK(reader.ReadRange(first.stamp_us, middle.stamp_us, &range));
  CHECK(range.ok);
  CHECK(!range.samples.empty());
  for (const RecordSample &sample : range.samples) {
    CHECK(sample.stamp_us >= first.stamp_us);
    CHECK(sample.stamp_us < middle.stamp_us);
  }
  // The first row of the run is inside the window and is exactly what was
  // written.
  CHECK(range.samples.front() == first);
  // No loss overlaps a window of samples the run did record.
  CHECK(range.gaps.empty());

  // A window that covers the hole reports it.
  const TimeUs hole_start_us = 1000 + hole_from * kStampStepUs;
  const TimeUs hole_end_us = 1000 + (hole_to + 2 * kStride) * kStampStepUs;
  RecordRange hole_range;
  CHECK(reader.ReadRange(hole_start_us, hole_end_us, &hole_range));
  CHECK(hole_range.gaps.size() == 1);
  // The gap is expressed on the recorded grid - with a stride of five, the two
  // samples that are missing are the ones at the two sequences, not the eight
  // control ticks between them - and it carries their stamps, so a reader can
  // place the loss in time without re-deriving it.
  CHECK(hole_range.gaps[0].first_seq == hole_from);
  CHECK(hole_range.gaps[0].last_seq == hole_to);
  CHECK(hole_range.gaps[0].first_stamp_us == 1000 + hole_from * kStampStepUs);
  CHECK(hole_range.gaps[0].last_stamp_us == 1000 + hole_to * kStampStepUs);
  CHECK(hole_range.gaps[0].dropped == 2);
  CHECK(hole_range.gaps[0].reason == RecordGapReason::QueueFull);
  // The rows around the hole are still read: a gap does not swallow the samples
  // beside it.
  CHECK(!hole_range.samples.empty());

  // An empty window reads nothing and is not an error.
  RecordRange empty;
  CHECK(reader.ReadRange(500, 500, &empty));
  CHECK(empty.ok);
  CHECK(empty.samples.empty());
  CHECK(empty.gaps.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk cut inside the fixed-width per-row fields is reported as a
// truncation, and the earlier rows it did hold whole are still read.
int TruncatedChunk() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  const std::filesystem::path chunk = run / reader.chunks().front().file;
  const std::uint64_t rows = reader.chunks().front().rows;
  const auto size = std::filesystem::file_size(chunk);
  // Cut in the middle of the per-row fields, which end where the joint columns
  // begin. The rows before the cut decode whole.
  const std::uint64_t fixed_bytes =
      size - kChunkHeaderBytes - JointColumnBytes(rows);
  std::filesystem::resize_file(chunk, kChunkHeaderBytes + fixed_bytes / 2);

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.truncated);
  CHECK(!range.samples.empty());
  // Exactly the rows the cut left whole, and no partial row after them.
  CHECK(range.samples.size() == (fixed_bytes / 2) / kFixedRowBytes);
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk cut inside the column-major joint columns is reported as a truncation
// and yields no rows at all: every row is missing the tail of a column, so a
// row handed out would carry zeros the run never stored. The intact chunks
// beside it are still read.
int JointTruncatedChunk() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  // Two full chunks and a short tail: cutting the first leaves the others read.
  const int count =
      2 * static_cast<int>(irmv::telemetry::kRecordChunkRows) + 10;
  CHECK(WriteRun(directory, count, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.chunks().size() >= 2);

  const irmv::telemetry::ChunkInfo broken = reader.chunks().front();
  const std::filesystem::path chunk = run / broken.file;
  const auto size = std::filesystem::file_size(chunk);
  const std::uint64_t joint_bytes = JointColumnBytes(broken.rows);
  std::filesystem::resize_file(chunk, size - joint_bytes / 2);

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.truncated);
  // Every intact chunk is read whole...
  std::size_t intact_rows = 0;
  for (std::size_t index = 1; index < reader.chunks().size(); ++index) {
    intact_rows += reader.chunks()[index].rows;
  }
  CHECK(range.samples.size() == intact_rows);
  // ...and not one row of the broken chunk: none lies in its time span, so the
  // rows are absent rather than present with zeroed joint columns.
  for (const RecordSample &sample : range.samples) {
    CHECK(!(sample.stamp_us >= broken.first_stamp_us &&
            sample.stamp_us <= broken.last_stamp_us));
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A byte flipped inside a chunk's payload is reported as corruption, and the
// damaged chunk contributes no rows: the intact chunks around it are still
// returned, and the range is not called a truncation - a cut file and a
// corrupted file call for different answers.
int CorruptPayload() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  const int count =
      2 * static_cast<int>(irmv::telemetry::kRecordChunkRows) + 10;
  CHECK(WriteRun(directory, count, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.chunks().size() == 3);

  const irmv::telemetry::ChunkInfo broken = reader.chunks().front();
  const std::filesystem::path chunk = run / broken.file;
  // The first byte of the first row's first applied-target value, in the
  // column-major joint columns: a reader that trusted it would hand out a joint
  // target this run never stored.
  CHECK(FlipByte(chunk, kChunkHeaderBytes + kFixedRowBytes));

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.corrupt);
  CHECK(!range.truncated);
  CHECK(range.error.find(broken.file) != std::string::npos);
  CHECK(range.error.find("checksum") != std::string::npos);
  // Every intact chunk is read whole, and not one row comes from the damaged
  // one: its rows are absent rather than present with a value its bytes never
  // held.
  CHECK(range.samples.size() == RowsOutsideChunk(reader, 0));
  CHECK(!TookRowsFromChunk(range, broken));
  std::filesystem::remove_all(directory);
  return 0;
}

// A byte flipped in the header's length field is corruption too, never a short
// file: the header's own checksum covers the length, so a reader cannot mistake
// a wrong length for a truncated payload and return the rows on either side of
// an offset that was never written.
int CorruptLength() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  const int count =
      2 * static_cast<int>(irmv::telemetry::kRecordChunkRows) + 10;
  CHECK(WriteRun(directory, count, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.chunks().size() == 3);

  const irmv::telemetry::ChunkInfo broken = reader.chunks().front();
  const std::filesystem::path chunk = run / broken.file;
  CHECK(FlipByte(chunk, kChunkLengthOffset));

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.corrupt);
  CHECK(!range.truncated);
  CHECK(range.error.find(broken.file) != std::string::npos);
  CHECK(range.error.find("header checksum") != std::string::npos);
  CHECK(range.samples.size() == RowsOutsideChunk(reader, 0));
  CHECK(!TookRowsFromChunk(range, broken));
  std::filesystem::remove_all(directory);
  return 0;
}

// An intact run reads back exactly what was written - every field of every row,
// across several chunks - with neither damage flag set: the length and the
// checksums a v3 writer adds change not one value a reader returns.
int IntactRunUnchanged() {
  const std::filesystem::path directory = MakeDirectory();
  std::vector<RecordSample> written;
  std::filesystem::path run;
  const int count =
      2 * static_cast<int>(irmv::telemetry::kRecordChunkRows) + 10;
  CHECK(WriteRun(directory, count, 0, 0, &written, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.chunks().size() == 3);

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(!range.truncated);
  CHECK(!range.corrupt);
  CHECK(range.error.empty());
  CHECK(range.samples.size() == written.size());
  for (std::size_t index = 0; index < written.size(); ++index) {
    CHECK(range.samples[index] == written[index]);
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A run of another schema version is refused whole rather than reinterpreted,
// and the message names the version so an operator sees which side must move.
int UnknownVersion() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 5, 0, 0, nullptr, &run));
  std::ofstream file(run / "meta.json", std::ios::trunc);
  file << R"({"schema_version":7})";
  file.close();
  RecordReader reader;
  std::string error;
  CHECK(!RecordReader::Open(run, &reader, &error));
  CHECK(error.find("7") != std::string::npos);
  std::filesystem::remove_all(directory);
  return 0;
}

// A v1 run - the schema before this build - is refused by name, never read with
// the v3 layout: its bytes mean something else, and its chunk has fewer flags
// and fewer columns, so decoding it would misalign every field.
int VersionOneRun() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 5, 0, 0, nullptr, &run));
  std::ofstream file(run / "meta.json", std::ios::trunc);
  file << R"({"schema_version":1})";
  file.close();
  RecordReader reader;
  std::string error;
  CHECK(!RecordReader::Open(run, &reader, &error));
  // The refusal names both the version the run declares and the one this build
  // implements; it never assumes the bytes are v3.
  CHECK(error.find("1") != std::string::npos);
  CHECK(error.find(std::to_string(irmv::telemetry::kRecordSchemaVersion)) !=
        std::string::npos);
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk that declares another schema version is refused with that version in
// the message, and yields no rows - rather than being decoded with the v3
// layout, which would silently misalign every field after the header.
int VersionOneChunk() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 5, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  const std::filesystem::path chunk = run / reader.chunks().front().file;
  // The schema version is the u32 right after the eight-byte magic. Patch it to
  // 1, the version this build no longer reads.
  std::fstream file(chunk, std::ios::binary | std::ios::in | std::ios::out);
  CHECK(static_cast<bool>(file));
  file.seekp(8);
  const char version_one[4] = {1, 0, 0, 0};
  file.write(version_one, sizeof(version_one));
  file.close();

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.truncated);
  CHECK(range.samples.empty());
  CHECK(range.error.find("1") != std::string::npos);
  std::filesystem::remove_all(directory);
  return 0;
}

// A run can be opened while the recorder is still writing it. The index is
// published before the first chunk and replaced after each committed chunk, so
// the reader sees the committed rows and is told the run is unfinished.
int ReadDuringRecording() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartReaderRecorder(&recorder, directory));
  Sequence seq = 1;
  for (int index = 0; index < 2000 && recorder.chunks() == 0; ++index) {
    CHECK(OfferUntilTaken(&recorder, seq));
    seq += kStride;
    if ((index % 32) == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  CHECK(recorder.chunks() > 0);

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(recorder.run_directory(), &reader, &error));
  CHECK(!reader.complete());
  CHECK(!reader.chunks().empty());
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(!range.samples.empty());

  CHECK(recorder.Stop());
  RecordReader finalized;
  CHECK(RecordReader::Open(recorder.run_directory(), &finalized, &error));
  CHECK(finalized.complete());
  CHECK(finalized.chunks().size() == recorder.chunks());
  std::filesystem::remove_all(directory);
  return 0;
}

// A crash between a chunk's rename and the next index replacement leaves a
// committed chunk the index does not name. Opening the run recovers it and
// reports the run as unfinished.
int RecoverCommittedChunkWithoutIndex() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, nullptr, &run));
  std::filesystem::remove(run / "index.json");
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(!reader.complete());
  CHECK(!reader.chunks().empty());
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.samples.size() == 30);
  std::filesystem::remove_all(directory);
  return 0;
}

// Recovery must not turn a damaged committed chunk into silence. With no
// index, a valid chunk and a chunk whose header no longer validates, the reader
// recovers the valid rows and reports the run corrupt by filename.
int DamagedRecoveredChunkIsReported() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 950, 0, 0, nullptr, &run));
  RecordReader indexed;
  std::string error;
  CHECK(RecordReader::Open(run, &indexed, &error));
  CHECK(indexed.chunks().size() >= 2);
  const std::string damaged = indexed.chunks().front().file;
  CHECK(std::filesystem::remove(run / "index.json"));
  CHECK(FlipByte(run / damaged, 0));

  RecordReader reader;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(!reader.complete());
  CHECK(reader.chunks().size() + 1 == indexed.chunks().size());
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.corrupt);
  CHECK(!range.truncated);
  CHECK(range.error.find(damaged) != std::string::npos);
  CHECK(!range.samples.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// A recorder killed before any finalization still leaves its committed chunks
// readable. The run is explicitly unfinished rather than silently presented as
// a complete one.
int ReadAfterSigkill() {
  const std::filesystem::path directory = MakeDirectory();
  const pid_t child = fork();
  if (child == 0) {
    Recorder recorder;
    if (!StartReaderRecorder(&recorder, directory)) {
      _exit(2);
    }
    Sequence seq = 1;
    for (int index = 0; index < 2000 && recorder.chunks() == 0; ++index) {
      if (!OfferUntilTaken(&recorder, seq)) {
        _exit(3);
      }
      seq += kStride;
    }
    if (recorder.chunks() == 0) {
      _exit(4);
    }
    // No destructor, no Stop(), no index finalization: exactly the state a
    // crash leaves.
    std::raise(SIGKILL);
    _exit(5);
  }
  int status = 0;
  CHECK(child > 0);
  CHECK(waitpid(child, &status, 0) == child);
  CHECK(WIFSIGNALED(status));
  CHECK(WTERMSIG(status) == SIGKILL);

  std::filesystem::path run;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_directory()) {
      run = entry.path();
    }
  }
  CHECK(!run.empty());
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(!reader.complete());
  CHECK(!reader.chunks().empty());
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(!range.samples.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += FullRead();
  failures += RangeRead();
  failures += TruncatedChunk();
  failures += JointTruncatedChunk();
  failures += CorruptPayload();
  failures += CorruptLength();
  failures += IntactRunUnchanged();
  failures += UnknownVersion();
  failures += VersionOneRun();
  failures += VersionOneChunk();
  failures += ReadDuringRecording();
  failures += RecoverCommittedChunkWithoutIndex();
  failures += DamagedRecoveredChunkIsReported();
  failures += ReadAfterSigkill();
  if (failures == 0) {
    return 0;
  }
  std::cerr << failures << " section(s) failed\n";
  return 1;
}
