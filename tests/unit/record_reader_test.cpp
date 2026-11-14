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

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
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
// fields, then the column-major joint columns, nine columns of
// `rows * joint_count` doubles, then the row-major validity block of one byte
// per value.
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
    6 * sizeof(std::uint64_t) + 9 * sizeof(double) + sizeof(std::uint64_t);

// The number of bytes the joint columns of a chunk of `rows` rows take: the
// applied target's five columns, the measured position, velocity and torque
// estimate, and the output stage's torque - nine doubles per joint per row -
// followed by the row-major validity block, one byte per value.
std::uint64_t JointColumnBytes(std::uint64_t rows) {
  return 9 * rows * kJoints * sizeof(double) +
         rows * kJoints * sizeof(std::uint8_t);
}

RecordSample MakeSample(Sequence state_seq,
                        std::uint32_t joint_count = kJoints) {
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
  // Three distinct axes per IMU vector: a decoder that shifted an axis into a
  // neighbour cannot agree with these values by accident.
  sample.imu_orientation_rpy = {0.11, -0.22, 0.33};
  sample.imu_angular_velocity = {1.1, -2.2, 3.3};
  sample.imu_linear_acceleration = {4.4, -5.5, 6.6};
  sample.imu_rx_stamp_us = sample.stamp_us + 7;
  sample.send_accepted = true;
  sample.output_valid = true;
  sample.joint_count = joint_count;
  for (std::uint32_t index = 0; index < joint_count; ++index) {
    sample.q[index] = 0.5 + 0.001 * state_seq + 0.01 * index;
    sample.dq[index] = 0.25 + 0.002 * state_seq;
    sample.tau_ff[index] = 0.125 + 0.003 * state_seq;
    sample.kp[index] = 50.0 + index;
    sample.kd[index] = 2.0 + 0.1 * index;
    // The feedback the tick used and the torque the output stage produced, both
    // of this same tick: the two column groups a v1 run could not carry.
    sample.measured_q[index] = 0.75 + 0.004 * state_seq + 0.02 * index;
    sample.measured_dq[index] = 0.35 + 0.005 * state_seq;
    sample.measured_tau_est[index] = 0.45 + 0.007 * state_seq + 0.03 * index;
    sample.output_torque[index] = 0.15 + 0.006 * state_seq;
    sample.joint_valid[index] = (index % 2) == 0;
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

// Rewrites a chunk header's payload length and its own checksum, so the
// decoder reaches the structural check with a header that is internally
// consistent but describes a payload its rows cannot hold. The header checksum
// is over the first 60 bytes, the length field among them.
bool SetPayloadLength(const std::filesystem::path &path, std::uint32_t length) {
  std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
  if (!file) {
    return false;
  }
  std::array<std::uint8_t, irmv::telemetry::kChunkHeaderBytes> header{};
  file.read(reinterpret_cast<char *>(header.data()),
            static_cast<std::streamsize>(header.size()));
  if (!file) {
    return false;
  }
  for (int index = 0; index < 4; ++index) {
    header[20 + static_cast<std::size_t>(index)] =
        static_cast<std::uint8_t>((length >> (8 * index)) & 0xFFU);
  }
  const std::uint32_t crc = irmv::telemetry::ChunkCrc32(
      header.data(), irmv::telemetry::kChunkHeaderCrcOffset);
  for (int index = 0; index < 4; ++index) {
    header[irmv::telemetry::kChunkHeaderCrcOffset + index] =
        static_cast<std::uint8_t>((crc >> (8 * index)) & 0xFFU);
  }
  file.seekp(0);
  file.write(reinterpret_cast<const char *>(header.data()),
             static_cast<std::streamsize>(header.size()));
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

// The metadata of a run of `joint_count` joints, as the recorder passes it
// through to meta.json. The reader checks the joint table against the chunks,
// so every run a test reads back has to declare the table it really wrote.
std::string ReaderMetadata(std::uint32_t joint_count) {
  std::string joints;
  for (std::uint32_t index = 0; index < joint_count; ++index) {
    if (index != 0) {
      joints += ",";
    }
    joints += "\"j" + std::to_string(index) + "\"";
  }
  return std::string(R"({"schema_version":)") +
         std::to_string(irmv::telemetry::kRecordSchemaVersion) +
         R"(,"config_id":"cfg","layout_id":"lay","backend":"fake","control_hz":500.0,"sample_hz":100,"joint_count":)" +
         std::to_string(joint_count) + R"(,"joints":[)" + joints +
         R"(],"units":{"position":"rad"}})";
}

// Starts a recorder whose metadata a reader can open and whose samples are the
// four-joint run this file reads.
bool StartReaderRecorder(Recorder *recorder,
                         const std::filesystem::path &directory,
                         std::uint32_t stride = kStride,
                         std::uint32_t joint_count = kJoints) {
  RecorderConfig config;
  config.enabled = true;
  config.sample_hz = 100;
  config.stride = stride;
  config.directory = directory;
  std::string error;
  const std::string metadata = ReaderMetadata(joint_count);
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
  const std::string metadata = ReaderMetadata(kJoints);
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
  CHECK(reader.metadata().joints[0] == "j0");
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
  CHECK(first.imu_rx_stamp_us == written.front().imu_rx_stamp_us);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    CHECK(first.imu_orientation_rpy[axis] ==
          written.front().imu_orientation_rpy[axis]);
    CHECK(first.imu_angular_velocity[axis] ==
          written.front().imu_angular_velocity[axis]);
    CHECK(first.imu_linear_acceleration[axis] ==
          written.front().imu_linear_acceleration[axis]);
  }
  for (std::uint32_t joint = 0; joint < kJoints; ++joint) {
    CHECK(first.measured_q[joint] == written.front().measured_q[joint]);
    CHECK(first.measured_dq[joint] == written.front().measured_dq[joint]);
    CHECK(first.measured_tau_est[joint] ==
          written.front().measured_tau_est[joint]);
    CHECK(first.joint_valid[joint] == written.front().joint_valid[joint]);
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
  // The first byte of the second row's fixed-width fields - the low byte of
  // its stamp. It is inside the payload, and a reader that trusted it would
  // hand out a tick time this run never stored.
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

// A schema-3 run - the layout before this build - is refused by name, never
// read with the schema-4 layout: its fixed rows and joint columns are shorter,
// so decoding it would misalign every field after the new columns.
int SchemaThreeRun() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 5, 0, 0, nullptr, &run));
  std::ofstream file(run / "meta.json", std::ios::trunc);
  file << R"({"schema_version":3})";
  file.close();
  RecordReader reader;
  std::string error;
  CHECK(!RecordReader::Open(run, &reader, &error));
  // The refusal names both the version the run declares and the one this build
  // implements; it never assumes the bytes are the current layout.
  CHECK(error.find("3") != std::string::npos);
  CHECK(error.find(std::to_string(irmv::telemetry::kRecordSchemaVersion)) !=
        std::string::npos);
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk that declares another schema version is refused with that version in
// the message, and yields no rows - rather than being decoded with the
// schema-4 layout, which would silently misalign every field after the header.
int SchemaThreeChunk() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 5, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  const std::filesystem::path chunk = run / reader.chunks().front().file;
  // The schema version is the u32 right after the eight-byte magic. Patch it to
  // 3, the version this build no longer reads.
  std::fstream file(chunk, std::ios::binary | std::ios::in | std::ios::out);
  CHECK(static_cast<bool>(file));
  file.seekp(8);
  const char version_three[4] = {3, 0, 0, 0};
  file.write(version_three, sizeof(version_three));
  file.close();

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.truncated);
  CHECK(range.samples.empty());
  CHECK(range.error.find("3") != std::string::npos);
  std::filesystem::remove_all(directory);
  return 0;
}

// A run that recorded no samples has no chunks, and reading it is an empty
// answer rather than an error: the metadata and a complete index are enough to
// say the run holds zero rows.
int ZeroRowRun() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 0, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  CHECK(reader.complete());
  CHECK(reader.chunks().empty());
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.samples.empty());
  CHECK(range.gaps.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// The protocol's maximum of 64 joints: the row and column sizes scale to the
// count the run declares, and every value of the widest chunk comes back.
int MaximumJoints() {
  const std::filesystem::path directory = MakeDirectory();
  constexpr std::uint32_t kAllJoints = irmv::protocol::kMaxJoints;
  Recorder recorder;
  CHECK(StartReaderRecorder(&recorder, directory, kStride, kAllJoints));
  std::vector<RecordSample> written;
  for (Sequence seq = 1; seq <= 2; ++seq) {
    const RecordSample sample = MakeSample(seq, kAllJoints);
    for (int attempt = 0; attempt < 500; ++attempt) {
      if (recorder.TryRecord(sample)) {
        written.push_back(sample);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  CHECK(written.size() == 2);
  CHECK(recorder.Stop());

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(recorder.run_directory(), &reader, &error));
  CHECK(reader.metadata().joint_count == kAllJoints);
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.samples.size() == written.size());
  for (std::size_t index = 0; index < written.size(); ++index) {
    CHECK(range.samples[index] == written[index]);
    CHECK(range.samples[index].joint_count == kAllJoints);
    CHECK(range.samples[index].measured_tau_est[kAllJoints - 1] ==
          written[index].measured_tau_est[kAllJoints - 1]);
    CHECK(range.samples[index].joint_valid[kAllJoints - 1] ==
          written[index].joint_valid[kAllJoints - 1]);
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A NaN the driver reported and a validity flag of false cross the writer and
// reader unchanged: the archive never substitutes a value for a failed
// measurement, and the flag stays the run's own.
int NaNAndInvalidJointKeepOriginalValues() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartReaderRecorder(&recorder, directory));
  RecordSample sample = MakeSample(1);
  sample.imu_valid = false;
  sample.imu_orientation_rpy[2] = std::numeric_limits<double>::quiet_NaN();
  sample.measured_tau_est[0] = std::numeric_limits<double>::quiet_NaN();
  sample.measured_tau_est[1] = 0.0;
  sample.joint_valid[0] = false;
  sample.joint_valid[1] = true;
  for (int attempt = 0; attempt < 500; ++attempt) {
    if (recorder.TryRecord(sample)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  CHECK(recorder.Stop());

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(recorder.run_directory(), &reader, &error));
  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.samples.size() == 1);
  const RecordSample &read = range.samples.front();
  CHECK(!read.imu_valid);
  CHECK(std::isnan(read.imu_orientation_rpy[2]));
  CHECK(std::isnan(read.measured_tau_est[0]));
  CHECK(!read.joint_valid[0]);
  CHECK(read.joint_valid[1]);
  CHECK(read.measured_tau_est[1] == 0.0);
  std::filesystem::remove_all(directory);
  return 0;
}

// A chunk cut inside the validity block yields no rows at all: a row whose
// validity is unknown cannot be handed out, and the run is reported as short.
int TruncatedValidityBlock() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  const std::filesystem::path chunk = run / reader.chunks().front().file;
  const std::uint64_t rows = reader.chunks().front().rows;
  const std::uint64_t validity_bytes =
      rows * static_cast<std::uint64_t>(kJoints);
  const auto size = std::filesystem::file_size(chunk);
  CHECK(size > validity_bytes);
  // Cut halfway through the row-major validity block, so every row is missing
  // at least the flags of its later joints.
  std::filesystem::resize_file(chunk, size - validity_bytes / 2);

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.truncated);
  CHECK(!range.corrupt);
  CHECK(range.samples.empty());
  std::filesystem::remove_all(directory);
  return 0;
}

// A byte flipped inside the validity block is corruption: the payload checksum
// covers the new block, so a validity flag edited in place is named and none of
// the damaged chunk's rows are returned.
int CorruptValidityBlock() {
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
  const std::uint64_t double_column_bytes =
      9 * static_cast<std::uint64_t>(broken.rows) * kJoints * sizeof(double);
  const std::uint64_t validity_offset = kChunkHeaderBytes +
                                        broken.rows * kFixedRowBytes +
                                        double_column_bytes + 5;
  CHECK(FlipByte(chunk, validity_offset));

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.corrupt);
  CHECK(!range.truncated);
  CHECK(range.error.find(broken.file) != std::string::npos);
  CHECK(range.error.find("checksum") != std::string::npos);
  CHECK(range.samples.size() == RowsOutsideChunk(reader, 0));
  CHECK(!TookRowsFromChunk(range, broken));
  std::filesystem::remove_all(directory);
  return 0;
}

// The raw chunk bytes, not a round trip, pin where the validity block sits:
// after the ninth double column and row-major at
// `rows * 9 * J + (row * J + joint)`. A writer and reader that both switched to
// a column-major block would still round-trip, so this test reads the file the
// writer really produced and checks the offset and the byte of every value.
int ValidityBlockByteLayout() {
  const std::filesystem::path directory = MakeDirectory();
  Recorder recorder;
  CHECK(StartReaderRecorder(&recorder, directory));
  RecordSample first = MakeSample(1);
  first.joint_valid[0] = true;
  first.joint_valid[1] = false;
  first.joint_valid[2] = true;
  first.joint_valid[3] = false;
  RecordSample second = MakeSample(2);
  second.joint_valid[0] = false;
  second.joint_valid[1] = true;
  second.joint_valid[2] = false;
  second.joint_valid[3] = true;
  for (const RecordSample &sample : {first, second}) {
    bool accepted = false;
    for (int attempt = 0; attempt < 500 && !accepted; ++attempt) {
      accepted = recorder.TryRecord(sample);
      if (!accepted) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    CHECK(accepted);
  }
  CHECK(recorder.Stop());

  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(recorder.run_directory(), &reader, &error));
  CHECK(reader.chunks().size() == 1);
  CHECK(reader.chunks().front().rows == 2);
  const std::filesystem::path chunk =
      recorder.run_directory() / reader.chunks().front().file;
  std::vector<std::uint8_t> bytes(
      static_cast<std::size_t>(std::filesystem::file_size(chunk)));
  {
    std::ifstream file(chunk, std::ios::binary);
    CHECK(static_cast<bool>(file));
    file.read(reinterpret_cast<char *>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
  }
  const std::uint64_t rows = reader.chunks().front().rows;
  const std::uint64_t values = rows * kJoints;
  const std::uint64_t block_start =
      kChunkHeaderBytes + rows * kFixedRowBytes + 9 * values * sizeof(double);
  CHECK(block_start + values == bytes.size());

  const auto read_u64 = [&bytes](std::uint64_t offset) {
    std::uint64_t value = 0;
    for (int index = 0; index < 8; ++index) {
      value |=
          static_cast<std::uint64_t>(bytes[static_cast<std::size_t>(offset) +
                                           static_cast<std::size_t>(index)])
          << (8 * index);
    }
    return value;
  };
  const auto bits_of = [](double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
  };

  // Fixed rows come first, then the double columns, then the validity block:
  // the byte before the block is the last value of the ninth column (the
  // second row's last torque estimate), and the first column starts where the
  // fixed rows end.
  CHECK(read_u64(kChunkHeaderBytes) == first.stamp_us);
  CHECK(read_u64(kChunkHeaderBytes + kFixedRowBytes) == second.stamp_us);
  CHECK(read_u64(block_start - sizeof(double)) ==
        bits_of(second.measured_tau_est[kJoints - 1]));
  CHECK(read_u64(kChunkHeaderBytes + rows * kFixedRowBytes) ==
        bits_of(first.q[0]));

  const std::array<bool, 2 * kJoints> expected{true,  false, true,  false,
                                               false, true,  false, true};
  for (std::uint64_t row = 0; row < rows; ++row) {
    for (std::uint64_t joint = 0; joint < kJoints; ++joint) {
      const std::uint8_t byte =
          bytes[static_cast<std::size_t>(block_start + row * kJoints + joint)];
      CHECK(byte == (expected[row * kJoints + joint] ? 1U : 0U));
    }
  }
  std::filesystem::remove_all(directory);
  return 0;
}

// A header whose payload length omits the validity block - the length a schema
// 3 writer declared - is named as a length the current rows cannot hold, not
// read as a short file: the validity bytes are part of this schema's payload.
int PayloadLengthOmittingValidity() {
  const std::filesystem::path directory = MakeDirectory();
  std::filesystem::path run;
  CHECK(WriteRun(directory, 30, 0, 0, nullptr, &run));
  RecordReader reader;
  std::string error;
  CHECK(RecordReader::Open(run, &reader, &error));
  const irmv::telemetry::ChunkInfo broken = reader.chunks().front();
  const std::filesystem::path chunk = run / broken.file;
  const std::uint64_t old_payload =
      std::filesystem::file_size(chunk) - kChunkHeaderBytes;
  const std::uint64_t without_validity =
      old_payload - static_cast<std::uint64_t>(broken.rows) * kJoints;
  CHECK(SetPayloadLength(chunk, static_cast<std::uint32_t>(without_validity)));

  RecordRange range;
  CHECK(reader.ReadRange(0, ~0ULL, &range));
  CHECK(range.ok);
  CHECK(range.corrupt);
  CHECK(!range.truncated);
  CHECK(range.error.find(broken.file) != std::string::npos);
  CHECK(range.error.find("payload length") != std::string::npos);
  CHECK(range.samples.empty());
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

int SimulationEvidence() {
  using namespace irmv::telemetry;
  const auto directory = MakeDirectory();
  const auto path = directory / "simulation.jsonl";
  irmv::protocol::Id run{};
  run.fill(0x12);
  const auto clock = SimulationRecordClock(run, 1000, 2000);
  nlohmann::json header = {
      {"schema", "irmv.simulation_evidence"},
      {"version", 1},
      {"record_type", "header"},
      {"clock", nlohmann::json::parse(EncodeRecordClock(clock))}};
  nlohmann::json row = {{"record_type", "event"},   {"sequence", 1},
                        {"kind", "write_accepted"}, {"logical_us", 1},
                        {"control_step", 0},        {"phase", 4},
                        {"state_seq", 87}};
  {
    std::ofstream f(path);
    f << header.dump() << '\n' << row.dump() << '\n';
  }
  std::string summary, error;
  int count = 0;
  CHECK(ReadSimulationEvidence(
      path, clock, [&](const std::string &) { ++count; }, &summary, &error));
  CHECK(count == 1);
  CHECK(!nlohmann::json::parse(summary)["evidence_complete"].get<bool>());
  {
    std::ofstream f(path, std::ios::app);
    f << R"({"record_type":"footer","offered":1,"written":1,"dropped":0,"write_failures":0})"
      << '\n';
  }
  CHECK(ReadSimulationEvidence(path, clock, {}, &summary, &error));
  CHECK(nlohmann::json::parse(summary)["evidence_complete"] == true);
  CHECK(nlohmann::json::parse(summary)["measured_feedback_complete"] == false);
  auto foreign = clock;
  foreign.clock_run_id = std::string(32, 'a');
  CHECK(!ReadSimulationEvidence(path, foreign, {}, &summary, &error));
  {
    std::ofstream f(path, std::ios::app);
    f << row.dump() << '\n';
  }
  CHECK(!ReadSimulationEvidence(path, clock, {}, &summary, &error));
  CHECK(nlohmann::json::parse(summary)["evidence_complete"] == false);
  {
    std::ofstream f(path);
    f << std::string(65537, 'x') << '\n';
  }
  CHECK(!ReadSimulationEvidence(path, clock, {}, &summary, &error));
  std::filesystem::remove_all(directory);
  return 0;
}

nlohmann::json SimulationMeasurementRow(std::uint64_t step) {
  return {{"record_type", "event"},
          {"sequence", step + 1},
          {"kind", "event_feedback"},
          {"control_step", step},
          {"phase", 1},
          {"logical_us", 1 + step * 2000},
          {"feedback_validated", true},
          {"feedback",
           {{"physics_step", step * 2},
            {"q", {0.}},
            {"dq", {0.}},
            {"tau", {0.}},
            {"imu_quaternion", {1., 0., 0., 0.}},
            {"imu_angular_velocity", {0., 0., 0.}},
            {"imu_specific_force", {0., 0., 9.81}},
            {"base_position", {0., 0., 0.}},
            {"base_velocity", {0., 0., 0., 0., 0., 0.}},
            {"base_quaternion", {1., 0., 0., 0.}}}}};
}

int SimulationMeasurementHorizon() {
  using namespace irmv::telemetry;
  using Json = nlohmann::json;
  const auto directory = MakeDirectory();
  const auto path = directory / "simulation.jsonl";
  irmv::protocol::Id run{};
  run.fill(0x12);
  const auto clock = SimulationRecordClock(run, 1000, 2000);
  const Json header = {{"schema", "irmv.simulation_evidence"},
                       {"version", 1},
                       {"record_type", "header"},
                       {"clock", Json::parse(EncodeRecordClock(clock))}};
  for (bool applied : {false, true}) {
    for (std::uint64_t last : {0, 2, 3}) {
      std::ofstream file(path);
      file << header.dump() << '\n';
      for (std::uint64_t step = 0; step <= last; ++step)
        file << SimulationMeasurementRow(step).dump() << '\n';
      Json marker = {{"record_type", "event"},
                     {"sequence", last + 2},
                     {"kind", applied ? "AppliedToStep" : "runner_final"},
                     {"control_step", applied ? 2 : 3},
                     {"phase", applied ? 3 : 4},
                     {"logical_us", applied ? 4001 : 6001},
                     {"coordinator_state", "Failed"},
                     {"stop_proven", false},
                     {"failure", "fixture failure"}};
      if (applied)
        marker["frame"] = {{"name", "AppliedToStep"},
                           {"payload",
                            {{"write_id", 17},
                             {"physics_step_begin", 4},
                             {"physics_step_end", 6}}}};
      file << marker.dump() << '\n'
           << Json{{"record_type", "footer"},
                   {"offered", last + 2},
                   {"written", last + 2},
                   {"dropped", 0},
                   {"write_failures", 0}}
                  .dump()
           << '\n';
      file.close();
      std::string summary, error;
      CHECK(ReadSimulationEvidence(path, clock, {}, &summary, &error));
      const auto result = Json::parse(summary);
      CHECK(result["evidence_complete"] == true);
      CHECK(result["measured_feedback_complete"] == (last == 3));
      if (!applied) CHECK(result["final"]["stop_proven"] == false);
    }
  }
  std::filesystem::remove_all(directory);
  return 0;
}

int SimulationInterruptedTail() {
  using namespace irmv::telemetry;
  using Json = nlohmann::json;
  const auto directory = MakeDirectory();
  const auto path = directory / "simulation.jsonl";
  irmv::protocol::Id run{};
  run.fill(0x12);
  const auto clock = SimulationRecordClock(run, 1000, 2000);
  const Json header = {{"schema", "irmv.simulation_evidence"},
                       {"version", 1},
                       {"record_type", "header"},
                       {"clock", Json::parse(EncodeRecordClock(clock))}};
  const std::string prefix =
      header.dump() + "\n" + SimulationMeasurementRow(0).dump() + "\n";
  const std::string clean =
      prefix +
      R"({"record_type":"footer","offered":1,"written":1,"dropped":0,"write_failures":0})" +
      "\n";
  {
    std::ofstream file(path);
    file << clean;
  }
  std::string summary, error;
  CHECK(ReadSimulationEvidence(path, clock, {}, &summary, &error));
  CHECK(Json::parse(summary)["measured_feedback_complete"] == true);
  for (const std::string suffix : {"{", " ", "{}\n"}) {
    {
      std::ofstream file(path);
      file << clean << suffix;
    }
    CHECK(!ReadSimulationEvidence(path, clock, {}, &summary, &error));
    CHECK(Json::parse(summary)["evidence_complete"] == false);
    CHECK(error.find("after simulation footer") != std::string::npos);
  }
  {
    std::ofstream file(path);
    file << prefix << R"({"record_type":"footer")";
  }
  CHECK(ReadSimulationEvidence(path, clock, {}, &summary, &error));
  CHECK(Json::parse(summary)["evidence_complete"] == false);
  CHECK(Json::parse(summary)["measured_feedback_complete"] == false);
  std::filesystem::remove_all(directory);
  return 0;
}

int main() {
  int failures = 0;
  failures += FullRead();
  failures += SimulationEvidence();
  failures += SimulationMeasurementHorizon();
  failures += SimulationInterruptedTail();
  failures += RangeRead();
  failures += TruncatedChunk();
  failures += JointTruncatedChunk();
  failures += CorruptPayload();
  failures += CorruptLength();
  failures += IntactRunUnchanged();
  failures += UnknownVersion();
  failures += SchemaThreeRun();
  failures += SchemaThreeChunk();
  failures += ZeroRowRun();
  failures += MaximumJoints();
  failures += NaNAndInvalidJointKeepOriginalValues();
  failures += TruncatedValidityBlock();
  failures += CorruptValidityBlock();
  failures += ValidityBlockByteLayout();
  failures += PayloadLengthOmittingValidity();
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
