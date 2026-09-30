#include "irmv/telemetry/record_reader.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace irmv::telemetry {
namespace {

using Json = nlohmann::json;

constexpr char kChunkMagic[8] = {'I', 'R', 'M', 'V', 'C', 'H', '0', '1'};

// A cursor over the raw bytes of one chunk. It never reads past the end: a
// short file is reported, not read into.
class Cursor {
 public:
  Cursor(const std::vector<std::uint8_t> &bytes, std::size_t offset)
      : bytes_(bytes), offset_(offset) {}

  bool U8(std::uint8_t *out) {
    if (offset_ + 1 > bytes_.size()) {
      return false;
    }
    *out = bytes_[offset_];
    offset_ += 1;
    return true;
  }

  bool U32(std::uint32_t *out) {
    if (offset_ + 4 > bytes_.size()) {
      return false;
    }
    std::uint32_t value = 0;
    for (int shift = 0; shift < 32; shift += 8) {
      value |= static_cast<std::uint32_t>(bytes_[offset_++]) << shift;
    }
    *out = value;
    return true;
  }

  bool U64(std::uint64_t *out) {
    if (offset_ + 8 > bytes_.size()) {
      return false;
    }
    std::uint64_t value = 0;
    for (int shift = 0; shift < 64; shift += 8) {
      value |= static_cast<std::uint64_t>(bytes_[offset_++]) << shift;
    }
    *out = value;
    return true;
  }

  bool Double(double *out) {
    std::uint64_t bits = 0;
    if (!U64(&bits)) {
      return false;
    }
    std::memcpy(out, &bits, sizeof(*out));
    return true;
  }

  bool Bytes(std::uint8_t *out, std::size_t count) {
    if (offset_ + count > bytes_.size()) {
      return false;
    }
    std::memcpy(out, bytes_.data() + offset_, count);
    offset_ += count;
    return true;
  }

 private:
  const std::vector<std::uint8_t> &bytes_;
  std::size_t offset_;
};

bool ReadFile(const std::filesystem::path &path,
              std::vector<std::uint8_t> *bytes) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }
  file.seekg(0, std::ios::end);
  const std::streamoff size = file.tellg();
  if (size < 0) {
    return false;
  }
  file.seekg(0, std::ios::beg);
  bytes->resize(static_cast<std::size_t>(size));
  if (size > 0) {
    file.read(reinterpret_cast<char *>(bytes->data()), size);
  }
  return file.good() || file.eof();
}

bool ReadJson(const std::filesystem::path &path, Json *out) {
  std::ifstream file(path);
  if (!file) {
    return false;
  }
  *out = Json::parse(file, nullptr, false);
  return !out->is_discarded();
}

// One chunk decoded into rows. The columns are read in the order the writer
// wrote them; a chunk that does not hold exactly the bytes its header promises
// is a truncation, and the rows it did hold are returned with `complete` false.
bool DecodeChunk(const std::vector<std::uint8_t> &bytes,
                 const RunMetadata &metadata, std::vector<RecordSample> *rows,
                 bool *complete) {
  *complete = false;
  if (bytes.size() < sizeof(kChunkMagic) ||
      std::memcmp(bytes.data(), kChunkMagic, sizeof(kChunkMagic)) != 0) {
    return false;
  }
  Cursor cursor(bytes, sizeof(kChunkMagic));
  std::uint32_t schema_version = 0;
  std::uint32_t joint_count = 0;
  std::uint32_t row_count = 0;
  std::uint64_t first_stamp = 0;
  std::uint64_t last_stamp = 0;
  std::uint64_t first_seq = 0;
  std::uint64_t last_seq = 0;
  if (!cursor.U32(&schema_version) || !cursor.U32(&joint_count) ||
      !cursor.U32(&row_count) || !cursor.U64(&first_stamp) ||
      !cursor.U64(&last_stamp) || !cursor.U64(&first_seq) ||
      !cursor.U64(&last_seq)) {
    return false;
  }
  if (schema_version != metadata.schema_version ||
      joint_count != metadata.joint_count) {
    // A chunk of another schema or another layout is not this run's data.
    return false;
  }
  std::vector<RecordSample> decoded;
  decoded.reserve(row_count);
  for (std::uint32_t index = 0; index < row_count; ++index) {
    RecordSample sample;
    std::uint64_t stamp = 0;
    std::uint64_t state_seq = 0;
    std::uint32_t state = 0;
    std::uint32_t output = 0;
    std::uint32_t stop = 0;
    std::uint64_t fault_id = 0;
    std::uint8_t healthy = 0;
    std::uint8_t damping_eligible = 0;
    std::uint8_t command_present = 0;
    std::uint8_t feedback_valid = 0;
    std::uint8_t imu_valid = 0;
    std::uint8_t send_accepted = 0;
    std::uint64_t epoch = 0;
    std::uint64_t command_seq = 0;
    std::uint64_t source_state_seq = 0;
    std::uint64_t command_rx_us = 0;
    std::uint64_t feedback_seq = 0;
    std::uint64_t feedback_stamp_us = 0;
    if (!cursor.U64(&stamp) || !cursor.U64(&state_seq) || !cursor.U32(&state) ||
        !cursor.U32(&output) || !cursor.U32(&stop) || !cursor.U64(&fault_id) ||
        !cursor.U8(&healthy) || !cursor.U8(&damping_eligible) ||
        !cursor.U8(&command_present) || !cursor.U8(&feedback_valid) ||
        !cursor.U8(&imu_valid) || !cursor.U8(&send_accepted) ||
        !cursor.Bytes(sample.producer_id.data(), sample.producer_id.size()) ||
        !cursor.U64(&epoch) || !cursor.U64(&command_seq) ||
        !cursor.U64(&source_state_seq) || !cursor.U64(&command_rx_us) ||
        !cursor.U64(&feedback_seq) || !cursor.U64(&feedback_stamp_us)) {
      *rows = std::move(decoded);
      return true;
    }
    sample.schema_version = schema_version;
    sample.stamp_us = stamp;
    sample.state_seq = state_seq;
    sample.execution_state = static_cast<protocol::ExecutionState>(state);
    sample.output_action = static_cast<protocol::OutputAction>(output);
    sample.stop_reason = static_cast<protocol::StopReason>(stop);
    sample.fault_id = fault_id;
    sample.healthy = healthy != 0;
    sample.damping_eligible = damping_eligible != 0;
    sample.command_present = command_present != 0;
    sample.feedback_valid = feedback_valid != 0;
    sample.imu_valid = imu_valid != 0;
    sample.send_accepted = send_accepted != 0;
    sample.epoch = epoch;
    sample.command_seq = command_seq;
    sample.source_state_seq = source_state_seq;
    sample.command_rx_us = command_rx_us;
    sample.feedback_seq = feedback_seq;
    sample.feedback_stamp_us = feedback_stamp_us;
    sample.joint_count = joint_count;
    decoded.push_back(sample);
  }
  // The joint columns follow, one column at a time, in the order the writer
  // wrote them. A column that runs past the end of the file is a truncation:
  // the rows decoded so far are kept and the chunk is reported incomplete.
  for (int column = 0; column < 5; ++column) {
    for (std::uint32_t row = 0; row < row_count; ++row) {
      for (std::uint32_t joint = 0; joint < joint_count; ++joint) {
        double value = 0.0;
        if (!cursor.Double(&value)) {
          *rows = std::move(decoded);
          return true;
        }
        RecordSample &sample = decoded[row];
        switch (column) {
          case 0:
            sample.q[joint] = value;
            break;
          case 1:
            sample.dq[joint] = value;
            break;
          case 2:
            sample.tau_ff[joint] = value;
            break;
          case 3:
            sample.kp[joint] = value;
            break;
          default:
            sample.kd[joint] = value;
            break;
        }
      }
    }
  }
  *complete = true;
  *rows = std::move(decoded);
  return true;
}

}  // namespace

bool RecordReader::Open(const std::filesystem::path &run_directory,
                        RecordReader *reader, std::string *error) {
  if (reader == nullptr) {
    return false;
  }
  RecordReader opened;
  opened.directory_ = run_directory;
  Json metadata;
  if (!ReadJson(run_directory / std::string(kRecordMetadataFile), &metadata) ||
      !metadata.is_object()) {
    *error = "the run has no readable " + std::string(kRecordMetadataFile);
    return false;
  }
  if (!metadata.contains("schema_version") ||
      metadata["schema_version"] != kRecordSchemaVersion) {
    // An unknown version is refused rather than reinterpreted: the bytes of
    // another schema mean something else.
    *error = "the run declares another schema version";
    return false;
  }
  opened.metadata_.schema_version = kRecordSchemaVersion;
  opened.metadata_.config_id = metadata.value("config_id", std::string());
  opened.metadata_.layout_id = metadata.value("layout_id", std::string());
  opened.metadata_.backend = metadata.value("backend", std::string());
  opened.metadata_.control_hz = metadata.value("control_hz", 0.0);
  opened.metadata_.sample_hz = metadata.value("sample_hz", 0U);
  opened.metadata_.joint_count = metadata.value("joint_count", 0U);
  if (metadata.contains("joints") && metadata["joints"].is_array()) {
    for (const auto &joint : metadata["joints"]) {
      if (joint.is_string()) {
        opened.metadata_.joints.push_back(joint.get<std::string>());
      }
    }
  }
  if (opened.metadata_.joints.size() != opened.metadata_.joint_count ||
      opened.metadata_.joint_count == 0 ||
      opened.metadata_.joint_count > protocol::kMaxJoints) {
    *error = "the run declares a joint table of another size";
    return false;
  }
  if (metadata.contains("units") && metadata["units"].is_object()) {
    opened.metadata_.units_json = metadata["units"].dump();
  }

  Json index;
  if (!ReadJson(run_directory / std::string(kRecordIndexFile), &index) ||
      !index.is_object()) {
    *error = "the run has no readable " + std::string(kRecordIndexFile);
    return false;
  }
  if (index.value("schema_version", 0U) != kRecordSchemaVersion) {
    *error = "the run index declares another schema version";
    return false;
  }
  if (index.contains("chunks") && index["chunks"].is_array()) {
    for (const auto &chunk : index["chunks"]) {
      ChunkInfo info;
      info.file = chunk.value("file", std::string());
      info.rows = chunk.value("rows", 0U);
      info.first_stamp_us = chunk.value("first_stamp_us", 0ULL);
      info.last_stamp_us = chunk.value("last_stamp_us", 0ULL);
      info.first_state_seq = chunk.value("first_state_seq", 0ULL);
      info.last_state_seq = chunk.value("last_state_seq", 0ULL);
      if (info.file.empty()) {
        *error = "the run index names a chunk without a file";
        return false;
      }
      opened.chunks_.push_back(std::move(info));
    }
  }
  if (index.contains("gaps") && index["gaps"].is_array()) {
    for (const auto &gap : index["gaps"]) {
      RecordGap entry;
      entry.first_seq = gap.value("first_seq", 0ULL);
      entry.last_seq = gap.value("last_seq", 0ULL);
      entry.first_stamp_us = gap.value("first_stamp_us", 0ULL);
      entry.last_stamp_us = gap.value("last_stamp_us", 0ULL);
      entry.dropped = gap.value("dropped", 0ULL);
      const std::string reason = gap.value("reason", std::string("none"));
      if (reason == "queue_full") {
        entry.reason = RecordGapReason::QueueFull;
      } else if (reason == "write_failed") {
        entry.reason = RecordGapReason::WriteFailed;
      } else if (reason == "stopped") {
        entry.reason = RecordGapReason::Stopped;
      } else {
        entry.reason = RecordGapReason::None;
      }
      opened.gaps_.push_back(entry);
    }
  }
  *reader = std::move(opened);
  return true;
}

bool RecordReader::ReadRange(protocol::TimeUs begin_us, protocol::TimeUs end_us,
                             RecordRange *range) const {
  if (range == nullptr) {
    return false;
  }
  RecordRange result;
  if (end_us <= begin_us) {
    // An empty range reads nothing and is not an error: a caller asking for a
    // window that holds no time is asking a question with an empty answer.
    result.ok = true;
    *range = std::move(result);
    return true;
  }
  for (const ChunkInfo &chunk : chunks_) {
    if (chunk.last_stamp_us < begin_us || chunk.first_stamp_us >= end_us) {
      continue;
    }
    std::vector<std::uint8_t> bytes;
    if (!ReadFile(directory_ / chunk.file, &bytes)) {
      result.truncated = true;
      continue;
    }
    std::vector<RecordSample> rows;
    bool complete = false;
    if (!DecodeChunk(bytes, metadata_, &rows, &complete)) {
      result.truncated = true;
      continue;
    }
    if (!complete) {
      result.truncated = true;
    }
    for (const RecordSample &sample : rows) {
      if (sample.stamp_us >= begin_us && sample.stamp_us < end_us) {
        result.samples.push_back(sample);
      }
    }
  }
  for (const RecordGap &gap : gaps_) {
    // A gap is reported when it touches the range at all: the caller asked
    // about a stretch of time, and a stretch that is missing inside it is part
    // of the answer.
    if ((gap.last_stamp_us != 0 && gap.last_stamp_us < begin_us) ||
        (gap.first_stamp_us != 0 && gap.first_stamp_us >= end_us)) {
      continue;
    }
    result.gaps.push_back(gap);
  }
  result.ok = true;
  *range = std::move(result);
  return true;
}

}  // namespace irmv::telemetry
