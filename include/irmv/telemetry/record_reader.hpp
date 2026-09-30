#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/record.hpp"

// The read side of a recorded run: the metadata, the chunks and the gaps a run
// left behind, read by time rather than by replaying the run.
//
// The reader is a pure consumer of files: it opens nothing but the run
// directory, takes no DDS endpoint, publishes nothing and cannot command a
// machine. A range read returns the samples whose controller stamp lies inside
// it, in the order the run recorded them, together with every gap that overlaps
// the range - so a caller can tell "the machine did nothing here" from "nothing
// was recorded here".
//
// Every structural fact is checked against the schema version and the run's own
// metadata before a row is handed out: an unknown version, a chunk that is
// short, over-long or of another joint count, and a chunk that was never
// renamed into place (a `.part` file) are refused or reported as a truncation
// instead of being reinterpreted.
namespace irmv::telemetry {

// What one run declares about itself.
struct RunMetadata {
  std::uint32_t schema_version{0};
  std::string config_id;
  std::string layout_id;
  std::string backend;
  double control_hz{0.0};
  std::uint32_t sample_hz{0};
  std::uint32_t joint_count{0};
  std::vector<std::string> joints;
  std::string units_json;
};

// One chunk as the index describes it.
struct ChunkInfo {
  std::string file;
  std::uint32_t rows{0};
  protocol::TimeUs first_stamp_us{0};
  protocol::TimeUs last_stamp_us{0};
  protocol::Sequence first_state_seq{0};
  protocol::Sequence last_state_seq{0};
};

// One read of a time range.
struct RecordRange {
  bool ok{false};
  std::string error;
  // The samples inside the range, ordered by the controller's stamp.
  std::vector<RecordSample> samples;
  // The gaps overlapping the range. A gap whose bounds fall outside it is
  // reported whole: a caller that reads a sub-range is told about the stretch
  // it asked about, and the gap's own bounds say what was missing.
  std::vector<RecordGap> gaps;
  // True when a chunk was short or a file was missing: the samples it did yield
  // are in `samples`, and what follows is not the whole run.
  bool truncated{false};
};

class RecordReader {
 public:
  // Opens one run directory: the metadata and the index must exist, parse, and
  // declare a schema version this build implements.
  static bool Open(const std::filesystem::path &run_directory,
                   RecordReader *reader, std::string *error);

  const RunMetadata &metadata() const { return metadata_; }
  const std::vector<ChunkInfo> &chunks() const { return chunks_; }
  const std::vector<RecordGap> &gaps() const { return gaps_; }
  const std::filesystem::path &directory() const { return directory_; }

  // Reads the samples whose controller stamp is in `[begin_us, end_us)`. An
  // empty or inverted range reads nothing and is not an error; a range beyond
  // the run reads everything.
  bool ReadRange(protocol::TimeUs begin_us, protocol::TimeUs end_us,
                 RecordRange *range) const;

 private:
  std::filesystem::path directory_;
  RunMetadata metadata_;
  std::vector<ChunkInfo> chunks_;
  std::vector<RecordGap> gaps_;
};

}  // namespace irmv::telemetry
