#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "irmv/protocol/types.hpp"
#include "irmv/telemetry/velocity_record.hpp"

// The display-side reduction of one recorded signal.
//
// A range query never ships every row of a long run: it ships the shape of the
// window, keeping the raw timestamp of the first, smallest, largest and last
// sample of each display bucket. A narrow spike inside one bucket therefore
// keeps its own time rather than being replaced by the bucket's midpoint, and a
// bucket never borrows points from another segment.
//
// The reducer is a pure model: it reads no clock, allocates only its own fixed
// bucket list and does not know about files, HTTP or the browser.
namespace irmv::web {

struct SeriesPoint {
  protocol::TimeUs stamp_us{0};
  std::uint64_t sequence{0};
  double value{0.0};
  bool valid{false};
  std::uint32_t segment{0};
};

struct ReducedPoint {
  protocol::TimeUs stamp_us{0};
  std::uint64_t sequence{0};
  double value{0.0};
  bool valid{false};
  std::uint32_t segment{0};
  // One or more of `first`, `min`, `max`, `last`, joined by `+`. A raw sample
  // is always exactly one of them, never an interpolated value.
  std::string aggregation;
};

struct ReducedSeries {
  std::vector<ReducedPoint> points;
  std::uint64_t input_points{0};
  std::uint64_t bucket_width_us{1};
};

class RangeReducer {
 public:
  RangeReducer(protocol::TimeUs begin_us, protocol::TimeUs end_us,
               std::uint32_t pixel_width);

  void Push(const SeriesPoint &point);
  ReducedSeries Finish();

 private:
  struct SelectedPoint {
    SeriesPoint point;
    bool used{false};
  };

  struct Bucket {
    std::uint64_t index{0};
    std::uint32_t segment{0};
    SelectedPoint first;
    SelectedPoint last;
    SelectedPoint minimum;
    SelectedPoint maximum;
  };

  std::uint64_t BucketIndex(protocol::TimeUs stamp_us) const;
  void Flush();
  void Emit(const std::vector<SelectedPoint> &selected);

  protocol::TimeUs begin_us_;
  protocol::TimeUs end_us_;
  std::uint64_t bucket_width_us_{1};
  std::vector<ReducedPoint> points_;
  std::uint64_t input_points_{0};
  Bucket bucket_;
  bool bucket_used_{false};
};

// Which recorded application value a cursor asks for.
enum class VelocityField { Forward, Lateral, Yaw };

// What a cursor sees at one instant.
struct HoldValue {
  bool found{false};
  double value{0.0};
  protocol::TimeUs stamp_us{0};
  protocol::TimeUs age_us{0};
  std::string reason;
};

// Holds the latest velocity sample forward to `cursor_us`, but only inside one
// contiguous recorded segment: a cadence break, an identity change, the end of
// the recording and the earliest indexed loss all end the hold at the instant
// the segment can no longer serve a value. A cursor outside every segment has
// no value; this function never invents one.
HoldValue HoldVelocityAt(const std::vector<telemetry::VelocitySample> &rows,
                         const std::vector<telemetry::VelocityLossSpan> &losses,
                         protocol::TimeUs cursor_us, std::uint32_t sample_hz,
                         VelocityField field);

}  // namespace irmv::web
