#include "irmv/web/range_reduce.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <utility>

namespace irmv::web {
namespace {

std::uint64_t PlannedPeriodUs(std::uint32_t sample_hz) {
  if (sample_hz == 0) {
    return 1;
  }
  return (1000000ULL + sample_hz / 2U) / sample_hz;
}

bool SameIdentity(const telemetry::VelocitySample &left,
                  const telemetry::VelocitySample &right) {
  return left.identity_known == right.identity_known &&
         left.runtime_id == right.runtime_id &&
         left.session_id == right.session_id;
}

double FieldValue(const telemetry::VelocitySample &row, VelocityField field) {
  switch (field) {
    case VelocityField::Forward:
      return row.forward;
    case VelocityField::Lateral:
      return row.lateral;
    case VelocityField::Yaw:
      return row.yaw;
  }
  return 0.0;
}

}  // namespace

RangeReducer::RangeReducer(protocol::TimeUs begin_us,
                           protocol::TimeUs end_us,
                           std::uint32_t pixel_width)
    : begin_us_(begin_us), end_us_(end_us) {
  // One display bucket per pixel, and at most four raw samples per bucket
  // (first, minimum, maximum, last), so a window never returns more than
  // 4 * pixel_width points.
  const std::uint64_t budget = std::max<std::uint64_t>(1, pixel_width);
  const std::uint64_t span = end_us > begin_us ? end_us - begin_us : 1;
  bucket_width_us_ = std::max<std::uint64_t>(1, (span + budget - 1) / budget);
}

std::uint64_t RangeReducer::BucketIndex(protocol::TimeUs stamp_us) const {
  if (stamp_us <= begin_us_) {
    return 0;
  }
  return (stamp_us - begin_us_) / bucket_width_us_;
}

void RangeReducer::Push(const SeriesPoint &point) {
  ++input_points_;
  const std::uint64_t index = BucketIndex(point.stamp_us);
  if (bucket_used_ && (index != bucket_.index ||
                       point.segment != bucket_.segment)) {
    Flush();
  }
  if (!bucket_used_) {
    bucket_ = Bucket{};
    bucket_.index = index;
    bucket_.segment = point.segment;
    bucket_used_ = true;
  }
  if (!bucket_.first.used) {
    bucket_.first = SelectedPoint{point, true};
  }
  bucket_.last = SelectedPoint{point, true};
  if (point.valid) {
    if (!bucket_.minimum.used || point.value < bucket_.minimum.point.value) {
      bucket_.minimum = SelectedPoint{point, true};
    }
    if (!bucket_.maximum.used || point.value > bucket_.maximum.point.value) {
      bucket_.maximum = SelectedPoint{point, true};
    }
  }
}

void RangeReducer::Flush() {
  if (!bucket_used_) {
    return;
  }
  std::vector<SelectedPoint> selected;
  selected.push_back(bucket_.first);
  if (bucket_.minimum.used) {
    selected.push_back(bucket_.minimum);
  }
  if (bucket_.maximum.used) {
    selected.push_back(bucket_.maximum);
  }
  selected.push_back(bucket_.last);
  Emit(selected);
  bucket_used_ = false;
}

void RangeReducer::Emit(const std::vector<SelectedPoint> &selected) {
  static const char *kLabels[] = {"first", "min", "max", "last"};
  struct Merge {
    SeriesPoint point;
    std::string label;
  };
  std::vector<Merge> merged;
  for (std::size_t index = 0; index < selected.size(); ++index) {
    if (!selected[index].used) {
      continue;
    }
    const SeriesPoint &point = selected[index].point;
    bool found = false;
    for (Merge &entry : merged) {
      if (entry.point.stamp_us == point.stamp_us &&
          entry.point.sequence == point.sequence) {
        entry.label += "+";
        entry.label += kLabels[index];
        found = true;
        break;
      }
    }
    if (!found) {
      merged.push_back(Merge{point, kLabels[index]});
    }
  }
  for (Merge &entry : merged) {
    ReducedPoint output;
    output.stamp_us = entry.point.stamp_us;
    output.sequence = entry.point.sequence;
    output.value = entry.point.value;
    output.valid = entry.point.valid;
    output.segment = entry.point.segment;
    output.aggregation = std::move(entry.label);
    points_.push_back(std::move(output));
  }
}

ReducedSeries RangeReducer::Finish() {
  Flush();
  ReducedSeries series;
  series.points = std::move(points_);
  series.input_points = input_points_;
  series.bucket_width_us = bucket_width_us_;
  return series;
}

HoldValue HoldVelocityAt(const std::vector<telemetry::VelocitySample> &rows,
                         const std::vector<telemetry::VelocityLossSpan> &losses,
                         protocol::TimeUs cursor_us, std::uint32_t sample_hz,
                         VelocityField field) {
  HoldValue result;
  if (rows.empty()) {
    result.reason = "no application samples";
    return result;
  }
  if (cursor_us < rows.front().stamp_us) {
    result.reason = "before the first application sample";
    return result;
  }
  const std::uint64_t period_us = PlannedPeriodUs(sample_hz);
  // Find the segment that owns the cursor. A segment ends at the first break
  // after its last row, and the earliest indexed loss inside the interval
  // bounds it even when a cadence gap or identity change is also present. The
  // table is inspected whole so the answer does not depend on loss order.
  std::size_t start = 0;
  while (start < rows.size()) {
    std::size_t end = start;
    std::uint64_t segment_end = rows[start].stamp_us + period_us;
    while (end + 1 < rows.size()) {
      const telemetry::VelocitySample &a = rows[end];
      const telemetry::VelocitySample &b = rows[end + 1];
      std::uint64_t next_end = a.stamp_us + period_us;
      bool broken = false;
      if (b.stamp_us <= a.stamp_us ||
          b.stamp_us - a.stamp_us > (period_us * 3) / 2) {
        broken = true;
      }
      if (!SameIdentity(a, b)) {
        broken = true;
      }
      for (const telemetry::VelocityLossSpan &loss : losses) {
        const protocol::TimeUs first = loss.first_stamp_us;
        const protocol::TimeUs last = loss.last_stamp_us;
        if (first < b.stamp_us && last >= a.stamp_us) {
          next_end = std::min(
              next_end, std::max<std::uint64_t>(first, a.stamp_us));
          broken = true;
        }
      }
      if (broken) {
        segment_end = next_end;
        break;
      }
      end += 1;
    }
    if (end + 1 >= rows.size()) {
      std::uint64_t last_end = rows.back().stamp_us + period_us;
      for (const telemetry::VelocityLossSpan &loss : losses) {
        if (rows.back().stamp_us < loss.first_stamp_us &&
            loss.first_stamp_us < last_end) {
          last_end = std::min(last_end, loss.first_stamp_us);
        }
      }
      segment_end = last_end;
    }
    const telemetry::VelocitySample &first = rows[start];
    const telemetry::VelocitySample &last = rows[end];
    if (cursor_us >= first.stamp_us && cursor_us < last.stamp_us) {
      // The cursor lies between two rows of this segment: hold the later one's
      // predecessor, provided the cursor is still inside the recorded cadence.
      std::size_t chosen = start;
      for (std::size_t index = start; index <= end; ++index) {
        if (rows[index].stamp_us <= cursor_us) {
          chosen = index;
        } else {
          break;
        }
      }
      const telemetry::VelocitySample &row = rows[chosen];
      if (cursor_us - row.stamp_us >= period_us) {
        result.reason = "older than one planned period";
        return result;
      }
      result.found = true;
      result.value = FieldValue(row, field);
      result.stamp_us = row.stamp_us;
      result.age_us = cursor_us - row.stamp_us;
      return result;
    }
    if (cursor_us == last.stamp_us) {
      result.found = true;
      result.value = FieldValue(last, field);
      result.stamp_us = last.stamp_us;
      result.age_us = 0;
      return result;
    }
    if (cursor_us >= last.stamp_us && cursor_us < segment_end) {
      result.found = true;
      result.value = FieldValue(last, field);
      result.stamp_us = last.stamp_us;
      result.age_us = cursor_us - last.stamp_us;
      return result;
    }
    start = end + 1;
  }
  result.reason = "no recorded application segment covers the cursor";
  return result;
}

}  // namespace irmv::web
