#include "irmv/telemetry/record_context.hpp"

#include <cstddef>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

namespace irmv::telemetry {
namespace {

constexpr std::uint64_t kMicrosPerSecond = 1000000;
constexpr std::string_view kWhitespace = " \t\r\n";

bool IsBootIdCharacter(char character) {
  const bool digit = character >= '0' && character <= '9';
  const bool lower = character >= 'a' && character <= 'f';
  const bool upper = character >= 'A' && character <= 'F';
  return digit || lower || upper || character == '-';
}

// The kernel writes one UUID line. An empty, unreadable or malformed file is
// reported as a failure rather than turned into an identity that could be
// mistaken for a real boot.
bool ReadBootId(const std::string& path, std::string* boot_id,
                std::string* reason) {
  std::ifstream file(path);
  if (!file.is_open()) {
    *reason = "cannot open " + path;
    return false;
  }
  std::string text;
  std::getline(file, text);
  if (file.bad()) {
    *reason = "cannot read " + path;
    return false;
  }
  const std::size_t begin = text.find_first_not_of(kWhitespace);
  if (begin == std::string::npos) {
    *reason = path + " is empty";
    return false;
  }
  const std::size_t end = text.find_last_not_of(kWhitespace);
  std::string value = text.substr(begin, end - begin + 1);
  if (value.size() > 64) {
    *reason = path + " is longer than 64 bytes";
    return false;
  }
  for (const char character : value) {
    if (!IsBootIdCharacter(character)) {
      *reason = path + " is not a boot id";
      return false;
    }
  }
  *boot_id = std::move(value);
  return true;
}

}  // namespace

bool LoadRecordClockIdentityFromFile(std::string_view boot_id_path,
                                     RecordClockIdentity* identity,
                                     std::string* error) {
  if (identity == nullptr) {
    if (error != nullptr) {
      *error = "clock identity output is null";
    }
    return false;
  }
  identity->clock = std::string(kRecordClock);
  identity->boot_id.clear();
  identity->available = false;

  std::string boot_id;
  std::string reason;
  if (!ReadBootId(std::string(boot_id_path), &boot_id, &reason)) {
    if (error != nullptr) {
      *error = "clock identity unavailable: " + reason +
               "; automatic cross-process merging of this run is disabled";
    }
    return false;
  }
  identity->boot_id = std::move(boot_id);
  identity->available = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool LoadRecordClockIdentity(RecordClockIdentity* identity,
                             std::string* error) {
  return LoadRecordClockIdentityFromFile(kBootIdPath, identity, error);
}

SampleSchedule::SampleSchedule(std::uint32_t sample_hz) noexcept {
  // Rounded to the nearest microsecond, so a rate that does not divide one
  // second keeps its phase over a run instead of drifting every sample.
  const std::uint64_t rate = sample_hz;
  if (rate == 0 || rate > kMicrosPerSecond) {
    return;
  }
  period_us_ = (kMicrosPerSecond + rate / 2) / rate;
}

bool SampleSchedule::Due(protocol::TimeUs now_us) noexcept {
  if (period_us_ == 0) {
    return false;
  }
  if (!started_) {
    started_ = true;
    next_due_us_ = now_us;
  }
  if (now_us < next_due_us_) {
    return false;
  }

  // Advance the phase grid past now in one arithmetic step. The number of
  // missed deadlines is not replayed, so a stall yields exactly the current
  // sample and the next deadline stays on the established grid.
  const protocol::TimeUs elapsed = now_us - next_due_us_;
  const protocol::TimeUs steps = elapsed / period_us_ + 1;
  constexpr protocol::TimeUs kMax =
      std::numeric_limits<protocol::TimeUs>::max();
  if (steps > (kMax - next_due_us_) / period_us_) {
    next_due_us_ = kMax;
  } else {
    next_due_us_ += steps * period_us_;
  }
  return true;
}

}  // namespace irmv::telemetry
