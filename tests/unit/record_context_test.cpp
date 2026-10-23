// The two pieces of record context that are pure or startup-only: the
// independent sample plan both processes run at the configured rate, and the
// clock identity a run carries so two processes on one host can be lined up.
//
// The sample plan is exercised through explicit timestamps only - it never
// reads a clock - so a stalled loop, an exact deadline and the first call are
// all reproducible. The clock identity is read from a temporary file instead of
// the real /proc, so the test proves the failure path without depending on the
// host.
#include "irmv/telemetry/record_context.hpp"

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <system_error>
#include <utility>

#include "support/check.hpp"

namespace {

using irmv::telemetry::kRecordClock;
using irmv::telemetry::LoadRecordClockIdentityFromFile;
using irmv::telemetry::RecordClockIdentity;
using irmv::telemetry::SampleSchedule;

constexpr char kBootId[] = "01234567-89ab-cdef-0123-456789abcdef";

std::filesystem::path TemporaryBootIdPath(const char* suffix) {
  return std::filesystem::temp_directory_path() /
         (std::string("irmv-record-context-") + std::to_string(getpid()) +
          suffix);
}

bool WriteText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream file(path, std::ios::trunc);
  file << text;
  return file.good();
}

// Removes its file when it goes out of scope, including when a CHECK below
// returns early, so a failed test never leaves a boot-id file behind.
class TemporaryFileGuard {
 public:
  explicit TemporaryFileGuard(std::filesystem::path path) noexcept
      : path_(std::move(path)) {}
  ~TemporaryFileGuard() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  TemporaryFileGuard(const TemporaryFileGuard&) = delete;
  TemporaryFileGuard& operator=(const TemporaryFileGuard&) = delete;

  const std::filesystem::path& Path() const noexcept { return path_; }

 private:
  std::filesystem::path path_;
};

}  // namespace

int main() {
  // A 100 Hz plan with its origin at 3000 us emits on the integer grid
  // 3000/13000/23000 and nowhere in between.
  {
    SampleSchedule schedule(100);
    CHECK(schedule.Enabled());
    CHECK(schedule.PeriodUs() == 10000);
    CHECK(schedule.Due(3000));
    CHECK(!schedule.Due(12999));
    CHECK(schedule.Due(13000));
    CHECK(!schedule.Due(22999));
    CHECK(schedule.Due(23000));
  }

  // A continuous 200 Hz plan emits at 0/5000/10000 and is not due before the
  // next deadline.
  {
    SampleSchedule schedule(200);
    CHECK(schedule.PeriodUs() == 5000);
    CHECK(schedule.Due(0));
    CHECK(!schedule.Due(4999));
    CHECK(schedule.Due(5000));
    CHECK(!schedule.Due(9999));
    CHECK(schedule.Due(10000));
    CHECK(!schedule.Due(14999));
    CHECK(schedule.Due(15000));
  }

  // After a 35 ms stall the plan reports one current sample and re-anchors the
  // next deadline past now: the missed 5000/10000/... deadlines are never
  // replayed and the next sample is at 40000, not at the old 10000.
  {
    SampleSchedule schedule(200);
    CHECK(schedule.Due(0));
    CHECK(schedule.Due(35000));
    CHECK(!schedule.Due(35001));
    CHECK(!schedule.Due(39999));
    CHECK(schedule.Due(40000));
  }

  // A zero rate has no schedule at all rather than dividing by zero.
  {
    SampleSchedule schedule(0);
    CHECK(!schedule.Enabled());
    CHECK(!schedule.Due(0));
    CHECK(!schedule.Due(123456789));
  }

  const TemporaryFileGuard boot_id_file(TemporaryBootIdPath(".boot_id"));
  const TemporaryFileGuard empty_file(TemporaryBootIdPath(".empty"));
  const TemporaryFileGuard missing_file(TemporaryBootIdPath(".missing"));
  std::error_code ignored;
  std::filesystem::remove(missing_file.Path(), ignored);

  // The successful read carries the stable clock name and the trimmed boot id.
  CHECK(WriteText(boot_id_file.Path(), std::string(kBootId) + "\n"));
  {
    RecordClockIdentity identity;
    std::string error{"stale"};
    CHECK(LoadRecordClockIdentityFromFile(boot_id_file.Path().string(),
                                          &identity, &error));
    CHECK(identity.available);
    CHECK(identity.clock == kRecordClock);
    CHECK(identity.boot_id == kBootId);
    CHECK(error.empty());
  }

  // An unreadable boot id is not fatal: the identity is explicitly marked
  // unavailable and the diagnostic says cross-process merging is refused.
  {
    RecordClockIdentity identity;
    std::string error;
    const bool loaded = LoadRecordClockIdentityFromFile(
        missing_file.Path().string(), &identity, &error);
    CHECK(!loaded);
    CHECK(!identity.available);
    CHECK(identity.clock == kRecordClock);
    CHECK(identity.boot_id.empty());
    CHECK(!error.empty());
    CHECK(error.find("cross-process") != std::string::npos);
  }

  // An empty file is a read failure too, and a null error out-param is allowed.
  CHECK(WriteText(empty_file.Path(), ""));
  {
    RecordClockIdentity identity;
    CHECK(!LoadRecordClockIdentityFromFile(empty_file.Path().string(),
                                           &identity, nullptr));
    CHECK(!identity.available);
    CHECK(identity.boot_id.empty());
  }

  // A null identity is refused without touching the error string.
  {
    std::string error;
    CHECK(!LoadRecordClockIdentityFromFile(boot_id_file.Path().string(),
                                           nullptr, &error));
    CHECK(!error.empty());
  }

  std::cout << "record_context_test: all checks passed\n";
  return 0;
}
