#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "irmv/telemetry/fault_record.hpp"

// Internal to the recorder: the worker-side writer of `faults.jsonl`. The
// reader shares the line contract in `fault_record.hpp`; this side only
// serializes one occurrence and appends it.
namespace irmv::telemetry {

class FaultWriter {
 public:
  explicit FaultWriter(std::filesystem::path path);
  ~FaultWriter();

  FaultWriter(const FaultWriter &) = delete;
  FaultWriter &operator=(const FaultWriter &) = delete;

  // Creates/truncates the file for a new run. Called by the control thread
  // before the worker starts.
  bool Prepare(std::string *error);

  // Appends one line and flushes it. Returns false when the hook refused the
  // write or the device failed; the caller counts and reports the loss, and
  // never treats the occurrence as persisted. Reopens the file on the next call
  // after a failure.
  bool Write(const RecordedFault &fault);

 private:
  struct State;
  std::unique_ptr<State> state_;
};

// Test-only seam: called on the worker immediately before one line is written.
// A test can block inside it (a blocked device) or return false (a failed
// write). Production never sets it.
void SetFaultLineWriteHookForTest(bool (*hook)(void *context),
                                  void *context) noexcept;

}  // namespace irmv::telemetry
