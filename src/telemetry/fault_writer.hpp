#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

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
  // never treats the occurrence as persisted. On any failure the writer rolls
  // the file back to its last committed byte, so a torn line can never merge
  // with the next one into a malformed complete line. It reopens the file on
  // the next call after a failure.
  bool Write(const RecordedFault &fault);

 private:
  struct State;
  // Closes the stream and truncates the file back to the last line that was
  // written and flushed in full.
  void RollBack() noexcept;

  std::unique_ptr<State> state_;
};

// Test-only seam: called on the worker before one line is written, with the
// serialized line. It returns the action:
//   0 - write the whole line;
//   1 - refuse it (no bytes are written);
//   2 - simulate a torn write (a prefix is written, then the sink fails).
// A test can also block inside the hook (a blocked device). Production never
// sets it. The function and its context are read as one pair under a mutex; a
// test must keep the context alive until the worker is quiescent, because an
// in-flight call may already hold the pair after removal. The `line` view is
// valid only for the duration of the call; a hook that needs the text must copy
// it.
void SetFaultLineWriteHookForTest(int (*hook)(void *context,
                                              std::string_view line),
                                  void *context) noexcept;

}  // namespace irmv::telemetry
