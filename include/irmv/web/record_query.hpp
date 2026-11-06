#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

// The read-only query surface of the experiment web workbench.
//
// One RecordQuery owns a configured records root and answers JSONL requests
// about the runs inside it. It is a pure reader: it opens no socket, publishes
// nothing and cannot command a machine. Run identities are opaque fixed-width
// hex strings resolved from the catalog, never file paths supplied by a caller.
//
// The wire contract is the one in the experiment-web plan: absolute times and
// sequences travel as decimal strings, a request is one JSON object and a reply
// is one line of JSON whose `request_id` is the one it was asked under. A
// request the build does not implement is answered with an explicit error, not
// a silent success with no data.
namespace irmv::web {

class RecordQuery {
 public:
  // `records_root` is the only directory this query will ever look inside. It
  // is canonicalized at construction; a run whose canonical directory escapes
  // it - through `..` or a symlink - is refused rather than read.
  explicit RecordQuery(std::filesystem::path records_root);
  ~RecordQuery();

  RecordQuery(const RecordQuery &) = delete;
  RecordQuery &operator=(const RecordQuery &) = delete;

  // Executes one request and returns one JSON object without a trailing
  // newline. It never throws for a malformed request: the reply carries
  // `ok:false` and an `error`.
  std::string Execute(std::string_view request_json);

 private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace irmv::web
