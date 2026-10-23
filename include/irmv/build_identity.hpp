#pragma once

#include <string_view>

namespace irmv {

// The revision this build was made from, as `git rev-parse HEAD` reported it
// when the build tree last checked it. The value lives in a generated source of
// the build tree, so asking for it never allocates, reads a file or runs Git.
// A build from a source copy without Git metadata reports "unknown"; it never
// invents a revision.
inline constexpr std::string_view kUnknownSdkGitSha{"unknown"};

std::string_view SdkGitSha() noexcept;

}  // namespace irmv
