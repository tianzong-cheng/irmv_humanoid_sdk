#pragma once

#include <array>
#include <cstdint>

#include "irmv/protocol/types.hpp"

namespace irmv::hardware {

struct TorqueOutput {
  std::uint32_t count{0};
  std::array<double, protocol::kMaxJoints> torque{};
};

struct WriteResult {
  // Acceptance means the host backend accepted the request. It does not prove
  // that physical hardware applied the requested torque.
  bool accepted{false};
  std::uint32_t error_code{0};
};

class Backend {
 public:
  virtual ~Backend() = default;

  virtual void Initialize() = 0;
  virtual protocol::FeedbackSnapshot Read(protocol::TimeUs now_us) = 0;
  virtual WriteResult Write(protocol::TimeUs now_us,
                            const TorqueOutput &output) = 0;
  virtual void Close() = 0;
};

}  // namespace irmv::hardware
