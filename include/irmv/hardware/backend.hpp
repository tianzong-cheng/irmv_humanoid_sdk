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

  // Non-blocking coherent read of the current feedback. A backend that has not
  // produced a frame yet reports the configured joint count with every sample
  // at its all-default value - unobserved, unconnected, no receive stamp, no
  // driver error, and the zeros of an untouched sample. That shape is the
  // documented "nothing observed yet" the runtime's boot grace waits for: a
  // frame that claims absence while carrying any value is treated as an
  // observation and judged by it, and a fabricated valid zero measurement is
  // never a substitute.
  virtual protocol::FeedbackSnapshot Read(protocol::TimeUs now_us) = 0;
  virtual WriteResult Write(protocol::TimeUs now_us,
                            const TorqueOutput &output) = 0;
  virtual void Close() = 0;
};

}  // namespace irmv::hardware
