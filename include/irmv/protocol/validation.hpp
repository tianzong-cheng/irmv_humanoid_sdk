#pragma once

#include <cstddef>

#include "irmv/protocol/types.hpp"

namespace irmv::protocol {

struct ValidationResult {
  bool ok{false};
  RejectReason reason{RejectReason::Malformed};
};

bool IsValid(ExecutionState value);
bool IsValid(OutputAction value);
bool IsValid(Operation value);
bool IsValid(ResultStatus value);
bool IsValid(RejectReason value);
bool IsValid(FaultReason value);
bool IsValid(StopReason value);

ValidationResult ValidateCommandShape(const JointCommand &command,
                                      std::size_t expected_count);

}  // namespace irmv::protocol
