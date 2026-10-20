#pragma once

#include <memory>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"

namespace irmv::hardware {

// Builds the backend the configuration selects. `fake` keeps the shipped
// scenario-observable backend; `real` is refused by a build that did not enable
// the vendor dependencies, never downgraded to fake.
//
// Throws std::runtime_error when the backend is unavailable. The caller is
// responsible for the startup sequence: nothing here touches a device.
std::unique_ptr<Backend> MakeBackend(const configuration::RuntimeConfig &config);

}  // namespace irmv::hardware
