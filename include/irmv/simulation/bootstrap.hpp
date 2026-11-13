#pragma once
#include "irmv/configuration/config.hpp"
#include "irmv/simulation/endpoint.hpp"
namespace irmv::simulation {
// Startup only. Artifact is owned by the independent launcher, never rewritten.
EndpointConfig LoadBootstrap(const configuration::RuntimeConfig&, Role,
                             const std::string& application_digest = {});
}  // namespace irmv::simulation
