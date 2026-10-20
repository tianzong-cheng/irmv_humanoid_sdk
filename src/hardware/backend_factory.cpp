#include "irmv/hardware/backend_factory.hpp"

#include <stdexcept>
#include <string>

#include "irmv/hardware/driver_port.hpp"
#include "irmv/hardware/fake_backend.hpp"
#include "irmv/hardware/real_backend.hpp"

namespace irmv::hardware {

std::unique_ptr<Backend> MakeBackend(
    const configuration::RuntimeConfig &config) {
  if (config.backend == "fake") {
    return std::make_unique<FakeBackend>(config);
  }
  if (config.backend == "real") {
#ifdef IRMV_HAS_REAL_BACKEND
    std::unique_ptr<DriverPort> port = MakeIrmvDriverPort();
    if (!port) {
      throw std::runtime_error("the real driver port is unavailable");
    }
    return std::make_unique<RealBackend>(config, std::move(port));
#else
    throw std::runtime_error(
        "this build does not include the real backend "
        "(configure with -DIRMV_ENABLE_REAL_BACKEND=ON)");
#endif
  }
  throw std::runtime_error("unknown backend \"" + config.backend + "\"");
}

}  // namespace irmv::hardware
