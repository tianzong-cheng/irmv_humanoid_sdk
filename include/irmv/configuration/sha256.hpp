#pragma once

#include <string>
#include <string_view>

namespace irmv::configuration {

std::string Sha256(std::string_view input);

}  // namespace irmv::configuration
