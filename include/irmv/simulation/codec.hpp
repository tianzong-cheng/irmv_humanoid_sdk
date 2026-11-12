#pragma once
#include "irmv/simulation/protocol.hpp"
namespace irmv::simulation {
struct ByteView {
  const std::uint8_t* data{nullptr};
  std::size_t size{0};
};
struct ByteBuffer {
  std::array<std::uint8_t, kMaxPacketBytes> bytes{};
  std::size_t size{0};
};
bool Encode(const Frame&, ByteBuffer*) noexcept;
bool Decode(ByteView, Frame*) noexcept;
}  // namespace irmv::simulation
