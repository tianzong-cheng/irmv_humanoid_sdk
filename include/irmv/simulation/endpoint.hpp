#pragma once
#include <atomic>
#include <filesystem>
#include <thread>

#include "irmv/simulation/protocol.hpp"
#include "irmv/transport/bounded_channel.hpp"
namespace irmv::simulation {
// SDK is the connecting peer. Coordinator owns listener and socket cleanup.
// Parent must already be a same-user 0700 directory. No paths are ever
// unlinked.
struct EndpointConfig {
  std::filesystem::path socket_path;
  protocol::Id run_id{};
  Hello local_hello{};
  Hello peer_hello{};
  std::uint32_t startup_timeout_ms{10000};
  std::uint32_t io_timeout_ms{10000};
};
class Endpoint {
 public:
  ~Endpoint();
  // Lifecycle on control thread, before/after channel use. Single-use endpoint.
  void Start(const EndpointConfig&);
  // Only control thread produces outbound and consumes inbound. Worker owns
  // opposite sides and all socket calls. Overflow fails the run, never evicts.
  bool TrySend(const Frame&) noexcept;
  // Producer-owned readiness: lets the outer driver wait without overflowing.
  bool SendAvailable() const noexcept {
    return Ready() && !Failed() && !outbound_.full();
  }
  bool TryTake(Frame*) noexcept;
  void Stop();
  bool Ready() const noexcept { return ready_.load(); }
  bool Failed() const noexcept { return failed_.load(); }
  std::uint64_t Rejected() const noexcept { return rejected_.load(); }

 private:
  void Run() noexcept;
  EndpointConfig config_;
  transport::BoundedChannel<Frame, kFrameQueueCapacity> inbound_, outbound_;
  std::thread worker_;
  std::atomic<bool> stop_{false}, ready_{false}, failed_{false};
  std::atomic<std::uint64_t> rejected_{0};
  bool started_{false};
  int directory_{-1};
};
}  // namespace irmv::simulation
