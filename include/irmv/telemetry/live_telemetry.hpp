#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>

namespace irmv::telemetry {
struct RecordSample;
struct RecordedFault;
struct VelocitySample;
}  // namespace irmv::telemetry

// The recorder worker's low-latency tail, outside every control loop.
//
// A recorder worker is the only producer on its socket and the service's
// receive thread is the only consumer. Every attempt is a non-blocking Unix
// datagram: an absent receiver, a full socket or a service restart costs the
// message and is counted, never a wait, a retry loop or an allocation on a
// control thread. The envelope is versioned and carries its own transport
// sequence, which is independent of the sample and fault ordinals on disk.
//
// The tail is temporary. The committed record stays authoritative; a receiver
// reconciles the two by source, run path and identity, and a message that never
// reached the service is not a hole in the history.
namespace irmv::telemetry {

struct LiveTelemetryOptions {
  // Empty disables the outlet. The composition layer reads the environment once
  // at startup; the hot path never does.
  std::filesystem::path socket_path;
};

class LiveTelemetrySender {
 public:
  // `metadata_json` is the run's metadata object, optionally augmented with
  // `source` (runtime/application) and `run_path`. A malformed document leaves
  // the sender disabled, named by `error`, rather than throwing.
  LiveTelemetrySender(LiveTelemetryOptions options,
                      std::string metadata_json) noexcept;
  ~LiveTelemetrySender();

  LiveTelemetrySender(const LiveTelemetrySender &) = delete;
  LiveTelemetrySender &operator=(const LiveTelemetrySender &) = delete;

  // Sends one envelope. Never blocks, throws or allocates on a control path;
  // it is called from the recorder worker only. Returns false when the outlet
  // is disabled, the payload is invalid or larger than 64 KiB, or the send was
  // refused. Each call advances the transport sequence.
  bool TrySend(std::string_view kind, std::string_view payload_json) noexcept;

  bool enabled() const noexcept;
  std::uint64_t sent() const noexcept;
  std::uint64_t failures() const noexcept;
  std::uint64_t oversize() const noexcept;
  std::uint64_t transport_seq() const noexcept;
  const std::string &error() const noexcept;

 private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

// The payload encoders shared by the recorder workers. Times, sequences and
// ordinals are decimal strings so a JSON Number never rounds a 64-bit value.
// Non-finite doubles are the explicit strings "NaN", "+Inf" and "-Inf".
std::string EncodeLiveSample(const RecordSample &sample);
std::string EncodeLiveFault(const RecordedFault &fault);
std::string EncodeLiveVelocity(const VelocitySample &sample);

// A deterministic seam for the core test: when installed, every send is handed
// to the hook instead of the socket. A zero return is a successful send, any
// other value is a refusal. Production installs no hook.
void SetLiveTelemetrySendHookForTest(int (*hook)(void *context,
                                                 std::string_view bytes),
                                     void *context) noexcept;

}  // namespace irmv::telemetry
