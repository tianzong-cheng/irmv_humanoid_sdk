#pragma once

// Internal scaffolding of the two DDS transports in this directory: the runtime
// transport (DdsTransport reads requests, commands and heartbeats and writes
// results and states) and its application mirror (ClientTransport, the other
// direction of the same five topics). Both run the same bounded loop, so the
// wait period, the read batch, the stop bound, the interface binding and the
// guarded read/write policy live in one definition and cannot drift apart.
//
// This header is internal to src/transport and is not installed: nothing
// outside the two transport translation units includes it.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <string>

#include "irmv/protocol/types.hpp"

namespace irmv::transport {

// How long the transport thread may wait for inbound traffic before it looks at
// the outbound channels again. The value bounds both the delay of an outbound
// sample and the time Stop() needs to be noticed.
inline constexpr int kWaitTimeoutMs = 10;

// Samples read from one reader per wake-up. Bounded so that a burst cannot keep
// the thread from writing pending output or from noticing a stop request.
inline constexpr int kReadBatch = 16;

// Bound Stop() gives the transport thread to leave its loop before it joins.
// The loop only blocks in bounded operations (a ten millisecond WaitSet wait
// and DDS writes bounded by their max_blocking_time), so the thread leaves well
// inside this bound and the join that follows returns immediately.
inline constexpr int kStopTimeoutMs = 2000;

inline protocol::TimeUs NowUs() {
  return static_cast<protocol::TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Pins the participant to one network interface. The pinned C++ API takes a
// configuration string on the participant constructor and parses it as XML when
// it starts with '<'; this is the only per-participant configuration route the
// pinned version has.
inline std::string InterfaceConfig(const std::string &interface) {
  return "<CycloneDDS><Domain "
         "id=\"any\"><General><Interfaces><NetworkInterface "
         "name=\"" +
         interface + "\"/></Interfaces></General></Domain></CycloneDDS>";
}

// A reliable writer can refuse a sample when its history is full and the
// blocking time expires. The sample is counted instead of being lost silently,
// and the transport thread keeps running. The return value tells the one caller
// that needs the distinction - the heartbeat writer, whose publication is its
// own chain step - everything else ignores it.
template <typename Writer, typename Sample>
bool TryWrite(Writer &writer, const Sample &sample,
              std::atomic<std::uint64_t> &dropped) {
  try {
    writer.write(sample);
    return true;
  } catch (const std::exception &) {
    dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
}

// Every read in the transport thread runs behind this guard. The thread entry
// function must not let an exception escape it, so a failing DDS call ends the
// transport loop cleanly - the same policy as the guarded WaitSet wait - and is
// counted on its own counter, apart from samples that fail validation.
template <typename Body>
bool ReadGuarded(std::atomic<std::uint64_t> &read_failures, Body body) {
  try {
    body();
    return true;
  } catch (const std::exception &) {
    read_failures.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
}

}  // namespace irmv::transport
