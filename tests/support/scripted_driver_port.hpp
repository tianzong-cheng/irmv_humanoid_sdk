#pragma once

#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "irmv/hardware/driver_port.hpp"

namespace irmv::hardware {

// A test-only DriverPort. It is not installed and it is never selected by the
// production factory: the tests inject it directly.
class ScriptedDriverPort final : public DriverPort {
 public:
  struct Send {
    TorqueOutput output;
    WriteResult result;
  };

  void Open(const configuration::HardwareConfig &hardware) override {
    ++open_calls;
    opened_hardware = hardware;
    if (open_error) {
      throw std::runtime_error(open_error_message);
    }
    opened = true;
  }

  protocol::FeedbackSnapshot ReadLatest() override {
    ++read_calls;
    return snapshot;
  }

  WriteResult SendTorque(const TorqueOutput &output) override {
    ++send_calls;
    WriteResult result = next_result;
    sends.push_back(Send{output, result});
    return result;
  }

  void Close() noexcept override {
    ++close_calls;
    opened = false;
  }

  // Script.
  protocol::FeedbackSnapshot snapshot;
  WriteResult next_result{true, 0};
  bool open_error{false};
  std::string open_error_message{"scripted open failure"};

  // Observations.
  bool opened{false};
  int open_calls{0};
  int read_calls{0};
  int send_calls{0};
  int close_calls{0};
  configuration::HardwareConfig opened_hardware;
  std::vector<Send> sends;
};

}  // namespace irmv::hardware
