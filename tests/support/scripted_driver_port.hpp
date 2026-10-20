#pragma once

#include <cstdint>
#include <functional>
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

  struct Command {
    std::string bus_name;
    std::uint32_t motor_id;
    std::uint8_t command;
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
    if (before_read) {
      before_read();
    }
    return snapshot;
  }

  WriteResult SendTorque(const TorqueOutput &output) override {
    ++send_calls;
    WriteResult result = next_result;
    sends.push_back(Send{output, result});
    return result;
  }

  WriteResult EnableMotor(const std::string &bus_name,
                          std::uint32_t motor_id) override {
    ++enable_calls;
    enabled.push_back(bus_name + ":" + std::to_string(motor_id));
    return next_enable_result;
  }

  WriteResult SendMotorCommand(const std::string &bus_name,
                               std::uint32_t motor_id,
                               std::uint8_t command) override {
    ++command_calls;
    commands.push_back(Command{bus_name, motor_id, command});
    if (on_command) {
      on_command(commands.back());
    }
    return next_command_result;
  }

  void Close() noexcept override {
    ++close_calls;
    opened = false;
  }

  // Script.
  protocol::FeedbackSnapshot snapshot;
  std::function<void()> before_read;
  std::function<void(const Command &)> on_command;
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
  WriteResult next_enable_result{true, 0};
  int enable_calls{0};
  std::vector<std::string> enabled;
  WriteResult next_command_result{true, 0};
  int command_calls{0};
  std::vector<Command> commands;
};

}  // namespace irmv::hardware
