#include "irmv/maintenance/commands.hpp"

#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

namespace irmv::maintenance {
namespace {

namespace hardware = irmv::hardware;

constexpr int kExitOk = 0;
constexpr int kExitFailure = 1;
constexpr int kExitUsage = 2;

constexpr std::uint8_t kDisableCommand = 0x00;
constexpr std::uint8_t kResetZeroCommand = 0xFF;

std::atomic<bool> g_stop{false};

extern "C" void HandleStop(int) { g_stop.store(true); }

std::uint64_t MonotonicUs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

struct Arguments {
  std::filesystem::path config_path;
  std::string command;
  bool execute{false};
  std::vector<std::string> joints;
  double torque_nm{0.0};
  bool torque_given{false};
  long long duration_ms{0};
  bool duration_given{false};
  std::uint32_t rate_hz{500};
  double position_tolerance{0.0};
  double velocity_tolerance{0.0};
  long long verify_timeout_ms{0};
};

bool ReadValue(const std::vector<std::string> &arguments, std::size_t *index,
               std::string *value) {
  if (*index + 1 >= arguments.size()) {
    return false;
  }
  *value = arguments[++(*index)];
  return true;
}

bool ParseDouble(const std::string &text, double *value) {
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool ParseInteger(const std::string &text, long long *value) {
  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool Parse(std::vector<std::string> arguments, Arguments *parsed,
           std::string *error) {
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string &argument = arguments[index];
    std::string value;
    if (argument == "--config") {
      if (!ReadValue(arguments, &index, &value)) {
        *error = "--config requires a path";
        return false;
      }
      parsed->config_path = value;
    } else if (argument == "--execute") {
      parsed->execute = true;
    } else if (argument == "--joint") {
      if (!ReadValue(arguments, &index, &value)) {
        *error = "--joint requires a name";
        return false;
      }
      parsed->joints.push_back(value);
    } else if (argument == "--torque-nm") {
      if (!ReadValue(arguments, &index, &value) ||
          !ParseDouble(value, &parsed->torque_nm)) {
        *error = "--torque-nm requires a finite number";
        return false;
      }
      parsed->torque_given = true;
    } else if (argument == "--duration-ms") {
      if (!ReadValue(arguments, &index, &value) ||
          !ParseInteger(value, &parsed->duration_ms) ||
          parsed->duration_ms <= 0) {
        *error = "--duration-ms requires a positive integer";
        return false;
      }
      parsed->duration_given = true;
    } else if (argument == "--rate-hz") {
      long long rate = 0;
      if (!ReadValue(arguments, &index, &value) ||
          !ParseInteger(value, &rate) || rate <= 0 || rate > 100000) {
        *error = "--rate-hz requires a positive integer";
        return false;
      }
      parsed->rate_hz = static_cast<std::uint32_t>(rate);
    } else if (argument == "--verify-position-tolerance-rad") {
      if (!ReadValue(arguments, &index, &value) ||
          !ParseDouble(value, &parsed->position_tolerance) ||
          parsed->position_tolerance <= 0.0) {
        *error = "--verify-position-tolerance-rad requires a positive number";
        return false;
      }
    } else if (argument == "--verify-velocity-tolerance-rad-s") {
      if (!ReadValue(arguments, &index, &value) ||
          !ParseDouble(value, &parsed->velocity_tolerance) ||
          parsed->velocity_tolerance <= 0.0) {
        *error = "--verify-velocity-tolerance-rad-s requires a positive number";
        return false;
      }
    } else if (argument == "--verify-timeout-ms") {
      if (!ReadValue(arguments, &index, &value) ||
          !ParseInteger(value, &parsed->verify_timeout_ms) ||
          parsed->verify_timeout_ms <= 0) {
        *error = "--verify-timeout-ms requires a positive integer";
        return false;
      }
    } else if (argument.size() > 2 && argument.rfind("--", 0) == 0) {
      *error = "unknown option " + argument;
      return false;
    } else if (parsed->command.empty()) {
      parsed->command = argument;
    } else {
      *error = "unexpected argument " + argument;
      return false;
    }
  }
  if (parsed->config_path.empty()) {
    *error = "--config is required";
    return false;
  }
  if (parsed->command.empty()) {
    *error = "a subcommand is required";
    return false;
  }
  return true;
}

bool InstallStopHandlers() {
  return std::signal(SIGINT, HandleStop) != SIG_ERR &&
         std::signal(SIGTERM, HandleStop) != SIG_ERR;
}

const configuration::HardwareJointConfig *FindJoint(
    const configuration::HardwareConfig &hardware, const std::string &name) {
  for (const auto &joint : hardware.joints) {
    if (joint.name == name) {
      return &joint;
    }
  }
  return nullptr;
}

std::size_t JointIndex(const configuration::HardwareConfig &hardware,
                       const std::string &name) {
  for (std::size_t index = 0; index < hardware.joints.size(); ++index) {
    if (hardware.joints[index].name == name) {
      return index;
    }
  }
  return hardware.joints.size();
}

const configuration::BusConfig *FindBus(
    const configuration::HardwareConfig &hardware, const std::string &name) {
  for (const auto &bus : hardware.buses) {
    if (bus.name == name) {
      return &bus;
    }
  }
  return nullptr;
}

void PrintUsage(std::ostream &out) {
  out << "usage: irmv_maintenance --config PATH <subcommand> [options]\n"
         "subcommands:\n"
         "  heartbeat-decode       show fresh feedback, error codes and age\n"
         "  motor-load-test        send torque to explicit joints\n"
         "  motor-load-test-can1   same, restricted to configured channel 1\n"
         "  motor-zero-reset       disable then reset zero for explicit "
         "joints\n"
         "  motor-zero-reset-single  the same for exactly one joint\n"
         "options:\n"
         "  --execute              perform writes (default: dry run)\n"
         "  --joint NAME           target joint (repeatable, required)\n"
         "  --torque-nm N          load torque, within runtime.torque_limits\n"
         "  --duration-ms N        load duration or heartbeat window\n"
         "  --rate-hz N            load/read rate (default 500)\n"
         "  --verify-position-tolerance-rad N\n"
         "  --verify-velocity-tolerance-rad-s N\n"
         "  --verify-timeout-ms N\n";
}

int Heartbeat(const configuration::RuntimeConfig &config,
              const Arguments &arguments, hardware::DriverPort &port,
              std::ostream &out) {
  const long long duration_ms =
      arguments.duration_given ? arguments.duration_ms : 1000;
  if (!InstallStopHandlers()) {
    out << "heartbeat-decode: cannot install stop handlers\n";
    return kExitFailure;
  }
  const auto &hardware = *config.hardware;
  port.Open(hardware);
  const auto period = std::chrono::microseconds(
      1'000'000 / std::max<std::uint32_t>(arguments.rate_hz, 1));
  auto deadline = std::chrono::steady_clock::now();
  const auto end = deadline + std::chrono::milliseconds(duration_ms);
  while (!g_stop.load() && std::chrono::steady_clock::now() < end) {
    const protocol::FeedbackSnapshot snapshot = port.ReadLatest();
    const std::uint64_t now = MonotonicUs();
    for (std::size_t index = 0; index < hardware.joints.size(); ++index) {
      const configuration::HardwareJointConfig &mapping =
          hardware.joints[index];
      const protocol::JointFeedback &joint = snapshot.joints[index];
      const bool have_stamp = joint.rx_stamp_us != 0;
      out << mapping.name << " bus=" << mapping.bus
          << " motor=" << mapping.motor_id << " error=0x" << std::hex
          << joint.error_code << std::dec;
      if (have_stamp && now >= joint.rx_stamp_us) {
        out << " age_us=" << (now - joint.rx_stamp_us);
      } else {
        out << " age_us=none";
      }
      out << " freq_hz=" << std::fixed << std::setprecision(1) << joint.freq_hz
          << std::setprecision(6) << " q=" << joint.q << " dq=" << joint.dq
          << " tau_est=" << joint.tau_est << " valid=" << (joint.valid ? 1 : 0)
          << " connected=" << (joint.connected ? 1 : 0)
          << " temperature=unsupported voltage=unsupported\n";
    }
    out.flush();
    deadline += period;
    std::this_thread::sleep_until(deadline);
  }
  port.Close();
  return kExitOk;
}

int LoadTest(const configuration::RuntimeConfig &config,
             const Arguments &arguments, bool channel_one_only,
             hardware::DriverPort &port, std::ostream &out) {
  const auto &hardware = *config.hardware;
  if (arguments.joints.empty()) {
    out << "motor-load-test: --joint is required; no joint is driven by "
           "default\n";
    return kExitUsage;
  }
  if (!arguments.torque_given || !arguments.duration_given) {
    out << "motor-load-test: --torque-nm and --duration-ms are required\n";
    return kExitUsage;
  }
  std::vector<std::size_t> indices;
  for (const std::string &name : arguments.joints) {
    const std::size_t index = JointIndex(hardware, name);
    if (index == hardware.joints.size()) {
      out << "motor-load-test: unknown joint " << name << "\n";
      return kExitUsage;
    }
    const auto &mapping = hardware.joints[index];
    const configuration::BusConfig *bus = FindBus(hardware, mapping.bus);
    if (bus == nullptr) {
      out << "motor-load-test: joint " << name << " names an unknown bus\n";
      return kExitFailure;
    }
    if (channel_one_only && bus->channel != 1) {
      out << "motor-load-test-can1: joint " << name
          << " is not on a configured channel 1 bus\n";
      return kExitUsage;
    }
    if (std::abs(arguments.torque_nm) > config.torque_limits[index]) {
      out << "motor-load-test: torque " << arguments.torque_nm
          << " exceeds torque_limits[" << name
          << "]=" << config.torque_limits[index] << "\n";
      return kExitUsage;
    }
    indices.push_back(index);
  }

  const std::uint32_t count =
      static_cast<std::uint32_t>(hardware.joints.size());
  hardware::TorqueOutput output;
  output.count = count;
  for (const std::size_t index : indices) {
    output.torque[index] = arguments.torque_nm;
  }

  out << (arguments.execute ? "execute" : "dry-run") << ": "
      << arguments.joints.size() << " joint(s) at " << arguments.torque_nm
      << " Nm for " << arguments.duration_ms << " ms at " << arguments.rate_hz
      << " Hz\n";
  for (std::size_t index = 0; index < hardware.joints.size(); ++index) {
    out << "  " << hardware.joints[index].name
        << " torque=" << output.torque[index] << "\n";
  }
  if (!arguments.execute) {
    return kExitOk;
  }

  if (!InstallStopHandlers()) {
    out << "motor-load-test: cannot install stop handlers\n";
    return kExitFailure;
  }
  port.Open(hardware);
  for (const std::size_t index : indices) {
    const configuration::HardwareJointConfig &mapping = hardware.joints[index];
    const hardware::WriteResult enabled =
        port.EnableMotor(mapping.bus, mapping.motor_id);
    if (!enabled.accepted) {
      out << "motor-load-test: enable refused for " << mapping.name << " (0x"
          << std::hex << enabled.error_code << std::dec << ")\n";
      port.Close();
      return kExitFailure;
    }
  }
  const auto period = std::chrono::microseconds(
      1'000'000 / std::max<std::uint32_t>(arguments.rate_hz, 1));
  auto deadline = std::chrono::steady_clock::now();
  const auto end = deadline + std::chrono::milliseconds(arguments.duration_ms);
  bool failure = false;
  std::uint64_t sends = 0;
  while (!g_stop.load() && std::chrono::steady_clock::now() < end) {
    const hardware::WriteResult result = port.SendTorque(output);
    if (!result.accepted) {
      out << "motor-load-test: send refused (0x" << std::hex
          << result.error_code << std::dec << ")\n";
      failure = true;
      break;
    }
    ++sends;
    deadline += period;
    std::this_thread::sleep_until(deadline);
  }

  hardware::TorqueOutput zero;
  zero.count = count;
  const hardware::WriteResult stopped = port.SendTorque(zero);
  if (!stopped.accepted) {
    out << "motor-load-test: the closing zero frame was refused (0x" << std::hex
        << stopped.error_code << std::dec << ")\n";
    failure = true;
  }
  out << "motor-load-test: " << sends << " frame(s) accepted; closing zero "
      << (stopped.accepted ? "accepted" : "refused") << "\n";
  port.Close();
  return failure ? kExitFailure : kExitOk;
}

int ZeroReset(const configuration::RuntimeConfig &config,
              const Arguments &arguments, bool single,
              hardware::DriverPort &port, MaintenancePort &maintenance,
              std::ostream &out) {
  const auto &hardware = *config.hardware;
  if (arguments.joints.empty()) {
    out << "motor-zero-reset: --joint is required; the broadcast path is "
           "refused because the controlled set cannot be proven\n";
    return kExitUsage;
  }
  if (single && arguments.joints.size() != 1) {
    out << "motor-zero-reset-single: exactly one --joint is required\n";
    return kExitUsage;
  }
  if (arguments.position_tolerance <= 0.0 ||
      arguments.velocity_tolerance <= 0.0 || arguments.verify_timeout_ms <= 0) {
    out << "motor-zero-reset: --verify-position-tolerance-rad, "
           "--verify-velocity-tolerance-rad-s and --verify-timeout-ms are "
           "required positive values\n";
    return kExitUsage;
  }
  for (const std::string &name : arguments.joints) {
    if (FindJoint(hardware, name) == nullptr) {
      out << "motor-zero-reset: unknown joint " << name << "\n";
      return kExitUsage;
    }
  }
  out << (arguments.execute ? "execute" : "dry-run") << ": reset zero for";
  for (const std::string &name : arguments.joints) {
    out << " " << name;
  }
  out << " after disable; position tolerance " << arguments.position_tolerance
      << " rad, velocity tolerance " << arguments.velocity_tolerance
      << " rad/s, timeout " << arguments.verify_timeout_ms << " ms\n";
  out << "  firmware persistence is not acknowledged by this protocol; a "
         "successful fresh readback is not a reboot verification\n";
  if (!arguments.execute) {
    return kExitOk;
  }

  if (!InstallStopHandlers()) {
    out << "motor-zero-reset: cannot install stop handlers\n";
    return kExitFailure;
  }
  port.Open(hardware);
  // A pre-reset observation is required for every target: a silent motor is
  // not a calibrated one. Opening an asynchronous driver does not guarantee
  // that the first heartbeat has arrived, so this waits within the caller's
  // verification deadline instead of reading the all-default sample once and
  // declaring the calibration impossible.
  {
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(arguments.verify_timeout_ms);
    std::string missing;
    bool ready = false;
    while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
      const protocol::FeedbackSnapshot snapshot = port.ReadLatest();
      const std::uint64_t now_us = MonotonicUs();
      missing.clear();
      for (const std::string &name : arguments.joints) {
        const std::size_t index = JointIndex(hardware, name);
        const protocol::JointFeedback &joint = snapshot.joints[index];
        const bool fresh =
            joint.valid && joint.rx_stamp_us != 0 &&
            joint.rx_stamp_us <= now_us &&
            now_us - joint.rx_stamp_us <= config.timeouts.feedback_us;
        if (!fresh) {
          missing = name;
          break;
        }
      }
      if (missing.empty()) {
        ready = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!ready) {
      if (missing.empty()) {
        missing = arguments.joints.front();
      }
      out << "motor-zero-reset: no fresh pre-reset feedback for " << missing
          << "; nothing was written\n";
      port.Close();
      return kExitFailure;
    }
  }

  // Each reset owns its own baseline. A heartbeat that arrived while the motor
  // was being disabled is not a read-back of the reset.
  std::vector<std::uint64_t> reset_time(arguments.joints.size(), 0);
  std::vector<protocol::Sequence> baseline_seq(arguments.joints.size(), 0);
  for (std::size_t target = 0; target < arguments.joints.size(); ++target) {
    const std::string &name = arguments.joints[target];
    const configuration::HardwareJointConfig *mapping =
        FindJoint(hardware, name);
    const hardware::WriteResult disabled =
        maintenance.Disable(mapping->bus, mapping->motor_id);
    if (!disabled.accepted) {
      out << "motor-zero-reset: disable refused for " << name << " (0x"
          << std::hex << disabled.error_code << std::dec << ")\n";
      port.Close();
      return kExitFailure;
    }
    baseline_seq[target] = port.ReadLatest().snapshot_seq;
    const hardware::WriteResult reset =
        maintenance.ResetZero(mapping->bus, mapping->motor_id);
    reset_time[target] = MonotonicUs();
    if (!reset.accepted) {
      out << "motor-zero-reset: reset refused for " << name << " (0x"
          << std::hex << reset.error_code << std::dec << ")\n";
      port.Close();
      return kExitFailure;
    }
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(arguments.verify_timeout_ms);
  std::vector<bool> verified(arguments.joints.size(), false);
  while (std::chrono::steady_clock::now() < deadline) {
    const protocol::FeedbackSnapshot snapshot = port.ReadLatest();
    bool all = true;
    for (std::size_t target = 0; target < arguments.joints.size(); ++target) {
      if (verified[target]) {
        continue;
      }
      const std::string &name = arguments.joints[target];
      const std::size_t index = JointIndex(hardware, name);
      const protocol::JointFeedback &joint = snapshot.joints[index];
      const configuration::HardwareJointConfig *mapping =
          FindJoint(hardware, name);
      // After the reset the raw rotor angle is zero; the configured offset
      // implies this joint-side reading.
      const double expected = -static_cast<double>(mapping->direction) *
                              mapping->encoder_offset /
                              mapping->reduction_ratio;
      const bool after_reset = joint.rx_stamp_us >= reset_time[target] &&
                               snapshot.snapshot_seq > baseline_seq[target];
      if (after_reset && joint.valid && joint.error_code == 0 &&
          std::abs(joint.q - expected) <= arguments.position_tolerance &&
          std::abs(joint.dq) <= arguments.velocity_tolerance) {
        verified[target] = true;
      } else {
        all = false;
      }
    }
    if (all) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  bool success = true;
  for (std::size_t target = 0; target < arguments.joints.size(); ++target) {
    out << "motor-zero-reset: " << arguments.joints[target]
        << (verified[target] ? " fresh readback passed"
                             : " fresh readback failed")
        << "\n";
    success = success && verified[target];
  }
  port.Close();
  return success ? kExitOk : kExitFailure;
}

}  // namespace

hardware::WriteResult DriverPortMaintenancePort::Disable(
    const std::string &bus_name, std::uint32_t motor_id) {
  return port_.SendMotorCommand(bus_name, motor_id, kDisableCommand);
}

hardware::WriteResult DriverPortMaintenancePort::ResetZero(
    const std::string &bus_name, std::uint32_t motor_id) {
  return port_.SendMotorCommand(bus_name, motor_id, kResetZeroCommand);
}

int Run(const std::vector<std::string> &arguments, hardware::DriverPort &port,
        MaintenancePort &maintenance, std::ostream &out) {
  g_stop.store(false);
  Arguments parsed;
  std::string error;
  if (!Parse(arguments, &parsed, &error)) {
    out << "irmv_maintenance: " << error << "\n";
    PrintUsage(out);
    return kExitUsage;
  }

  configuration::RuntimeConfig config;
  try {
    config = configuration::LoadConfig(parsed.config_path);
  } catch (const std::exception &exception) {
    out << "irmv_maintenance: configuration error: " << exception.what()
        << "\n";
    return kExitFailure;
  }
  if (!config.hardware) {
    out << "irmv_maintenance: the configuration carries no hardware table\n";
    return kExitFailure;
  }

  try {
    if (parsed.command == "heartbeat-decode") {
      return Heartbeat(config, parsed, port, out);
    }
    if (parsed.command == "motor-load-test") {
      return LoadTest(config, parsed, false, port, out);
    }
    if (parsed.command == "motor-load-test-can1") {
      return LoadTest(config, parsed, true, port, out);
    }
    if (parsed.command == "motor-zero-reset") {
      return ZeroReset(config, parsed, false, port, maintenance, out);
    }
    if (parsed.command == "motor-zero-reset-single") {
      return ZeroReset(config, parsed, true, port, maintenance, out);
    }
  } catch (const std::exception &exception) {
    out << "irmv_maintenance: " << exception.what() << "\n";
    return kExitFailure;
  }

  out << "irmv_maintenance: unknown subcommand " << parsed.command << "\n";
  PrintUsage(out);
  return kExitUsage;
}

}  // namespace irmv::maintenance
