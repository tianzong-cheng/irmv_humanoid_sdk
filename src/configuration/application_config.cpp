#include "irmv/configuration/application_config.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "irmv/configuration/sha256.hpp"
#include "parse_helpers.hpp"

namespace irmv::configuration {
namespace {

using detail::Fail;
using detail::FiniteMember;
using detail::JointValueRule;
using detail::Json;
using detail::Member;
using detail::ObjectMember;
using detail::PositiveIntegerMember;
using detail::ReadJointMap;
using detail::RejectUnknownJointKeys;

// The action names are declared once, in the order the canonical digest and the
// duplicate-binding precedence use. `required` is the mode set the behaviour
// FSM needs; the velocity actions may stay unbound and are then inert.
struct InputBinding {
  const char *name;
  InputAction action;
  bool required;
};

constexpr InputBinding kInputBindings[] = {
    {"emergency_stop", InputAction::EmergencyStop, true},
    {"damping", InputAction::Damping, true},
    {"initialize", InputAction::Initialize, true},
    {"zero_torque", InputAction::ZeroTorque, true},
    {"reset", InputAction::Reset, true},
    {"exit", InputAction::Exit, true},
    {"velocity_forward", InputAction::VelocityForward, false},
    {"velocity_backward", InputAction::VelocityBackward, false},
    {"velocity_left", InputAction::VelocityLeft, false},
    {"velocity_right", InputAction::VelocityRight, false},
    {"velocity_yaw_left", InputAction::VelocityYawLeft, false},
    {"velocity_yaw_right", InputAction::VelocityYawRight, false},
};

// Emergency stop is the reserved space bar: the terminal and every scripted
// stop path must reach it without knowing the rest of the map.
constexpr char kEmergencyStopKey = ' ';

// Initialization gains are read through the same joint-map completeness rules
// as every runtime joint map. kp must hold position at the initialization
// target, so it is positive for every initialization joint; kd only has to
// damp, so it may be zero. Nothing here supplies a fallback gain: every joint
// must be configured explicitly.
void ReadGainMap(const Json &mapping, const std::string &path,
                 const std::vector<std::string> &joint_names,
                 InitializationGains &gains) {
  if (!mapping.is_object()) {
    Fail(path, "must be an object keyed by joint name");
  }
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    const std::string entry_path = path + "." + joint_names[index];
    const auto entry = mapping.find(joint_names[index]);
    if (entry == mapping.end()) {
      Fail(entry_path, "is required");
    }
    if (!entry->is_object()) {
      Fail(entry_path, "must be an object with kp and kd");
    }
    const double kp = FiniteMember(*entry, "kp", entry_path);
    const double kd = FiniteMember(*entry, "kd", entry_path);
    if (kp < 0.0) {
      Fail(entry_path + ".kp", "must be non-negative");
    }
    if (kp == 0.0) {
      Fail(entry_path + ".kp", "must be positive to hold position");
    }
    if (kd < 0.0) {
      Fail(entry_path + ".kd", "must be non-negative");
    }
    gains.kp[index] = kp;
    gains.kd[index] = kd;
  }
  RejectUnknownJointKeys(mapping, path, joint_names);
}

void ReadTrajectory(const Json &object, const std::string &path,
                    TrajectoryConfig &trajectory) {
  trajectory.duration_us = PositiveIntegerMember(object, "duration_us", path);
  trajectory.position_tolerance =
      FiniteMember(object, "position_tolerance", path);
  if (trajectory.position_tolerance <= 0.0) {
    Fail(path + ".position_tolerance", "must be positive");
  }
  trajectory.velocity_tolerance =
      FiniteMember(object, "velocity_tolerance", path);
  if (trajectory.velocity_tolerance <= 0.0) {
    Fail(path + ".velocity_tolerance", "must be positive");
  }
  trajectory.stable_window_us =
      PositiveIntegerMember(object, "stable_window_us", path);
  trajectory.deadline_us = PositiveIntegerMember(object, "deadline_us", path);
  if (trajectory.deadline_us <= trajectory.duration_us) {
    Fail(path + ".deadline_us", "must exceed duration_us");
  }
  // The deadline also has to leave room for the whole stability window after
  // the ramp: a deadline before `duration_us + stable_window_us` could never
  // complete, so such a configuration would load already doomed. The equality
  // is admissible - a window that completes exactly on the deadline is still in
  // time. The subtraction is safe because the check above proved
  // `deadline_us > duration_us`.
  if (trajectory.stable_window_us >
      trajectory.deadline_us - trajectory.duration_us) {
    Fail(path + ".deadline_us",
         "must leave room for stable_window_us (" +
             std::to_string(trajectory.stable_window_us) +
             " us) after duration_us (" +
             std::to_string(trajectory.duration_us) + " us)");
  }
}

void ReadInitialization(const Json &object, const std::string &path,
                        const RuntimeConfig &runtime,
                        InitializationConfig &initialization) {
  ReadJointMap(Member(object, "target_position", path),
               path + ".target_position", runtime.joint_names,
               JointValueRule::Finite, initialization.target_q);
  for (std::size_t index = 0; index < runtime.joint_count(); ++index) {
    const double target = initialization.target_q[index];
    if (target < runtime.position_min[index] ||
        target > runtime.position_max[index]) {
      Fail(path + ".target_position." + runtime.joint_names[index],
           "is outside the joint position limits");
    }
  }
  ReadGainMap(Member(object, "gains", path), path + ".gains",
              runtime.joint_names, initialization.gains);
  ReadTrajectory(ObjectMember(object, "trajectory", path), path + ".trajectory",
                 initialization.trajectory);
}

// The application periods are host supervision of the runtime, so they are
// checked against the runtime timeouts they sit inside instead of being trusted
// on their own.
void ReadSupervision(const Json &object, const std::string &path,
                     const RuntimeConfig &runtime,
                     SupervisionConfig &supervision) {
  supervision.target_period_us =
      PositiveIntegerMember(object, "target_period_us", path);
  supervision.heartbeat_period_us =
      PositiveIntegerMember(object, "heartbeat_period_us", path);
  supervision.request_retry_us =
      PositiveIntegerMember(object, "request_retry_us", path);
  supervision.request_deadline_us =
      PositiveIntegerMember(object, "request_deadline_us", path);
  supervision.state_timeout_us =
      PositiveIntegerMember(object, "state_timeout_us", path);
  if (supervision.request_deadline_us <= supervision.request_retry_us) {
    Fail(path + ".request_deadline_us", "must exceed request_retry_us");
  }
  if (supervision.heartbeat_period_us >= runtime.timeouts.supervisor_us) {
    Fail(path + ".heartbeat_period_us",
         "must be less than runtime.safety.timeouts.supervisor_us");
  }
  if (supervision.state_timeout_us > runtime.timeouts.source_state_us) {
    // The client's "fresh" is the host receive age against state_timeout_us
    // while the initialization generator refuses a snapshot whose *runtime*
    // publication age exceeds source_state_us. A larger state_timeout_us would
    // therefore let the upper layer call a snapshot fresh that its own
    // generator must reject, which latches InitializationFailed instead of
    // skipping a frame.
    Fail(path + ".state_timeout_us",
         "must not exceed runtime.safety.timeouts.source_state_us (" +
             std::to_string(runtime.timeouts.source_state_us) + " us)");
  }
  const protocol::TimeUs command_deadline =
      std::min(runtime.timeouts.command_us, runtime.timeouts.source_state_us);
  if (supervision.target_period_us >= command_deadline) {
    Fail(path + ".target_period_us",
         "must be less than runtime.safety.timeouts.command_us and "
         "runtime.safety.timeouts.source_state_us (" +
             std::to_string(command_deadline) + " us)");
  }
}

void ReadInput(const Json &object, const std::string &path,
               InputConfig &input) {
  input.intent_timeout_us =
      PositiveIntegerMember(object, "intent_timeout_us", path);
  input.repeat_guard_us =
      PositiveIntegerMember(object, "repeat_guard_us", path);
  const Json &bindings = Member(object, "bindings", path);
  if (!bindings.is_object()) {
    Fail(path + ".bindings", "must be an object keyed by action name");
  }
  std::array<const char *, 256> owners{};
  for (const InputBinding &binding : kInputBindings) {
    const std::string binding_path = path + ".bindings." + binding.name;
    const auto value = bindings.find(binding.name);
    if (value == bindings.end()) {
      if (binding.required) {
        Fail(binding_path, "is required");
      }
      continue;
    }
    if (!value->is_string()) {
      Fail(binding_path, "must be a one-character string");
    }
    const std::string key = value->get<std::string>();
    if (key.size() != 1) {
      Fail(binding_path, "must be exactly one byte");
    }
    const std::size_t byte =
        static_cast<std::size_t>(static_cast<unsigned char>(key[0]));
    if (binding.action == InputAction::EmergencyStop) {
      if (key[0] != kEmergencyStopKey) {
        Fail(binding_path, "must be the space character");
      }
    } else if (key[0] == kEmergencyStopKey) {
      Fail(binding_path,
           "must not claim the space reserved for emergency_stop");
    }
    if (owners[byte] != nullptr) {
      Fail(binding_path, std::string("is already bound to ") + owners[byte]);
    }
    owners[byte] = binding.name;
    input.actions[byte] = binding.action;
  }
  for (auto item = bindings.begin(); item != bindings.end(); ++item) {
    bool known = false;
    for (const InputBinding &binding : kInputBindings) {
      if (item.key() == binding.name) {
        known = true;
        break;
      }
    }
    if (!known) {
      Fail(path + ".bindings." + item.key(), "does not name a known action");
    }
  }
}

// Canonical form of the application+input sections only. The joint maps are
// rebuilt from the parsed arrays in robot.joints order and the bindings from
// the action table, so key order in the file does not enter the digest and no
// runtime parameter can.
Json CanonicalApplication(const ApplicationConfig &config) {
  Json target_positions = Json::object();
  Json gains = Json::object();
  for (std::size_t index = 0; index < config.joint_count(); ++index) {
    const std::string &name = config.joint_names()[index];
    target_positions[name] = config.initialization.target_q[index];
    gains[name] = {{"kp", config.initialization.gains.kp[index]},
                   {"kd", config.initialization.gains.kd[index]}};
  }
  Json bindings = Json::object();
  for (const InputBinding &binding : kInputBindings) {
    for (std::size_t byte = 0; byte < config.input.actions.size(); ++byte) {
      if (config.input.actions[byte] == binding.action) {
        bindings[binding.name] = std::string(1, static_cast<char>(byte));
        break;
      }
    }
  }
  return {
      {"application",
       {{"initialization",
         {{"target_position", target_positions},
          {"gains", gains},
          {"trajectory",
           {{"duration_us", config.initialization.trajectory.duration_us},
            {"position_tolerance",
             config.initialization.trajectory.position_tolerance},
            {"velocity_tolerance",
             config.initialization.trajectory.velocity_tolerance},
            {"stable_window_us",
             config.initialization.trajectory.stable_window_us},
            {"deadline_us", config.initialization.trajectory.deadline_us}}}}},
        {"supervision",
         {{"target_period_us", config.supervision.target_period_us},
          {"heartbeat_period_us", config.supervision.heartbeat_period_us},
          {"request_retry_us", config.supervision.request_retry_us},
          {"request_deadline_us", config.supervision.request_deadline_us},
          {"state_timeout_us", config.supervision.state_timeout_us}}}}},
      {"input",
       {{"intent_timeout_us", config.input.intent_timeout_us},
        {"repeat_guard_us", config.input.repeat_guard_us},
        {"bindings", bindings}}}};
}

}  // namespace

ApplicationConfig LoadApplicationConfig(const std::filesystem::path &path,
                                        const ConfigOverrides &overrides) {
  ApplicationConfig config;
  // The runtime snapshot comes from the phase-1 loader, so config_id/layout_id
  // are byte-identical to what the runtime publishes. The application sections
  // are read from a second parse of the same file, because LoadConfig keeps its
  // document private: startup-only cost, and the runtime digest still comes
  // from LoadConfig's own canonical form.
  config.runtime = LoadConfig(path, overrides);
  const Json document = detail::LoadDocument(path);
  const std::string application_path = "application";
  const Json &application = ObjectMember(document, "application", "");
  ReadInitialization(
      ObjectMember(application, "initialization", application_path),
      application_path + ".initialization", config.runtime,
      config.initialization);
  ReadSupervision(ObjectMember(application, "supervision", application_path),
                  application_path + ".supervision", config.runtime,
                  config.supervision);
  ReadInput(ObjectMember(document, "input", ""), "input", config.input);
  config.application_config_id = Sha256(CanonicalApplication(config).dump());
  return config;
}

}  // namespace irmv::configuration
