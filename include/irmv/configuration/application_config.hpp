#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "irmv/configuration/config.hpp"
#include "irmv/protocol/types.hpp"

namespace irmv::configuration {

// Position and damping gains of the initialization target. The arrays follow
// the canonical `robot.joints` order, like every other joint array in
// RuntimeConfig.
struct InitializationGains {
  JointArray kp{};
  JointArray kd{};
};

// One initialization move: how long it may take, how close to the target the
// move must be to count as finished, how long that closeness must hold, and a
// final bound on the whole move that must exceed the duration.
struct TrajectoryConfig {
  protocol::TimeUs duration_us{0};
  double position_tolerance{0.0};  // rad
  double velocity_tolerance{0.0};  // rad/s
  protocol::TimeUs stable_window_us{0};
  protocol::TimeUs deadline_us{0};  // final bound; must exceed duration_us
};

struct InitializationConfig {
  JointArray target_q{};  // rad, within each joint's position limits
  InitializationGains gains;
  TrajectoryConfig trajectory;
};

// Periods and deadlines of one application session. They are host-side
// supervision of the frozen runtime protocol, not runtime parameters.
struct SupervisionConfig {
  protocol::TimeUs target_period_us{0};     // JointCommand / Ready-hold period
  protocol::TimeUs heartbeat_period_us{0};  // lease refresh period
  protocol::TimeUs request_retry_us{0};     // identical-payload resend interval
  protocol::TimeUs request_deadline_us{0};  // final deadline of one request
  protocol::TimeUs state_timeout_us{0};     // no fresh RobotState => lost
};

// One byte of terminal input, mapped to the application event it raises.
enum class InputAction {
  None = 0,
  Damping,
  Initialize,
  ZeroTorque,
  EmergencyStop,
  Reset,
  Exit,
  VelocityForward,
  VelocityBackward,
  VelocityLeft,
  VelocityRight,
  VelocityYawLeft,
  VelocityYawRight,
};

struct InputConfig {
  // Byte -> action table. Bytes the file does not bind stay InputAction::None;
  // nothing derives a fallback binding, so an unbound byte is inert.
  std::array<InputAction, 256> actions{};
  protocol::TimeUs intent_timeout_us{0};  // body-velocity intent expiry
  protocol::TimeUs repeat_guard_us{0};    // one-shot key repeat suppression
};

// The whole application configuration: the runtime snapshot plus the
// application and input sections of the same file.
struct ApplicationConfig {
  // The snapshot the runtime loads, parsed by LoadConfig, so
  // runtime_config_id/layout_id are byte-identical to what the runtime
  // publishes. Read-only for the application.
  RuntimeConfig runtime;
  // Sha256 of the canonical application+input sections only. Changing an
  // application parameter changes this digest and never runtime.config_id.
  std::string application_config_id;
  InitializationConfig initialization;
  SupervisionConfig supervision;
  InputConfig input;

  const std::vector<std::string> &joint_names() const {
    return runtime.joint_names;
  }
  std::size_t joint_count() const { return runtime.joint_count(); }
  const std::string &layout_id() const { return runtime.layout_id; }
  const std::string &runtime_config_id() const { return runtime.config_id; }
  const DdsConfig &dds() const { return runtime.dds; }
};

// Parses the runtime section through LoadConfig and validates `application` and
// `input` from the same document. Configuration errors carry the JSON field
// path in ConfigError.
ApplicationConfig LoadApplicationConfig(const std::filesystem::path &path,
                                        const ConfigOverrides &overrides = {});

}  // namespace irmv::configuration
