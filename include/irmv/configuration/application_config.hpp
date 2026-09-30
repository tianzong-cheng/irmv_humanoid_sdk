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
  // No fresh RobotState within this age (the *host* receive age of the
  // snapshot) means the session is lost.
  //
  // LoadApplicationConfig refuses a value above
  // runtime.safety.timeouts.source_state_us. The two ages are measured against
  // different clocks - the client's freshness against this host's receive
  // stamp, the initializer's usability rule against the runtime's own
  // publication stamp - so a larger value here would let the upper layer call a
  // snapshot fresh that its target generator must refuse, latching
  // InitializationFailed instead of skipping that frame.
  protocol::TimeUs state_timeout_us{0};
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

// One experimental gain override of one model joint. A field the file does not
// mention leaves the model's own value in place, and the loader records which
// of the two it changed; both values are finite and non-negative, like every
// gain this deployment may send. The override never relaxes a runtime boundary:
// robot_runtime validates and clamps what it receives regardless of this file.
struct PolicyJointOverride {
  std::string joint;
  bool overrides_kp{false};
  double kp{0.0};
  bool overrides_kd{false};
  double kd{0.0};
};

// The overrides that belong to exactly one model, keyed by the complete
// SHA-256 of the model file. Complete-digest matching is deliberate: a digest
// prefix or a file name would let a re-export inherit an experiment's gains
// without anybody noticing.
struct PolicyOverrides {
  std::string model_sha256;  // lower-case hexadecimal, 64 characters
  std::vector<PolicyJointOverride> joints;
};

// Where the local locomotion policy comes from and how long one policy step
// may take. The gains themselves are not configured here: they are part of the
// model's own deployment contract, so the model stays the single source of the
// trained values and this file only points at the model and records the
// differences that were agreed for it.
struct PolicyConfig {
  // Empty when this deployment ships no local policy, which is what the
  // shipped Fake configuration does. Selecting the local source without a path
  // is refused at startup rather than falling back to another model.
  std::filesystem::path model_path;
  // Deadline of one policy step: a submitted observation that produced no
  // result within this age is a fault of the local source. It must exceed the
  // model's own policy_period_us, which is checked when the model is loaded,
  // because the period is a property of the model and not of this file.
  protocol::TimeUs inference_timeout_us{0};
  std::vector<PolicyOverrides> overrides;
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
  PolicyConfig policy;

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
