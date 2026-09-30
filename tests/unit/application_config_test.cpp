#include "irmv/configuration/application_config.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include "irmv/configuration/config.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::ApplicationConfig;
using irmv::configuration::ConfigError;
using irmv::configuration::ConfigOverrides;
using irmv::configuration::InputAction;
using irmv::configuration::LoadApplicationConfig;
using irmv::configuration::LoadConfig;

class TemporaryConfig {
 public:
  explicit TemporaryConfig(std::string_view contents) {
    static unsigned int next_id = 0;
    path_ =
        std::filesystem::temp_directory_path() /
        ("irmv-application-config-test-" + std::to_string(next_id++) + ".json");
    std::ofstream output(path_);
    output << contents;
  }

  ~TemporaryConfig() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

std::string ReplaceOnce(std::string value, std::string_view from,
                        std::string_view to) {
  const auto position = value.find(from);
  if (position == std::string::npos) {
    return {};
  }
  value.replace(position, from.size(), to);
  return value;
}

// The runtime section is the phase-1 fake configuration; only the application
// and input placeholders change between cases, so every negative case differs
// from the valid file in exactly one place.
constexpr std::string_view kConfigTemplate = R"({
  "schema_version": 1,
  "deployment": "fake_only",
  "backend": "fake",
  "robot": {
    "units": {"position": "rad", "velocity": "rad_s", "torque": "Nm"},
    "joints": [
      {"name": "test_left", "position_min": -2, "position_max": 2},
      {"name": "test_right", "position_min": -2, "position_max": 2}
    ]
  },
  "runtime": {
    "control_hz": 500,
    "damping": {"kd": {"test_right": 2, "test_left": 2}},
    "safety": {
      "joint_velocity_limits": {"test_right": 10, "test_left": 10},
      "torque_limits": {"test_left": 20, "test_right": 20},
      "target_velocity_limits": {"test_left": 10, "test_right": 10},
      "timeouts": {"command_us": 100000, "source_state_us": 150000,
        "feedback_us": 20000, "imu_us": 20000, "supervisor_us": 300000}
    },
    "boot": {"frequency_grace_us": 1000000, "healthy_window_us": 100000,
      "deadline_us": 2000000, "minimum_frequency_hz": 400},
    "recovery": {"healthy_window_us": 100000, "minimum_frequency_hz": 400},
    "imu_required": false,
    "continuity": {
      "position_delta": {"test_left": 0.1, "test_right": 0.1},
      "velocity_delta": {"test_left": 0.5, "test_right": 0.5},
      "torque_delta": {"test_left": 5, "test_right": 5}
    },
    "state_window_capacity": 1024
  },
@@APPLICATION@@
@@INPUT@@
  "telemetry": {"level": "INFO", "directory": "logs", "rotate_bytes": 1048576,
    "retained_files": 3, "queue_capacity": 1024},
  "dds": {"domain": 1, "interface": "lo"}
})";

// Both joint maps are deliberately keyed out of canonical order and carry a
// different value per joint, so a reader that expands by key order instead of
// robot.joints order is caught.
constexpr std::string_view kApplicationBody = R"({
    "initialization": {
      "target_position": {"test_right": 0.25, "test_left": -0.5},
      "gains": {
        "test_left": {"kp": 10.0, "kd": 1.0},
        "test_right": {"kp": 12.5, "kd": 1.5}
      },
      "trajectory": {
        "duration_us": 500000,
        "position_tolerance": 0.02,
        "velocity_tolerance": 0.05,
        "stable_window_us": 100000,
        "deadline_us": 2000000
      }
    },
    "supervision": {
      "target_period_us": 20000,
      "heartbeat_period_us": 50000,
      "request_retry_us": 50000,
      "request_deadline_us": 1000000,
      "state_timeout_us": 150000
    },
    "policy": {
      "model_path": null,
      "inference_timeout_us": 100000
    },
    "policy_overrides": [
      {
        "model_sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        "joints": {
          "test_right": {"kp": 30.0},
          "test_left": {"kp": 25.0, "kd": 3.0}
        }
      }
    ]
  })";

constexpr std::string_view kBindings = R"({
    "emergency_stop": " ",
    "damping": "d",
    "initialize": "i",
    "zero_torque": "z",
    "reset": "r",
    "exit": "x",
    "velocity_forward": "w",
    "velocity_backward": "s",
    "velocity_left": "a",
    "velocity_right": "f",
    "velocity_yaw_left": "q",
    "velocity_yaw_right": "e"
  })";

constexpr std::string_view kInputTemplate = R"({
    "intent_timeout_us": 500000,
    "repeat_guard_us": 250000,
    "bindings": @@BINDINGS@@
  })";

std::string ApplicationSection(std::string_view body) {
  return "\"application\": " + std::string(body) + ",";
}

std::string InputBody(std::string_view bindings) {
  return ReplaceOnce(std::string(kInputTemplate), "@@BINDINGS@@", bindings);
}

std::string InputSection(std::string_view body) {
  return "\"input\": " + std::string(body) + ",";
}

std::string ConfigWith(std::string_view application_section,
                       std::string_view input_section) {
  std::string text(kConfigTemplate);
  text = ReplaceOnce(text, "@@APPLICATION@@", application_section);
  text = ReplaceOnce(text, "@@INPUT@@", input_section);
  return text;
}

std::string ValidWithApplication(std::string_view body) {
  return ConfigWith(ApplicationSection(body),
                    InputSection(InputBody(kBindings)));
}

std::string ValidWithInput(std::string_view body) {
  return ConfigWith(ApplicationSection(kApplicationBody), InputSection(body));
}

std::string ValidApplicationAfter(std::string_view from, std::string_view to) {
  return ValidWithApplication(
      ReplaceOnce(std::string(kApplicationBody), from, to));
}

std::string ValidInputAfter(std::string_view from, std::string_view to) {
  return ValidWithInput(ReplaceOnce(InputBody(kBindings), from, to));
}

std::string ValidBindingsAfter(std::string_view from, std::string_view to) {
  return ValidWithInput(
      InputBody(ReplaceOnce(std::string(kBindings), from, to)));
}

bool ThrowsAt(std::string_view contents, std::string_view field_path) {
  const TemporaryConfig file(contents);
  try {
    static_cast<void>(LoadApplicationConfig(file.path()));
  } catch (const ConfigError& error) {
    return error.field_path() == field_path &&
           std::string_view(error.what()).find(field_path) !=
               std::string_view::npos;
  }
  return false;
}

bool ThrowsWithMessage(std::string_view contents, std::string_view field_path,
                       std::string_view needle) {
  const TemporaryConfig file(contents);
  try {
    static_cast<void>(LoadApplicationConfig(file.path()));
  } catch (const ConfigError& error) {
    return error.field_path() == field_path &&
           std::string_view(error.what()).find(needle) !=
               std::string_view::npos;
  }
  return false;
}

// One row per canonical leaf, in the order CanonicalApplication enumerates the
// sections: target position and kp/kd of both joints, the five trajectory
// fields, the five supervision fields, the policy path and deadline, the
// override set, the two input deadlines and one binding. A row that stops
// changing application_config_id would mean the canonical form no longer
// restates that field.
struct DigestMutation {
  const char* from;
  const char* to;
  bool in_input;
};

const DigestMutation kDigestMutations[] = {
    {"\"test_left\": -0.5", "\"test_left\": -0.25", false},
    {"\"test_right\": 0.25", "\"test_right\": 0.5", false},
    {"\"kp\": 10.0", "\"kp\": 11.0", false},
    {"\"kd\": 1.0", "\"kd\": 2.0", false},
    {"\"kp\": 12.5", "\"kp\": 13.0", false},
    {"\"kd\": 1.5", "\"kd\": 2.5", false},
    {"\"duration_us\": 500000", "\"duration_us\": 600000", false},
    {"\"position_tolerance\": 0.02", "\"position_tolerance\": 0.03", false},
    {"\"velocity_tolerance\": 0.05", "\"velocity_tolerance\": 0.06", false},
    {"\"stable_window_us\": 100000", "\"stable_window_us\": 120000", false},
    {"\"deadline_us\": 2000000", "\"deadline_us\": 2500000", false},
    {"\"target_period_us\": 20000", "\"target_period_us\": 25000", false},
    {"\"heartbeat_period_us\": 50000", "\"heartbeat_period_us\": 60000", false},
    {"\"request_retry_us\": 50000", "\"request_retry_us\": 60000", false},
    {"\"request_deadline_us\": 1000000", "\"request_deadline_us\": 1200000",
     false},
    {"\"state_timeout_us\": 150000", "\"state_timeout_us\": 140000", false},
    {"\"inference_timeout_us\": 100000", "\"inference_timeout_us\": 150000",
     false},
    {"\"model_path\": null", "\"model_path\": \"policy.onnx\"", false},
    {"\"kp\": 30.0", "\"kp\": 31.0", false},
    {"\"kd\": 3.0", "\"kd\": 4.0", false},
    {"\"model_sha256\": "
     "\"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\"",
     "\"model_sha256\": "
     "\"fedcba9876543210fedcba9876543210fedcba9876543210fedcba9876543210\"",
     false},
    {"\"intent_timeout_us\": 500000", "\"intent_timeout_us\": 600000", true},
    {"\"repeat_guard_us\": 250000", "\"repeat_guard_us\": 300000", true},
    {"\"velocity_left\": \"a\"", "\"velocity_left\": \"g\"", true},
};

ConfigOverrides OverrideDomain(std::uint32_t domain) {
  ConfigOverrides overrides;
  overrides.dds_domain = domain;
  return overrides;
}

}  // namespace

int main() {
  const TemporaryConfig valid(ConfigWith(ApplicationSection(kApplicationBody),
                                         InputSection(InputBody(kBindings))));
  const ApplicationConfig config = LoadApplicationConfig(valid.path());
  const auto runtime_config = LoadConfig(valid.path());

  // The runtime snapshot is the byte-identical phase-1 result: the application
  // thresholds never enter the runtime digest.
  CHECK(config.runtime_config_id() == runtime_config.config_id);
  CHECK(config.layout_id() == runtime_config.layout_id);
  CHECK(config.runtime.config_id == runtime_config.config_id);
  CHECK(config.dds().domain == 1);
  CHECK(config.dds().interface == "lo");

  // Joint arrays follow the canonical robot.joints order, not the map order of
  // the file.
  CHECK(config.joint_count() == 2);
  CHECK(config.joint_names()[0] == "test_left");
  CHECK(config.joint_names()[1] == "test_right");
  CHECK(config.initialization.target_q[0] == -0.5);
  CHECK(config.initialization.target_q[1] == 0.25);
  CHECK(config.initialization.gains.kp[0] == 10.0);
  CHECK(config.initialization.gains.kp[1] == 12.5);
  CHECK(config.initialization.gains.kd[0] == 1.0);
  CHECK(config.initialization.gains.kd[1] == 1.5);
  CHECK(config.initialization.trajectory.duration_us == 500000);
  CHECK(config.initialization.trajectory.position_tolerance == 0.02);
  CHECK(config.initialization.trajectory.velocity_tolerance == 0.05);
  CHECK(config.initialization.trajectory.stable_window_us == 100000);
  CHECK(config.initialization.trajectory.deadline_us == 2000000);
  CHECK(config.supervision.target_period_us == 20000);
  CHECK(config.supervision.heartbeat_period_us == 50000);
  CHECK(config.supervision.request_retry_us == 50000);
  CHECK(config.supervision.request_deadline_us == 1000000);
  CHECK(config.supervision.state_timeout_us == 150000);
  CHECK(config.input.intent_timeout_us == 500000);
  CHECK(config.input.repeat_guard_us == 250000);
  CHECK(config.input.actions[' '] == InputAction::EmergencyStop);
  CHECK(config.input.actions['d'] == InputAction::Damping);
  CHECK(config.input.actions['i'] == InputAction::Initialize);
  CHECK(config.input.actions['z'] == InputAction::ZeroTorque);
  CHECK(config.input.actions['r'] == InputAction::Reset);
  CHECK(config.input.actions['x'] == InputAction::Exit);
  CHECK(config.input.actions['w'] == InputAction::VelocityForward);
  CHECK(config.input.actions['s'] == InputAction::VelocityBackward);
  CHECK(config.input.actions['a'] == InputAction::VelocityLeft);
  CHECK(config.input.actions['f'] == InputAction::VelocityRight);
  CHECK(config.input.actions['q'] == InputAction::VelocityYawLeft);
  CHECK(config.input.actions['e'] == InputAction::VelocityYawRight);
  CHECK(config.input.actions['j'] == InputAction::None);
  CHECK(config.application_config_id.size() == 64);

  // The shipped file is the one both entry points read.
  const ApplicationConfig fixture = LoadApplicationConfig("config/sdk.json");
  const auto fixture_runtime = LoadConfig("config/sdk.json");
  CHECK(fixture.runtime_config_id() == fixture_runtime.config_id);
  CHECK(fixture.layout_id() == fixture_runtime.layout_id);
  CHECK(fixture.joint_count() == 2);
  CHECK(fixture.initialization.target_q[0] == 0.0);
  CHECK(fixture.initialization.target_q[1] == 0.0);
  CHECK(fixture.initialization.gains.kp[0] == 10.0);
  CHECK(fixture.initialization.gains.kp[1] == 10.0);
  CHECK(fixture.initialization.gains.kd[0] == 1.0);
  CHECK(fixture.initialization.gains.kd[1] == 1.0);
  CHECK(fixture.initialization.trajectory.duration_us == 500000);
  CHECK(fixture.initialization.trajectory.position_tolerance == 0.02);
  CHECK(fixture.initialization.trajectory.velocity_tolerance == 0.05);
  CHECK(fixture.initialization.trajectory.stable_window_us == 100000);
  CHECK(fixture.initialization.trajectory.deadline_us == 2000000);
  CHECK(fixture.supervision.target_period_us == 20000);
  CHECK(fixture.supervision.heartbeat_period_us == 50000);
  CHECK(fixture.supervision.request_retry_us == 50000);
  CHECK(fixture.supervision.request_deadline_us == 1000000);
  CHECK(fixture.supervision.state_timeout_us == 150000);
  CHECK(fixture.input.intent_timeout_us == 500000);
  CHECK(fixture.input.repeat_guard_us == 250000);
  CHECK(fixture.input.actions[' '] == InputAction::EmergencyStop);
  CHECK(fixture.input.actions['e'] == InputAction::VelocityYawRight);
  CHECK(fixture.application_config_id.size() == 64);

  // The application digest is canonical: a file that only reorders the
  // application and input keys describes the same application.
  constexpr std::string_view kReorderedApplicationBody = R"({
    "supervision": {
      "state_timeout_us": 150000,
      "request_deadline_us": 1000000,
      "request_retry_us": 50000,
      "heartbeat_period_us": 50000,
      "target_period_us": 20000
    },
    "initialization": {
      "trajectory": {
        "deadline_us": 2000000,
        "stable_window_us": 100000,
        "velocity_tolerance": 0.05,
        "position_tolerance": 0.02,
        "duration_us": 500000
      },
      "gains": {
        "test_right": {"kd": 1.5, "kp": 12.5},
        "test_left": {"kd": 1.0, "kp": 10.0}
      },
      "target_position": {"test_right": 0.25, "test_left": -0.5}
    },
    "policy_overrides": [
      {
        "joints": {
          "test_left": {"kd": 3.0, "kp": 25.0},
          "test_right": {"kp": 30.0}
        },
        "model_sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
      }
    ],
    "policy": {
      "inference_timeout_us": 100000,
      "model_path": null
    }
  })";
  constexpr std::string_view kReorderedBindings = R"({
    "velocity_yaw_right": "e",
    "velocity_yaw_left": "q",
    "velocity_right": "f",
    "velocity_left": "a",
    "velocity_backward": "s",
    "velocity_forward": "w",
    "exit": "x",
    "reset": "r",
    "zero_torque": "z",
    "initialize": "i",
    "damping": "d",
    "emergency_stop": " "
  })";
  const TemporaryConfig reordered(
      ConfigWith(ApplicationSection(kReorderedApplicationBody),
                 InputSection(InputBody(kReorderedBindings))));
  const ApplicationConfig reordered_config =
      LoadApplicationConfig(reordered.path());
  CHECK(reordered_config.initialization.target_q[0] == -0.5);
  CHECK(reordered_config.initialization.gains.kp[1] == 12.5);
  CHECK(reordered_config.application_config_id == config.application_config_id);
  CHECK(reordered_config.runtime_config_id() == config.runtime_config_id());
  CHECK(reordered_config.layout_id() == config.layout_id());

  // An application parameter changes the application digest and never the
  // runtime identity.
  const TemporaryConfig retuned(ValidApplicationAfter(
      "\"target_period_us\": 20000", "\"target_period_us\": 25000"));
  const ApplicationConfig retuned_config =
      LoadApplicationConfig(retuned.path());
  CHECK(retuned_config.supervision.target_period_us == 25000);
  CHECK(retuned_config.application_config_id != config.application_config_id);
  CHECK(retuned_config.runtime_config_id() == config.runtime_config_id());

  const TemporaryConfig rebound(ValidBindingsAfter("\"velocity_left\": \"a\"",
                                                   "\"velocity_left\": \"g\""));
  const ApplicationConfig rebound_config =
      LoadApplicationConfig(rebound.path());
  CHECK(rebound_config.input.actions['g'] == InputAction::VelocityLeft);
  CHECK(rebound_config.input.actions['a'] == InputAction::None);
  CHECK(rebound_config.application_config_id != config.application_config_id);
  CHECK(rebound_config.runtime_config_id() == config.runtime_config_id());

  // Every application/input leaf reaches the digest: mutating any one of them
  // changes application_config_id while the runtime identity stays put. This
  // pins the fields CanonicalApplication restates by name.
  const std::string base_application(kApplicationBody);
  const std::string base_input = InputBody(kBindings);
  for (const DigestMutation& mutation : kDigestMutations) {
    const std::string& base = mutation.in_input ? base_input : base_application;
    CHECK(base.find(mutation.from) != std::string::npos);
    const TemporaryConfig mutated(
        mutation.in_input ? ValidInputAfter(mutation.from, mutation.to)
                          : ValidApplicationAfter(mutation.from, mutation.to));
    const ApplicationConfig mutated_config =
        LoadApplicationConfig(mutated.path());
    CHECK(mutated_config.application_config_id != config.application_config_id);
    CHECK(mutated_config.runtime_config_id() == config.runtime_config_id());
  }

  // A runtime-only command line override reaches the runtime digest and leaves
  // the application digest alone.
  const ApplicationConfig overridden =
      LoadApplicationConfig(valid.path(), OverrideDomain(7));
  const auto overridden_runtime = LoadConfig(valid.path(), OverrideDomain(7));
  CHECK(overridden.dds().domain == 7);
  CHECK(overridden.runtime_config_id() == overridden_runtime.config_id);
  CHECK(overridden.runtime_config_id() != config.runtime_config_id());
  CHECK(overridden.layout_id() == config.layout_id());
  CHECK(overridden.application_config_id == config.application_config_id);

  const std::string& initialization = "application.initialization";
  const std::string& gains = "application.initialization.gains";
  const std::string& trajectory = "application.initialization.trajectory";
  const std::string& supervision = "application.supervision";
  const std::string& bindings = "input.bindings";

  // Every joint map must cover exactly robot.joints: missing and unknown joints
  // report the exact field path.
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"target_position\": {\"test_right\": 0.25, "
                            "\"test_left\": -0.5}",
                            "\"target_position\": {\"test_left\": -0.5}"),
      initialization + ".target_position.test_right"));
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"target_position\": {\"test_right\": 0.25, "
                            "\"test_left\": -0.5}",
                            "\"target_position\": {\"test_right\": 0.25, "
                            "\"test_left\": -0.5, \"test_extra\": 0.0}"),
      initialization + ".target_position.test_extra"));
  CHECK(ThrowsAt(
      ValidApplicationAfter(
          ",\n        \"test_right\": {\"kp\": 12.5, \"kd\": 1.5}", ""),
      gains + ".test_right"));
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"test_right\": {\"kp\": 12.5, \"kd\": 1.5}",
                            "\"test_right\": {\"kp\": 12.5, \"kd\": 1.5}, "
                            "\"test_extra\": {\"kp\": 1.0, \"kd\": 1.0}"),
      gains + ".test_extra"));
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"target_position\": {\"test_right\": 0.25, "
                            "\"test_left\": -0.5}",
                            "\"target_position\": []"),
      initialization + ".target_position"));

  // Gains are non-negative; an initialization joint additionally needs kp > 0
  // to hold position. That last rule is initialization-only: this contract has
  // no tracking gains, so no generic Tracking assertion is added here.
  CHECK(ThrowsAt(ValidApplicationAfter("\"kp\": 10.0", "\"kp\": -1.0"),
                 gains + ".test_left.kp"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"kd\": 1.0", "\"kd\": -1.0"),
                 gains + ".test_left.kd"));
  // A non-number value exercises the type half of the same check.
  CHECK(ThrowsAt(ValidApplicationAfter("\"kp\": 10.0", "\"kp\": \"10\""),
                 gains + ".test_left.kp"));
  // JSON cannot express a non-finite number, so an overflowing literal is
  // rejected by the document parse at `$`; FiniteNumber's "must be finite"
  // branch is defensive and unreachable from a document.
  CHECK(ThrowsAt(ValidApplicationAfter("\"kp\": 10.0", "\"kp\": 1e400"), "$"));
  CHECK(ThrowsWithMessage(ValidApplicationAfter("\"kp\": 10.0", "\"kp\": 0.0"),
                          gains + ".test_left.kp", "position"));

  // Target positions must stay inside the physical limits of their joint.
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"test_left\": -0.5", "\"test_left\": 2.5"),
      initialization + ".target_position.test_left"));
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"test_left\": -0.5", "\"test_left\": -2.5"),
      initialization + ".target_position.test_left"));

  // The limits themselves are inclusive, kd may be zero (only kp has to hold
  // position), and an optional velocity action may stay unbound: its byte is
  // inert and the digest records that the binding is absent.
  std::string at_limits_body =
      ReplaceOnce(std::string(kApplicationBody), "\"test_left\": -0.5",
                  "\"test_left\": -2");
  at_limits_body =
      ReplaceOnce(at_limits_body, "\"test_right\": 0.25", "\"test_right\": 2");
  const TemporaryConfig at_limits(ValidWithApplication(at_limits_body));
  const ApplicationConfig at_limits_config =
      LoadApplicationConfig(at_limits.path());
  CHECK(at_limits_config.initialization.target_q[0] == -2.0);
  CHECK(at_limits_config.initialization.target_q[1] == 2.0);

  const TemporaryConfig zero_kd(
      ValidApplicationAfter("\"kd\": 1.0", "\"kd\": 0.0"));
  const ApplicationConfig zero_kd_config =
      LoadApplicationConfig(zero_kd.path());
  CHECK(zero_kd_config.initialization.gains.kd[0] == 0.0);

  const TemporaryConfig unbound_velocity(ValidWithInput(InputBody(ReplaceOnce(
      std::string(kBindings), ",\n    \"velocity_yaw_right\": \"e\"", ""))));
  const ApplicationConfig unbound_velocity_config =
      LoadApplicationConfig(unbound_velocity.path());
  CHECK(unbound_velocity_config.input.actions['e'] == InputAction::None);
  CHECK(unbound_velocity_config.input.actions['q'] ==
        InputAction::VelocityYawLeft);
  CHECK(unbound_velocity_config.application_config_id !=
        config.application_config_id);

  // Trajectory: positive duration, positive tolerances, positive stable window
  // and a deadline beyond the duration.
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"duration_us\": 500000", "\"duration_us\": 0"),
      trajectory + ".duration_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"position_tolerance\": 0.02",
                                       "\"position_tolerance\": 0"),
                 trajectory + ".position_tolerance"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"velocity_tolerance\": 0.05",
                                       "\"velocity_tolerance\": 0"),
                 trajectory + ".velocity_tolerance"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"stable_window_us\": 100000",
                                       "\"stable_window_us\": 0"),
                 trajectory + ".stable_window_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"deadline_us\": 2000000",
                                       "\"deadline_us\": 500000"),
                 trajectory + ".deadline_us"));
  // The deadline must also leave room for the whole stable window after the
  // ramp, or the move would load already unable to complete. Here
  // duration_us = 500000 and stable_window_us = 100000, so one microsecond
  // short of their sum (599999) is refused with the deadline's own field path
  // and a message naming both the window and the duration, and the exact sum
  // loads.
  CHECK(ThrowsWithMessage(ValidApplicationAfter("\"deadline_us\": 2000000",
                                                "\"deadline_us\": 599999"),
                          trajectory + ".deadline_us", "stable_window_us"));
  CHECK(ThrowsWithMessage(ValidApplicationAfter("\"deadline_us\": 2000000",
                                                "\"deadline_us\": 599999"),
                          trajectory + ".deadline_us", "duration_us"));
  const TemporaryConfig deadline_at_the_window_end(ValidApplicationAfter(
      "\"deadline_us\": 2000000", "\"deadline_us\": 600000"));
  const ApplicationConfig deadline_at_the_window_end_config =
      LoadApplicationConfig(deadline_at_the_window_end.path());
  CHECK(
      deadline_at_the_window_end_config.initialization.trajectory.deadline_us ==
      600000);

  // Supervision periods and deadlines must be positive and ordered against the
  // runtime timeouts they supervise.
  CHECK(ThrowsAt(ValidApplicationAfter("\"target_period_us\": 20000",
                                       "\"target_period_us\": 0"),
                 supervision + ".target_period_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"target_period_us\": 20000",
                                       "\"target_period_us\": 100000"),
                 supervision + ".target_period_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"heartbeat_period_us\": 50000",
                                       "\"heartbeat_period_us\": 300000"),
                 supervision + ".heartbeat_period_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"request_deadline_us\": 1000000",
                                       "\"request_deadline_us\": 50000"),
                 supervision + ".request_deadline_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"state_timeout_us\": 150000",
                                       "\"state_timeout_us\": 0"),
                 supervision + ".state_timeout_us"));
  // The supervisor's "fresh" (host receive age <= state_timeout_us) and the
  // initializer's usability rule (runtime publication age <= source_state_us)
  // are two independent thresholds, so the window the upper layer trusts must
  // stay inside the window its own generator can use. One microsecond above the
  // priority's source-state deadline is refused; equality is the boundary and
  // is accepted (the template's own values are exactly equal, below).
  CHECK(ThrowsWithMessage(ValidApplicationAfter("\"state_timeout_us\": 150000",
                                                "\"state_timeout_us\": 150001"),
                          supervision + ".state_timeout_us",
                          "source_state_us"));
  const TemporaryConfig boundary(ValidApplicationAfter(
      "\"state_timeout_us\": 150000", "\"state_timeout_us\": 150000"));
  const ApplicationConfig boundary_config =
      LoadApplicationConfig(boundary.path());
  CHECK(boundary_config.supervision.state_timeout_us ==
        boundary_config.runtime.timeouts.source_state_us);
  CHECK(boundary_config.supervision.state_timeout_us == 150000);

  // Input: the space/emergency-stop reservation, one byte per action, and the
  // required mode actions.
  CHECK(ThrowsAt(ValidBindingsAfter("\"damping\": \"d\"", "\"damping\": \" \""),
                 bindings + ".damping"));
  CHECK(ThrowsAt(ValidBindingsAfter("\"velocity_forward\": \"w\"",
                                    "\"velocity_forward\": \" \""),
                 bindings + ".velocity_forward"));
  CHECK(ThrowsAt(ValidBindingsAfter("\"emergency_stop\": \" \"",
                                    "\"emergency_stop\": \"e\""),
                 bindings + ".emergency_stop"));
  CHECK(ThrowsAt(ValidBindingsAfter("\"velocity_left\": \"a\"",
                                    "\"velocity_left\": \"w\""),
                 bindings + ".velocity_left"));
  CHECK(ThrowsAt(ValidBindingsAfter("    \"exit\": \"x\",\n", ""),
                 bindings + ".exit"));
  CHECK(
      ThrowsAt(ValidBindingsAfter("\"damping\": \"d\"", "\"damping\": \"dd\""),
               bindings + ".damping"));
  CHECK(
      ThrowsAt(ValidBindingsAfter("\"damping\": \"d\"", "\"damping\": \"界\""),
               bindings + ".damping"));
  CHECK(ThrowsAt(ValidBindingsAfter("\"damping\": \"d\"", "\"damping\": 3"),
                 bindings + ".damping"));
  CHECK(ThrowsAt(ValidBindingsAfter("\"velocity_yaw_right\": \"e\"",
                                    "\"velocity_yaw_right\": \"e\", "
                                    "\"jump\": \"j\""),
                 bindings + ".jump"));
  CHECK(ThrowsAt(ValidWithInput(InputBody("[]")), bindings));
  CHECK(ThrowsAt(ValidInputAfter("\"intent_timeout_us\": 500000",
                                 "\"intent_timeout_us\": 0"),
                 "input.intent_timeout_us"));
  CHECK(ThrowsAt(
      ValidInputAfter("\"repeat_guard_us\": 250000", "\"repeat_guard_us\": 0"),
      "input.repeat_guard_us"));
  CHECK(ThrowsAt(ValidInputAfter("\"bindings\"", "\"bindings_removed\""),
                 bindings));

  // Policy: the path may be absent, but what is written down must be usable;
  // an override must name a configured joint, change at least one gain, and
  // belong to exactly one model.
  const std::string policy = "application.policy";
  const std::string overrides = "application.policy_overrides";
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"model_path\": null", "\"model_path\": \"\""),
      policy + ".model_path"));
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"model_path\": null", "\"model_path\": 3"),
      policy + ".model_path"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"inference_timeout_us\": 100000",
                                       "\"inference_timeout_us\": 0"),
                 policy + ".inference_timeout_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"inference_timeout_us\": 100000",
                                       "\"inference_timeout_us\": 100000.5"),
                 policy + ".inference_timeout_us"));
  CHECK(ThrowsAt(ValidApplicationAfter("    \"policy\": {\n",
                                       "    \"policy_removed\": {\n"),
                 policy));
  CHECK(ThrowsAt(ValidApplicationAfter(
                     "\"model_sha256\": \"0123456789abcdef0123456789abcdef"
                     "0123456789abcdef0123456789abcdef\"",
                     "\"model_sha256\": \"0123456789ABCDEF0123456789abcdef"
                     "0123456789abcdef0123456789abcdef\""),
                 overrides + "[0].model_sha256"));
  CHECK(ThrowsAt(ValidApplicationAfter(
                     "\"model_sha256\": \"0123456789abcdef0123456789abcdef"
                     "0123456789abcdef0123456789abcdef\"",
                     "\"model_sha256\": \"0123456789abcdef\""),
                 overrides + "[0].model_sha256"));
  CHECK(
      ThrowsAt(ValidApplicationAfter(
                   "\"joints\": {\n",
                   "\"joints\": {\n          \"test_knob\": {\"kp\": 1.0},\n"),
               overrides + "[0].joints.test_knob"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"joints\": {\n",
                                       "\"joints\": {},\n"
                                       "        \"ignored\": {\n"),
                 overrides + "[0].joints"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"test_right\": {\"kp\": 30.0}",
                                       "\"test_right\": {}"),
                 overrides + "[0].joints.test_right"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"test_right\": {\"kp\": 30.0}",
                                       "\"test_right\": {\"kp\": -1.0}"),
                 overrides + "[0].joints.test_right.kp"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"test_right\": {\"kp\": 30.0}",
                                       "\"test_right\": {\"ki\": 30.0}"),
                 overrides + "[0].joints.test_right.ki"));

  {
    // The parsed overrides keep the file's own per-joint differences and the
    // record of which gain each entry replaces.
    const TemporaryConfig policed(
        ConfigWith(ApplicationSection(kApplicationBody),
                   InputSection(InputBody(kBindings))));
    const ApplicationConfig policed_config =
        LoadApplicationConfig(policed.path());
    CHECK(policed_config.policy.model_path.empty());
    CHECK(policed_config.policy.inference_timeout_us == 100000);
    CHECK(policed_config.policy.overrides.size() == 1);
    const auto& entry = policed_config.policy.overrides.front();
    CHECK(entry.model_sha256 ==
          "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
    CHECK(entry.joints.size() == 2);
    // The reader keeps the entries in the order the JSON object iterates them;
    // the digest is what makes the order canonical, so the assertions look the
    // joints up by name.
    const auto right = std::find_if(
        entry.joints.begin(), entry.joints.end(),
        [](const auto& candidate) { return candidate.joint == "test_right"; });
    const auto left = std::find_if(
        entry.joints.begin(), entry.joints.end(),
        [](const auto& candidate) { return candidate.joint == "test_left"; });
    CHECK(right != entry.joints.end() && left != entry.joints.end());
    CHECK(right->overrides_kp && !right->overrides_kd);
    CHECK(right->kp == 30.0);
    CHECK(left->overrides_kp && left->overrides_kd);
    CHECK(left->kp == 25.0 && left->kd == 3.0);
  }

  {
    const TemporaryConfig model(ValidApplicationAfter(
        "\"model_path\": null", "\"model_path\": \"policy.onnx\""));
    const ApplicationConfig model_config = LoadApplicationConfig(model.path());
    CHECK(model_config.policy.model_path ==
          std::filesystem::path("policy.onnx"));
  }

  {
    // A second entry for the same digest is dropped nowhere: it would never
    // apply, so it is refused instead.
    const std::string digest =
        "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
    const std::string duplicate =
        "\"policy_overrides\": [{\"model_sha256\": \"" + digest +
        "\", \"joints\": {\"test_left\": {\"kp\": 1.0}}},";
    CHECK(ThrowsAt(ValidApplicationAfter("\"policy_overrides\": [", duplicate),
                   overrides + "[1].model_sha256"));
  }

  // A section present but malformed, or missing entirely, is reported at the
  // section path.
  CHECK(ThrowsAt(
      ConfigWith("\"application\": 5,", InputSection(InputBody(kBindings))),
      "application"));
  CHECK(ThrowsAt(ConfigWith("", InputSection(InputBody(kBindings))),
                 "application"));
  CHECK(ThrowsAt(ValidWithInput("[]"), "input"));
  CHECK(
      ThrowsAt(ConfigWith(ApplicationSection(kApplicationBody), ""), "input"));
  CHECK(ThrowsAt(ValidApplicationAfter("\"initialization\": {",
                                       "\"initialization_removed\": {"),
                 initialization));
  CHECK(ThrowsAt(
      ValidApplicationAfter("\"trajectory\": {", "\"trajectory_removed\": {"),
      trajectory));

  return 0;
}
