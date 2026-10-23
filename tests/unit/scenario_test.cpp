// The fake-scenario parser and its IMU override.
//
// A scenario is the only way a test injects feedback into the fake runtime, so
// what the parser accepts is part of the fixture contract. These cases pin the
// optional IMU vectors: exactly three finite numbers each, applied to the
// injected snapshot, with every existing field still optional.
//
// The parser lives in the runtime executable's own translation unit, so this
// test compiles that file directly instead of adding a library boundary the
// production binary does not need.

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/fake_backend.hpp"
#include "irmv/runtime/runtime_loop.hpp"
#include "support/check.hpp"

namespace {

using irmv::configuration::RuntimeConfig;
using irmv::hardware::FakeBackend;
using irmv::runtime::FakeScenario;
using irmv::runtime::LoadFakeScenario;
using irmv::runtime::ScenarioApplier;
using irmv::runtime::ScenarioError;

std::filesystem::path g_directory;

FakeScenario LoadText(const std::string &name, const std::string &text,
                      const RuntimeConfig &config) {
  const std::filesystem::path path = g_directory / name;
  std::ofstream file(path);
  file << text;
  file.close();
  return LoadFakeScenario(path, config);
}

bool RejectedAt(const std::string &name, const std::string &text,
                const RuntimeConfig &config, const std::string &field) {
  try {
    static_cast<void>(LoadText(name, text, config));
  } catch (const ScenarioError &error) {
    return error.field_path().find(field) != std::string::npos;
  }
  return false;
}

bool Near(double left, double right) {
  return std::fabs(left - right) <= 1e-12;
}

}  // namespace

int main(int argc, char **argv) {
  const std::filesystem::path source_root =
      argc > 1 ? std::filesystem::path(argv[1]) : std::filesystem::path(".");
  g_directory = std::filesystem::temp_directory_path() /
                ("irmv-scenario-test-" + std::to_string(::getpid()));
  std::error_code code;
  std::filesystem::remove_all(g_directory, code);
  std::filesystem::create_directories(g_directory);

  RuntimeConfig config;
  try {
    config =
        irmv::configuration::LoadConfig(source_root / "config" / "sdk.json");
  } catch (const std::exception &error) {
    std::fprintf(stderr, "scenario_test: cannot load config: %s\n",
                 error.what());
    return 1;
  }

  // The vectors are parsed and applied to the injected snapshot. rx_stamp_us 0
  // is the "fresh" offset, so the applied snapshot carries the current time.
  const std::string valid =
      R"({"entries":[{"elapsed_us":0,"feedback":{"count":2,"imu":{)"
      R"("valid":true,"rx_stamp_us":0,)"
      R"("orientation_rpy":[0.1,-0.2,0.3],)"
      R"("angular_velocity":[0.4,-0.5,0.6],)"
      R"("linear_acceleration":[0.7,-0.8,9.81]}}}]})";
  FakeScenario scenario = LoadText("valid.json", valid, config);
  CHECK(scenario.entries.size() == 1);
  if (scenario.entries.size() == 1 && scenario.entries[0].feedback &&
      scenario.entries[0].feedback->imu) {
    const auto &imu = *scenario.entries[0].feedback->imu;
    CHECK(imu.orientation_rpy.has_value());
    CHECK(imu.angular_velocity.has_value());
    CHECK(imu.linear_acceleration.has_value());
  }

  FakeBackend backend(config);
  backend.Initialize();
  ScenarioApplier applier(std::move(scenario), config);
  applier.ApplyDue(0, 123456, backend);
  const auto injected = backend.Read(123456);
  CHECK(injected.imu.valid);
  CHECK(injected.imu.rx_stamp_us == 123456);
  CHECK(Near(injected.imu.orientation_rpy[0], 0.1));
  CHECK(Near(injected.imu.orientation_rpy[1], -0.2));
  CHECK(Near(injected.imu.orientation_rpy[2], 0.3));
  CHECK(Near(injected.imu.angular_velocity[0], 0.4));
  CHECK(Near(injected.imu.angular_velocity[1], -0.5));
  CHECK(Near(injected.imu.angular_velocity[2], 0.6));
  CHECK(Near(injected.imu.linear_acceleration[0], 0.7));
  CHECK(Near(injected.imu.linear_acceleration[1], -0.8));
  CHECK(Near(injected.imu.linear_acceleration[2], 9.81));

  // Backward compatibility: every existing field, including imu.valid and
  // rx_stamp_us, is enough on its own; an omitted vector keeps the previous
  // (initially zero) value.
  const std::string legacy = R"({"entries":[{"elapsed_us":0,"feedback":{"imu":)"
                             R"({"valid":true,"rx_stamp_us":0}}}]})";
  FakeScenario legacy_scenario = LoadText("legacy.json", legacy, config);
  FakeBackend legacy_backend(config);
  legacy_backend.Initialize();
  ScenarioApplier legacy_applier(std::move(legacy_scenario), config);
  legacy_applier.ApplyDue(0, 1000, legacy_backend);
  const auto legacy_snapshot = legacy_backend.Read(1000);
  CHECK(legacy_snapshot.imu.valid);
  CHECK(Near(legacy_snapshot.imu.orientation_rpy[0], 0.0));
  CHECK(Near(legacy_snapshot.imu.angular_velocity[1], 0.0));
  CHECK(Near(legacy_snapshot.imu.linear_acceleration[2], 0.0));

  // Wrong shapes are refused at load time, naming the offending vector. JSON
  // itself cannot represent NaN or infinity, so the only file-level way to
  // offer one is an overflowing literal such as 1e999; this library rejects
  // that at parse time and the parser reports the document as `$` (invalid
  // JSON), not as the vector field. The finite-value guard in NumberValue
  // remains for any non-finite value constructed in memory, but it is not
  // reachable from a JSON document and this test does not claim otherwise.
  CHECK(RejectedAt("short.json",
                   R"({"entries":[{"elapsed_us":0,"feedback":{"imu":)"
                   R"({"orientation_rpy":[0.1,0.2]}}}]})",
                   config, "orientation_rpy"));
  CHECK(RejectedAt("long.json",
                   R"({"entries":[{"elapsed_us":0,"feedback":{"imu":)"
                   R"({"angular_velocity":[0.1,0.2,0.3,0.4]}}}]})",
                   config, "angular_velocity"));
  CHECK(RejectedAt("scalar.json",
                   R"({"entries":[{"elapsed_us":0,"feedback":{"imu":)"
                   R"({"linear_acceleration":9.81}}}]})",
                   config, "linear_acceleration"));
  CHECK(RejectedAt("nonfinite.json",
                   R"({"entries":[{"elapsed_us":0,"feedback":{"imu":)"
                   R"({"orientation_rpy":[0.1,0.2,1e999]}}}]})",
                   config, "$"));

  std::filesystem::remove_all(g_directory, code);
  std::puts("scenario_test: IMU vectors parsed, applied and validated");
  return 0;
}
