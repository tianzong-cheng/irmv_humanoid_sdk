#include "irmv/telemetry/record_context.hpp"

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <utility>

namespace irmv::telemetry {
namespace {

constexpr std::uint64_t kMicrosPerSecond = 1000000;
constexpr std::string_view kWhitespace = " \t\r\n";

bool IsBootIdCharacter(char character) {
  const bool digit = character >= '0' && character <= '9';
  const bool lower = character >= 'a' && character <= 'f';
  const bool upper = character >= 'A' && character <= 'F';
  return digit || lower || upper || character == '-';
}

// The kernel writes one UUID line. An empty, unreadable or malformed file is
// reported as a failure rather than turned into an identity that could be
// mistaken for a real boot.
bool ReadBootId(const std::string& path, std::string* boot_id,
                std::string* reason) {
  std::ifstream file(path);
  if (!file.is_open()) {
    *reason = "cannot open " + path;
    return false;
  }
  std::string text;
  std::getline(file, text);
  if (file.bad()) {
    *reason = "cannot read " + path;
    return false;
  }
  const std::size_t begin = text.find_first_not_of(kWhitespace);
  if (begin == std::string::npos) {
    *reason = path + " is empty";
    return false;
  }
  const std::size_t end = text.find_last_not_of(kWhitespace);
  std::string value = text.substr(begin, end - begin + 1);
  if (value.size() > 64) {
    *reason = path + " is longer than 64 bytes";
    return false;
  }
  for (const char character : value) {
    if (!IsBootIdCharacter(character)) {
      *reason = path + " is not a boot id";
      return false;
    }
  }
  *boot_id = std::move(value);
  return true;
}

}  // namespace

std::string SimulationRecordMetadata(const configuration::RuntimeConfig& c) {
  if (!c.simulation) return "{}";
  const auto& s = *c.simulation;
  // The run directory is one child of recorder.directory. Keep the entire
  // experiment tree together when moving it; no source/install path is baked
  // in.
  const auto relative =
      std::filesystem::path("..") /
      std::filesystem::relative(s.socket_directory / "simulation.jsonl",
                                c.telemetry.recorder.directory);
  return nlohmann::json{
      {"sidecar", relative.generic_string()},
      {"sidecar_schema", "irmv.simulation_evidence"},
      {"sidecar_version", 1},
      {"engine", s.engine},
      {"engine_sha256", s.engine_sha256},
      {"asset_sha256", s.asset_sha256},
      {"config_sha256", c.config_id},
      {"random_seed", "not configured"},
      {"logical_origin_us", 1},
      {"physics_dt_us", s.physics_dt_us},
      {"control_dt_us", c.control_period_us},
      {"host_clock", "linux_clock_monotonic"},
      {"host_origin",
       "kernel boot; elapsed host durations are not control latency"},
      {"logical_origin",
       "1us at control step0; stop phase4 can advance state_seq at frozen "
       "time"},
      {"versions",
       "actual engine factory and dependency versions in sidecar header"}}
      .dump();
}

std::string EncodeRecordClock(const RecordClockIdentity& c) {
  return nlohmann::json{{"clock", c.clock},
                        {"boot_id", c.boot_id},
                        {"available", c.available},
                        {"clock_domain", c.clock_domain},
                        {"clock_run_id", c.clock_run_id},
                        {"logical_origin_us", c.logical_origin_us},
                        {"physics_dt_us", c.physics_dt_us},
                        {"control_dt_us", c.control_dt_us}}
      .dump();
}
bool DecodeRecordClock(std::string_view text, RecordClockIdentity* c) {
  if (!c) return false;
  *c = {};
  try {
    const auto j = nlohmann::json::parse(text);
    c->clock = j.value("clock", std::string());
    c->boot_id = j.value("boot_id", std::string());
    c->available = j.value("available", false);
    c->clock_domain = j.value("clock_domain", std::string("host_monotonic"));
    if (c->clock_domain == "host_monotonic") return c->clock == kRecordClock;
    if (c->clock_domain != "simulation" || c->clock != "simulation")
      return false;
    c->clock_run_id = j.at("clock_run_id").get<std::string>();
    if (c->clock_run_id.size() != 32 ||
        c->clock_run_id.find_first_not_of("0123456789abcdef") !=
            std::string::npos ||
        c->clock_run_id.find_first_not_of('0') == std::string::npos)
      return false;
    for (const char* key :
         {"logical_origin_us", "physics_dt_us", "control_dt_us"})
      if (!j.at(key).is_number_unsigned()) return false;
    c->logical_origin_us = j.at("logical_origin_us");
    c->physics_dt_us = j.at("physics_dt_us");
    c->control_dt_us = j.at("control_dt_us");
    return c->available && c->logical_origin_us == 1 && c->physics_dt_us > 0 &&
           c->control_dt_us >= c->physics_dt_us &&
           c->control_dt_us % c->physics_dt_us == 0;
  } catch (...) {
    return false;
  }
}
bool SameRecordClock(const RecordClockIdentity& a,
                     const RecordClockIdentity& b) {
  if (!a.available || !b.available || a.clock_domain != b.clock_domain ||
      a.clock != b.clock)
    return false;
  if (a.clock_domain == "simulation")
    return !a.clock_run_id.empty() && a.clock_run_id == b.clock_run_id &&
           a.logical_origin_us == b.logical_origin_us &&
           a.physics_dt_us == b.physics_dt_us &&
           a.control_dt_us == b.control_dt_us;
  return a.clock_domain == "host_monotonic" && a.clock == kRecordClock &&
         !a.boot_id.empty() && a.boot_id == b.boot_id;
}
RecordClockIdentity SimulationRecordClock(const protocol::Id& run,
                                          protocol::TimeUs physics,
                                          protocol::TimeUs control) {
  RecordClockIdentity c;
  c.clock = "simulation";
  c.clock_domain = "simulation";
  c.available = true;
  constexpr char hex[] = "0123456789abcdef";
  for (auto b : run) {
    c.clock_run_id += hex[b >> 4];
    c.clock_run_id += hex[b & 15];
  }
  c.logical_origin_us = 1;
  c.physics_dt_us = physics;
  c.control_dt_us = control;
  return c;
}

bool LoadRecordClockIdentityFromFile(std::string_view boot_id_path,
                                     RecordClockIdentity* identity,
                                     std::string* error) {
  if (identity == nullptr) {
    if (error != nullptr) {
      *error = "clock identity output is null";
    }
    return false;
  }
  *identity = {};
  identity->clock = std::string(kRecordClock);
  identity->boot_id.clear();
  identity->available = false;

  std::string boot_id;
  std::string reason;
  if (!ReadBootId(std::string(boot_id_path), &boot_id, &reason)) {
    if (error != nullptr) {
      *error = "clock identity unavailable: " + reason +
               "; automatic cross-process merging of this run is disabled";
    }
    return false;
  }
  identity->boot_id = std::move(boot_id);
  identity->available = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool LoadRecordClockIdentity(RecordClockIdentity* identity,
                             std::string* error) {
  return LoadRecordClockIdentityFromFile(kBootIdPath, identity, error);
}

SampleSchedule::SampleSchedule(std::uint32_t sample_hz) noexcept {
  // Rounded to the nearest microsecond, so a rate that does not divide one
  // second keeps its phase over a run instead of drifting every sample.
  const std::uint64_t rate = sample_hz;
  if (rate == 0 || rate > kMicrosPerSecond) {
    return;
  }
  period_us_ = (kMicrosPerSecond + rate / 2) / rate;
}

bool SampleSchedule::Due(protocol::TimeUs now_us) noexcept {
  if (period_us_ == 0) {
    return false;
  }
  if (!started_) {
    started_ = true;
    next_due_us_ = now_us;
  }
  if (now_us < next_due_us_) {
    return false;
  }

  // Advance the phase grid past now in one arithmetic step. The number of
  // missed deadlines is not replayed, so a stall yields exactly the current
  // sample and the next deadline stays on the established grid.
  const protocol::TimeUs elapsed = now_us - next_due_us_;
  const protocol::TimeUs steps = elapsed / period_us_ + 1;
  constexpr protocol::TimeUs kMax =
      std::numeric_limits<protocol::TimeUs>::max();
  if (steps > (kMax - next_due_us_) / period_us_) {
    next_due_us_ = kMax;
  } else {
    next_due_us_ += steps * period_us_;
  }
  return true;
}

}  // namespace irmv::telemetry
