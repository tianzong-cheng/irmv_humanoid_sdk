#include "irmv/simulation/bootstrap.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <stdexcept>
namespace irmv::simulation {
namespace {
struct Fd {
  int value{-1};
  ~Fd() {
    if (value >= 0) close(value);
  }
};
template <std::size_t N>
std::array<std::uint8_t, N> Hex(const std::string& s) {
  if (s.size() != 2 * N || s.find_first_not_of("0123456789abcdef") != s.npos)
    throw std::runtime_error("bootstrap hexadecimal identity");
  std::array<std::uint8_t, N> out{};
  bool nonzero = false;
  const std::string digits = "0123456789abcdef";
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = 16 * digits.find(s[2 * i]) + digits.find(s[2 * i + 1]);
    nonzero |= out[i] != 0;
  }
  if (!nonzero) throw std::runtime_error("bootstrap zero identity");
  return out;
}
}  // namespace
EndpointConfig LoadBootstrap(const configuration::RuntimeConfig& c, Role role,
                             const std::string& app) {
  if (!c.simulation || c.backend != "sim")
    throw std::runtime_error("simulation bootstrap without sim config");
  const auto& s = *c.simulation;
  Fd dir;
  dir.value = open("/", O_RDONLY | O_CLOEXEC | O_DIRECTORY);
  for (const auto& part : s.socket_directory.relative_path()) {
    if (part == "..") throw std::runtime_error("bootstrap parent traversal");
    int next = openat(dir.value, part.c_str(),
                      O_RDONLY | O_CLOEXEC | O_DIRECTORY | O_NOFOLLOW);
    close(dir.value);
    dir.value = next;
    if (next < 0) throw std::runtime_error("unsafe bootstrap directory");
  }
  struct stat st {};
  if (fstat(dir.value, &st) || st.st_uid != geteuid() ||
      (st.st_mode & 0777) != 0700)
    throw std::runtime_error("bootstrap requires private 0700 directory");
  Fd fd;
  fd.value = openat(dir.value, "bootstrap.json",
                    O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
  if (fd.value < 0 || fstat(fd.value, &st) || !S_ISREG(st.st_mode) ||
      st.st_uid != geteuid() || (st.st_mode & 0022) || st.st_size <= 0 ||
      st.st_size > 65536)
    throw std::runtime_error("unsafe or oversized bootstrap.json");
  std::string text(static_cast<std::size_t>(st.st_size), '\0');
  std::size_t got = 0;
  while (got < text.size()) {
    auto n = read(fd.value, text.data() + got, text.size() - got);
    if (n <= 0) throw std::runtime_error("bootstrap read");
    got += static_cast<std::size_t>(n);
  }
  char extra;
  if (read(fd.value, &extra, 1) != 0)
    throw std::runtime_error("bootstrap changed");
  const auto j = nlohmann::json::parse(text);
  EndpointConfig out;
  out.socket_path = s.socket_directory / "bridge.sock";
  out.run_id = Hex<16>(j.at("run_id").get<std::string>());
  auto& h = out.local_hello;
  h.role = role;
  h.runtime_id = Hex<16>(j.at("runtime_id").get<std::string>());
  if (h.runtime_id == out.run_id)
    throw std::runtime_error("bootstrap IDs must be independent");
  h.config_digest = Hex<32>(j.at("config_digest").get<std::string>());
  h.application_digest = Hex<32>(j.at("application_digest").get<std::string>());
  h.engine_digest = Hex<32>(j.at("engine_digest").get<std::string>());
  auto integer = [&](const char* key) {
    if (!j.at(key).is_number_unsigned() && !j.at(key).is_number_integer())
      throw std::runtime_error("bootstrap timestep type");
    if (j.at(key).get<std::int64_t>() <= 0)
      throw std::runtime_error("bootstrap timestep");
    return j.at(key).get<std::uint64_t>();
  };
  h.physics_dt_us = integer("physics_dt_us");
  h.control_dt_us = integer("control_dt_us");
  if (h.config_digest != Hex<32>(c.config_id) ||
      h.engine_digest != Hex<32>(s.engine_sha256) ||
      h.physics_dt_us != s.physics_dt_us ||
      h.control_dt_us != c.control_period_us ||
      (role == Role::Application && h.application_digest != Hex<32>(app)))
    throw std::runtime_error(
        "bootstrap does not match normalized SDK configuration");
  out.peer_hello = h;
  out.peer_hello.role = Role::Coordinator;
  out.startup_timeout_ms = (s.host_startup_timeout_us + 999) / 1000;
  out.io_timeout_ms = (s.host_phase_timeout_us + 999) / 1000;
  return out;
}
}  // namespace irmv::simulation
