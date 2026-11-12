#include <chrono>
#include <iostream>
#include <thread>

#include "irmv/simulation/endpoint.hpp"
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  irmv::simulation::EndpointConfig c;
  c.socket_path = argv[1];
  c.run_id[0] = 1;
  c.local_hello.role = irmv::simulation::Role::Runtime;
  c.peer_hello.role = irmv::simulation::Role::Coordinator;
  c.startup_timeout_ms = 150;
  c.io_timeout_ms = 100;
  irmv::simulation::Endpoint e;
  try {
    e.Start(c);
  } catch (...) {
    std::cout << "rejected\n";
    return 0;
  }
  if (std::string(argv[2]) == "stop") {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    e.Stop();
    std::cout << "stopped\n";
    return 0;
  }
  auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
  bool ready = false;
  bool exchanged = false;
  while (std::chrono::steady_clock::now() < end && !e.Failed()) {
    ready = ready || e.Ready();
    if (std::string(argv[2]) == "echo") {
      irmv::simulation::Frame f;
      if (e.TryTake(&f)) {
        exchanged = e.TrySend(f);
      }
    }
    if (std::string(argv[2]) == "send" && e.Ready()) {
      irmv::simulation::Frame f;
      f.identity.run_id = c.run_id;
      f.payload = irmv::simulation::Abort{};
      for (int i = 0; i < 10000; ++i)
        if (!e.TrySend(f)) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  bool failed = e.Failed();
  e.Stop();
  if (std::string(argv[2]) == "echo") {
    std::cout << exchanged << " exchanged\n";
    return 0;
  }
  std::cout << ready << " " << failed << " " << e.Rejected() << "\n";
}
