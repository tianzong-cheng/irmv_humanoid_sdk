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
  const bool blocked = std::string(argv[2]) == "blocked";
  const auto begin = std::chrono::steady_clock::now();
  auto next_send = begin;
  int received = 0;
  auto end = begin + std::chrono::milliseconds(blocked ? 2000 : 350);
  bool ready = false;
  bool exchanged = false;
  while (std::chrono::steady_clock::now() < end && !e.Failed()) {
    ready = ready || e.Ready();
    if (blocked && e.Ready()) {
      irmv::simulation::Frame incoming;
      while (e.TryTake(&incoming)) ++received;
      if (std::chrono::steady_clock::now() >= next_send) {
        irmv::simulation::Frame outgoing;
        outgoing.identity.run_id = c.run_id;
        irmv::simulation::Feedback feedback;
        feedback.q.count = feedback.dq.count = feedback.tau.count = 64;
        outgoing.payload = feedback;
        e.TrySend(outgoing);
        next_send =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
      }
    }
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
  std::cout << ready << " " << failed << " " << e.Rejected();
  if (blocked) {
    std::cout << " " << received << " "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - begin)
                     .count();
  }
  std::cout << "\n";
}
