#pragma once
#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"
#include "irmv/simulation/protocol.hpp"
namespace irmv::hardware {
class SimBackend;
}
namespace irmv::simulation {
struct FeedbackFrame {
  StepIdentity identity{};
  Feedback feedback{};
};
struct TorqueFrame {
  StepIdentity identity{};
  Torque torque{};
};
// Entirely control-thread-owned, including all methods and Backend access.
// Loop driver installs decoded feedback and drains one torque before advancing.
// One outstanding torque slot; draining is host handoff, never physics
// completion.
class SimulationExchange {
 public:
  explicit SimulationExchange(std::uint32_t joint_count);
  // Advancing a control step requires increasing logical time. A strictly later
  // phase at the same step may reuse an identical feedback snapshot for a
  // frozen-time stop. Duplicate identities and pending output are refused.
  bool InstallFeedback(const FeedbackFrame&) noexcept;
  bool TryTakeTorque(TorqueFrame*) noexcept;

 private:
  friend class irmv::hardware::SimBackend;
  std::uint32_t count_;
  FeedbackFrame feedback_{};
  TorqueFrame torque_{};
  bool observed_{false}, pending_{false}, written_{false};
  std::uint64_t write_id_{0};
};
}  // namespace irmv::simulation
namespace irmv::hardware {
class SimBackend final : public Backend {
 public:
  SimBackend(const configuration::RuntimeConfig&,
             simulation::SimulationExchange&);
  void Initialize() override;
  protocol::FeedbackSnapshot Read(protocol::TimeUs) override;
  WriteResult Write(protocol::TimeUs, const TorqueOutput&) override;
  void Close() override;

 private:
  simulation::SimulationExchange& exchange_;
  bool open_{false};
};
}  // namespace irmv::hardware
