#include "irmv/hardware/sim_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace irmv::simulation {
SimulationExchange::SimulationExchange(std::uint32_t count) : count_(count) {
  if (!count || count > kMaxJoints)
    throw std::invalid_argument("simulation joints");
}
bool SimulationExchange::InstallFeedback(const FeedbackFrame& f) noexcept {
  if (f.identity.run_id == protocol::Id{}) return false;
  if (pending_ || f.feedback.q.count != count_ ||
      f.feedback.dq.count != count_ || f.feedback.tau.count != count_ ||
      f.identity.logical_us == 0)
    return false;
  if (observed_) {
    const auto& previous = feedback_.identity;
    if (f.identity.run_id != previous.run_id ||
        f.identity.control_step < previous.control_step)
      return false;
    if (f.identity.control_step == previous.control_step) {
      const auto& a = f.feedback;
      const auto& b = feedback_.feedback;
      // Frozen-time stop phases may reuse the exact snapshot, never relabel a
      // changed measurement as the same physical sample.
      if (f.identity.phase <= previous.phase ||
          f.identity.logical_us != previous.logical_us ||
          a.physics_step != b.physics_step ||
          a.engine_time_s != b.engine_time_s || a.q.values != b.q.values ||
          a.dq.values != b.dq.values || a.tau.values != b.tau.values ||
          a.imu_quaternion != b.imu_quaternion ||
          a.imu_angular_velocity != b.imu_angular_velocity ||
          a.imu_specific_force != b.imu_specific_force ||
          a.base_position != b.base_position ||
          a.base_velocity != b.base_velocity || a.valid != b.valid ||
          a.imu_valid != b.imu_valid)
        return false;
    } else if (f.identity.logical_us <= previous.logical_us)
      return false;
  }
  for (std::size_t i = 0; i < count_; ++i)
    if (!std::isfinite(f.feedback.q.values[i]) ||
        !std::isfinite(f.feedback.dq.values[i]) ||
        !std::isfinite(f.feedback.tau.values[i]))
      return false;
  double norm = 0;
  for (double v : f.feedback.imu_quaternion) {
    if (!std::isfinite(v)) return false;
    norm += v * v;
  }
  if (f.feedback.imu_valid && std::abs(norm - 1) > 1e-6) return false;
  for (double v : f.feedback.imu_angular_velocity)
    if (!std::isfinite(v)) return false;
  for (double v : f.feedback.imu_specific_force)
    if (!std::isfinite(v)) return false;
  feedback_ = f;
  observed_ = true;
  written_ = false;
  return true;
}
bool SimulationExchange::TryTakeTorque(TorqueFrame* out) noexcept {
  if (!out || !pending_) return false;
  *out = torque_;
  pending_ = false;
  return true;
}
}  // namespace irmv::simulation
namespace irmv::hardware {
SimBackend::SimBackend(const configuration::RuntimeConfig& c,
                       simulation::SimulationExchange& x)
    : exchange_(x) {
  if (c.joint_count() != x.count_)
    throw std::invalid_argument("simulation joint shape");
}
void SimBackend::Initialize() { open_ = true; }
void SimBackend::Close() { open_ = false; }
protocol::FeedbackSnapshot SimBackend::Read(protocol::TimeUs) {
  protocol::FeedbackSnapshot r{};
  r.count = exchange_.count_;
  if (!open_ || !exchange_.observed_) return r;
  const auto& f = exchange_.feedback_;
  const auto& v = f.feedback;
  r.stamp_us = f.identity.logical_us;
  r.snapshot_seq = f.identity.control_step + 1;
  for (std::size_t i = 0; i < r.count; ++i) {
    auto& j = r.joints[i];
    j.valid = v.valid;
    j.connected = v.valid;
    j.q = v.q.values[i];
    j.dq = v.dq.values[i];
    j.tau_est = v.tau.values[i];
    j.rx_stamp_us = r.stamp_us;
  }
  r.imu.valid = v.imu_valid;
  r.imu.rx_stamp_us = r.stamp_us;
  r.imu.angular_velocity = v.imu_angular_velocity;
  r.imu.linear_acceleration = v.imu_specific_force;
  if (v.imu_valid) {
    const auto& q = v.imu_quaternion;
    double w = q[0], x = q[1], y = q[2], z = q[3];
    r.imu.orientation_rpy = {
        std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)),
        std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))};
  }
  return r;
}
WriteResult SimBackend::Write(protocol::TimeUs now,
                              const TorqueOutput& output) {
  auto& x = exchange_;
  if (!open_ || !x.observed_ || x.pending_ || x.written_ ||
      output.count != x.count_ || now != x.feedback_.identity.logical_us)
    return {false, 1};
  for (std::size_t i = 0; i < output.count; ++i)
    if (!std::isfinite(output.torque[i])) return {false, 2};
  x.torque_ = {};
  x.torque_.identity = x.feedback_.identity;
  auto& t = x.torque_.torque;
  t.write_id = ++x.write_id_;
  t.accepted = true;
  t.tau.count = output.count;
  t.tau.values = output.torque;
  x.pending_ = x.written_ = true;
  return {true, 0};
}
}  // namespace irmv::hardware
