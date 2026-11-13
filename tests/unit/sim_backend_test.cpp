#include "irmv/hardware/sim_backend.hpp"

#include <cmath>
#include <limits>

#include "irmv/hardware/backend_factory.hpp"
#include "support/check.hpp"
int main() {
  using namespace irmv;
  configuration::RuntimeConfig c;
  c.joint_names = {"a", "b"};
  simulation::SimulationExchange x(2);
  hardware::SimBackend b(c, x);
  b.Initialize();
  CHECK(b.Read(100).count == 2);
  CHECK(!b.Read(100).joints[0].valid);
  CHECK(b.Read(100).joints[0].rx_stamp_us == 0);
  simulation::FeedbackFrame f;
  f.identity.run_id[0] = 1;
  f.feedback.q.count = f.feedback.dq.count = f.feedback.tau.count = 2;
  f.feedback.valid = f.feedback.imu_valid = true;
  f.feedback.imu_quaternion = {std::sqrt(0.5), 0, 0, std::sqrt(0.5)};
  f.feedback.q.values[0] = 0.3;
  auto invalid_run = f;
  invalid_run.identity.run_id = {};
  CHECK(!x.InstallFeedback(invalid_run));
  CHECK(x.InstallFeedback(f));
  auto r = b.Read(999);
  CHECK(r.stamp_us == 1);
  CHECK(r.joints[0].q == 0.3);
  CHECK(std::abs(r.imu.orientation_rpy[2] - std::acos(-1) / 2) < 1e-10);
  CHECK(b.Read(9999).joints[0].rx_stamp_us == 1);
  hardware::TorqueOutput t;
  t.count = 2;
  t.torque[0] = 4;
  auto wrong = t;
  wrong.count = 1;
  CHECK(!b.Write(1, wrong).accepted);
  wrong = t;
  wrong.torque[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(!b.Write(1, wrong).accepted);
  CHECK(!x.TryTakeTorque(nullptr));
  CHECK(b.Write(1, t).accepted);
  CHECK(!b.Write(1, t).accepted);
  auto next = f;
  next.identity.control_step = 1;
  next.identity.logical_us = 2001;
  CHECK(!x.InstallFeedback(next));
  simulation::TorqueFrame out;
  CHECK(x.TryTakeTorque(&out));
  CHECK(out.torque.accepted);
  CHECK(out.torque.tau.values[0] == 4);
  CHECK(!b.Write(1, t).accepted);
  CHECK(x.InstallFeedback(next));
  CHECK(b.Read(999999).joints[0].freq_hz == 500);
  CHECK(b.Read(9999999).joints[0].freq_hz == 500);
  CHECK(!b.Write(1, t).accepted);
  CHECK(b.Write(2001, t).accepted);
  CHECK(x.TryTakeTorque(&out));
  next.identity.control_step = 2;
  next.identity.logical_us = 4001;
  next.feedback.imu_quaternion = {2, 0, 0, 0};
  CHECK(!x.InstallFeedback(next));
  next.feedback.imu_quaternion = {1, 0, 0, 0};
  next.feedback.q.values[0] = std::numeric_limits<double>::infinity();
  CHECK(!x.InstallFeedback(next));
  next.feedback.q.values[0] = 0;
  next.feedback.imu_quaternion = {std::sqrt(.5), std::sqrt(.5), 0, 0};
  CHECK(x.InstallFeedback(next));
  CHECK(std::abs(b.Read(1).imu.orientation_rpy[0] - std::acos(-1) / 2) < 1e-10);
  next.identity.control_step = 3;
  next.identity.logical_us = 6001;
  next.feedback.imu_quaternion = {std::cos(.25), 0, std::sin(.25), 0};
  CHECK(x.InstallFeedback(next));
  CHECK(std::abs(b.Read(1).imu.orientation_rpy[1] - .5) < 1e-10);
  // Frozen-time stop output uses a new phase, never a fresh measurement stamp.
  auto stop_phase = next;
  stop_phase.identity.phase = 1;
  CHECK(x.InstallFeedback(stop_phase));
  CHECK(b.Read(999999).stamp_us == 6001);
  CHECK(b.Read(999999).joints[0].freq_hz == 500);
  CHECK(b.Write(6001, t).accepted);
  CHECK(!x.InstallFeedback(stop_phase));
  CHECK(x.TryTakeTorque(&out));
  CHECK(out.identity.phase == 1);
  stop_phase.identity.phase = 2;
  stop_phase.feedback.q.values[0] = 42;
  CHECK(!x.InstallFeedback(stop_phase));
  b.Close();
  CHECK(!b.Write(2001, t).accepted);
  c.backend = "sim";
  bool refused = false;
  try {
    hardware::MakeBackend(c);
  } catch (...) {
    refused = true;
  }
  CHECK(refused);
  CHECK(hardware::MakeBackend(c, x) != nullptr);
  return 0;
}
