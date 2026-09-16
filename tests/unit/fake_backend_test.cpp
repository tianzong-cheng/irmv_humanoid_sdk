#include "irmv/hardware/fake_backend.hpp"

#include <array>
#include <cstddef>

#include "irmv/configuration/config.hpp"
#include "support/check.hpp"

namespace {

irmv::configuration::RuntimeConfig TwoJointConfig() {
  irmv::configuration::RuntimeConfig config;
  config.joint_names = {"test_left", "test_right"};
  return config;
}

}  // namespace

int main() {
  using irmv::hardware::FakeBackend;
  using irmv::hardware::OutputRecord;
  using irmv::hardware::TorqueOutput;
  using irmv::protocol::FeedbackSnapshot;

  const auto config = TwoJointConfig();
  FakeBackend backend(config);
  backend.Initialize();

  const auto empty = backend.Read(0);
  CHECK(empty.count == 2);
  CHECK(!empty.joints[0].valid);
  CHECK(!empty.joints[1].valid);
  CHECK(!empty.imu.valid);

  FeedbackSnapshot snapshot{};
  snapshot.snapshot_seq = 17;
  snapshot.stamp_us = 19;
  snapshot.count = 2;
  snapshot.joints[0].valid = true;
  snapshot.joints[0].connected = true;
  snapshot.joints[0].q = 1.25;
  snapshot.joints[0].dq = -2.5;
  snapshot.joints[0].tau_est = 3.75;
  snapshot.joints[0].rx_stamp_us = 20;
  snapshot.joints[0].freq_hz = 499.5;
  snapshot.joints[0].error_code = 0x80000000u;
  snapshot.joints[1].valid = true;
  snapshot.joints[1].rx_stamp_us = 21;
  snapshot.imu.valid = true;
  snapshot.imu.orientation_rpy = {0.1, 0.2, 0.3};
  snapshot.imu.angular_velocity = {1.1, 1.2, 1.3};
  snapshot.imu.linear_acceleration = {2.1, 2.2, 2.3};
  snapshot.imu.rx_stamp_us = 22;
  backend.Inject(snapshot);

  const auto first_read = backend.Read(100);
  const auto second_read = backend.Read(200);
  CHECK(first_read.snapshot_seq == 17);
  CHECK(first_read.stamp_us == 19);
  CHECK(first_read.count == 2);
  CHECK(first_read.joints[0].valid);
  CHECK(first_read.joints[0].connected);
  CHECK(first_read.joints[0].q == 1.25);
  CHECK(first_read.joints[0].dq == -2.5);
  CHECK(first_read.joints[0].tau_est == 3.75);
  CHECK(first_read.joints[0].rx_stamp_us == 20);
  CHECK(first_read.joints[0].freq_hz == 499.5);
  CHECK(second_read.joints[0].rx_stamp_us == 20);
  CHECK(second_read.joints[0].error_code == 0x80000000u);
  CHECK(second_read.imu.valid);
  CHECK(second_read.imu.orientation_rpy ==
        (std::array<double, 3>{0.1, 0.2, 0.3}));
  CHECK(second_read.imu.angular_velocity ==
        (std::array<double, 3>{1.1, 1.2, 1.3}));
  CHECK(second_read.imu.linear_acceleration ==
        (std::array<double, 3>{2.1, 2.2, 2.3}));
  CHECK(second_read.imu.rx_stamp_us == 22);

  TorqueOutput output{};
  output.count = 2;
  output.torque[0] = 4.0;
  output.torque[1] = -5.0;
  const auto accepted = backend.Write(300, output);
  CHECK(accepted.accepted);
  CHECK(accepted.error_code == 0);
  CHECK(backend.LastOutput().has_value());
  CHECK(backend.LastOutput()->count == 2);
  CHECK(backend.LastOutput()->torque[0] == 4.0);
  CHECK(backend.LastOutput()->torque[1] == -5.0);

  std::array<OutputRecord, 1> accepted_record{};
  CHECK(backend.DrainOutputHistory(accepted_record.data(),
                                   accepted_record.size()) == 1);
  CHECK(accepted_record[0].write_stamp_us == 300);
  CHECK(accepted_record[0].result.accepted);
  CHECK(!accepted_record[0].hardware_applied);
  CHECK(accepted_record[0].output.torque[0] == 4.0);

  backend.SetWriteAccepted(false);
  const auto rejected = backend.Write(301, output);
  CHECK(!rejected.accepted);
  CHECK(rejected.error_code != 0);
  std::array<OutputRecord, 1> rejected_record{};
  CHECK(backend.DrainOutputHistory(rejected_record.data(),
                                   rejected_record.size()) == 1);
  CHECK(!rejected_record[0].result.accepted);
  CHECK(rejected_record[0].result.error_code == rejected.error_code);
  CHECK(!rejected_record[0].hardware_applied);

  backend.SetWriteAccepted(true);
  backend.Close();
  const auto after_close = backend.Write(302, output);
  CHECK(!after_close.accepted);
  CHECK(after_close.error_code != 0);

  FakeBackend bounded_backend(config);
  bounded_backend.Initialize();
  for (std::size_t i = 0; i < FakeBackend::kOutputHistoryCapacity + 2; ++i) {
    output.torque[0] = static_cast<double>(i);
    CHECK(bounded_backend.Write(i, output).accepted);
  }
  CHECK(bounded_backend.output_history_overflow_count() == 2);
  std::array<OutputRecord, FakeBackend::kOutputHistoryCapacity> records{};
  CHECK(bounded_backend.DrainOutputHistory(records.data(), records.size()) ==
        records.size());
  CHECK(records.front().write_stamp_us == 2);
  CHECK(records.front().output.torque[0] == 2.0);
  CHECK(records.back().write_stamp_us ==
        FakeBackend::kOutputHistoryCapacity + 1);
  CHECK(records.back().output.torque[0] ==
        static_cast<double>(FakeBackend::kOutputHistoryCapacity + 1));
  CHECK(bounded_backend.DrainOutputHistory(records.data(), records.size()) ==
        0);

  return 0;
}
