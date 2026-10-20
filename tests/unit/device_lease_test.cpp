#include "irmv/hardware/device_lease.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "support/check.hpp"

namespace {

using irmv::configuration::BusConfig;
using irmv::configuration::HardwareConfig;
using irmv::hardware::DeviceLease;

class TemporaryLockDirectory {
 public:
  TemporaryLockDirectory() {
    static unsigned int next_id = 0;
    path_ = std::filesystem::temp_directory_path() /
            ("irmv-lock-test-" + std::to_string(::getpid()) + "-" +
             std::to_string(next_id++));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryLockDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  const std::filesystem::path &path() const { return path_; }

 private:
  std::filesystem::path path_;
};

BusConfig Bus(std::uint32_t device_index, std::uint32_t channel) {
  BusConfig bus;
  bus.device_type = 41;
  bus.device_index = device_index;
  bus.channel = channel;
  bus.arbitration_bps = 1000000;
  bus.data_bps = 5000000;
  return bus;
}

HardwareConfig OneBus(const std::string &name, std::uint32_t device_index,
                      std::uint32_t channel) {
  HardwareConfig config;
  BusConfig bus = Bus(device_index, channel);
  bus.name = name;
  config.buses.push_back(std::move(bus));
  return config;
}

// Runs one acquisition in a child. Returns the child's result byte: 0 held, 1
// threw, 2 Acquire returned an empty lease; 100 means the child exited without
// reporting (the abnormal-exit case).
int ChildResult(const std::filesystem::path &directory,
                const HardwareConfig &config, bool report, int hold_ms = 0) {
  int descriptors[2];
  if (::pipe(descriptors) != 0) {
    return 101;
  }
  const pid_t child = ::fork();
  if (child == 0) {
    ::close(descriptors[0]);
    int result = 0;
    try {
      DeviceLease lease = DeviceLease::Acquire(config, directory);
      if (!lease.held()) {
        result = 2;
      }
      if (hold_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
      }
    } catch (const std::exception &) {
      result = 1;
    }
    if (report) {
      const char byte = static_cast<char>(result);
      static_cast<void>(::write(descriptors[1], &byte, 1));
    }
    ::_exit(0);
  }
  ::close(descriptors[1]);
  char byte = static_cast<char>(100);
  const ssize_t received = ::read(descriptors[0], &byte, 1);
  ::close(descriptors[0]);
  int status = 0;
  static_cast<void>(::waitpid(child, &status, 0));
  return received == 1 ? static_cast<int>(byte) : 100;
}

bool AcquireFails(const HardwareConfig &config,
                  const std::filesystem::path &directory,
                  std::string_view expected) {
  try {
    static_cast<void>(DeviceLease::Acquire(config, directory));
  } catch (const std::exception &error) {
    return std::string_view(error.what()).find(expected) !=
           std::string_view::npos;
  }
  return false;
}

}  // namespace

int main() {
  const TemporaryLockDirectory lock_directory;

  // One physical device is one lock, whatever channel or bus name the document
  // chooses.
  {
    DeviceLease parent =
        DeviceLease::Acquire(OneBus("can0", 0, 0), lock_directory.path());
    CHECK(parent.held());
    CHECK(parent.identities() == std::vector<std::string>{"can-41-0"});
    CHECK(ChildResult(lock_directory.path(), OneBus("can0", 0, 1), true) == 1);
    CHECK(ChildResult(lock_directory.path(), OneBus("left_leg", 0, 0), true) ==
          1);
  }

  // A different device index coexists.
  {
    DeviceLease parent =
        DeviceLease::Acquire(OneBus("can0", 0, 0), lock_directory.path());
    CHECK(ChildResult(lock_directory.path(), OneBus("can2", 1, 0), true) == 0);
  }

  // A failed second acquisition releases the first. The child holds device 1
  // for a moment; this process asks for device 0 then device 1 and must fail on
  // the second, after which device 0 is free again.
  {
    int descriptors[2];
    CHECK(::pipe(descriptors) == 0);
    const pid_t child = ::fork();
    if (child == 0) {
      ::close(descriptors[0]);
      DeviceLease lease =
          DeviceLease::Acquire(OneBus("held", 1, 0), lock_directory.path());
      const char ready = 0;
      static_cast<void>(::write(descriptors[1], &ready, 1));
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      ::_exit(0);
    }
    ::close(descriptors[1]);
    char ready = 1;
    CHECK(::read(descriptors[0], &ready, 1) == 1);
    ::close(descriptors[0]);

    HardwareConfig both;
    both.buses.push_back(Bus(0, 0));
    both.buses[0].name = "a";
    both.buses.push_back(Bus(1, 0));
    both.buses[1].name = "b";
    CHECK(AcquireFails(both, lock_directory.path(), "can-41-1"));
    {
      DeviceLease free_after_failure =
          DeviceLease::Acquire(OneBus("a", 0, 0), lock_directory.path());
      CHECK(free_after_failure.held());
    }
    int status = 0;
    static_cast<void>(::waitpid(child, &status, 0));
  }

  // A normal exit releases the lock.
  {
    CHECK(ChildResult(lock_directory.path(), OneBus("can0", 0, 0), true) == 0);
    DeviceLease lease =
        DeviceLease::Acquire(OneBus("can0", 0, 0), lock_directory.path());
    CHECK(lease.held());
    lease.Release();
    CHECK(!lease.held());
  }

  // An exit that never runs a destructor releases it too: the OS owns the
  // flock, not the C++ object.
  {
    CHECK(ChildResult(lock_directory.path(), OneBus("can0", 0, 0), false) ==
          100);
    DeviceLease lease =
        DeviceLease::Acquire(OneBus("can0", 0, 0), lock_directory.path());
    CHECK(lease.held());
  }

  // Re-entering Acquire from one process contends like another process would.
  {
    DeviceLease first =
        DeviceLease::Acquire(OneBus("can0", 0, 0), lock_directory.path());
    CHECK(
        AcquireFails(OneBus("can0", 0, 0), lock_directory.path(), "can-41-0"));
  }

  // The IMU is part of the same lease, keyed by its normalised identity.
  {
    HardwareConfig with_imu = OneBus("can0", 0, 0);
    with_imu.imu = irmv::configuration::ImuConfig{"YIS/IMU 0", 921600};
    DeviceLease lease = DeviceLease::Acquire(with_imu, lock_directory.path());
    CHECK(lease.identities() ==
          (std::vector<std::string>{"can-41-0", "imu-YIS_IMU_0"}));
  }
  return 0;
}
