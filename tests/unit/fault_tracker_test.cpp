#include "irmv/runtime/fault_tracker.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <new>

#include "irmv/protocol/types.hpp"
#include "support/check.hpp"

namespace {

// Counts every dynamic allocation of the test binary. The tracker must never
// allocate: its slot table and its observation batch live inside the object.
std::size_t allocation_count = 0;

}  // namespace

void *operator new(std::size_t size) {
  ++allocation_count;
  void *pointer = std::malloc(size == 0 ? 1 : size);
  if (pointer == nullptr) {
    throw std::bad_alloc();
  }
  return pointer;
}

void *operator new[](std::size_t size) { return ::operator new(size); }

void *operator new(std::size_t size, const std::nothrow_t &) noexcept {
  ++allocation_count;
  return std::malloc(size == 0 ? 1 : size);
}

void *operator new[](std::size_t size, const std::nothrow_t &) noexcept {
  return ::operator new(size, std::nothrow);
}

void operator delete(void *pointer) noexcept { std::free(pointer); }

void operator delete[](void *pointer) noexcept { std::free(pointer); }

void operator delete(void *pointer, std::size_t) noexcept {
  std::free(pointer);
}

void operator delete[](void *pointer, std::size_t) noexcept {
  std::free(pointer);
}

void operator delete(void *pointer, const std::nothrow_t &) noexcept {
  std::free(pointer);
}

void operator delete[](void *pointer, const std::nothrow_t &) noexcept {
  std::free(pointer);
}

namespace {

using irmv::protocol::FaultCondition;
using irmv::protocol::FaultDetail;
using irmv::protocol::kNoJointIndex;
using irmv::runtime::FaultTracker;
using irmv::runtime::kFaultObservationCapacity;

FaultDetail MakeDetail(FaultCondition condition, std::uint32_t joint_index,
                       std::uint32_t error_code = 0,
                       double observed_value = 0.0, const char *message = "") {
  FaultDetail detail;
  detail.condition = condition;
  detail.joint_index = joint_index;
  detail.error_code = error_code;
  detail.observed_value = observed_value;
  static_cast<void>(detail.message.Assign(message));
  return detail;
}

// One tracker tick: observe every listed detail, close the tick and report how
// many observations newly appeared. The new-observation batch of the tick can
// then be inspected through the tracker.
std::size_t ObserveTick(FaultTracker &tracker,
                        std::initializer_list<FaultDetail> details) {
  tracker.BeginTick();
  std::size_t newly = 0;
  for (const FaultDetail &detail : details) {
    if (tracker.Observe(detail)) {
      ++newly;
    }
  }
  tracker.EndTick();
  return newly;
}

const FaultDetail *FindNew(const FaultTracker &tracker,
                           FaultCondition condition,
                           std::uint32_t joint_index) {
  const auto &batch = tracker.NewObservations();
  for (std::uint32_t index = 0; index < batch.count; ++index) {
    if (batch.details[index].condition == condition &&
        batch.details[index].joint_index == joint_index) {
      return &batch.details[index];
    }
  }
  return nullptr;
}

int TestAppearPersistDisappearReappear() {
  FaultTracker tracker;
  const FaultDetail velocity =
      MakeDetail(FaultCondition::JointVelocityExceeded, 3, 0, 12.5,
                 "joint velocity limit exceeded");

  CHECK(ObserveTick(tracker, {velocity}) == 1);
  CHECK(tracker.NewObservations().count == 1);
  CHECK(tracker.NewObservations().dropped == 0);
  const FaultDetail *first =
      FindNew(tracker, FaultCondition::JointVelocityExceeded, 3);
  CHECK(first != nullptr);
  CHECK(first->observed_value == 12.5);
  CHECK(first->message.View() == "joint velocity limit exceeded");

  // The same condition on the next tick is not new.
  CHECK(ObserveTick(tracker, {velocity}) == 0);
  CHECK(tracker.NewObservations().count == 0);

  // A tick that does not carry it ends the occurrence.
  CHECK(ObserveTick(tracker, {}) == 0);
  CHECK(tracker.NewObservations().count == 0);

  // It reappears after the silent tick.
  CHECK(ObserveTick(tracker, {velocity}) == 1);
  CHECK(tracker.NewObservations().count == 1);

  // A second silent tick, then a different observed value near the threshold:
  // the key does not include the value, so this is still the same occurrence.
  CHECK(ObserveTick(tracker, {}) == 0);
  const FaultDetail near =
      MakeDetail(FaultCondition::JointVelocityExceeded, 3, 0, 10.001,
                 "joint velocity limit exceeded");
  CHECK(ObserveTick(tracker, {near}) == 1);
  CHECK(ObserveTick(tracker,
                    {MakeDetail(FaultCondition::JointVelocityExceeded, 3, 0,
                                10.002, "joint velocity limit exceeded")}) ==
        0);
  return 0;
}

int TestErrorCodesAreKeyedForDriverAndWriteFailures() {
  FaultTracker tracker;
  const FaultDetail error_a =
      MakeDetail(FaultCondition::JointDriverError, 2, 7, 0.0, "driver error");
  const FaultDetail error_b =
      MakeDetail(FaultCondition::JointDriverError, 2, 9, 0.0, "driver error");

  CHECK(ObserveTick(tracker, {error_a}) == 1);
  CHECK(ObserveTick(tracker, {error_a}) == 0);
  // A different error code is a different occurrence...
  CHECK(ObserveTick(tracker, {error_b}) == 1);
  const FaultDetail *second =
      FindNew(tracker, FaultCondition::JointDriverError, 2);
  CHECK(second != nullptr);
  CHECK(second->error_code == 9);
  // ...while code A returns as a new occurrence only after disappearing.
  CHECK(ObserveTick(tracker, {}) == 0);
  CHECK(ObserveTick(tracker, {error_a}) == 1);

  // The two write-failure conditions are keyed by error code as well.
  const FaultDetail primary =
      MakeDetail(FaultCondition::PrimaryWriteFailed, kNoJointIndex, 4);
  const FaultDetail primary_other =
      MakeDetail(FaultCondition::PrimaryWriteFailed, kNoJointIndex, 5);
  const FaultDetail fallback =
      MakeDetail(FaultCondition::FallbackWriteFailed, kNoJointIndex, 4);
  CHECK(ObserveTick(tracker, {primary}) == 1);
  CHECK(ObserveTick(tracker, {primary, primary_other}) == 1);
  CHECK(ObserveTick(tracker, {primary, primary_other, fallback}) == 1);
  return 0;
}

int TestKeysIgnoreErrorCodeOutsideFailureConditions() {
  FaultTracker tracker;
  // JointFeedbackInvalid has no error-code dimension: the same invalid sample
  // reported with a changing driver status stays the same occurrence.
  const FaultDetail invalid_a =
      MakeDetail(FaultCondition::JointFeedbackInvalid, 1, 4, 0.0,
                 "invalid joint feedback sample");
  const FaultDetail invalid_b =
      MakeDetail(FaultCondition::JointFeedbackInvalid, 1, 5, 0.0,
                 "invalid joint feedback sample");
  CHECK(ObserveTick(tracker, {invalid_a}) == 1);
  CHECK(ObserveTick(tracker, {invalid_b}) == 0);
  CHECK(tracker.NewObservations().count == 0);

  // Time and message are not part of the key either.
  FaultDetail stale = MakeDetail(FaultCondition::JointFeedbackStale, 0, 0, 0.0,
                                 "stale joint feedback sample");
  stale.sample_stamp_us = 10;
  stale.detected_stamp_us = 20;
  CHECK(ObserveTick(tracker, {stale}) == 1);
  stale.sample_stamp_us = 11;
  stale.detected_stamp_us = 21;
  static_cast<void>(stale.message.Assign("a different message"));
  CHECK(ObserveTick(tracker, {stale}) == 0);
  return 0;
}

int TestJointsAndConditionsAreSeparateKeys() {
  FaultTracker tracker;
  const FaultDetail velocity_0 =
      MakeDetail(FaultCondition::JointVelocityExceeded, 0);
  const FaultDetail velocity_1 =
      MakeDetail(FaultCondition::JointVelocityExceeded, 1);
  const FaultDetail position_0 =
      MakeDetail(FaultCondition::JointPositionExceeded, 0);

  // Two joints of the same type appear together.
  CHECK(ObserveTick(tracker, {velocity_0, velocity_1}) == 2);
  CHECK(tracker.NewObservations().count == 2);

  // The same joint with a different condition is a separate occurrence, while
  // the two velocity conditions persist.
  CHECK(ObserveTick(tracker, {velocity_0, velocity_1, position_0}) == 1);
  CHECK(tracker.NewObservations().count == 1);
  CHECK(FindNew(tracker, FaultCondition::JointPositionExceeded, 0) != nullptr);
  CHECK(FindNew(tracker, FaultCondition::JointVelocityExceeded, 1) == nullptr);

  // Both types on the same joint persist on the following tick.
  CHECK(ObserveTick(tracker, {velocity_0, position_0}) == 0);
  return 0;
}

int TestSameTickDuplicatesAreDeduplicated() {
  FaultTracker tracker;
  const FaultDetail error =
      MakeDetail(FaultCondition::JointDriverError, 5, 3, 0.0, "driver error");
  CHECK(ObserveTick(tracker, {error, error}) == 1);
  CHECK(tracker.NewObservations().count == 1);

  // A duplicate that carries a different message or observed value is still
  // the same observation of the same condition.
  const FaultDetail error_again =
      MakeDetail(FaultCondition::JointDriverError, 5, 3, 99.0, "other");
  CHECK(ObserveTick(tracker, {error, error_again}) == 0);
  CHECK(tracker.NewObservations().count == 0);
  return 0;
}

int TestBatchOverflowKeepsTracking() {
  FaultTracker tracker;
  // 1100 distinct driver-error keys in one tick exceed the batch capacity.
  // Slot tracking still learns every key and the overflow counter reports the
  // observations that did not fit.
  constexpr std::uint32_t kObservations = 1100;
  tracker.BeginTick();
  std::uint32_t newly = 0;
  for (std::uint32_t index = 0; index < kObservations; ++index) {
    const FaultDetail detail = MakeDetail(FaultCondition::JointDriverError,
                                          index % 64, 10'000 + index);
    if (tracker.Observe(detail)) {
      ++newly;
    }
  }
  tracker.EndTick();
  CHECK(newly == kObservations);
  CHECK(tracker.NewObservations().count == kFaultObservationCapacity);
  CHECK(tracker.NewObservations().dropped ==
        kObservations - kFaultObservationCapacity);

  // The keys learned while the batch was full are still active, so they do not
  // reappear on the next tick, and the counter is reset for the new tick.
  const FaultDetail late =
      MakeDetail(FaultCondition::JointDriverError, 1099 % 64, 10'000 + 1099);
  CHECK(ObserveTick(tracker, {late}) == 0);
  CHECK(tracker.NewObservations().count == 0);
  CHECK(tracker.NewObservations().dropped == 0);

  // A dropped occurrence that disappears and returns is reported again.
  CHECK(ObserveTick(tracker, {}) == 0);
  CHECK(ObserveTick(tracker, {late}) == 1);
  CHECK(tracker.NewObservations().count == 1);
  return 0;
}

int TestFullTableDoesNotRereportSustainedKeys() {
  FaultTracker tracker;
  // Saturate every slot (twice the batch capacity) with distinct error keys.
  const std::size_t keys = 2 * kFaultObservationCapacity;
  tracker.BeginTick();
  for (std::size_t index = 0; index < keys; ++index) {
    static_cast<void>(
        tracker.Observe(MakeDetail(FaultCondition::JointDriverError,
                                   static_cast<std::uint32_t>(index % 64),
                                   1'000 + static_cast<std::uint32_t>(index))));
  }
  tracker.EndTick();
  CHECK(tracker.NewObservations().count == kFaultObservationCapacity);
  CHECK(tracker.NewObservations().dropped == keys - kFaultObservationCapacity);

  // The next tick introduces one new key while every slot still holds a
  // sustained key. Evicting a previously active slot would report other
  // sustained keys as new later in this tick, so the new key stays untracked:
  // it is not reported as new and the loss moves the counter.
  tracker.BeginTick();
  const FaultDetail untracked =
      MakeDetail(FaultCondition::JointDriverError, 63, 999'999);
  CHECK(!tracker.Observe(untracked));
  CHECK(tracker.NewObservations().dropped == 1);

  std::size_t rereported = 0;
  for (std::size_t index = 0; index < keys; ++index) {
    const FaultDetail sustained =
        MakeDetail(FaultCondition::JointDriverError,
                   static_cast<std::uint32_t>(index % 64),
                   1'000 + static_cast<std::uint32_t>(index));
    if (tracker.Observe(sustained)) {
      ++rereported;
    }
  }
  tracker.EndTick();
  CHECK(rereported == 0);
  CHECK(tracker.NewObservations().count == 0);
  CHECK(tracker.NewObservations().dropped == 1);
  return 0;
}

int TestNoDynamicAllocation() {
  // Positive control: the monitor must observe a real allocation, otherwise a
  // broken override would let the zero-allocation assertion pass vacuously.
  const std::size_t control_before = allocation_count;
  int *probe = new int(7);
  delete probe;
  CHECK(allocation_count == control_before + 1);

  // Construction and the steady-state tick must not allocate. The tracker is
  // preallocated: its slot table and observation batch are members.
  FaultTracker tracker;
  const FaultDetail detail =
      MakeDetail(FaultCondition::JointFeedbackNonFinite, 0, 0, 0.0,
                 "non-finite joint feedback sample");
  const std::size_t before = allocation_count;
  tracker.BeginTick();
  const bool newly = tracker.Observe(detail);
  const bool duplicate = tracker.Observe(detail);
  tracker.EndTick();
  const std::size_t after = allocation_count;
  CHECK(newly);
  CHECK(!duplicate);
  CHECK(after == before);
  CHECK(tracker.NewObservations().count == 1);
  CHECK(tracker.NewObservations().details[0].condition ==
        FaultCondition::JointFeedbackNonFinite);
  return 0;
}

}  // namespace

int main() {
  int result = TestAppearPersistDisappearReappear();
  if (result != 0) {
    return result;
  }
  result = TestErrorCodesAreKeyedForDriverAndWriteFailures();
  if (result != 0) {
    return result;
  }
  result = TestKeysIgnoreErrorCodeOutsideFailureConditions();
  if (result != 0) {
    return result;
  }
  result = TestJointsAndConditionsAreSeparateKeys();
  if (result != 0) {
    return result;
  }
  result = TestSameTickDuplicatesAreDeduplicated();
  if (result != 0) {
    return result;
  }
  result = TestBatchOverflowKeepsTracking();
  if (result != 0) {
    return result;
  }
  result = TestFullTableDoesNotRereportSustainedKeys();
  if (result != 0) {
    return result;
  }
  return TestNoDynamicAllocation();
}
