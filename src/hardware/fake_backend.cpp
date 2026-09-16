#include "irmv/hardware/fake_backend.hpp"

#include <algorithm>

namespace irmv::hardware {
namespace {

constexpr std::uint32_t kBackendClosedError = 1;
constexpr std::uint32_t kConfiguredWriteRejectionError = 2;

}  // namespace

FakeBackend::FakeBackend(const configuration::RuntimeConfig &config)
    : joint_count_(static_cast<std::uint32_t>(config.joint_count())) {}

void FakeBackend::Initialize() {
  initialized_ = true;
  feedback_ = {};
  feedback_.count = joint_count_;
  last_output_.reset();
  output_history_head_ = 0;
  output_history_size_ = 0;
  output_history_overflow_count_ = 0;
}

protocol::FeedbackSnapshot FakeBackend::Read(protocol::TimeUs now_us) {
  static_cast<void>(now_us);
  return feedback_;
}

WriteResult FakeBackend::Write(protocol::TimeUs now_us,
                               const TorqueOutput &output) {
  WriteResult result;
  if (!initialized_) {
    result.error_code = kBackendClosedError;
  } else if (!write_accepted_) {
    result.error_code = kConfiguredWriteRejectionError;
  } else {
    result.accepted = true;
  }

  last_output_ = output;
  RecordOutput(OutputRecord{now_us, output, result, false});
  return result;
}

void FakeBackend::Close() { initialized_ = false; }

void FakeBackend::Inject(const protocol::FeedbackSnapshot &snapshot) {
  feedback_ = snapshot;
}

void FakeBackend::SetWriteAccepted(bool accepted) {
  write_accepted_ = accepted;
}

std::optional<TorqueOutput> FakeBackend::LastOutput() const {
  return last_output_;
}

std::size_t FakeBackend::DrainOutputHistory(OutputRecord *destination,
                                            std::size_t capacity) {
  if (destination == nullptr || capacity == 0) {
    return 0;
  }

  const std::size_t drain_count = std::min(capacity, output_history_size_);
  for (std::size_t i = 0; i < drain_count; ++i) {
    destination[i] =
        output_history_[(output_history_head_ + i) % kOutputHistoryCapacity];
  }
  output_history_head_ =
      (output_history_head_ + drain_count) % kOutputHistoryCapacity;
  output_history_size_ -= drain_count;
  return drain_count;
}

std::uint64_t FakeBackend::output_history_overflow_count() const noexcept {
  return output_history_overflow_count_;
}

void FakeBackend::RecordOutput(const OutputRecord &record) {
  if (output_history_size_ == kOutputHistoryCapacity) {
    output_history_head_ = (output_history_head_ + 1) % kOutputHistoryCapacity;
    --output_history_size_;
    ++output_history_overflow_count_;
  }

  const std::size_t insertion_index =
      (output_history_head_ + output_history_size_) % kOutputHistoryCapacity;
  output_history_[insertion_index] = record;
  ++output_history_size_;
}

}  // namespace irmv::hardware
