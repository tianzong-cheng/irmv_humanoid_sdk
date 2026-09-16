#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "irmv/configuration/config.hpp"
#include "irmv/hardware/backend.hpp"

namespace irmv::hardware {

struct OutputRecord {
  protocol::TimeUs write_stamp_us{0};
  TorqueOutput output;
  WriteResult result;
  // FakeBackend never actuates physical hardware, including accepted writes.
  bool hardware_applied{false};
};

class FakeBackend final : public Backend {
 public:
  static constexpr std::size_t kOutputHistoryCapacity = 64;

  explicit FakeBackend(const configuration::RuntimeConfig &config);

  void Initialize() override;
  protocol::FeedbackSnapshot Read(protocol::TimeUs now_us) override;
  WriteResult Write(protocol::TimeUs now_us,
                    const TorqueOutput &output) override;
  void Close() override;

  // All methods are control-thread owned. Runtime integrations must transfer
  // external injections onto that thread before calling Inject.
  void Inject(const protocol::FeedbackSnapshot &snapshot);
  void SetWriteAccepted(bool accepted);
  std::optional<TorqueOutput> LastOutput() const;

  std::size_t DrainOutputHistory(OutputRecord *destination,
                                 std::size_t capacity);
  std::uint64_t output_history_overflow_count() const noexcept;

 private:
  void RecordOutput(const OutputRecord &record);

  std::uint32_t joint_count_{0};
  bool initialized_{false};
  bool write_accepted_{true};
  protocol::FeedbackSnapshot feedback_{};
  std::optional<TorqueOutput> last_output_;
  std::array<OutputRecord, kOutputHistoryCapacity> output_history_{};
  std::size_t output_history_head_{0};
  std::size_t output_history_size_{0};
  std::uint64_t output_history_overflow_count_{0};
};

}  // namespace irmv::hardware
