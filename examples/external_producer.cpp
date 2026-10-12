// The example external producer of the Fake deployment: a target generator that
// the application selects, prepares and authorizes, and that then publishes one
// JointCommand per round period on the execution command topic.
//
// It is an example and a test peer, never a machine-safe program. The gains,
// amplitudes and tolerances it uses are demonstration values of the Fake
// deployment and are not a validated machine limit; nothing here decides what a
// real machine may be commanded, the runtime's own guards do.
//
// The lifecycle is the frozen one of protocol/producer.idl. This process never
// acquires a session, never requests a mode and never supervises: it announces
// itself, answers the preparations meant for it, publishes only after the
// authorization the runtime granted and stops the moment a stop notice names
// it. A frame published without an authorization is refused by the runtime
// anyway, so the point of the lifecycle is that nothing reaches the execution
// topic before the runtime applied the context.
//
//   external_producer --config <sdk.json> --domain <N> [--rate <hz>]
//                     [--producer-id <32 hex>] [--duration-us <us>]
//
// Every lifecycle edge is printed on stdout and flushed, so an operator and the
// integration test can follow it:
//
//   producer=announced id=<hex>
//   producer=preparing prepare_id=N
//   producer=prepared prepare_id=N source_seq=N
//   producer=authorized epoch=N first_seq=N
//   producer=publishing seq=N
//   producer=stopped reason=<name>
//   producer=exiting
//
// It exits 0 on every orderly end of the process - a --duration-us that elapsed
// or a SIGINT/SIGTERM - and keeps running otherwise. A stop notice stops the
// publication, not the process: a later preparation may authorize a new
// context. Every DDS refusal is counted and reported on stderr at exit instead
// of being dropped silently.
//
// Exit codes: 0 orderly end, 2 the command line was rejected, 3 startup
// (configuration, DDS or a period the runtime could never keep) failed.

#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <dds/dds.hpp>
#include <exception>
#include <filesystem>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "execution.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/configuration/config.hpp"
#include "irmv/protocol/producer.hpp"
#include "irmv/protocol/types.hpp"
#include "irmv/transport/codec.hpp"
#include "irmv/transport/producer_codec.hpp"
#include "producer.hpp"

namespace {

namespace codec = irmv::transport::codec;
namespace producer_codec = irmv::transport::producer_codec;
namespace wire = irmv::protocol::wire;

using irmv::configuration::ApplicationConfig;
using irmv::protocol::AuthorizationNotice;
using irmv::protocol::Id;
using irmv::protocol::JointCommand;
using irmv::protocol::kJointCommandTopic;
using irmv::protocol::kNoId;
using irmv::protocol::kProducerAnnouncementTopic;
using irmv::protocol::kProducerAuthorizationTopic;
using irmv::protocol::kProducerPrepareRequestTopic;
using irmv::protocol::kProducerPrepareResultTopic;
using irmv::protocol::kProducerStopTopic;
using irmv::protocol::kProducerVelocityIntentTopic;
using irmv::protocol::kRobotStateTopic;
using irmv::protocol::PrepareRequest;
using irmv::protocol::PrepareResult;
using irmv::protocol::ProducerAnnouncement;
using irmv::protocol::ProducerFailureReason;
using irmv::protocol::ProducerStatus;
using irmv::protocol::RobotState;
using irmv::protocol::Sequence;
using irmv::protocol::StopNotice;
using irmv::protocol::StopReason;
using irmv::protocol::TimeUs;
using irmv::protocol::VelocityIntentMessage;

constexpr int kExitOk = 0;
constexpr int kExitUsage = 2;
constexpr int kExitStartup = 3;

// The round period is the cadence of the whole lifecycle, not only of the
// commands: the announcement, the preparation and the publication all move at
// it. The application refuses a producer whose period is not strictly below the
// runtime's own command timeout, because every frame such a producer sent would
// arrive after the one before it expired.
constexpr TimeUs kAnnounceIntervalUs = 500'000;
constexpr TimeUs kLoopSleepUs = 1'000;

// Demonstration walk of the Fake deployment: a small radius, a slow cycle and
// modest phase biases. The position_delta and target_velocity_limits of the
// configuration are clamped against below, so a value here never overrides the
// runtime's own continuity guards.
constexpr double kPi = 3.14159265358979323846;
constexpr double kStepAmplitude = 0.05;  // rad at unit forward velocity
constexpr double kStepFrequencyHz = 0.5;
constexpr double kLateralPhase = 0.8;  // rad of left/right bias at unit lateral
constexpr double kYawPhase = 0.8;      // rad of common offset at unit yaw

// Set by the SIGINT/SIGTERM handler and read by the loop. The handler does
// nothing else - no logging, no allocation, no DDS call - which is why it may
// only touch this flag. std::atomic is required to be lock free for that to
// hold, which the assertion below pins.
std::atomic<bool> g_stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "the stop flag must be usable from a signal handler");

extern "C" void HandleStopSignal(int signal_number) {
  static_cast<void>(signal_number);
  g_stop_requested.store(true, std::memory_order_relaxed);
}

bool InstallStopHandlers() {
  struct sigaction action {};
  action.sa_handler = HandleStopSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  return ::sigaction(SIGINT, &action, nullptr) == 0 &&
         ::sigaction(SIGTERM, &action, nullptr) == 0;
}

TimeUs MonotonicUs() {
  return static_cast<TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::string IdHex(const Id &id) {
  constexpr char kHex[] = "0123456789abcdef";
  std::string text(2 * id.size(), '0');
  for (std::size_t index = 0; index < id.size(); ++index) {
    text[2 * index] = kHex[(id[index] >> 4) & 0x0f];
    text[2 * index + 1] = kHex[id[index] & 0x0f];
  }
  return text;
}

bool ParseHexId(std::string_view text, Id *id) {
  if (text.size() != 2 * id->size()) {
    return false;
  }
  for (std::size_t index = 0; index < id->size(); ++index) {
    unsigned value = 0;
    for (std::size_t nibble = 0; nibble < 2; ++nibble) {
      const unsigned char byte =
          static_cast<unsigned char>(text[2 * index + nibble]);
      if (byte >= '0' && byte <= '9') {
        value = value * 16 + (byte - '0');
      } else if (byte >= 'a' && byte <= 'f') {
        value = value * 16 + (byte - 'a' + 10);
      } else if (byte >= 'A' && byte <= 'F') {
        value = value * 16 + (byte - 'A' + 10);
      } else {
        return false;
      }
    }
    (*id)[index] = static_cast<std::uint8_t>(value);
  }
  return true;
}

// The identity of this process. It comes from the system random source like the
// runtime's session ids, with the process id and the clock as the fallback, and
// it is never the all-zero "none" identity: the supervisor refuses a producer
// without one because it could not tell it apart from another.
Id MakeProducerId() {
  Id id{};
  bool filled = false;
  try {
    std::random_device entropy;
    std::size_t index = 0;
    while (index < id.size()) {
      const std::random_device::result_type value = entropy();
      for (std::size_t byte = 0; byte < sizeof(value) && index < id.size();
           ++byte, ++index) {
        id[index] = static_cast<std::uint8_t>((value >> (8U * byte)) & 0xFFU);
      }
    }
    filled = true;
  } catch (const std::exception &) {
    // `filled` stays false: the fallback below takes over.
  }
  if (!filled || id == kNoId) {
    std::uint64_t mixed =
        (static_cast<std::uint64_t>(::getpid()) << 32) ^ MonotonicUs();
    for (std::size_t index = 0; index < id.size(); ++index) {
      mixed = mixed * 6364136223846793005ULL + 1442695040888963407ULL;
      id[index] = static_cast<std::uint8_t>((mixed >> 33) & 0xFFU);
    }
  }
  if (id == kNoId) {
    id[0] = 1;
  }
  return id;
}

// Pins the participant to the interface the runtime binds, exactly like the
// transport does: both processes must meet on one domain and one network.
std::string InterfaceConfig(const std::string &interface_name) {
  return "<CycloneDDS><Domain "
         "id=\"any\"><General><Interfaces><NetworkInterface "
         "name=\"" +
         interface_name + "\"/></Interfaces></General></Domain></CycloneDDS>";
}

const char *StopReasonName(StopReason reason) {
  switch (reason) {
    case StopReason::None:
      return "None";
    case StopReason::Requested:
      return "Requested";
    case StopReason::CommandTimeout:
      return "CommandTimeout";
    case StopReason::SourceStateTimeout:
      return "SourceStateTimeout";
    case StopReason::SupervisorTimeout:
      return "SupervisorTimeout";
  }
  return "Unknown";
}

void Print(const std::string &line) {
  std::fwrite(line.data(), 1, line.size(), stdout);
  std::fputc('\n', stdout);
  std::fflush(stdout);
}

// ---------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------

constexpr std::string_view kUsageLine =
    "usage: external_producer --config PATH [--domain N] [--rate HZ]"
    " [--producer-id HEX] [--duration-us US]";

struct Options {
  std::filesystem::path config_path;
  std::uint32_t domain{1};
  double rate_hz{50.0};
  std::optional<Id> producer_id;
  std::optional<TimeUs> duration_us;
};

struct CommandLine {
  bool ok{false};
  int exit_code{kExitUsage};
  std::string error;
  Options options;
};

CommandLine ParseCommandLine(int argc, char **argv) {
  CommandLine result;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    const bool has_value = index + 1 < argc;
    const auto value = [&]() -> std::string {
      return has_value ? std::string(argv[++index]) : std::string();
    };
    if (argument == "--config") {
      if (!has_value) {
        result.error = "--config requires a path";
        return result;
      }
      result.options.config_path = value();
      continue;
    }
    if (argument == "--domain") {
      if (!has_value) {
        result.error = "--domain requires a value";
        return result;
      }
      const std::string text = value();
      std::size_t consumed = 0;
      std::uint64_t domain = 0;
      try {
        domain = std::stoull(text, &consumed);
      } catch (const std::exception &) {
        result.error = "--domain must be an integer, got \"" + text + "\"";
        return result;
      }
      if (consumed != text.size() ||
          domain > irmv::configuration::kMaxDdsDomain) {
        result.error = "--domain must be between 0 and " +
                       std::to_string(irmv::configuration::kMaxDdsDomain) +
                       ", got \"" + text + "\"";
        return result;
      }
      result.options.domain = static_cast<std::uint32_t>(domain);
      continue;
    }
    if (argument == "--rate") {
      if (!has_value) {
        result.error = "--rate requires a value";
        return result;
      }
      const std::string text = value();
      try {
        result.options.rate_hz = std::stod(text);
      } catch (const std::exception &) {
        result.error = "--rate must be a number, got \"" + text + "\"";
        return result;
      }
      if (!(result.options.rate_hz > 0.0) ||
          !std::isfinite(result.options.rate_hz)) {
        result.error = "--rate must be finite and positive";
        return result;
      }
      continue;
    }
    if (argument == "--producer-id") {
      if (!has_value) {
        result.error = "--producer-id requires a hex identity";
        return result;
      }
      Id id{};
      if (!ParseHexId(value(), &id) || id == kNoId) {
        result.error = "--producer-id must be 32 hex digits and non-zero";
        return result;
      }
      result.options.producer_id = id;
      continue;
    }
    if (argument == "--duration-us") {
      if (!has_value) {
        result.error = "--duration-us requires a value";
        return result;
      }
      const std::string text = value();
      std::size_t consumed = 0;
      std::uint64_t duration = 0;
      try {
        duration = std::stoull(text, &consumed);
      } catch (const std::exception &) {
        result.error = "--duration-us must be an integer, got \"" + text + "\"";
        return result;
      }
      if (consumed != text.size() || duration == 0) {
        result.error =
            "--duration-us must be a positive integer, got \"" + text + "\"";
        return result;
      }
      result.options.duration_us = static_cast<TimeUs>(duration);
      continue;
    }
    result.error = "unknown argument \"" + std::string(argument) + "\"";
    return result;
  }

  if (result.options.config_path.empty()) {
    result.error = "--config is required";
    return result;
  }
  result.ok = true;
  result.exit_code = kExitOk;
  return result;
}

// ---------------------------------------------------------------------------
// The process
// ---------------------------------------------------------------------------

// One external producer over real DDS. It owns one participant, the six
// lifecycle endpoints and the two execution endpoints it uses, and one thread -
// this one - polls them all. Nothing waits on DDS without a bound: every read
// is a non-blocking take and every write is bounded by the finite blocking time
// of its reliable endpoint, so the stop handler and the publish cadence stay
// responsive whatever a peer does.
class ExternalProducer {
 public:
  ExternalProducer(const ApplicationConfig &config, const Options &options,
                   Id own_id, TimeUs period_us)
      : config_(config),
        own_id_(own_id),
        period_us_(period_us),
        duration_us_(options.duration_us),
        participant_(options.domain, dds::domain::qos::DomainParticipantQos(),
                     nullptr, dds::core::status::StatusMask::none(),
                     config.dds().interface.empty()
                         ? std::string()
                         : InterfaceConfig(config.dds().interface)),
        publisher_(participant_),
        subscriber_(participant_),
        announcement_topic_(participant_,
                            std::string(kProducerAnnouncementTopic.name)),
        prepare_request_topic_(participant_,
                               std::string(kProducerPrepareRequestTopic.name)),
        prepare_result_topic_(participant_,
                              std::string(kProducerPrepareResultTopic.name)),
        authorization_topic_(participant_,
                             std::string(kProducerAuthorizationTopic.name)),
        stop_topic_(participant_, std::string(kProducerStopTopic.name)),
        intent_topic_(participant_,
                      std::string(kProducerVelocityIntentTopic.name)),
        command_topic_(participant_, std::string(kJointCommandTopic.name)),
        state_topic_(participant_, std::string(kRobotStateTopic.name)),
        announcement_writer_(
            publisher_, announcement_topic_,
            producer_codec::ToWriterQos(kProducerAnnouncementTopic.qos)),
        prepare_result_writer_(
            publisher_, prepare_result_topic_,
            producer_codec::ToWriterQos(kProducerPrepareResultTopic.qos)),
        command_writer_(publisher_, command_topic_,
                        codec::ToWriterQos(kJointCommandTopic.qos)),
        prepare_request_reader_(
            subscriber_, prepare_request_topic_,
            producer_codec::ToReaderQos(kProducerPrepareRequestTopic.qos)),
        authorization_reader_(
            subscriber_, authorization_topic_,
            producer_codec::ToReaderQos(kProducerAuthorizationTopic.qos)),
        stop_reader_(subscriber_, stop_topic_,
                     producer_codec::ToReaderQos(kProducerStopTopic.qos)),
        intent_reader_(
            subscriber_, intent_topic_,
            producer_codec::ToReaderQos(kProducerVelocityIntentTopic.qos)),
        state_reader_(subscriber_, state_topic_,
                      codec::ToReaderQos(kRobotStateTopic.qos)) {
    // The walk is measured from the target the configuration initializes to:
    // the first frame is a step away from it, and every later frame is clamped
    // against the one before, so the stream never jumps.
    last_q_ = config_.initialization.target_q;
  }

  int Run() {
    start_us_ = MonotonicUs();
    next_announce_us_ = start_us_;
    next_publish_us_ = start_us_;
    for (;;) {
      const TimeUs now_us = MonotonicUs();
      if (g_stop_requested.load(std::memory_order_relaxed) ||
          (duration_us_.has_value() && now_us - start_us_ >= *duration_us_)) {
        return FinishOrderly();
      }
      DrainStates();
      DrainIntents();
      DrainPrepareRequests();
      AnswerPrepare(now_us);
      DrainAuthorizations(now_us);
      DrainStops();
      MaybeAnnounce(now_us);
      MaybePublish(now_us);
      std::this_thread::sleep_for(std::chrono::microseconds(kLoopSleepUs));
    }
  }

 private:
  template <typename Writer, typename Sample>
  bool TryWrite(Writer &writer, const Sample &sample) {
    try {
      writer.write(sample);
      return true;
    } catch (const std::exception &) {
      // A reliable writer refuses a sample when its history stays full past the
      // blocking time. It is counted and the lifecycle continues.
      ++write_failures_;
      return false;
    }
  }

  template <typename Reader, typename ProtocolT, typename Fn>
  void TakeAndConvert(Reader &reader, Fn convert) {
    try {
      const auto samples =
          reader.select().state(dds::sub::status::DataState::any()).take();
      for (const auto &sample : samples) {
        if (!sample.info().valid()) {
          continue;
        }
        ProtocolT converted;
        if (convert(sample.data(), converted)) {
          Consume(converted);
        } else {
          // A payload the codec refuses is discarded whole; it is reported at
          // exit instead of being applied partially.
          ++rejected_samples_;
        }
      }
    } catch (const std::exception &) {
      ++read_failures_;
    }
  }

  // The newest RobotState this process actually read is the only source
  // sequence a frame may name: a sequence never read describes an observation
  // this process cannot vouch for.
  void DrainStates() {
    TakeAndConvert<decltype(state_reader_), RobotState>(
        state_reader_, [](const wire::RobotState &from, RobotState &to) {
          return codec::FromWire(from, to);
        });
  }

  void Consume(const RobotState &state) {
    if (!have_state_ || state.state_seq > state_seq_) {
      have_state_ = true;
      state_seq_ = state.state_seq;
      runtime_id_ = state.runtime_id;
    }
  }

  // The body-velocity intent the operator holds. Only this process's intents
  // matter and only an increasing sequence is a new intent: a repeated or older
  // one carries no information at all.
  void DrainIntents() {
    TakeAndConvert<decltype(intent_reader_), VelocityIntentMessage>(
        intent_reader_,
        [](const wire::VelocityIntentMessage &from, VelocityIntentMessage &to) {
          return producer_codec::FromWire(from, to);
        });
  }

  void Consume(const VelocityIntentMessage &intent) {
    if (intent.producer_process_id != own_id_) {
      return;
    }
    if (have_intent_ && intent.seq <= last_intent_seq_) {
      return;
    }
    AdoptIntent(intent);
  }

  void AdoptIntent(const VelocityIntentMessage &intent) {
    intent_ = intent;
    intent_stamp_us_ = intent.send_stamp_us;
    intent_valid_for_us_ = intent.valid_for_us;
    have_intent_ = true;
    if (intent.seq > last_intent_seq_) {
      last_intent_seq_ = intent.seq;
    }
  }

  // Only a preparation of this process is this process's to answer; another
  // producer's request is another registration's and is left alone.
  void DrainPrepareRequests() {
    TakeAndConvert<decltype(prepare_request_reader_), PrepareRequest>(
        prepare_request_reader_,
        [](const wire::PrepareRequest &from, PrepareRequest &to) {
          return producer_codec::FromWire(from, to);
        });
  }

  void Consume(const PrepareRequest &request) {
    if (request.producer_process_id != own_id_) {
      return;
    }
    pending_prepare_ = request;
    Print("producer=preparing prepare_id=" +
          std::to_string(request.prepare_id));
  }

  // The answer waits until at least one state was read: a first frame without a
  // source sequence this process saw is not one the application may attach to
  // an observation.
  void AnswerPrepare(TimeUs now_us) {
    if (!pending_prepare_.has_value() || !have_state_) {
      return;
    }
    const PrepareRequest request = *pending_prepare_;
    pending_prepare_.reset();
    // The intent travels with the request: the first frame is prepared for the
    // velocity the application asked it to track, not for the last one seen.
    AdoptIntent(request.intent);

    PrepareResult result;
    result.producer_process_id = own_id_;
    result.prepare_id = request.prepare_id;
    result.runtime_id = request.runtime_id;
    result.session_id = request.session_id;
    result.producer_id = request.producer_id;
    result.epoch = request.epoch;
    result.status = ProducerStatus::Ready;
    result.reason = ProducerFailureReason::None;
    // The frame is built with the identity this process generated it under.
    // The authorization is stamped by the application, so the epoch and the
    // command sequence stay zero here.
    FillTarget(&result.first_command, 0, 0, now_us);

    wire::PrepareResult sample;
    producer_codec::ToWire(result, sample);
    if (TryWrite(prepare_result_writer_, sample)) {
      last_prepare_id_ = request.prepare_id;
      Print(
          "producer=prepared prepare_id=" + std::to_string(request.prepare_id) +
          " source_seq=" + std::to_string(state_seq_));
    }
  }

  // The authorization of the preparation this process last answered is what
  // lets it publish. The notice is a notice, not the source of its rights: the
  // runtime refuses a frame of a withdrawn epoch all the same.
  void DrainAuthorizations(TimeUs now_us) {
    TakeAndConvert<decltype(authorization_reader_), AuthorizationNotice>(
        authorization_reader_,
        [](const wire::AuthorizationNotice &from, AuthorizationNotice &to) {
          return producer_codec::FromWire(from, to);
        });
  }

  void Consume(const AuthorizationNotice &notice) {
    if (notice.producer_process_id != own_id_ || last_prepare_id_ == 0 ||
        notice.prepare_id != last_prepare_id_) {
      return;
    }
    authorized_ = true;
    grant_epoch_ = notice.epoch;
    grant_producer_ = notice.producer_id;
    // The routing identity of the context: the runtime's command gate drops a
    // frame whose runtime, session or epoch does not name the context it
    // granted, and these two are the notice's - a producer that published
    // frames without them would be publishing into nothing, silently, which is
    // exactly what the runtime's target deadline would then report.
    grant_runtime_id_ = notice.runtime_id;
    grant_session_id_ = notice.session_id;
    // The runtime adopted the application's first frame at first_command_seq;
    // this stream continues it, so numbering restarts at the one after it
    // rather than at one of its own.
    next_command_seq_ = notice.first_command_seq + 1;
    announce_active_ = false;
    next_publish_us_ = MonotonicUs();
    Print("producer=authorized epoch=" + std::to_string(grant_epoch_) +
          " first_seq=" + std::to_string(notice.first_command_seq));
  }

  // A stop notice that names this producer stops the publication at once. It is
  // not an instruction to exit: a later preparation may authorize a new
  // context, so the process goes on reading.
  void DrainStops() {
    TakeAndConvert<decltype(stop_reader_), StopNotice>(
        stop_reader_, [](const wire::StopNotice &from, StopNotice &to) {
          return producer_codec::FromWire(from, to);
        });
  }

  void Consume(const StopNotice &notice) {
    if (!NamesUs(notice)) {
      return;
    }
    announce_active_ = false;
    if (authorized_) {
      authorized_ = false;
      Print(std::string("producer=stopped reason=") +
            StopReasonName(notice.reason));
    }
  }

  bool NamesUs(const StopNotice &notice) const {
    return notice.producer_process_id == own_id_ ||
           notice.producer_id == own_id_ ||
           (authorized_ && notice.producer_id == grant_producer_);
  }

  // A registration the application missed must be recoverable without a
  // restart, so the announcement repeats until it is authorized or stopped.
  void MaybeAnnounce(TimeUs now_us) {
    if (!announce_active_ || authorized_ || now_us < next_announce_us_) {
      return;
    }
    next_announce_us_ = now_us + kAnnounceIntervalUs;

    ProducerAnnouncement announcement;
    announcement.producer_process_id = own_id_;
    announcement.runtime_id = runtime_id_;
    announcement.layout_id.Assign(config_.layout_id());
    announcement.target_period_us = period_us_;
    announcement.reads_robot_state = true;
    announcement.status = ProducerStatus::Ready;
    announcement.reason = ProducerFailureReason::None;

    wire::ProducerAnnouncement sample;
    producer_codec::ToWire(announcement, sample);
    if (TryWrite(announcement_writer_, sample) && !announced_) {
      announced_ = true;
      Print("producer=announced id=" + IdHex(own_id_));
    }
  }

  // One frame per round period, only while an authorization is held and only
  // from a state this process read. The sequence strictly increases from
  // first_command_seq + 1; a restart of it would be a discontinuity the command
  // gate refuses.
  void MaybePublish(TimeUs now_us) {
    if (!authorized_ || !have_state_ || now_us < next_publish_us_) {
      return;
    }
    next_publish_us_ = now_us + period_us_;
    JointCommand frame;
    FillTarget(&frame, grant_epoch_, next_command_seq_, now_us);
    wire::JointCommand sample;
    codec::ToWire(frame, sample);
    if (TryWrite(command_writer_, sample)) {
      Print("producer=publishing seq=" + std::to_string(next_command_seq_));
      ++next_command_seq_;
    }
  }

  // The target: the initialization position as the base plus a per-joint
  // sinusoidal step whose amplitude is the forward intent, whose phases the
  // lateral intent biases by side and the yaw intent offsets as a whole. Every
  // frame is clamped to the configuration's continuity delta and target
  // velocity limit, so consecutive frames stay inside the guards the runtime
  // applies to them anyway, and no value is ever non-finite.
  void FillTarget(JointCommand *frame, Sequence epoch, Sequence command_seq,
                  TimeUs now_us) {
    const std::size_t count = config_.joint_count();
    // The whole routing identity the runtime's command gate checks: the
    // context it granted is named by the runtime, the session, the producer and
    // the epoch together, and any one of them missing makes every frame of this
    // stream an ignored sample.
    // A prepared first frame is not published - it travels inside the
    // application's Tracking request, which completes its routing identity - so
    // it names no context yet: the identity below is the one the runtime
    // granted, and it is this process's own before there is one.
    frame->runtime_id = authorized_ ? grant_runtime_id_ : kNoId;
    frame->session_id = authorized_ ? grant_session_id_ : kNoId;
    frame->producer_id = authorized_ ? grant_producer_ : own_id_;
    frame->epoch = epoch;
    frame->command_seq = command_seq;
    frame->source_state_seq = state_seq_;
    frame->stamp_us = now_us;
    frame->layout_id.Assign(config_.layout_id());
    frame->joints.count = static_cast<std::uint32_t>(count);

    double forward = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;
    IntentVelocity(now_us, &forward, &lateral, &yaw);

    const double amplitude = kStepAmplitude * forward;
    const double omega = 2.0 * kPi * kStepFrequencyHz;
    const double elapsed_s = static_cast<double>(now_us - start_us_) / 1e6;
    const double phase_base = omega * elapsed_s;
    for (std::size_t index = 0; index < count; ++index) {
      const double side = (index % 2 == 0) ? 1.0 : -1.0;
      const double phase =
          phase_base + side * kLateralPhase * lateral + kYawPhase * yaw;
      double q =
          config_.initialization.target_q[index] + amplitude * std::sin(phase);
      double dq = amplitude * omega * std::cos(phase);
      if (!std::isfinite(q)) {
        q = config_.initialization.target_q[index];
      }
      if (!std::isfinite(dq)) {
        dq = 0.0;
      }
      q = std::min(std::max(q, config_.runtime.position_min[index]),
                   config_.runtime.position_max[index]);
      const double velocity_limit =
          config_.runtime.target_velocity_limits[index];
      dq = std::min(std::max(dq, -velocity_limit), velocity_limit);
      // The per-frame step, measured against the frame before this one, never
      // exceeds the continuity delta of the joint.
      const double step_limit =
          config_.runtime.continuity.position_delta[index];
      const double delta = q - last_q_[index];
      if (delta > step_limit) {
        q = last_q_[index] + step_limit;
      } else if (delta < -step_limit) {
        q = last_q_[index] - step_limit;
      }
      last_q_[index] = q;
      frame->joints.q[index] = q;
      frame->joints.dq[index] = dq;
      frame->joints.tau_ff[index] = 0.0;
      frame->joints.kp[index] = config_.initialization.gains.kp[index];
      frame->joints.kd[index] = config_.initialization.gains.kd[index];
    }
  }

  // The newest intent, or zero where there is none: an intent older than its
  // validity period is not a velocity to track, and each component is bounded
  // to the unit interval so a stray intent cannot enlarge the walk.
  void IntentVelocity(TimeUs now_us, double *forward, double *lateral,
                      double *yaw) const {
    *forward = 0.0;
    *lateral = 0.0;
    *yaw = 0.0;
    if (!have_intent_ || intent_valid_for_us_ == 0) {
      return;
    }
    const TimeUs age =
        now_us >= intent_stamp_us_ ? now_us - intent_stamp_us_ : 0;
    if (age > intent_valid_for_us_) {
      return;
    }
    *forward = std::min(std::max(intent_.forward, -1.0), 1.0);
    *lateral = std::min(std::max(intent_.lateral, -1.0), 1.0);
    *yaw = std::min(std::max(intent_.yaw, -1.0), 1.0);
    if (!std::isfinite(*forward)) {
      *forward = 0.0;
    }
    if (!std::isfinite(*lateral)) {
      *lateral = 0.0;
    }
    if (!std::isfinite(*yaw)) {
      *yaw = 0.0;
    }
  }

  int FinishOrderly() {
    if (authorized_) {
      authorized_ = false;
      // The process was asked to end while it was publishing: the stream stops
      // as it would on a stop notice, and the exit follows.
      Print("producer=stopped reason=Requested");
    }
    Print("producer=exiting");
    if (write_failures_ != 0 || read_failures_ != 0 || rejected_samples_ != 0) {
      std::fprintf(stderr,
                   "external_producer: refusals write_failures=%llu "
                   "read_failures=%llu rejected_samples=%llu\n",
                   static_cast<unsigned long long>(write_failures_),
                   static_cast<unsigned long long>(read_failures_),
                   static_cast<unsigned long long>(rejected_samples_));
    }
    return kExitOk;
  }

  const ApplicationConfig &config_;
  Id own_id_{};
  TimeUs period_us_{0};
  std::optional<TimeUs> duration_us_;

  dds::domain::DomainParticipant participant_;
  dds::pub::Publisher publisher_;
  dds::sub::Subscriber subscriber_;
  dds::topic::Topic<wire::ProducerAnnouncement> announcement_topic_;
  dds::topic::Topic<wire::PrepareRequest> prepare_request_topic_;
  dds::topic::Topic<wire::PrepareResult> prepare_result_topic_;
  dds::topic::Topic<wire::AuthorizationNotice> authorization_topic_;
  dds::topic::Topic<wire::StopNotice> stop_topic_;
  dds::topic::Topic<wire::VelocityIntentMessage> intent_topic_;
  dds::topic::Topic<wire::JointCommand> command_topic_;
  dds::topic::Topic<wire::RobotState> state_topic_;
  dds::pub::DataWriter<wire::ProducerAnnouncement> announcement_writer_;
  dds::pub::DataWriter<wire::PrepareResult> prepare_result_writer_;
  dds::pub::DataWriter<wire::JointCommand> command_writer_;
  dds::sub::DataReader<wire::PrepareRequest> prepare_request_reader_;
  dds::sub::DataReader<wire::AuthorizationNotice> authorization_reader_;
  dds::sub::DataReader<wire::StopNotice> stop_reader_;
  dds::sub::DataReader<wire::VelocityIntentMessage> intent_reader_;
  dds::sub::DataReader<wire::RobotState> state_reader_;

  // The walk's state: the last frame this process produced, and the monotonic
  // origin its phase is measured from.
  std::array<double, irmv::protocol::kMaxJoints> last_q_{};
  TimeUs start_us_{0};

  // The lifecycle state.
  bool announce_active_{true};
  bool announced_{false};
  TimeUs next_announce_us_{0};
  TimeUs next_publish_us_{0};
  bool have_state_{false};
  Sequence state_seq_{0};
  Id runtime_id_{};
  bool have_intent_{false};
  VelocityIntentMessage intent_{};
  TimeUs intent_stamp_us_{0};
  TimeUs intent_valid_for_us_{0};
  Sequence last_intent_seq_{0};
  std::optional<PrepareRequest> pending_prepare_;
  Sequence last_prepare_id_{0};
  bool authorized_{false};
  Id grant_producer_{};
  Id grant_runtime_id_{};
  Id grant_session_id_{};
  Sequence grant_epoch_{0};
  Sequence next_command_seq_{0};

  // Every refusal is counted so the exit can report it.
  std::uint64_t write_failures_{0};
  std::uint64_t read_failures_{0};
  std::uint64_t rejected_samples_{0};
};

}  // namespace

int main(int argc, char **argv) {
  // The whole command line is validated before anything else: a rejected
  // invocation touches no DDS entity and no configuration file. A malformed
  // --producer-id is refused here, which is as early as the refusal can be.
  const CommandLine command_line = ParseCommandLine(argc, argv);
  if (!command_line.ok) {
    std::fprintf(stderr, "external_producer: %s\n%.*s\n",
                 command_line.error.c_str(),
                 static_cast<int>(kUsageLine.size()), kUsageLine.data());
    return command_line.exit_code;
  }
  if (!InstallStopHandlers()) {
    std::fprintf(stderr,
                 "external_producer: cannot install the stop handlers\n");
    return kExitStartup;
  }

  // The configuration is loaded before any endpoint exists, so a missing or
  // unreadable file is a diagnostic and not a half-built process.
  ApplicationConfig config;
  try {
    config = irmv::configuration::LoadApplicationConfig(
        command_line.options.config_path);
  } catch (const std::exception &error) {
    std::fprintf(stderr, "external_producer: cannot load %s: %s\n",
                 command_line.options.config_path.c_str(), error.what());
    return kExitStartup;
  }

  const Id own_id = command_line.options.producer_id.value_or(MakeProducerId());
  if (own_id == kNoId) {
    std::fprintf(stderr, "external_producer: refusing the all-zero identity\n");
    return kExitStartup;
  }

  // The round period the announcement declares. It must be strictly below the
  // runtime's command timeout, or the supervisor refuses the announcement and
  // no frame such a producer sent would ever keep a context alive. A rate this
  // fast cannot be expressed in whole microseconds either.
  const double period = 1e6 / command_line.options.rate_hz;
  if (!std::isfinite(period) || period < 1.0) {
    std::fprintf(stderr,
                 "external_producer: --rate %g is beyond a 1 us "
                 "period\n",
                 command_line.options.rate_hz);
    return kExitStartup;
  }
  const TimeUs period_us = static_cast<TimeUs>(std::llround(period));
  if (config.runtime.timeouts.command_us == 0 ||
      period_us >= config.runtime.timeouts.command_us) {
    std::fprintf(
        stderr,
        "external_producer: --rate %g gives a %llu us period, not "
        "strictly below the runtime command timeout of %llu us\n",
        command_line.options.rate_hz,
        static_cast<unsigned long long>(period_us),
        static_cast<unsigned long long>(config.runtime.timeouts.command_us));
    return kExitStartup;
  }

  try {
    ExternalProducer producer(config, command_line.options, own_id, period_us);
    return producer.Run();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "external_producer: internal error: %s\n",
                 error.what());
    return kExitStartup;
  } catch (...) {
    std::fprintf(stderr, "external_producer: internal error: unknown\n");
    return kExitStartup;
  }
}
