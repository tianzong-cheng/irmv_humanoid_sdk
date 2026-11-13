#include "irmv/application/application_loop.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <thread>

#include "irmv/application/behavior_fsm.hpp"
#include "irmv/application/input_router.hpp"
#include "irmv/application/policy_session_record.hpp"
#include "irmv/application/producer_supervisor.hpp"
#include "irmv/application/robot_client.hpp"
#include "irmv/build_identity.hpp"
#include "irmv/configuration/application_config.hpp"
#include "irmv/policy/action.hpp"
#include "irmv/policy/inference.hpp"
#include "irmv/policy/observation.hpp"
#include "irmv/policy/onnx_model.hpp"
#include "irmv/simulation/application_driver.hpp"
#include "irmv/telemetry/record_context.hpp"
#include "irmv/telemetry/velocity_record.hpp"
#include "irmv/transport/client_transport.hpp"
#include "irmv/transport/producer_transport.hpp"

namespace irmv::application {

namespace {

using Clock = std::chrono::steady_clock;

// Bytes read from the terminal in one iteration, and how many reads one
// iteration performs: bounded so a stream nobody drains cannot starve the
// supervision checks, generous enough for a burst of keystrokes.
constexpr std::size_t kReadChunk = 64;
constexpr int kReadAttempts = 64;

// ---------------------------------------------------------------------------
// Names of the enumerations of the status line
// ---------------------------------------------------------------------------

const char *BehaviorName(BehaviorState state) {
  switch (state) {
    case BehaviorState::Starting:
      return "Starting";
    case BehaviorState::Standby:
      return "Standby";
    case BehaviorState::Damping:
      return "Damping";
    case BehaviorState::Initializing:
      return "Initializing";
    case BehaviorState::Ready:
      return "Ready";
    case BehaviorState::Locomotion:
      return "Locomotion";
    case BehaviorState::Fault:
      return "Fault";
  }
  return "Unknown";
}

const char *StateName(protocol::ExecutionState state) {
  switch (state) {
    case protocol::ExecutionState::Boot:
      return "Boot";
    case protocol::ExecutionState::ZeroTorque:
      return "ZeroTorque";
    case protocol::ExecutionState::Damping:
      return "Damping";
    case protocol::ExecutionState::Tracking:
      return "Tracking";
    case protocol::ExecutionState::Fault:
      return "Fault";
  }
  return "Unknown";
}

const char *OutputName(protocol::OutputAction action) {
  switch (action) {
    case protocol::OutputAction::ZeroTorque:
      return "ZeroTorque";
    case protocol::OutputAction::Damping:
      return "Damping";
    case protocol::OutputAction::Tracking:
      return "Tracking";
  }
  return "Unknown";
}

const char *OperationName(protocol::Operation operation) {
  switch (operation) {
    case protocol::Operation::AcquireSession:
      return "AcquireSession";
    case protocol::Operation::Damping:
      return "Damping";
    case protocol::Operation::ZeroTorque:
      return "ZeroTorque";
    case protocol::Operation::Tracking:
      return "Tracking";
    case protocol::Operation::Reset:
      return "Reset";
  }
  return "-";
}

// The upper layer's own latch. "-" while it holds none, so the display never
// reports a cause for a behaviour that is not Faulted.
const char *FaultCauseName(FaultCause cause) {
  switch (cause) {
    case FaultCause::None:
      return "-";
    case FaultCause::OperatorEmergencyStop:
      return "OperatorEmergencyStop";
    case FaultCause::RuntimeFault:
      return "RuntimeFault";
    case FaultCause::RuntimeUnreachable:
      return "RuntimeUnreachable";
    case FaultCause::RuntimeRestarted:
      return "RuntimeRestarted";
    case FaultCause::InitializationFailed:
      return "InitializationFailed";
    case FaultCause::RequestTimeout:
      return "RequestTimeout";
    case FaultCause::UnexpectedContext:
      return "UnexpectedContext";
  }
  return "-";
}

// The lower layer's own reason for leaving tracking, as the last trustworthy
// state reported it. "-" means it reported none (or is not known), never that a
// stop happened.
const char *StopReasonName(protocol::StopReason reason) {
  switch (reason) {
    case protocol::StopReason::None:
      return "-";
    case protocol::StopReason::Requested:
      return "Requested";
    case protocol::StopReason::CommandTimeout:
      return "CommandTimeout";
    case protocol::StopReason::SourceStateTimeout:
      return "SourceStateTimeout";
    case protocol::StopReason::SupervisorTimeout:
      return "SupervisorTimeout";
  }
  return "-";
}

const char *RejectName(protocol::RejectReason reason) {
  switch (reason) {
    case protocol::RejectReason::None:
      return "-";
    case protocol::RejectReason::VersionMismatch:
      return "VersionMismatch";
    case protocol::RejectReason::Malformed:
      return "Malformed";
    case protocol::RejectReason::WrongInstance:
      return "WrongInstance";
    case protocol::RejectReason::WrongSession:
      return "WrongSession";
    case protocol::RejectReason::SessionBusy:
      return "SessionBusy";
    case protocol::RejectReason::StaleRequest:
      return "StaleRequest";
    case protocol::RejectReason::RequestConflict:
      return "RequestConflict";
    case protocol::RejectReason::EpochMismatch:
      return "EpochMismatch";
    case protocol::RejectReason::InvalidTransition:
      return "InvalidTransition";
    case protocol::RejectReason::InvalidCommand:
      return "InvalidCommand";
    case protocol::RejectReason::StaleSource:
      return "StaleSource";
    case protocol::RejectReason::Discontinuity:
      return "Discontinuity";
    case protocol::RejectReason::RecoveryNotReady:
      return "RecoveryNotReady";
    case protocol::RejectReason::FaultMismatch:
      return "FaultMismatch";
    case protocol::RejectReason::WriteFailed:
      return "WriteFailed";
    case protocol::RejectReason::QueueFull:
      return "QueueFull";
  }
  return "-";
}

// ---------------------------------------------------------------------------
// The external producer source
// ---------------------------------------------------------------------------

// Why a producer stopped being the selected one, as the line reports it.
const char *ProducerStopName(ProducerStop stop) {
  switch (stop) {
    case ProducerStop::None:
      return "-";
    case ProducerStop::Stopped:
      return "Stopped";
    case ProducerStop::PrepareFailed:
      return "PrepareFailed";
    case ProducerStop::PrepareTimeout:
      return "PrepareTimeout";
    case ProducerStop::Replaced:
      return "Replaced";
    case ProducerStop::Incompatible:
      return "Incompatible";
  }
  return "-";
}

// The lower-case hexadecimal rendering of one protocol identity. The operator
// compares these lines with the producer's own log and with the runtime's
// trace, both of which name the same bytes the same way.
std::array<char, 33> IdText(const protocol::Id &id) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::array<char, 33> text{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    text[index * 2] = kDigits[id[index] >> 4];
    text[index * 2 + 1] = kDigits[id[index] & 0x0F];
  }
  return text;
}

// ---------------------------------------------------------------------------
// Clock and pacing
// ---------------------------------------------------------------------------

protocol::TimeUs MonotonicUs() {
  const auto elapsed = Clock::now().time_since_epoch();
  return static_cast<protocol::TimeUs>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

// The loop period: the shortest period the behaviour asks for, so neither a
// target nor a heartbeat can be late because the iteration was waiting for the
// other one. The configuration guarantees positive periods; the guard keeps a
// hand-written one from turning the loop into a spin.
protocol::TimeUs LoopPeriodUs(const configuration::ApplicationConfig &config) {
  const protocol::TimeUs target = config.supervision.target_period_us;
  const protocol::TimeUs heartbeat = config.supervision.heartbeat_period_us;
  const protocol::TimeUs period = target < heartbeat ? target : heartbeat;
  return period == 0 ? 1 : period;
}

// Reads everything the terminal has, bounded per iteration. Returns true when
// the input device is lost, which is an EOF or a terminal error and asks for
// the ordered exit - never for an emergency stop and never for a claim that the
// hardware stopped.
//
// Precondition: STDIN_FILENO is already O_NONBLOCK (main's terminal guard set
// it). That flag is what makes "nothing to read" return EAGAIN instead of
// waiting for a byte, so this read, the pacing poll at the end of the iteration
// and the supervision heartbeat stay independent of the typing hand.
bool ReadTerminal(InputRouter *router, std::array<char, kReadChunk> *buffer,
                  protocol::TimeUs now_us) {
  for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
    const ssize_t count = ::read(STDIN_FILENO, buffer->data(), buffer->size());
    if (count > 0) {
      for (ssize_t index = 0; index < count; ++index) {
        const char key = (*buffer)[static_cast<std::size_t>(index)];
        // A terminal byte stream carries presses; a release of a one-shot key
        // is the absence of a byte, which InputRouter's repeat guard already
        // treats as one event per press.
        static_cast<void>(router->OnKey(key, true, now_us));
      }
      continue;
    }
    if (count == 0) {
      router->OnInputLost();
      return true;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    router->OnInputLost();
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Transport adapter
// ---------------------------------------------------------------------------

// The three forwards of ClientSink to the transport, plus the one display fact
// the client does not expose: the operation of the request that was handed over
// last. irmv_transport must not depend on irmv_application, so ClientTransport
// cannot implement ClientSink itself, and the application executable is the
// only place that knows both.
class StageSink final : public ClientSink {
 public:
  explicit StageSink(transport::ClientTransport &transport,
                     simulation::ApplicationDriver *simulation = nullptr)
      : transport_(transport), simulation_(simulation) {}

  bool PublishRequest(const protocol::ExecutionRequest &request) override {
    // Recorded before the lane decides: the display shows the operation of the
    // request the client is waiting on, whether or not the transport could
    // queue that copy of it.
    last_operation_ = request.operation;
    have_operation_ = true;
    return simulation_ ? simulation_->PublishRequest(request)
                       : transport_.PublishRequest(request);
  }

  bool PublishCommand(const protocol::JointCommand &command) override {
    return simulation_ ? simulation_->PublishCommand(command)
                       : transport_.PublishCommand(command);
  }

  bool PublishHeartbeat(
      const protocol::SupervisorHeartbeat &heartbeat) override {
    return simulation_ ? simulation_->PublishHeartbeat(heartbeat)
                       : transport_.PublishHeartbeat(heartbeat);
  }

  const char *PendingOperation() const {
    return have_operation_ ? OperationName(last_operation_) : "-";
  }

 private:
  transport::ClientTransport &transport_;
  simulation::ApplicationDriver *simulation_;
  protocol::Operation last_operation_{protocol::Operation::AcquireSession};
  bool have_operation_{false};
};

// ---------------------------------------------------------------------------
// Status line
// ---------------------------------------------------------------------------

// One rendering of the status line. Every field is a name of a frozen
// enumeration, so the line is compared field by field and nothing on this path
// allocates.
struct StatusFields {
  const char *behavior{"Unknown"};
  const char *runtime{"Unknown"};
  const char *output{"Unknown"};
  const char *session{"none"};
  const char *pending{"-"};
  const char *reject{"-"};
  const char *fresh{"no"};
  const char *init{"-"};
  // The upper layer's latched cause and the lower layer's own stop reason:
  // without them an operator cannot tell an operator emergency stop from an
  // unreachable runtime, or a lower command timeout from a supervisor timeout.
  const char *cause{"-"};
  const char *stop{"-"};
};

bool SameFields(const StatusFields &left, const StatusFields &right) {
  return std::strcmp(left.behavior, right.behavior) == 0 &&
         std::strcmp(left.runtime, right.runtime) == 0 &&
         std::strcmp(left.output, right.output) == 0 &&
         std::strcmp(left.session, right.session) == 0 &&
         std::strcmp(left.pending, right.pending) == 0 &&
         std::strcmp(left.reject, right.reject) == 0 &&
         std::strcmp(left.fresh, right.fresh) == 0 &&
         std::strcmp(left.init, right.init) == 0 &&
         std::strcmp(left.cause, right.cause) == 0 &&
         std::strcmp(left.stop, right.stop) == 0;
}

void PrintStatus(const StatusFields &fields) {
  // Sized for the longest names of every field with room to spare; a line that
  // did not fit would be truncated, which the length clamp below turns into a
  // shorter line rather than an unterminated one.
  std::array<char, 320> line{};
  const int written = std::snprintf(
      line.data(), line.size(),
      "[application] behavior=%s runtime=%s output=%s session=%s pending=%s "
      "reject=%s fresh=%s init=%s cause=%s stop=%s\n",
      fields.behavior, fields.runtime, fields.output, fields.session,
      fields.pending, fields.reject, fields.fresh, fields.init, fields.cause,
      fields.stop);
  if (written <= 0) {
    return;
  }
  const std::size_t length =
      std::min<std::size_t>(static_cast<std::size_t>(written), line.size() - 1);
  static_cast<void>(std::fwrite(line.data(), 1, length, stdout));
  // A harness reads the pipe while the process runs: nothing may sit in a
  // buffer until the exit.
  static_cast<void>(std::fflush(stdout));
}

const char *SessionField(const RobotClient &client, const ClientView &view) {
  if (view.session_owned) {
    return "owned";
  }
  if (view.session_lost || client.session_id() != protocol::kNoId) {
    return "lost";
  }
  return "none";
}

// The pending request of the display: the operation of the request the client
// is waiting on, and nothing while none is pending.
void PendingField(const RobotClient &client, const StageSink &sink,
                  StatusFields *fields) {
  fields->pending = client.request_pending() ? sink.PendingOperation() : "-";
}

// ---------------------------------------------------------------------------
// Startup and the supervision loop
// ---------------------------------------------------------------------------

int LoadConfiguration(const ApplicationOptions &options,
                      configuration::ApplicationConfig *config) {
  configuration::ConfigOverrides overrides;
  if (options.dds_domain) {
    overrides.dds_domain = *options.dds_domain;
  }
  try {
    *config =
        configuration::LoadApplicationConfig(options.config_path, overrides);
  } catch (const std::exception &error) {
    // No claim about hardware, a device or the lower process: the invocation,
    // the file or a field of it was refused, and the process leaves before the
    // transport - the first thing that touches DDS - is constructed.
    std::fprintf(stderr, "application: %s\n", error.what());
    return kExitStartupError;
  }
  return kExitOk;
}

}  // namespace

// The metadata of one application run's velocity directory. It is what makes
// the directory self-describing: the schema identity, the build revision, the
// two configuration digests, the clock the stamps are taken from, the sampling
// rate, the units of the three values, and the explicit model status. The model
// block never invents a name: a local run names the file name of the model that
// was actually loaded, while an external or sourceless run states that absence
// with a null name, and a local run without a loaded model says `unloaded`.
std::string BuildVelocityMetadata(
    const configuration::ApplicationConfig &config, LocomotionSource source,
    const policy::OnnxModel *model, const telemetry::RecordClockIdentity &clock,
    const std::string &clock_error, protocol::TimeUs loop_period_us,
    protocol::TimeUs sample_period_us) {
  using nlohmann::json;
  json metadata;
  metadata["schema"] = std::string(telemetry::kVelocitySchema);
  metadata["version"] = telemetry::kVelocitySchemaVersion;
  metadata["sdk_git_sha"] = std::string(irmv::SdkGitSha());
  metadata["application_config_id"] = config.application_config_id;
  metadata["config_id"] = config.runtime_config_id();
  metadata["layout_id"] = config.layout_id();
  metadata["joint_count"] = config.joint_count();
  metadata["sample_hz"] = config.runtime.telemetry.recorder.sample_hz;
  // The application's own pacing, so a reader can see whether the sample
  // deadline or the nominal loop period determined the cadence. Neither is
  // guessed: both come from the configuration and the schedule in effect.
  metadata["loop_period_us"] = loop_period_us;
  metadata["sampling"] = {
      {"paced_by_deadline", loop_period_us > sample_period_us}};
  metadata["source"] =
      source == LocomotionSource::Local
          ? "local"
          : (source == LocomotionSource::External ? "external" : "none");
  metadata["clock"] = {{"clock", clock.clock},
                       {"boot_id", clock.boot_id},
                       {"available", clock.available}};
  if (!clock.available) {
    metadata["clock"]["diagnostic"] = clock_error;
  }
  metadata["units"] = {
      {"forward", "m_s"}, {"lateral", "m_s"}, {"yaw", "rad_s"}, {"time", "us"}};
  json model_info;
  if (source == LocomotionSource::Local && model != nullptr) {
    model_info["status"] = "loaded";
    model_info["name"] =
        std::filesystem::path(config.policy.model_path).filename().string();
    model_info["path"] = config.policy.model_path.string();
    model_info["sha256"] = model->sha256();
    model_info["contract_sha256"] = model->contract_sha256();
  } else {
    // An external source is its own producer and this application loaded no
    // model; a sourceless run has none either. Both are explicit states with no
    // fabricated name or digest.
    model_info["status"] = source == LocomotionSource::Local
                               ? "unloaded"
                               : metadata["source"].get<std::string>();
    model_info["name"] = nullptr;
    model_info["path"] = nullptr;
    model_info["sha256"] = nullptr;
    model_info["contract_sha256"] = nullptr;
  }
  metadata["model"] = std::move(model_info);
  metadata["row"] =
      "one due body-velocity intent sample: the values the application held, "
      "the runtime/session identity and latest state sequence in force, and "
      "the "
      "monotonic host stamp; a due deadline missed by the loop is not replayed";
  return metadata.dump(1);
}

// One line of the session sidecar, written the moment a locomotion context is
// confirmed - the point at which the model, the gains and the identity the
// runtime granted are all known at once. An external context is recorded from
// the first frame the runtime adopted, because the source declared the gains of
// that frame and this application overrode none of them.
void RecordPolicySession(application::PolicySessionRecorder *recorder,
                         LocomotionSource source, const RobotClient &client,
                         const BehaviorFsm &fsm,
                         const policy::ResolvedGains &resolved_gains,
                         const policy::OnnxModel *model,
                         const configuration::ApplicationConfig &config,
                         const protocol::JointCommand &first_frame) {
  if (recorder == nullptr) {
    return;
  }
  application::PolicySessionEvent event;
  event.runtime_id = client.runtime_id();
  event.session_id = client.session_id();
  event.epoch = fsm.locomotion_epoch();
  event.external = source == LocomotionSource::External;
  if (event.external) {
    event.producer_process_id = fsm.locomotion_producer();
    application::FillFrameGains(first_frame, config.joint_names(),
                                &event.gains);
  } else if (model != nullptr) {
    // The name is the file name of the model path that was actually loaded -
    // never a title invented for the run and never derived from the digest.
    event.model_name =
        std::filesystem::path(config.policy.model_path).filename().string();
    event.model_sha256 = model->sha256();
    event.model_path = config.policy.model_path;
    event.model_input = model->contract().input.name;
    event.model_output = model->contract().output.name;
    event.contract_sha256 = model->contract_sha256();
    application::FillGainOverrides(resolved_gains, &event.gains);
  }
  if (!recorder->TryRecord(event)) {
    std::fprintf(stderr, "application: session record dropped\n");
    std::fflush(stderr);
  }
}

int RunApplication(const ApplicationOptions &options,
                   const std::atomic<bool> &stop_requested) {
  configuration::ApplicationConfig config;
  const int load_error = LoadConfiguration(options, &config);
  if (load_error != kExitOk) {
    return load_error;
  }

  // The transport is the first thing of this process that touches DDS, and
  // after this point every decision of the loop is a decision about a live
  // link.
  std::optional<simulation::EndpointConfig> simulation_config;
  std::unique_ptr<simulation::SimulationClock> simulation_clock;
  if (config.runtime.backend == "sim") {
    try {
      simulation_config = simulation::LoadBootstrap(
          config.runtime, simulation::Role::Application,
          config.application_config_id);
      simulation_clock = std::make_unique<simulation::SimulationClock>(
          simulation_config->run_id, config.runtime.control_period_us);
    } catch (const std::exception &error) {
      std::fprintf(stderr, "application: simulation startup: %s\n",
                   error.what());
      return kExitStartupError;
    }
  }
  transport::ClientTransport transport(config);
  std::unique_ptr<simulation::ApplicationDriver> simulation_driver;
  if (simulation_clock) {
    if (!transport.ConfigureSimulation(*simulation_clock))
      return kExitStartupError;
    simulation_driver = std::make_unique<simulation::ApplicationDriver>(
        *simulation_config, *simulation_clock, transport);
  }
  try {
    transport.Start();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "application: transport start failed: %s\n",
                 error.what());
    return kExitStartupError;
  }

  // The identity, the client, the behaviour and the input router, in that
  // order: the client is the only writer of the protocol, the behaviour is the
  // only decision maker, and the router is the only reader of the keyboard.
  //
  // The source of this run is decided before the behaviour exists: what the
  // operator asked for, or - with no explicit request - the local model this
  // configuration carries. A deployment that configures no model therefore runs
  // without a locomotion source, exactly as it did before this phase, instead
  // of refusing to start.
  const LocomotionSource source = options.locomotion_source.value_or(
      config.policy.model_path.empty() ? LocomotionSource::None
                                       : LocomotionSource::Local);
  if (simulation_driver && source == LocomotionSource::External) {
    std::fprintf(stderr,
                 "application: external simulation source is unsupported\n");
    return kExitStartupError;
  }
  StageSink sink(transport, simulation_driver.get());
  RobotClient client(sink, config, MakeSupervisorId());
  BehaviorFsm fsm(config, client, source);
  InputRouter router(config);

  // The local policy source: its model, the gains its frames are sent with and
  // the worker that runs it. Everything that could refuse the model is decided
  // here, before the supervision loop exists, so a deployment that cannot run
  // its policy never starts supervising a link.
  std::shared_ptr<policy::OnnxModel> model;
  std::unique_ptr<policy::InferenceWorker> worker;
  policy::ResolvedGains gains;
  if (source == LocomotionSource::Local) {
    if (config.policy.model_path.empty()) {
      std::fprintf(stderr,
                   "application: the local locomotion source needs "
                   "application.policy.model_path\n");
      transport.Stop();
      return kExitStartupError;
    }
    try {
      model = policy::OnnxModel::Load(config.policy.model_path,
                                      config.joint_names());
    } catch (const std::exception &error) {
      std::fprintf(stderr, "application: %s\n", error.what());
      transport.Stop();
      return kExitStartupError;
    }
    if (!policy::CoversEveryJoint(model->contract(), config.joint_count())) {
      std::fprintf(stderr,
                   "application: the policy commands %zu joints but the "
                   "deployment has %zu\n",
                   model->contract().action.joints.size(),
                   config.joint_count());
      transport.Stop();
      return kExitStartupError;
    }
    if (model->contract().policy_period_us >
        config.policy.inference_timeout_us) {
      // The deadline has to leave room for at least one policy step, or every
      // step of the model would be latched as a failure before it could finish.
      std::fprintf(
          stderr,
          "application: application.policy.inference_timeout_us (%llu) "
          "is below the model's policy period (%llu)\n",
          static_cast<unsigned long long>(config.policy.inference_timeout_us),
          static_cast<unsigned long long>(model->contract().policy_period_us));
      transport.Stop();
      return kExitStartupError;
    }
    if (!policy::ResolveGains(model->contract(), model->mapping(),
                              config.joint_names(), config.policy,
                              model->sha256(), &gains)) {
      std::fprintf(stderr,
                   "application: the model's gains cannot be applied to this "
                   "deployment\n");
      transport.Stop();
      return kExitStartupError;
    }
    worker = std::make_unique<policy::InferenceWorker>(model);
    std::printf(
        "[application] policy=%s sha256=%s joints=%zu period_us=%llu "
        "timeout_us=%llu overrides=%u\n",
        config.policy.model_path.c_str(), model->sha256().c_str(),
        model->contract().action.joints.size(),
        static_cast<unsigned long long>(model->contract().policy_period_us),
        static_cast<unsigned long long>(config.policy.inference_timeout_us),
        gains.override_count);
    for (std::uint32_t index = 0; index < gains.override_count; ++index) {
      const policy::GainOverride &record = gains.overrides[index];
      std::printf(
          "[application] override joint=%.*s kp=%s %.6f->%.6f "
          "kd=%s %.6f->%.6f\n",
          static_cast<int>(record.joint.View().size()),
          record.joint.View().data(), record.kp_overridden ? "yes" : "no",
          record.model_kp, record.applied_kp,
          record.kd_overridden ? "yes" : "no", record.model_kd,
          record.applied_kd);
    }
    std::fflush(stdout);
  }
  if (simulation_driver) {
    try {
      configuration::ValidateSimulationPolicy(
          config, source == LocomotionSource::Local ? "local" : "none",
          model ? model->contract().policy_period_us : 0);
      // Warm the actual session under the startup host deadline before Hello.
      policy::PolicyRequest warmup;
      warmup.observation.count = model->contract().observation.dimension;
      worker->Submit(warmup);
      policy::PolicyResponse warmed;
      const auto end = simulation::DriverHostUs() +
                       config.runtime.simulation->host_startup_timeout_us;
      while (!worker->TryTake(&warmed)) {
        if (stop_requested.load() || simulation::DriverHostUs() >= end)
          throw std::runtime_error(
              "simulation model warmup cancelled or timed out");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (!warmed.ok)
        throw std::runtime_error("simulation model warmup failed");
      simulation_driver->Start();
    } catch (const std::exception &error) {
      std::fprintf(stderr, "application: simulation policy/startup: %s\n",
                   error.what());
      return kExitStartupError;
    }
  }
  // The external source: a producer process that registers over DDS. The
  // transport of that interface exists only in this mode, so a local or
  // sourceless run opens no producer endpoint at all. A producer that never
  // arrives is not a startup error: the application supervises its link as
  // always and reports that the configured source has nobody to run it.
  ProducerSupervisor supervisor(config);
  std::unique_ptr<transport::ProducerTransport> producer_transport;
  if (source == LocomotionSource::External) {
    producer_transport = std::make_unique<transport::ProducerTransport>(config);
    try {
      producer_transport->Start();
    } catch (const std::exception &error) {
      std::fprintf(stderr,
                   "application: the producer transport did not start: %s\n",
                   error.what());
      transport.Stop();
      return kExitStartupError;
    }
    std::printf(
        "[application] policy=none source=external prepare_timeout_us=%llu "
        "registration_timeout_us=%llu\n",
        static_cast<unsigned long long>(
            config.external_producer.prepare_timeout_us),
        static_cast<unsigned long long>(
            config.external_producer.registration_timeout_us));
    std::fflush(stdout);
  } else if (source == LocomotionSource::None) {
    std::printf("[application] policy=none source=none\n");
    std::fflush(stdout);
  }

  // The application's own recording. The velocity recorder comes first: it
  // owns the `app_run_*` directory and the independent sample stream, and the
  // policy session sidecar is written beside it in that same directory so two
  // applications can never share one sessions file. Recording is best-effort:
  // a run it cannot record still supervises the link, and the failure is
  // reported instead of turning into a startup error.
  const protocol::TimeUs period_us = LoopPeriodUs(config);
  telemetry::SampleSchedule velocity_schedule(
      config.runtime.telemetry.recorder.sample_hz);
  std::unique_ptr<telemetry::VelocityRecorder> velocity_recorder;
  std::unique_ptr<application::PolicySessionRecorder> session_recorder;
  std::uint64_t velocity_sample_seq = 0;

  if (config.runtime.telemetry.recorder.enabled) {
    telemetry::RecordClockIdentity record_clock;
    std::string record_clock_error;
    if (!telemetry::LoadRecordClockIdentity(&record_clock,
                                            &record_clock_error)) {
      // The run may still be saved, but it cannot be merged automatically with
      // another process's stamps: an identity is what proves both share a boot.
      std::fprintf(stderr,
                   "application: velocity record clock identity unavailable: "
                   "%s\n",
                   record_clock_error.c_str());
      std::fflush(stderr);
    }
    if (!velocity_schedule.Enabled()) {
      std::fprintf(stderr,
                   "application: velocity record unavailable: sample_hz %u "
                   "is not representable\n",
                   config.runtime.telemetry.recorder.sample_hz);
      std::fflush(stderr);
    } else if (!CanServeSampleRate(period_us, velocity_schedule.PeriodUs())) {
      // The poll floor cannot wake the loop once per sample period; recording
      // slower than the metadata would claim is refused rather than faked.
      std::fprintf(stderr,
                   "application: velocity record unavailable: loop period "
                   "%llu us cannot serve %u Hz\n",
                   static_cast<unsigned long long>(period_us),
                   config.runtime.telemetry.recorder.sample_hz);
      std::fflush(stderr);
    } else {
      if (period_us > velocity_schedule.PeriodUs()) {
        // The loop's nominal period is longer than the sample period; the wait
        // cap below is what wakes it once per sample instead of leaving the
        // stream short of the configured rate.
        std::printf(
            "[application] velocity record sample_hz=%u loop_period_us=%llu "
            "paced_by_sample_deadline=yes\n",
            config.runtime.telemetry.recorder.sample_hz,
            static_cast<unsigned long long>(period_us));
        std::fflush(stdout);
      }
      velocity_recorder = std::make_unique<telemetry::VelocityRecorder>();
      telemetry::LiveTelemetryOptions live_telemetry;
      if (const char *socket = std::getenv("IRMV_WEB_TELEMETRY_SOCKET");
          socket != nullptr && *socket != '\0') {
        live_telemetry.socket_path = socket;
      }
      std::string error;
      if (!velocity_recorder->Start(
              config.runtime.telemetry.recorder,
              BuildVelocityMetadata(config, source, model.get(), record_clock,
                                    record_clock_error, period_us,
                                    velocity_schedule.PeriodUs()),
              &error, live_telemetry)) {
        std::fprintf(stderr, "application: velocity record unavailable: %s\n",
                     error.c_str());
        std::fflush(stderr);
        velocity_recorder.reset();
      }
    }
    if (velocity_recorder) {
      // The sidecar lands in this run's own directory, not the shared recorder
      // base: two applications writing the same file would merge their
      // sessions into one stream no identity could separate.
      session_recorder = std::make_unique<application::PolicySessionRecorder>(
          velocity_recorder->run_directory().string());
      if (!session_recorder->started()) {
        std::fprintf(stderr, "application: session record unavailable: %s\n",
                     session_recorder->error().c_str());
        std::fflush(stderr);
      }
    } else {
      std::fprintf(stderr,
                   "application: session record unavailable: the application "
                   "run directory was not created\n");
      std::fflush(stderr);
    }
  }

  // The local source's own state across iterations: the authorization
  // generation that drops a result of a withdrawn stop, the previous arming
  // (so the transition out of it can be detected), whether a result is on its
  // way, and the raw action the observation carries.
  std::uint64_t generation = 0;
  simulation::InferencePhase inference_phase;
  bool armed_previously = false;
  bool result_outstanding = false;
  bool submitted_once = false;
  protocol::TimeUs last_submit_us = 0;
  std::array<float, protocol::kMaxJoints> last_action{};

  // The external source's own state across iterations: when this loop started
  // waiting for a producer, when the candidate it holds was produced, the last
  // velocity intent it published, whether the report of a producer that never
  // arrived was already made, and the authorization edge the notice follows.
  protocol::TimeUs producer_wait_start_us = 0;
  bool have_candidate_us = false;
  protocol::TimeUs candidate_us = 0;
  bool have_intent = false;
  policy::VelocityCommand last_intent;
  protocol::Sequence intent_seq = 0;
  protocol::TimeUs last_intent_us = 0;
  bool registration_reported = false;
  bool authorized_previously = false;
  bool supervised_armed_previously = false;
  // The locomotion context the previous iteration observed, for either source:
  // the session record is written on the transition into one, and the local
  // source has no authorization notice to hang that off.
  bool locomotion_previously = false;
  protocol::Id announced_producer = protocol::kNoId;

  std::array<char, kReadChunk> bytes{};

  StatusFields last{};
  bool have_status = false;
  bool input_lost = false;

  // The loop's own pacing, counted the way the runtime counts its ticks: an
  // iteration is an overrun when it started more than two loop periods after
  // the one before it, and the worst gap is what tells an operator whether the
  // process kept its schedule. The runtime prints the same numbers in its
  // shutdown line; without them on this side, a supervision stall can only be
  // guessed at from a lease that expired.
  std::uint64_t iterations = 0;
  std::uint64_t overruns = 0;
  protocol::TimeUs worst_gap_us = 0;
  protocol::TimeUs previous_start_us = 0;
  // The two halves of the transport as this thread sees them: draining the
  // inbound lanes, and the sends the behaviour asked for. They are measured
  // apart from the loop's own gap because a stall of either one is a stall of
  // the process, and the runtime's supervision deadline does not care which of
  // them it was - while an operator deciding what to fix does.
  protocol::TimeUs worst_rx_us = 0;
  protocol::TimeUs worst_tx_us = 0;

  while (true) {
    if (simulation_driver) {
      simulation::StepPermit permit;
      while (!simulation_driver->TakePermit(&permit)) {
        if (simulation_driver->Failed() ||
            stop_requested.load(std::memory_order_relaxed)) {
          std::fprintf(stderr,
                       "application: simulation aborted; successful stop "
                       "requires Task 8\n");
          return kExitStopUnknown;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    // Two instants per iteration, both from the one monotonic host clock and
    // both named for what they time. `input_now` is read before the terminal
    // is drained and stamps only the keystrokes that read produced: a key is
    // one event in the byte stream, and its timestamp is the moment it was
    // read, not the moment its effect is judged. `decision_now` is read after
    // the transport events of the iteration were drained, raised by
    // DecisionInstant to the newest receive stamp among them, and it is the
    // instant of every supervision judgement (the router tick, the client
    // poll, the request exit, the Step of each input event, the sends and the
    // status cadence). The raise is why the second read exists: a state the
    // transport thread received while this iteration was starting is stamped
    // after `input_now`, so judging it with `input_now` would call a legal
    // state stale in the initializer while the client's clamped receive age
    // calls it fresh. With `decision_now` the loop has `stamp_us <= rx_us <=
    // decision_now` for every event of the iteration, and no decision predates
    // the data it acts on. The loop reads no other clock, and the runtime's
    // stamps are only ever compared against a host receive stamp.
    const protocol::TimeUs input_now =
        simulation_clock ? simulation_clock->NowUs() : MonotonicUs();

    // (1) The terminal. A read that would block is not the loop's pacing: the
    // poll at the end of the iteration is, so the terminal can never stall the
    // heartbeat.
    if (!input_lost) {
      input_lost = ReadTerminal(&router, &bytes, input_now);
    }

    // (2) Everything the transport thread received since the last iteration.
    const transport::ClientEvents &events = simulation_driver
                                                ? simulation_driver->Events()
                                                : transport.TakeEvents();
    protocol::TimeUs newest_rx_us = 0;
    for (std::size_t index = 0; index < events.result_count; ++index) {
      client.ObserveResult(events.results[index].value,
                           events.results[index].rx_us);
      if (events.results[index].rx_us > newest_rx_us) {
        newest_rx_us = events.results[index].rx_us;
      }
    }
    if (events.has_state) {
      client.ObserveState(events.state, events.state_rx_us);
      if (events.state_rx_us > newest_rx_us) {
        newest_rx_us = events.state_rx_us;
      }
    }
    const protocol::TimeUs decision_now = DecisionInstant(
        simulation_clock ? simulation_clock->NowUs() : MonotonicUs(),
        newest_rx_us);
    ++iterations;
    if (previous_start_us != 0) {
      const protocol::TimeUs gap =
          (simulation_driver ? MonotonicUs() : decision_now) -
          previous_start_us;
      if (gap > worst_gap_us) {
        worst_gap_us = gap;
      }
      // An overrun is an iteration that skipped at least one whole turn of the
      // loop: the wait that paces it is one period long, so a gap of one period
      // is the cadence itself and not a late iteration.
      if (gap > 2 * period_us) {
        ++overruns;
      }
    }
    previous_start_us = simulation_driver ? MonotonicUs() : decision_now;
    router.Tick(decision_now);

    // (3) Exactly one poll per iteration: the outcome it reports is edge
    // triggered, and the behaviour consumes the serial of this poll for its
    // own outcome handling.
    const protocol::TimeUs poll_start_us = MonotonicUs();
    const ClientView view = client.Poll(decision_now);
    const protocol::TimeUs poll_us = MonotonicUs() - poll_start_us;
    if (poll_us > worst_rx_us) {
      worst_rx_us = poll_us;
    }

    // The signal is not a keystroke and never a control path of its own: it
    // asks for the same ordered exit the exit key does.
    if (stop_requested.load(std::memory_order_relaxed)) {
      fsm.RequestExit(decision_now);
    }

    // (4) One Step per input event, then one final Step that does the periodic
    // work, and (5) the sends each Step asked for. The sends happen per Step
    // because a Step's target belongs to the iteration that produced it.
    BehaviorStep behavior{};
    bool initialization_refused = false;
    protocol::TimeUs sends_us = 0;
    for (;;) {
      const InputEvent event = router.TakeEvent();
      behavior =
          fsm.Step(decision_now, event, view, simulation_driver != nullptr);
      if (behavior.initialization_refused) {
        initialization_refused = true;
      }
      if (behavior.send_command) {
        const protocol::TimeUs send_start_us = MonotonicUs();
        static_cast<void>(client.SendCommand(&behavior.command, decision_now));
        sends_us += MonotonicUs() - send_start_us;
      }
      if (behavior.send_heartbeat) {
        const protocol::TimeUs send_start_us = MonotonicUs();
        static_cast<void>(client.SendHeartbeat(decision_now));
        sends_us += MonotonicUs() - send_start_us;
      }
      if (behavior.exit_ready || event == InputEvent::None) {
        break;
      }
    }
    if (sends_us > worst_tx_us) {
      worst_tx_us = sends_us;
    }

    // (5a) The independent velocity sample. It is taken after this iteration's
    // input and expiry processing, so the intent it stores is the one the
    // iteration acted on, and it is gated by its own schedule rather than by
    // the loop's period: a due deadline missed by a stall yields one sample
    // with this iteration's real stamp, never a replayed burst of fabricated
    // rows. `decision_now` is the iteration's own clock read; sampling adds
    // none.
    if (velocity_recorder && velocity_schedule.Due(decision_now)) {
      telemetry::VelocitySample sample;
      sample.stamp_us = decision_now;
      sample.sample_seq = velocity_sample_seq++;
      sample.identity_known = client.runtime_id() != protocol::kNoId &&
                              client.session_id() != protocol::kNoId;
      if (sample.identity_known) {
        sample.runtime_id = client.runtime_id();
        sample.session_id = client.session_id();
      }
      sample.latest_state_seq = view.state_known ? view.state.state_seq : 0;
      const VelocityIntent &intent = router.velocity_intent();
      sample.forward = intent.forward;
      sample.lateral = intent.lateral;
      sample.yaw = intent.yaw;
      // A full handoff or a stopped recorder costs the sample, never the
      // iteration; the recorder counts it and names the loss in its index.
      static_cast<void>(velocity_recorder->TryRecord(sample));
    }

    // (5b) The local policy source. The behaviour above decided whether it is
    // wanted - a stop, a fault or the exit clears that - and this is the only
    // place that runs it: the observation is submitted to the worker, the
    // newest result is taken back and the frame it produced is offered to the
    // behaviour, which is what publishes it. The loop never blocks on the
    // model, and a result that is late, of a withdrawn authorization or not
    // usable is a latched upper fault instead of a frame.
    if (worker) {
      if (!behavior.locomotion_armed && armed_previously) {
        // The authorization ended: every result still on its way belongs to it,
        // the next one starts from the contract's own initial action, and the
        // generation makes the identity of the two unmistakable.
        ++generation;
        if (simulation_driver) inference_phase.Cancel();
        policy::PolicyResponse withdrawn;
        while (worker->TryTake(&withdrawn)) {
        }
        result_outstanding = false;
        submitted_once = false;
        policy::ResetLastAction(model->contract().action, &last_action);
      }
      armed_previously = behavior.locomotion_armed;
      const bool generating = behavior.locomotion_armed &&
                              (behavior.state == BehaviorState::Ready ||
                               behavior.state == BehaviorState::Locomotion);
      if (generating && !result_outstanding &&
          (!submitted_once || decision_now - last_submit_us >=
                                  model->contract().policy_period_us) &&
          view.state_known && view.state_fresh) {
        const VelocityIntent &intent = router.velocity_intent();
        policy::VelocityCommand command;
        command.forward = intent.forward;
        command.lateral = intent.lateral;
        command.yaw = intent.yaw;
        policy::Observation observation;
        if (!policy::BuildObservation(model->contract(), model->mapping(),
                                      view.state, command, last_action,
                                      &observation)) {
          // A snapshot the observation cannot be built from is a failure of the
          // local source, not a reason to feed the model something else.
          fsm.LatchLocalPolicyFault(decision_now);
        } else {
          policy::PolicyRequest request;
          request.generation = generation;
          request.epoch = fsm.locomotion_epoch();
          request.state_seq = view.state.state_seq;
          request.state_stamp_us = view.state.stamp_us;
          request.submitted_us = decision_now;
          request.observation = observation;
          if (!simulation_driver ||
              inference_phase.Begin(decision_now, generation, request.epoch,
                                    model->contract().policy_period_us)) {
            worker->Submit(request);
            result_outstanding = true;
            submitted_once = true;
            last_submit_us = decision_now;
          }
        }
      }
      if (result_outstanding &&
          decision_now - last_submit_us > config.policy.inference_timeout_us) {
        // No result inside the deadline the configuration gave this source.
        // The step is written off here, and the deadline below is what makes
        // the result of it - which the worker may still be computing - a value
        // that drives nothing instead of a frame.
        result_outstanding = false;
        fsm.LatchLocalPolicyFault(decision_now);
      }
      policy::PolicyResponse response;
      auto take_response = [&]() {
        while (worker->TryTake(&response)) {
          if (!simulation_driver ||
              inference_phase.Matches(response.submitted_us,
                                      response.generation, response.epoch))
            return true;
        }
        return false;
      };
      bool have_response = take_response();
      if (simulation_driver && result_outstanding && !have_response) {
        const auto end = simulation::DriverHostUs() +
                         config.runtime.simulation->host_phase_timeout_us;
        while (!have_response) {
          if (stop_requested.load() || simulation_driver->Failed() ||
              simulation::DriverHostUs() >= end) {
            std::fprintf(
                stderr,
                "application: simulation inference cancelled or timed out\n");
            return kExitStopUnknown;
          }
          if (!input_lost)
            input_lost = ReadTerminal(&router, &bytes, decision_now);
          const auto event = router.TakeEvent();
          if (event != InputEvent::None) {
            behavior = fsm.Step(decision_now, event, view, true);
            if (behavior.send_command)
              client.SendCommand(&behavior.command, decision_now);
            if (behavior.send_heartbeat) client.SendHeartbeat(decision_now);
            if (!behavior.locomotion_armed) {
              ++generation;
              inference_phase.Cancel();
              result_outstanding = false;
              submitted_once = false;
              break;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          have_response = take_response();
        }
      }
      if (have_response) {
        // The result of the step this loop is waiting for is the only one that
        // may reopen the lane or drive a frame. The identity is checked whole -
        // the submission instant that names the step, the local generation, the
        // authorization epoch - and so are the response deadline and the source
        // state it was computed from: a result of a withdrawn authorization, of
        // a step this loop already wrote off as late, or of a state the runtime
        // would no longer accept is dropped here, before it can touch
        // `last_action` or become a candidate.
        const bool current =
            result_outstanding && response.submitted_us == last_submit_us;
        if (current) {
          if (simulation_driver)
            inference_phase.Complete(response.submitted_us, response.generation,
                                     response.epoch);
          result_outstanding = false;
        }
        switch (policy::JudgePolicyResult(
            response, generation, fsm.locomotion_epoch(), decision_now,
            config.policy.inference_timeout_us, last_submit_us)) {
          case policy::PolicyVerdict::Ignored:
            // A result of another authorization or of another step: it drives
            // nothing, and the current generation is unaffected.
            break;
          case policy::PolicyVerdict::Failed:
            fsm.LatchLocalPolicyFault(decision_now);
            break;
          case policy::PolicyVerdict::Usable:
            if (!view.state_known || !view.state_fresh) {
              break;
            }
            if (!policy::PolicySourceStateUsable(
                    response, view.state.state_seq, view.state.stamp_us,
                    config.runtime.timeouts.source_state_us)) {
              // The state the action was computed from is not one the runtime
              // can still look the target up against.
              fsm.LatchLocalPolicyFault(decision_now);
              break;
            }
            {
              // The frame is stamped with the state the action was computed
              // from - not with the newest one the loop happens to hold -
              // because that sequence is what the runtime checks the target's
              // source age against.
              protocol::RobotState source = view.state;
              source.state_seq = response.state_seq;
              source.stamp_us = response.state_stamp_us;
              protocol::JointCommand frame;
              // The action of this step is what the observation of the next one
              // carries, in the stage the contract names, and the same step is
              // what the frame carries: both come from one action, so neither
              // can describe a step the other does not.
              if (!policy::BuildCommand(
                      model->contract(), model->mapping(), gains,
                      response.action.data(), response.action_count, source,
                      config.layout_id(), fsm.locomotion_producer(),
                      fsm.locomotion_epoch(), 0, &frame)) {
                fsm.LatchLocalPolicyFault(decision_now);
              } else if (!policy::LastActionFromAction(
                             model->contract().action, response.action.data(),
                             response.action_count, &last_action)) {
                fsm.LatchLocalPolicyFault(decision_now);
              } else if (behavior.state == BehaviorState::Ready) {
                // Not authorized yet: the frame is the candidate the behaviour
                // turns into the first frame of its request, and nothing of it
                // is published before that request is confirmed.
                fsm.OfferLocomotionCandidate(frame, decision_now);
              } else if (behavior.state == BehaviorState::Locomotion) {
                fsm.OfferLocomotionTarget(frame, decision_now);
              }
            }
            break;
        }
      }
    }

    if (simulation_driver && worker && behavior.locomotion_armed) {
      behavior = fsm.Step(decision_now, InputEvent::None, view);
      if (behavior.send_command)
        client.SendCommand(&behavior.command, decision_now);
      if (behavior.send_heartbeat) client.SendHeartbeat(decision_now);
    }
    // (5c) The external producer source. The behaviour above decided whether a
    // source is wanted and the runtime is the only thing that authorizes an
    // execution context; this is the one place that talks to a producer
    // process. It selects the producer that announced itself, asks it to
    // prepare a first frame, offers the frame it produced to the behaviour as
    // the candidate of the next authorization, tells it when that
    // authorization became live, keeps it supplied with the operator's velocity
    // intent, and tells it to stop when the authorization ends.
    //
    // Nothing of a preparation reaches the execution topic: the frame a
    // producer prepared travels inside the Tracking request, which is the
    // runtime's decision, and only the producer of an authorized context
    // publishes a frame stream - which is why the behaviour publishes none in
    // this mode.
    if (producer_transport) {
      if (producer_wait_start_us == 0) {
        producer_wait_start_us = decision_now;
      }
      const transport::ProducerEvents &producer_events =
          producer_transport->TakeEvents();
      for (std::size_t index = 0; index < producer_events.announcement_count;
           ++index) {
        const protocol::ProducerAnnouncement &announcement =
            producer_events.announcements[index];
        // A registration is taken only while the application is idle: Ready,
        // with no locomotion authorization armed or in flight. That is the
        // state a context is asked from, so a new process cannot take anything
        // away from a producer whose frames a live context is made of - the
        // runtime gates a context against the process whose frame it adopted,
        // and replacing that process mid-context would tell it to stop while
        // the machine is still following it. A producer that is not accepted
        // announces itself again on its own cadence, so this delays a selection
        // instead of losing one.
        if (behavior.state != BehaviorState::Ready ||
            behavior.locomotion_armed) {
          continue;
        }
        const bool was_selected = supervisor.selected();
        const protocol::Id selected_producer = supervisor.producer_process_id();
        if (!supervisor.Select(announcement)) {
          // An announcement this deployment cannot use: another protocol
          // version, another layout, another target period or a producer that
          // is not ready. Nothing is prepared for it, so nothing of it can
          // become a target.
          continue;
        }
        if (was_selected &&
            !(selected_producer == announcement.producer_process_id)) {
          // The selection moved from one process to another: a candidate the
          // behaviour still holds was produced by the process that is no longer
          // selected, and the first frame of the next request must name the
          // process whose frames the runtime will gate. The candidate goes with
          // the selection it was offered for, and the new producer is prepared
          // for the context the operator asks for next.
          fsm.DropLocomotionCandidate();
          have_candidate_us = false;
        }
        if (!(announcement.producer_process_id == announced_producer)) {
          announced_producer = announcement.producer_process_id;
          const std::array<char, 33> producer_text =
              IdText(announcement.producer_process_id);
          std::printf(
              "[application] producer=selected id=%s period_us=%llu\n",
              producer_text.data(),
              static_cast<unsigned long long>(announcement.target_period_us));
          std::fflush(stdout);
        }
      }
      for (std::size_t index = 0; index < producer_events.prepare_result_count;
           ++index) {
        const ProducerOutcome outcome = supervisor.Observe(
            producer_events.prepare_results[index], view.state, decision_now);
        if (outcome == ProducerOutcome::Candidate) {
          fsm.OfferLocomotionCandidate(supervisor.candidate(), decision_now);
          have_candidate_us = true;
          candidate_us = decision_now;
          std::printf("[application] producer=prepared prepare_id=%llu\n",
                      static_cast<unsigned long long>(supervisor.prepare_id()));
          std::fflush(stdout);
        } else if (outcome == ProducerOutcome::Failed) {
          have_candidate_us = false;
          std::printf("[application] producer=failed reason=%s\n",
                      ProducerStopName(supervisor.failure()));
          std::fflush(stdout);
        }
      }
      protocol::StopNotice stop_notice;
      if (supervisor.TakeStopNotice(&stop_notice)) {
        static_cast<void>(producer_transport->PublishStop(stop_notice));
      }
      if (supervisor.ExpirePreparation(decision_now)) {
        // A preparation the producer did not answer inside its deadline: the
        // frame of it is not a first frame any more. The producer stays
        // registered, and the next preparation is a new request.
        have_candidate_us = false;
        std::printf("[application] producer=failed reason=%s\n",
                    ProducerStopName(supervisor.failure()));
        std::fflush(stdout);
      }

      // The velocity intent the producer should track, at this source's own
      // cadence: the newest one the operator holds, republished when it changes
      // and otherwise once per target period, so the producer never has to
      // guess whether a stale intent still holds. An intent the router expired
      // is a zero command like any other, and it is published the same way.
      const VelocityIntent &intent = router.velocity_intent();
      policy::VelocityCommand current_intent;
      current_intent.forward = intent.forward;
      current_intent.lateral = intent.lateral;
      current_intent.yaw = intent.yaw;
      const bool intent_changed =
          !have_intent || current_intent.forward != last_intent.forward ||
          current_intent.lateral != last_intent.lateral ||
          current_intent.yaw != last_intent.yaw;
      protocol::VelocityIntentMessage intent_message;
      intent_message.producer_process_id = supervisor.producer_process_id();
      intent_message.seq = intent_seq + 1;
      intent_message.forward = current_intent.forward;
      intent_message.lateral = current_intent.lateral;
      intent_message.yaw = current_intent.yaw;
      intent_message.send_stamp_us = decision_now;
      intent_message.valid_for_us = config.input.intent_timeout_us;
      if (supervisor.selected() &&
          (intent_changed || decision_now - last_intent_us >=
                                 config.supervision.target_period_us)) {
        static_cast<void>(
            producer_transport->PublishVelocityIntent(intent_message));
        have_intent = true;
        last_intent = current_intent;
        last_intent_us = decision_now;
        intent_seq = intent_message.seq;
      }

      // One preparation at a time, and only from the state a locomotion
      // authorization can be asked from. A candidate that is already usable is
      // not replaced; one that outlived the preparation deadline is not a first
      // frame any more, so a new preparation takes its place.
      const bool candidate_usable =
          have_candidate_us && decision_now - candidate_us <=
                                   config.external_producer.prepare_timeout_us;
      if (supervisor.selected() && view.session_owned &&
          behavior.state == BehaviorState::Ready && !supervisor.preparing() &&
          !candidate_usable) {
        protocol::PrepareRequest request;
        if (supervisor.Prepare(client.runtime_id(), client.session_id(),
                               intent_message, decision_now,
                               config.external_producer.prepare_timeout_us,
                               &request)) {
          // A request the outbound lane refuses is a preparation the producer
          // never hears of; its deadline is what ends it, and the transport
          // counts the loss.
          static_cast<void>(producer_transport->PublishPrepareRequest(request));
          have_candidate_us = false;
        }
      }

      // The runtime applied the context this request asked for: the producer
      // may publish. The notice carries the identity the runtime granted and
      // the sequence of the first frame the request carried, so the producer
      // continues the one stream instead of restarting a numbering of its own
      // that the runtime's command gate would refuse. The identity is the one
      // the behaviour stamped on the request - the process whose frame the
      // runtime adopted - and not the producer that happens to be selected
      // now: a notice that named another process would authorize a stream the
      // runtime refuses.
      const bool authorized = behavior.state == BehaviorState::Locomotion;
      if (authorized && !authorized_previously) {
        const protocol::AuthorizationNotice notice = supervisor.Authorized(
            client.runtime_id(), client.session_id(), fsm.locomotion_producer(),
            fsm.locomotion_epoch(), fsm.locomotion_first_command_seq());
        static_cast<void>(producer_transport->PublishAuthorization(notice));
        std::printf(
            "[application] producer=authorized epoch=%llu first_seq=%llu\n",
            static_cast<unsigned long long>(notice.epoch),
            static_cast<unsigned long long>(notice.first_command_seq));
        std::fflush(stdout);
      }
      authorized_previously = authorized;

      // The authorization ended - a stop, a fault, the exit, or the runtime
      // withdrawing the context on its own: the producer is told to stop, and
      // the preparation and its candidate are dropped so that nothing of a
      // withdrawn context can prepare the next one.
      if (!behavior.locomotion_armed && supervised_armed_previously) {
        static_cast<void>(supervisor.Revoke(ProducerStop::Stopped));
        have_candidate_us = false;
      }
      supervised_armed_previously = behavior.locomotion_armed;

      // A producer that never announced itself inside the configured window:
      // reported once, and never fatal. The loop keeps supervising the link, so
      // a producer that starts later is still selected, prepared and
      // authorized.
      if (!supervisor.selected() && !registration_reported &&
          decision_now - producer_wait_start_us >=
              config.external_producer.registration_timeout_us) {
        registration_reported = true;
        std::printf("[application] producer=missing timeout_us=%llu\n",
                    static_cast<unsigned long long>(
                        config.external_producer.registration_timeout_us));
        std::fflush(stdout);
      }
    }

    // (5c) The session record: one line per confirmed locomotion context,
    // whichever source produced it. The local source publishes its own stream
    // and the external one hands the stream to a producer, but what produced
    // the context - the model and the gains, or the process and the gains its
    // first frame declared - is the same record, and this is the iteration in
    // which the runtime applied the request and the identity it granted is
    // known.
    if (behavior.state == BehaviorState::Locomotion && !locomotion_previously) {
      RecordPolicySession(session_recorder.get(), source, client, fsm, gains,
                          model.get(), config, supervisor.candidate());
    }
    locomotion_previously = behavior.state == BehaviorState::Locomotion;

    // (6) The display: only when a field changes, so a mode request, a
    // transition, a refusal or a lost session is reported exactly once and an
    // idle run stays silent.
    StatusFields fields;
    fields.behavior = BehaviorName(behavior.state);
    fields.runtime = behavior.runtime_state_known
                         ? StateName(behavior.runtime_state)
                         : "Unknown";
    fields.output = behavior.runtime_state_known
                        ? OutputName(behavior.runtime_output)
                        : "Unknown";
    fields.session = SessionField(client, view);
    PendingField(client, sink, &fields);
    fields.reject = RejectName(behavior.last_reject_reason);
    fields.fresh = view.state_fresh ? "yes" : "no";
    if (initialization_refused) {
      fields.init = "refused";
    } else if (behavior.state == BehaviorState::Initializing ||
               behavior.state == BehaviorState::Ready) {
      fields.init = "ok";
    }
    // The latched cause is the upper layer's own statement and stays visible
    // while it holds; the lower stop reason is a present fact of the state, so
    // it is shown only while a current state is known.
    fields.cause = FaultCauseName(behavior.fault_cause);
    fields.stop = behavior.runtime_state_known
                      ? StopReasonName(behavior.runtime_stop_reason)
                      : "-";
    if (!have_status || !SameFields(fields, last)) {
      PrintStatus(fields);
      last = fields;
      have_status = true;
    }

    // (7) The exit. `exit_ready` is the FSM's own statement that the sequence
    // it was asked for is over: either a fresh lower state shows the damping
    // stop, or the sequence reached its own deadline and proved nothing.
    if (simulation_driver && behavior.exit_ready) {
      std::fprintf(stderr,
                   "application: simulation exit requested; successful leave "
                   "requires Task 8\n");
      return kExitStopUnknown;
    }
    if (behavior.exit_ready) {
      const char *lower = behavior.runtime_state_known
                              ? StateName(behavior.runtime_state)
                              : "Unknown";
      // The loop's own pacing first, the way the runtime prints its ticks and
      // overruns in the clean-shutdown line, then the outcome of the exit.
      std::printf(
          "application: loop iterations=%llu overruns=%llu worst_gap_us=%llu "
          "worst_rx_us=%llu worst_tx_us=%llu period_us=%llu\n",
          static_cast<unsigned long long>(iterations),
          static_cast<unsigned long long>(overruns),
          static_cast<unsigned long long>(worst_gap_us),
          static_cast<unsigned long long>(worst_rx_us),
          static_cast<unsigned long long>(worst_tx_us),
          static_cast<unsigned long long>(period_us));
      // The link's own counters, printed beside the loop's pacing: a stall of
      // the processes is what the pacing shows, and a silence on the wire that
      // no control loop caused is what these show - a lane that refused a
      // sample, a sample that failed validation, a read that ended a loop. The
      // application is the only place they exist; the runtime prints its own.
      // The heartbeat chain, application side: what this process generated,
      // what it enqueued, and what its transport thread really handed to the
      // DDS writer - the three steps between the supervision loop and the
      // wire. Together with the runtime's read/accepted/rejected counts and
      // the lease's age in its own status line, they say which step a silence
      // belongs to; the loop's own gap says nothing about the write, because
      // the write happens on the transport thread.
      std::printf(
          "application: link results_dropped=%llu states_dropped=%llu "
          "requests_dropped=%llu commands_dropped=%llu heartbeats_dropped=%llu "
          "inbound_rejected=%llu read_failures=%llu heartbeats_queued=%llu "
          "heartbeats_published=%llu last_queued_seq=%llu "
          "last_published_seq=%llu last_published_us=%llu "
          "worst_heartbeat_gap_us=%llu heartbeats_late=%llu\n",
          static_cast<unsigned long long>(transport.dropped_result_count()),
          static_cast<unsigned long long>(transport.dropped_state_count()),
          static_cast<unsigned long long>(transport.dropped_request_count()),
          static_cast<unsigned long long>(transport.dropped_command_count()),
          static_cast<unsigned long long>(transport.dropped_heartbeat_count()),
          static_cast<unsigned long long>(transport.inbound_rejected_count()),
          static_cast<unsigned long long>(
              transport.inbound_read_failure_count()),
          static_cast<unsigned long long>(transport.heartbeats_queued_count()),
          static_cast<unsigned long long>(
              transport.heartbeats_published_count()),
          static_cast<unsigned long long>(
              transport.last_queued_heartbeat_seq()),
          static_cast<unsigned long long>(
              transport.last_published_heartbeat_seq()),
          static_cast<unsigned long long>(
              transport.last_published_heartbeat_us()),
          static_cast<unsigned long long>(transport.worst_heartbeat_gap_us()),
          static_cast<unsigned long long>(transport.heartbeats_late_count()));
      int exit_code = kExitStopUnknown;
      if (behavior.exit_stop_confirmed) {
        std::printf("application: exit stop=confirmed state=%s\n", lower);
        exit_code = kExitOk;
      } else {
        std::printf("application: exit stop=unknown lower=%s\n", lower);
      }
      static_cast<void>(std::fflush(stdout));
      // Nothing of this process keeps running behind the exit line: the
      // transport thread is joined here, and the runtime is not touched - its
      // own watchdog owns the link this process leaves behind. Shutdown asks
      // the session sidecar to finish before the velocity recorder finalizes
      // its directory and index, so the two records of a run normally complete
      // in that order. A sidecar that misses its deadline is left to its
      // detached worker and may finish after the velocity index, so this is a
      // best-effort order, not a guarantee.
      if (session_recorder && !session_recorder->Stop()) {
        std::fprintf(stderr,
                     "application: policy session record did not finalize "
                     "inside %u ms; its worker will finish on its own\n",
                     kPolicySessionStopTimeoutMs);
        std::fflush(stderr);
      }
      if (velocity_recorder && !velocity_recorder->Stop()) {
        std::fprintf(stderr,
                     "application: velocity record did not finalize inside "
                     "%u ms; its worker will finish on its own\n",
                     telemetry::kVelocityStopTimeoutMs);
        std::fflush(stderr);
      }
      if (worker) {
        worker->Stop();
      }
      if (producer_transport) {
        // The producer is told the context is over before the endpoint that
        // tells it is closed. It is a notice, not a revocation: the runtime
        // withdrew the epoch on its own when the ordered stop was applied. The
        // revocation is asked for here as well, because an exit that completed
        // in the same iteration the authorization ended may not have reached
        // the block that does it on the way.
        static_cast<void>(supervisor.Revoke(ProducerStop::Stopped));
        protocol::StopNotice stop_notice;
        if (supervisor.TakeStopNotice(&stop_notice)) {
          static_cast<void>(producer_transport->PublishStop(stop_notice));
        }
        producer_transport->Stop();
      }
      transport.Stop();
      return exit_code;
    }

    if (simulation_driver) {
      const auto end = simulation::DriverHostUs() +
                       config.runtime.simulation->host_phase_timeout_us;
      auto report = [&](const auto &sample) {
        simulation::MessageIdentity id;
        if (!simulation::Identify(sample, &id)) return false;
        while (!simulation_driver->Processed(id)) {
          if (simulation_driver->Failed() || stop_requested.load() ||
              simulation::DriverHostUs() >= end)
            return false;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return true;
      };
      for (std::size_t i = 0; i < events.result_count; ++i)
        if (!report(events.results[i].value)) return kExitStopUnknown;
      if (events.has_state && !report(events.state)) return kExitStopUnknown;
      while (!simulation_driver->CompletePhase(simulation_driver->Outputs())) {
        if (simulation_driver->Failed() || stop_requested.load() ||
            simulation::DriverHostUs() >= end)
          return kExitStopUnknown;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      continue;
    }
    // The wait that paces the iteration, and the terminal wait with it. Once
    // the input device is gone the descriptor reports it forever, so the
    // pacing becomes a plain sleep instead of a spin. In both forms the wait is
    // the loop's own period capped at the next velocity sample deadline: the
    // cap can only shorten this idle stretch, so every heartbeat and FSM
    // deadline - judged at the next iteration's decision instant - keeps the
    // same bound it had before.
    const protocol::TimeUs next_sample_us =
        velocity_recorder ? velocity_schedule.NextDeadlineUs() : 0;
    if (input_lost) {
      const protocol::TimeUs budget =
          WaitBudgetUs(period_us, next_sample_us, decision_now);
      std::this_thread::sleep_for(std::chrono::microseconds(budget));
    } else {
      struct pollfd descriptor {};
      descriptor.fd = STDIN_FILENO;
      descriptor.events = POLLIN;
      const int wait_timeout_ms =
          WaitTimeoutMs(period_us, next_sample_us, decision_now);
      // An interrupted wait and every other result just end the wait: the next
      // iteration re-reads the clock and re-runs its checks, which is also what
      // keeps a signalled exit bounded.
      static_cast<void>(::poll(&descriptor, 1, wait_timeout_ms));
    }
  }
}

}  // namespace irmv::application
