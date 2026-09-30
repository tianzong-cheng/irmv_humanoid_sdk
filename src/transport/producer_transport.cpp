#include "irmv/transport/producer_transport.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <dds/dds.hpp>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "dds_loop.hpp"
#include "irmv/transport/producer_codec.hpp"
#include "producer.hpp"

namespace irmv::transport {
namespace {

namespace wire = irmv::protocol::wire;

}  // namespace

// Every DDS entity lives here and is created by the constructor, which only
// runs from Start(). The transport thread owns all of them after that.
//
// The application writes the four outbound messages and reads the two inbound
// ones, which is the opposite direction of ClientTransport on the same
// participant configuration.
struct ProducerTransport::Impl {
  Impl(const configuration::ApplicationConfig &config, ProducerTransport &owner)
      : owner(owner),
        participant(config.dds().domain,
                    dds::domain::qos::DomainParticipantQos(), nullptr,
                    dds::core::status::StatusMask::none(),
                    config.dds().interface.empty()
                        ? std::string()
                        : InterfaceConfig(config.dds().interface)),
        publisher(participant),
        subscriber(participant),
        announcement_topic(
            participant,
            std::string(protocol::kProducerAnnouncementTopic.name)),
        prepare_request_topic(
            participant,
            std::string(protocol::kProducerPrepareRequestTopic.name)),
        prepare_result_topic(
            participant,
            std::string(protocol::kProducerPrepareResultTopic.name)),
        authorization_topic(
            participant,
            std::string(protocol::kProducerAuthorizationTopic.name)),
        stop_topic(participant, std::string(protocol::kProducerStopTopic.name)),
        intent_topic(participant,
                     std::string(protocol::kProducerVelocityIntentTopic.name)),
        prepare_request_writer(publisher, prepare_request_topic,
                               producer_codec::ToWriterQos(
                                   protocol::kProducerPrepareRequestTopic.qos)),
        authorization_writer(publisher, authorization_topic,
                             producer_codec::ToWriterQos(
                                 protocol::kProducerAuthorizationTopic.qos)),
        stop_writer(
            publisher, stop_topic,
            producer_codec::ToWriterQos(protocol::kProducerStopTopic.qos)),
        intent_writer(publisher, intent_topic,
                      producer_codec::ToWriterQos(
                          protocol::kProducerVelocityIntentTopic.qos)),
        announcement_reader(subscriber, announcement_topic,
                            producer_codec::ToReaderQos(
                                protocol::kProducerAnnouncementTopic.qos)),
        prepare_result_reader(subscriber, prepare_result_topic,
                              producer_codec::ToReaderQos(
                                  protocol::kProducerPrepareResultTopic.qos)),
        announcement_condition(announcement_reader,
                               dds::sub::status::DataState::any()),
        prepare_result_condition(prepare_result_reader,
                                 dds::sub::status::DataState::any()) {
    waitset += announcement_condition;
    waitset += prepare_result_condition;
  }

  void Start() { thread = std::thread(&Impl::Run, this); }

  void Run();
  bool ReadInbound();
  void ReadAnnouncements();
  void ReadPrepareResults();
  void WriteOutbound();
  void Stop();

  ProducerTransport &owner;
  dds::domain::DomainParticipant participant;
  dds::pub::Publisher publisher;
  dds::sub::Subscriber subscriber;
  dds::topic::Topic<wire::ProducerAnnouncement> announcement_topic;
  dds::topic::Topic<wire::PrepareRequest> prepare_request_topic;
  dds::topic::Topic<wire::PrepareResult> prepare_result_topic;
  dds::topic::Topic<wire::AuthorizationNotice> authorization_topic;
  dds::topic::Topic<wire::StopNotice> stop_topic;
  dds::topic::Topic<wire::VelocityIntentMessage> intent_topic;
  dds::pub::DataWriter<wire::PrepareRequest> prepare_request_writer;
  dds::pub::DataWriter<wire::AuthorizationNotice> authorization_writer;
  dds::pub::DataWriter<wire::StopNotice> stop_writer;
  dds::pub::DataWriter<wire::VelocityIntentMessage> intent_writer;
  dds::sub::DataReader<wire::ProducerAnnouncement> announcement_reader;
  dds::sub::DataReader<wire::PrepareResult> prepare_result_reader;
  dds::sub::cond::ReadCondition announcement_condition;
  dds::sub::cond::ReadCondition prepare_result_condition;
  dds::core::cond::WaitSet waitset;
  std::thread thread;
  std::atomic<bool> stop_requested{false};
  std::mutex done_mutex;
  std::condition_variable done_cv;
  bool finished{false};
};

void ProducerTransport::Impl::Run() {
  while (!stop_requested.load(std::memory_order_relaxed)) {
    bool data_available = false;
    try {
      data_available =
          !waitset.wait(dds::core::Duration::from_millisecs(kWaitTimeoutMs))
               .empty();
    } catch (const dds::core::TimeoutError &) {
      // The pinned WaitSet reports a timeout by throwing; that is the normal
      // "nothing to read within the wait window" path.
      data_available = false;
    } catch (const std::exception &) {
      // A failing wait means the entities this thread owns are unusable. Leave
      // the loop instead of spinning on the error: the channels keep working
      // and started() turns false.
      break;
    }
    if (data_available && !ReadInbound()) {
      break;
    }
    WriteOutbound();
  }

  {
    const std::lock_guard<std::mutex> lock(done_mutex);
    finished = true;
  }
  done_cv.notify_all();
  owner.started_.store(false, std::memory_order_release);
}

bool ProducerTransport::Impl::ReadInbound() {
  // Each lane is drained behind the same guard: a failing read ends the
  // transport loop cleanly and is counted, and the lane that follows is left
  // untouched.
  return ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadAnnouncements(); }) &&
         ReadGuarded(owner.inbound_read_failures_,
                     [this]() { ReadPrepareResults(); });
}

void ProducerTransport::Impl::ReadAnnouncements() {
  auto samples = announcement_reader.select()
                     .max_samples(kReadBatch)
                     .state(dds::sub::status::DataState::any())
                     .take();
  for (const auto &sample : samples) {
    if (!sample.info().valid()) {
      continue;
    }
    protocol::ProducerAnnouncement value;
    if (!producer_codec::FromWire(sample.data(), value)) {
      owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    // The lane is FIFO: an announcement that does not fit is refused and
    // counted, and the ones already queued keep the order the producers
    // declared themselves in.
    if (!owner.announcements_.TryPush(value)) {
      owner.dropped_announcements_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void ProducerTransport::Impl::ReadPrepareResults() {
  auto samples = prepare_result_reader.select()
                     .max_samples(kReadBatch)
                     .state(dds::sub::status::DataState::any())
                     .take();
  for (const auto &sample : samples) {
    if (!sample.info().valid()) {
      continue;
    }
    protocol::PrepareResult value;
    if (!producer_codec::FromWire(sample.data(), value)) {
      owner.inbound_rejected_.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    if (!owner.prepare_results_.TryPush(value)) {
      owner.dropped_prepare_results_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void ProducerTransport::Impl::WriteOutbound() {
  // The lifecycle lanes are drained in the order the interface defines them:
  // a stop is written before a preparation the application may have asked for
  // in the same iteration, because the stop belongs to the context that is
  // already over and must not wait behind a new request.
  protocol::StopNotice stop;
  while (owner.stops_.TryPop(stop)) {
    wire::StopNotice wire_stop;
    producer_codec::ToWire(stop, wire_stop);
    TryWrite(stop_writer, wire_stop, owner.dropped_stops_);
  }
  protocol::PrepareRequest request;
  while (owner.prepare_requests_.TryPop(request)) {
    wire::PrepareRequest wire_request;
    producer_codec::ToWire(request, wire_request);
    TryWrite(prepare_request_writer, wire_request,
             owner.dropped_prepare_requests_);
  }
  protocol::AuthorizationNotice authorization;
  while (owner.authorizations_.TryPop(authorization)) {
    wire::AuthorizationNotice wire_authorization;
    producer_codec::ToWire(authorization, wire_authorization);
    TryWrite(authorization_writer, wire_authorization,
             owner.dropped_authorizations_);
  }

  // The velocity intent keeps one entry, so a take either hands it over or
  // reports nothing published; the wire representation is built only in the
  // first case.
  protocol::VelocityIntentMessage intent;
  if (owner.intents_.TryTake(intent)) {
    wire::VelocityIntentMessage wire_intent;
    producer_codec::ToWire(intent, wire_intent);
    TryWrite(intent_writer, wire_intent, owner.dropped_intents_);
  }
}

void ProducerTransport::Impl::Stop() {
  stop_requested.store(true, std::memory_order_relaxed);
  if (!thread.joinable()) {
    return;
  }
  {
    std::unique_lock<std::mutex> lock(done_mutex);
    done_cv.wait_for(lock, std::chrono::milliseconds(kStopTimeoutMs),
                     [this] { return finished; });
  }
  thread.join();
}

ProducerTransport::ProducerTransport(
    const configuration::ApplicationConfig &config)
    : config_(config) {}

ProducerTransport::~ProducerTransport() { Stop(); }

void ProducerTransport::Start() {
  if (impl_) {
    return;
  }
  // The same invariant ClientTransport keeps: started() is true only while an
  // owned implementation has a launched thread that has not left its loop.
  std::unique_ptr<Impl> impl = std::make_unique<Impl>(config_, *this);
  impl_ = std::move(impl);
  started_.store(true, std::memory_order_release);
  try {
    impl_->Start();
  } catch (...) {
    started_.store(false, std::memory_order_release);
    impl_.reset();
    throw;
  }
}

void ProducerTransport::Stop() {
  if (!impl_) {
    return;
  }
  impl_->Stop();
  impl_.reset();
  started_.store(false, std::memory_order_release);
}

bool ProducerTransport::started() const {
  return started_.load(std::memory_order_acquire);
}

bool ProducerTransport::PublishPrepareRequest(
    const protocol::PrepareRequest &request) {
  if (!started()) {
    dropped_prepare_requests_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!prepare_requests_.TryPush(request)) {
    dropped_prepare_requests_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

bool ProducerTransport::PublishAuthorization(
    const protocol::AuthorizationNotice &notice) {
  if (!started()) {
    dropped_authorizations_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!authorizations_.TryPush(notice)) {
    dropped_authorizations_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

bool ProducerTransport::PublishStop(const protocol::StopNotice &notice) {
  if (!started()) {
    dropped_stops_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!stops_.TryPush(notice)) {
    dropped_stops_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

bool ProducerTransport::PublishVelocityIntent(
    const protocol::VelocityIntentMessage &intent) {
  if (!started()) {
    dropped_intents_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  bool evicted = false;
  if (!intents_.Publish(intent, &evicted)) {
    dropped_intents_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (evicted) {
    // The superseded intent was never written, and nobody would have wanted it:
    // an intent the producer has not seen yet is worth less than the one that
    // replaces it.
    dropped_intents_.fetch_add(1, std::memory_order_relaxed);
  }
  return true;
}

const ProducerEvents &ProducerTransport::TakeEvents() {
  // The batch describes this one call: a take that finds nothing costs one pass
  // over two empty lanes.
  events_.announcement_count = 0;
  protocol::ProducerAnnouncement announcement;
  while (events_.announcement_count < events_.announcements.size() &&
         announcements_.TryPop(announcement)) {
    events_.announcements[events_.announcement_count] = announcement;
    ++events_.announcement_count;
  }

  events_.prepare_result_count = 0;
  protocol::PrepareResult result;
  while (events_.prepare_result_count < events_.prepare_results.size() &&
         prepare_results_.TryPop(result)) {
    events_.prepare_results[events_.prepare_result_count] = result;
    ++events_.prepare_result_count;
  }
  return events_;
}

std::uint64_t ProducerTransport::dropped_announcement_count() const {
  return dropped_announcements_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::dropped_prepare_result_count() const {
  return dropped_prepare_results_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::dropped_prepare_request_count() const {
  return dropped_prepare_requests_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::dropped_authorization_count() const {
  return dropped_authorizations_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::dropped_stop_count() const {
  return dropped_stops_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::dropped_velocity_intent_count() const {
  return dropped_intents_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::inbound_rejected_count() const {
  return inbound_rejected_.load(std::memory_order_relaxed);
}

std::uint64_t ProducerTransport::inbound_read_failure_count() const {
  return inbound_read_failures_.load(std::memory_order_relaxed);
}

}  // namespace irmv::transport
