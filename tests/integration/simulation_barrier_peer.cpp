#include <chrono>
#include <dds/dds.hpp>
#include <iostream>
#include <memory>
#include <thread>

#include "irmv/configuration/application_config.hpp"
#include "irmv/transport/client_transport.hpp"
#include "irmv/transport/codec.hpp"
#include "irmv/transport/dds_transport.hpp"
#include "support/check.hpp"
using namespace irmv;
namespace {
template <class Predicate>
bool WaitFor(Predicate predicate) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  do {
    if (predicate()) return true;
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}
// A business value may be dequeued before the DDS worker publishes its ingress
// receipt. Retry only the captured delivered identity, without taking events
// again or confusing receipt/processing with FSM acceptance.
template <class Transport>
bool ReportProcessed(Transport& transport,
                     const simulation::MessageIdentity& delivered) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  do {
    if (transport.SimulationFailed()) return false;
    if (transport.ReportSimulationProcessed(delivered))
      return !transport.SimulationFailed();
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}
}  // namespace
int main(int argc, char** argv) {
  CHECK(argc == 2);
  auto cfg = configuration::LoadConfig("config/sdk.json", {222});
  auto app = configuration::LoadApplicationConfig("config/sdk.json", {222});
  simulation::SimulationClock clock({1}, 2000);
  auto lower = std::make_unique<transport::DdsTransport>(cfg);
  auto upper = std::make_unique<transport::ClientTransport>(app);
  CHECK(lower->ConfigureSimulation(clock));
  CHECK(upper->ConfigureSimulation(clock));
  lower->Start();
  upper->Start();
  CHECK(!upper->ConfigureSimulation(clock));
  simulation::PhaseManifest phase;
  phase.identity.run_id[0] = 1;
  phase.identity.phase = 1;
  phase.sender = simulation::Role::Runtime;
  phase.consumer = simulation::Role::Application;
  phase.required = simulation::Disposition::BusinessConsumed;
  runtime::TickResult tick;
  tick.state.state_seq = 1;
  tick.state.stamp_us = 1;
  tick.state.feedback.count = 2;
  tick.result_count = 1;
  tick.results[0].request_id = 5;
  tick.results[0].status = protocol::ResultStatus::Rejected;
  simulation::MessageIdentity state, result;
  CHECK(simulation::Identify(tick.state, &state));
  CHECK(simulation::Identify(tick.results[0], &result));
  CHECK(phase.Add(state));
  CHECK(phase.Add(result));
  CHECK(phase.Close());
  simulation::Frame ack;
  auto empty = phase;
  empty.closed = false;
  empty.count = 0;
  empty.identity.phase = 0;
  CHECK(empty.Close());
  CHECK(!upper->SimulationPhaseInstalled(empty));
  CHECK(upper->BeginSimulationPhase(empty));
  CHECK(WaitFor([&] { return upper->SimulationPhaseInstalled(empty); }));
  CHECK(upper->TrySimulationAck(&ack));
  auto lower_empty = empty;
  lower_empty.sender = simulation::Role::Application;
  lower_empty.consumer = simulation::Role::Runtime;
  lower_empty.closed = false;
  CHECK(lower_empty.Close());
  CHECK(!lower->SimulationPhaseInstalled(lower_empty));
  CHECK(lower->BeginSimulationPhase(lower_empty));
  CHECK(WaitFor([&] { return lower->SimulationPhaseInstalled(lower_empty); }));
  CHECK(lower->TrySimulationAck(&ack));
  CHECK(upper->BeginSimulationPhase(phase));
  CHECK(!upper->SimulationPhaseInstalled(empty));
  CHECK(WaitFor([&] { return upper->SimulationPhaseInstalled(phase); }));
  auto mismatch = phase;
  mismatch.count = 1;
  mismatch.closed = false;
  CHECK(mismatch.Close());
  CHECK(!upper->SimulationPhaseInstalled(mismatch));
  CHECK(!upper->TrySimulationAck(&ack));
  // Installation was observed above, separately from DDS discovery and ingress.
  // Independent test publisher has its own DDS owner (this main thread).
  // Transports still have exactly their original worker and no helper DDS
  // calls.
  namespace wire = protocol::wire;
  dds::domain::DomainParticipant participant(222);
  dds::pub::Publisher publisher(participant);
  dds::topic::Topic<wire::RobotState> state_topic(
      participant, std::string(protocol::kRobotStateTopic.name));
  dds::topic::Topic<wire::ExecutionResult> result_topic(
      participant, std::string(protocol::kExecutionResultTopic.name));
  dds::pub::DataWriter<wire::RobotState> state_writer(
      publisher, state_topic,
      transport::codec::ToWriterQos(protocol::kRobotStateTopic.qos));
  dds::pub::DataWriter<wire::ExecutionResult> result_writer(
      publisher, result_topic,
      transport::codec::ToWriterQos(protocol::kExecutionResultTopic.qos));
  wire::RobotState wire_state;
  wire::ExecutionResult wire_result;
  transport::codec::ToWire(tick.state, wire_state);
  transport::codec::ToWire(tick.results[0], wire_result);
  CHECK(WaitFor([&] {
    return state_writer.publication_matched_status().current_count() > 0 &&
           result_writer.publication_matched_status().current_count() > 0;
  }));
  bool state_first = std::string(argv[1]) == "state-first";
  if (state_first)
    state_writer.write(wire_state);
  else
    result_writer.write(wire_result);
  const transport::ClientEvents* first_batch = nullptr;
  CHECK(WaitFor([&] {
    first_batch = &upper->TakeEvents();
    return state_first ? first_batch->has_state
                       : first_batch->result_count == 1;
  }));
  const auto& first = *first_batch;
  CHECK(first.has_state == state_first);
  CHECK(first.result_count == (state_first ? 0 : 1));
  simulation::MessageIdentity delivered_first;
  CHECK(state_first
            ? simulation::Identify(first.state, &delivered_first)
            : simulation::Identify(first.results[0].value, &delivered_first));
  CHECK(
      simulation::SameIdentity(delivered_first, state_first ? state : result));
  CHECK(ReportProcessed(*upper, delivered_first));
  CHECK(!upper->TrySimulationAck(&ack));
  if (state_first)
    result_writer.write(wire_result);
  else
    state_writer.write(wire_state);
  bool got_state = state_first, got_result = !state_first;
  simulation::MessageIdentity delivered_second;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (std::chrono::steady_clock::now() < deadline &&
         (!got_state || !got_result)) {
    const auto& events = upper->TakeEvents();
    if (events.has_state) {
      CHECK(events.state_rx_us == 1);
      CHECK(simulation::Identify(events.state, &delivered_second));
      got_state = true;
    }
    for (std::size_t i = 0; i < events.result_count; ++i) {
      CHECK(events.results[i].rx_us == 1);
      CHECK(simulation::Identify(events.results[i].value, &delivered_second));
      got_result = true;
    }
    std::this_thread::yield();
  }
  CHECK(got_state && got_result);
  CHECK(!upper->TrySimulationAck(&ack));  // dequeued, not processed
  CHECK(ReportProcessed(*upper, delivered_first));
  CHECK(!upper->TrySimulationAck(&ack));
  CHECK(
      simulation::SameIdentity(delivered_second, state_first ? result : state));
  CHECK(ReportProcessed(*upper, delivered_second));
  CHECK(upper->TrySimulationAck(&ack));
  CHECK(clock.Commit({{1}, 1, 0, 2001}));
  state_writer.write(wire_state);
  result_writer.write(wire_result);  // delayed old phase retransmission

  // Publication alone is not evidence the worker observed these resends.
  // Old-sample deduplication/no restamping is proved deterministically in the
  // core Ingress test; later exact ingress receipts check forward DDS progress.
  auto burst = phase;
  burst.closed = false;
  burst.count = 0;
  burst.identity.control_step = 1;
  burst.identity.logical_us = 2001;
  auto s3 = tick.state;
  s3.state_seq = 3;
  s3.stamp_us = 2001;
  auto s4 = s3;
  s4.state_seq = 4;
  simulation::MessageIdentity three, four;
  CHECK(simulation::Identify(s3, &three));
  CHECK(simulation::Identify(s4, &four));
  CHECK(burst.Add(three));
  CHECK(burst.Add(four));
  CHECK(burst.Close());
  CHECK(upper->BeginSimulationPhase(burst));
  CHECK(!upper->SimulationPhaseInstalled(phase));
  CHECK(WaitFor([&] { return upper->SimulationPhaseInstalled(burst); }));
  transport::codec::ToWire(s3, wire_state);
  state_writer.write(wire_state);
  simulation::PhaseReceipt ingress_receipt;
  CHECK(WaitFor([&] { return upper->TrySimulationReceipt(&ingress_receipt); }));
  CHECK(simulation::SameIdentity(ingress_receipt.message, three));
  transport::codec::ToWire(s4, wire_state);
  state_writer.write(wire_state);
  CHECK(WaitFor([&] { return upper->TrySimulationReceipt(&ingress_receipt); }));
  CHECK(simulation::SameIdentity(ingress_receipt.message, four));
  CHECK(upper->TakeEvents().state.state_seq == 3);
  CHECK(upper->ReportSimulationProcessed(three));
  CHECK(!upper->TrySimulationAck(&ack));
  CHECK(upper->TakeEvents().state.state_seq == 4);
  CHECK(upper->ReportSimulationProcessed(four));
  CHECK(upper->TrySimulationAck(&ack));
  // Reverse direction: two commands must survive without latest-value eviction.
  simulation::PhaseManifest inbound;
  inbound.identity.run_id[0] = 1;
  inbound.identity.control_step = 1;
  inbound.identity.logical_us = 2001;
  inbound.sender = simulation::Role::Application;
  inbound.consumer = simulation::Role::Runtime;
  inbound.required = simulation::Disposition::BusinessConsumed;
  protocol::JointCommand c1, c2;
  c1.command_seq = 1;
  c1.stamp_us = 1;
  c1.joints.count = 2;
  c2 = c1;
  c2.command_seq = 2;
  protocol::ExecutionRequest stop;
  stop.operation = protocol::Operation::Damping;
  stop.request_id = 9;
  protocol::SupervisorHeartbeat heartbeat;
  heartbeat.heartbeat_seq = 1;
  heartbeat.stamp_us = 1;
  simulation::MessageIdentity one, two, stop_id, hb_id;
  CHECK(simulation::Identify(c1, &one));
  CHECK(simulation::Identify(c2, &two));
  CHECK(simulation::Identify(stop, &stop_id));
  CHECK(simulation::Identify(heartbeat, &hb_id));
  CHECK(inbound.Add(one));
  CHECK(inbound.Add(two));
  CHECK(inbound.Add(stop_id));
  CHECK(inbound.Add(hb_id));
  CHECK(inbound.Close());
  CHECK(lower->BeginSimulationPhase(inbound));
  CHECK(!lower->SimulationPhaseInstalled(lower_empty));
  CHECK(WaitFor([&] { return lower->SimulationPhaseInstalled(inbound); }));
  mismatch = inbound;
  mismatch.count = 1;
  mismatch.closed = false;
  CHECK(mismatch.Close());
  CHECK(!lower->SimulationPhaseInstalled(mismatch));
  CHECK(upper->PublishCommand(c1));
  CHECK(upper->PublishRequest(stop));
  CHECK(upper->PublishHeartbeat(heartbeat));
  // Observe all first-burst admissions without dequeuing the business lanes.
  unsigned receipts = 0;
  CHECK(WaitFor([&] {
    if (lower->TrySimulationReceipt(&ingress_receipt)) ++receipts;
    return receipts == 3;
  }));
  CHECK(upper->PublishCommand(c2));
  CHECK(WaitFor([&] { return lower->TrySimulationReceipt(&ingress_receipt); }));
  CHECK(simulation::SameIdentity(ingress_receipt.message, two));
  const auto& input = lower->TakeInput();
  CHECK(input.command_count == 2);
  CHECK(input.stop_request_count == 1);
  CHECK(input.heartbeat_count == 1);
  CHECK(input.commands[0].value.command_seq == 1);
  CHECK(input.commands[1].value.command_seq == 2);
  CHECK(input.commands[0].rx_us == 2001 && input.commands[1].rx_us == 2001);
  CHECK(input.stop_requests[0].rx_us == 2001 &&
        input.heartbeats[0].rx_us == 2001);
  CHECK(!lower->TrySimulationAck(&ack));
  CHECK(lower->ReportSimulationProcessed(two));
  CHECK(lower->ReportSimulationProcessed(one));
  CHECK(lower->ReportSimulationProcessed(stop_id));
  CHECK(lower->ReportSimulationProcessed(hb_id));
  CHECK(lower->TrySimulationAck(&ack));
  // A changed payload under the same command sequence is rejected whole.
  c2.joints.q[0] = 1;
  CHECK(upper->PublishCommand(c2));
  CHECK(WaitFor([&] { return lower->SimulationFailed(); }));
  CHECK(lower->TakeInput().command_count == 0);
  upper->Stop();
  lower->Stop();
  std::cout << "installation-ready empty/nonempty both-transports " << argv[1]
            << "\n";
  return 0;
}
