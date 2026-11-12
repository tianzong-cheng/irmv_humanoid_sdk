#include <cstdlib>
#include <limits>
#include <memory>
#include <new>

#include "irmv/simulation/barrier.hpp"
#include "irmv/simulation/ingress.hpp"
#include "support/check.hpp"
namespace {
bool forbid_allocation = false;
}
void* operator new(std::size_t n) {
  if (forbid_allocation) std::abort();
  if (auto* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
using namespace irmv::simulation;
int main() {
  PhaseManifest m;
  m.identity.run_id[0] = 1;
  m.sender = Role::Runtime;
  m.consumer = Role::Application;
  m.required = Disposition::BusinessConsumed;
  Barrier b(Role::Application);
  CHECK(!b.Complete(m));
  CHECK(m.Close());
  CHECK(!b.Complete(m));  // empty inventory still needs actual consumer
  CHECK(b.Begin(m));
  Frame ack;
  CHECK(b.MakeAck(m, &ack));
  CHECK(b.Complete(m));
  Barrier coordinator(Role::Coordinator);
  CHECK(coordinator.Begin(m));
  CHECK(coordinator.ObserveAck(ack, Role::Application));
  CHECK(coordinator.Complete(m));
  PhaseManifest n = m;
  n.identity.control_step = 1;
  n.identity.logical_us = 2001;
  n.closed = false;
  irmv::protocol::RobotState s;
  s.state_seq = 7;
  MessageIdentity state;
  CHECK(Identify(s, &state));
  CHECK(n.Add(state));
  irmv::protocol::ExecutionResult r;
  r.request_id = 2;
  MessageIdentity result;
  CHECK(Identify(r, &result));
  CHECK(n.Add(result));
  CHECK(n.Add(result));  // identical declaration dedup
  CHECK(n.count == 2);
  CHECK(n.Close());
  CHECK(b.Begin(n));
  CHECK(!b.MakeAck(n, &ack));
  PhaseReceipt receipt{n.identity,        n.physics_dt_us,      n.control_dt_us,
                       Role::Application, Disposition::Ingress, state};
  CHECK(b.Observe(receipt));
  receipt.message = result;
  CHECK(b.Observe(receipt));
  CHECK(!b.Complete(n));
  receipt.disposition = Disposition::BusinessConsumed;
  CHECK(b.Observe(receipt));
  CHECK(!b.Complete(n));  // last result isn't entire inventory
  receipt.message = state;
  CHECK(b.Observe(receipt));
  CHECK(b.MakeAck(n, &ack));
  CHECK(b.Complete(n));
  CHECK(!coordinator.ObserveAck(ack, Role::Runtime));
  auto bad = n;
  bad.seal.manifest_digest[0] ^= 1;
  CHECK(!bad.Valid());
  bad = n;
  bad.messages[0].content_digest[0] ^= 1;
  CHECK(!bad.Valid());
  auto changed = state;
  changed.content_digest[0] ^= 1;
  receipt.message = changed;
  CHECK(!b.Observe(receipt));
  CHECK(!b.Complete(n));  // conflict poisons phase
  s.feedback.count = 65;
  CHECK(!Identify(s, &state));
  struct Clock : LogicalClock {
    irmv::protocol::TimeUs NowUs() const noexcept override { return 2001; }
  } clock;
  auto ingress = std::make_unique<Ingress>(Role::Application, clock);
  CHECK(ingress->ControlBegin(n));
  ingress->WorkerUpdate();
  irmv::protocol::TimeUs stamp = 0;
  CHECK(ingress->WorkerCheck(n.messages[0], &stamp) == 1);
  CHECK(stamp == 2001);
  CHECK(!ingress->ControlProcessed(n.messages[0]));
  ingress->WorkerStored(n.messages[0], true);
  CHECK(ingress->WorkerCheck(n.messages[0], &stamp) == 0);
  CHECK(ingress->ControlDequeued(n.messages[0]));
  CHECK(ingress->ControlProcessed(n.messages[0]));
  CHECK(!ingress->ControlAck(&ack));
  CHECK(ingress->WorkerCheck(n.messages[1], &stamp) == 1);
  ingress->WorkerStored(n.messages[1], true);
  CHECK(ingress->ControlDequeued(n.messages[1]));
  CHECK(!ingress->ControlAck(&ack));  // dequeue isn't business processing
  CHECK(ingress->ControlProcessed(n.messages[1]));
  CHECK(ingress->ControlAck(&ack));
  // Independent Python hashlib/struct golden, IDL traversal (no CDR padding).
  irmv::protocol::JointCommand command;
  command.command_seq = 17;
  command.source_state_seq = 8;
  command.stamp_us = 123;
  command.joints.count = 1;
  command.joints.q[0] = 1.25;
  command.joints.dq[0] = -2;
  command.joints.tau_ff[0] = 3;
  command.joints.kp[0] = 4;
  command.joints.kd[0] = 5;
  MessageIdentity golden;
  CHECK(Identify(command, &golden));
  const Digest expected{225, 153, 89,  202, 210, 167, 204, 143, 211, 223, 118,
                        117, 245, 174, 254, 117, 173, 226, 69,  146, 140, 49,
                        221, 89,  52,  225, 61,  198, 228, 249, 41,  177};
  CHECK(golden.content_digest == expected);
  command.joints.q[63] = std::numeric_limits<double>::quiet_NaN();
  MessageIdentity tail;
  CHECK(Identify(command, &tail));
  CHECK(SameIdentity(tail, golden));
  command.joints.q[0] = std::numeric_limits<double>::infinity();
  CHECK(!Identify(command, &tail));
  CHECK(!Identify(command, nullptr));
  CHECK(m.seal.manifest_digest ==
        Digest({227, 176, 196, 66,  152, 252, 28,  20, 154, 251, 244,
                200, 153, 111, 185, 36,  39,  174, 65, 228, 100, 155,
                147, 76,  164, 149, 153, 27,  120, 82, 184, 85}));
  auto capacity = std::make_unique<PhaseManifest>(m);
  capacity->closed = false;
  for (std::uint64_t i = 0; i < 64; ++i) {
    auto x = result;
    x.request_id = i;
    CHECK(capacity->Add(x));
  }
  auto extra = result;
  extra.request_id = 65;
  CHECK(!capacity->Add(extra));
  CHECK(capacity->Close());
  CHECK(capacity->Valid());
  auto history = std::make_unique<Barrier>(Role::Application);
  for (std::uint64_t i = 0; i < 33; ++i) {
    auto next = m;
    next.identity.control_step = i;
    next.identity.logical_us = 1 + i * 2000;
    CHECK(history->Begin(next));
    CHECK(history->MakeAck(next, &ack));
  }
  CHECK(!history->Begin(m));
  CHECK(!history->Complete(m));
  auto inconsistent = m;
  inconsistent.identity.control_step =
      1;  // time cannot stay frozen across steps
  CHECK(!history->Begin(inconsistent));
  auto timecheck = std::make_unique<Barrier>(Role::Application);
  CHECK(timecheck->Begin(m));
  CHECK(!timecheck->Begin(inconsistent));
  auto receiver = n;
  receiver.closed = false;
  receiver.count = 0;
  Frame declaration;
  declaration.identity = n.identity;
  declaration.payload = Manifest{n.sender, n.messages[0]};
  CHECK(receiver.Add(declaration));
  CHECK(receiver.Add(declaration));
  CHECK(receiver.count == 1);
  declaration.physics_dt_us = 500;
  CHECK(!receiver.Add(declaration));
  declaration.physics_dt_us = 1000;
  Frame closure;
  closure.identity = n.identity;
  closure.payload = n.seal;
  CHECK(!receiver.Close(closure));  // seal cannot replace missing inventory
  declaration.payload = Manifest{n.sender, n.messages[1]};
  CHECK(receiver.Add(declaration));
  CHECK(receiver.Close(closure));
  CHECK(receiver.Valid());
  closure.identity.phase++;
  CHECK(!receiver.Close(closure));
  auto exact = std::make_unique<Barrier>(Role::Coordinator);
  CHECK(exact->Begin(m));
  auto wrong = ack;
  wrong.identity = m.identity;
  wrong.physics_dt_us = 500;
  CHECK(!exact->ObserveAck(wrong, Role::Application));
  wrong = ack;
  wrong.identity = m.identity;
  std::get<PhaseAck>(wrong.payload).disposition = Disposition::Ingress;
  CHECK(!exact->ObserveAck(wrong, Role::Application));
  wrong = ack;
  wrong.identity = m.identity;
  std::get<PhaseAck>(wrong.payload).seal.manifest_digest[0] ^= 1;
  CHECK(!exact->ObserveAck(wrong, Role::Application));
  CHECK(!exact->Complete(m));
  // Exercise bounded canonical traversal/closure/receipts with heap forbidden.
  auto allocation_check = std::make_unique<Barrier>(Role::Application);
  auto bounded = n;
  bounded.closed = false;
  bounded.count = 0;
  irmv::protocol::RobotState full;
  full.feedback.count = 64;
  full.faults.count = 64;
  forbid_allocation = true;
  CHECK(Identify(full, &state));
  CHECK(bounded.Add(state));
  CHECK(bounded.Close());
  CHECK(allocation_check->Begin(bounded));
  CHECK(allocation_check->Observe({bounded.identity, 1000, 2000,
                                   Role::Application, Disposition::Ingress,
                                   state}));
  CHECK(allocation_check->Observe({bounded.identity, 1000, 2000,
                                   Role::Application,
                                   Disposition::BusinessConsumed, state}));
  CHECK(allocation_check->MakeAck(bounded, &ack));
  CHECK(allocation_check->Complete(bounded));
  forbid_allocation = false;
  return 0;
}
