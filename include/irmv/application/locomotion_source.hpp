#pragma once

// Who produces the joint targets of the locomotion behaviour. It is chosen once
// at startup and never changes while the process runs: switching sources is a
// stop, an initialization and a new authorization, not a hot swap.
//
// Both the behaviour and the supervision loop have to know it, which is why it
// lives on its own: the behaviour decides whether a source owns its own frame
// stream - an external producer publishes on the execution topic itself, a
// local one is relayed by this process - and the loop decides what to run.
namespace irmv::application {

enum class LocomotionSource {
  // No source at all: the operator may ask for the locomotion behaviour, but
  // nothing is there to produce a first frame, so the behaviour keeps holding
  // Ready. This is what a deployment without a configured model resolves to,
  // and it is the shipped Fake configuration's own mode.
  None,
  // The model named by `application.policy.model_path`, run in this process.
  Local,
  // A producer process that registers with this application over DDS. It
  // publishes the frames of an authorized context itself, under the producer
  // identity the runtime granted, so this process must not publish any.
  External,
};

}  // namespace irmv::application
