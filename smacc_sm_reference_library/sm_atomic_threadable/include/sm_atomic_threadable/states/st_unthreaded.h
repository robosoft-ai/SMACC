#pragma once

#include <smacc/smacc.h>

namespace sm_atomic_threadable {
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct StUnthreaded : smacc::SmaccState<StUnthreaded, SmAtomicThreadable> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<cl_keyboard::EvKeyPressD<
                     cl_keyboard::CbDefaultKeyboardBehavior, OrKeyboard>,
                 StThreaded, SUCCESS>>
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrListener, CbUnthreaded>();
    configure_orthogonal<OrTalker, CbTalker<1>>();
    configure_orthogonal<OrTalker, CbTalker<2>>();
    configure_orthogonal<OrKeyboard, cl_keyboard::CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { ROS_INFO("Entering StMain!"); }

  void onExit() { ROS_INFO("Exiting StMain!"); }
};
}  // namespace sm_atomic_threadable
