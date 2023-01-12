#pragma once

#include <smacc/smacc.h>

namespace sm_atomic_threadable {
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct StThreaded : smacc::SmaccState<StThreaded, SmAtomicThreadable> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<cl_keyboard::EvKeyPressA<
                     cl_keyboard::CbDefaultKeyboardBehavior, OrKeyboard>,
                 StUnthreaded, SUCCESS>>
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrListener, CbThreaded>();
    configure_orthogonal<OrTalker, CbTalker<1>>();
    configure_orthogonal<OrTalker, CbTalker<2>>();
    configure_orthogonal<OrKeyboard, cl_keyboard::CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { ROS_INFO("Entering StThreaded"); }

  void onExit() { ROS_INFO("Exiting StThreaded!"); }
};
}  // namespace sm_atomic_threadable
