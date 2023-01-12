#pragma once

#include <smacc/smacc.h>

#include "sm_atomic_threadable/states/st_threaded.h"

namespace sm_atomic_threadable {
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct StMain : smacc::SmaccState<StMain, SmAtomicThreadable> {
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<cl_keyboard::EvKeyPressD<
                     cl_keyboard::CbDefaultKeyboardBehavior, OrKeyboard>,
                 StThreaded, SUCCESS> >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrKeyboard, cl_keyboard::CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { ROS_INFO("Entering StMain!"); }

  void onExit() { ROS_INFO("Exiting StMain!"); }
};
}  // namespace sm_atomic_threadable
