#pragma once

#include <smacc/smacc.h>

// CLIENTS
#include <keyboard_client/cl_keyboard.h>
#include <ros_timer_client/cl_ros_timer.h>

#include "sm_atomic_threadable/clients/cl_listener/cl_listener.h"
#include "sm_atomic_threadable/clients/cl_talker/cl_talker.h"

// ORTHOGONALS
#include "sm_atomic_threadable/orthogonals/or_keyboard.h"
#include "sm_atomic_threadable/orthogonals/or_listener.h"
#include "sm_atomic_threadable/orthogonals/or_talker.h"
#include "sm_atomic_threadable/orthogonals/or_timer.h"

// CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include "sm_atomic_threadable/clients/cl_listener/client_behaviors/cb_threaded.h"
#include "sm_atomic_threadable/clients/cl_listener/client_behaviors/cb_unthreaded.h"
#include "sm_atomic_threadable/clients/cl_talker/client_behaviors/cb_talker.h"

using namespace boost;
using namespace smacc;

namespace sm_atomic_threadable {

// STATE
class StUnthreaded;
class StThreaded;

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmAtomicThreadable
    : public smacc::SmaccStateMachineBase<SmAtomicThreadable, StUnthreaded> {
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrListener>();
    this->createOrthogonal<OrTalker>();
    this->createOrthogonal<OrKeyboard>();
  }
};

}  // namespace sm_atomic_threadable

#include <sm_atomic_threadable/states/st_threaded.h>
#include <sm_atomic_threadable/states/st_unthreaded.h>
