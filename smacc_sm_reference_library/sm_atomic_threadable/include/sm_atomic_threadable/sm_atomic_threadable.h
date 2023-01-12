#pragma once

#include <smacc/smacc.h>

// CLIENTS
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
#include <sm_atomic_threadable/orthogonals/or_keyboard.h>

// CLIENT BEHAVIORS
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>

#include "sm_atomic_threadable/clients/cl_thread/client_behaviors/cb_thread.h"

using namespace boost;
using namespace smacc;

namespace sm_atomic_threadable {

// STATE
class StMain;
class StThreaded;

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmAtomicThreadable
    : public smacc::SmaccStateMachineBase<SmAtomicThreadable, StMain> {
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override { this->createOrthogonal<OrKeyboard>(); }
};

}  // namespace sm_atomic_threadable

#include <sm_atomic_threadable/states/st_main.h>
#include <sm_atomic_threadable/states/st_threaded.h>
