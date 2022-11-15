#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>

// ORTHOGONALS
#include <sm_atomic_threadable/orthogonals/or_timer.h>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>
#include <sm_atomic_threadable/clients/cl_thread/client_behaviors/cb_thread.h>

using namespace boost;
using namespace smacc;

namespace sm_atomic_threadable {

// STATE
class State1;
class State2;

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmAtomicThreadable
    : public smacc::SmaccStateMachineBase<SmAtomicThreadable, State1> {
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override { this->createOrthogonal<OrTimer>(); }
};

}  // namespace sm_atomic_threadable

#include <sm_atomic_threadable/states/st_state_1.h>
#include <sm_atomic_threadable/states/st_state_2.h>
