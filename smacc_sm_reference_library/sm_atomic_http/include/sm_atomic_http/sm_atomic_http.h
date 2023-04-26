#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <sm_atomic_http/clients/cl_http.h>

// ORTHOGONALS
#include <sm_atomic_http/orthogonals/or_http.h>
#include <sm_atomic_http/orthogonals/or_timer.h>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>
#include <sm_atomic_http/clients/client_behaviors/cb_request.h>

using namespace boost;
using namespace smacc;

namespace sm_atomic_http {

// STATE
class State1;
class State2;

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmAtomicHttp
    : public smacc::SmaccStateMachineBase<SmAtomicHttp, State1> {
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrHttp>();
  }
};

}  // namespace sm_atomic_http

#include <sm_atomic_http/states/st_state_1.h>
#include <sm_atomic_http/states/st_state_2.h>