#include <smacc/smacc.h>

// ORTHOGONALS
#include "orthogonals/or_subscriber.h"
#include "orthogonals/or_timer.h"

//CLIENTS
#include "clients/cl_numbers_subscription/cl_numbers_subscription.h"
#include <ros_timer_client/cl_ros_timer.h>

// CLIENT BEHAVIORS
#include "clients/cl_numbers_subscription/client_behaviors/cb_post_custom_event.h"
#include "clients/cl_numbers_subscription/client_behaviors/cb_print_message_number.h"

#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

using namespace boost;
using namespace smacc;
using namespace cl_ros_timer;


namespace sm_subscriber
{
// STATE
class State1;
class State2;

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmSubscriber : public smacc::SmaccStateMachineBase<SmSubscriber, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrSubscriber>();
    this->createOrthogonal<OrTimer>();
  }
};

}  // namespace sm_subscriber

#include "states/st_state_1.h"
#include "states/st_state_2.h"
