#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include <sm_atomic/clients/cl_odd_pub/cl_odd_pub.h>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>
#include <sm_atomic/clients/cl_odd_pub/client_behaviors/cb_odd_pub.h>

using namespace sm_atomic::cl_odd_pub;

using namespace boost;
using namespace smacc;

// ORTHOGONALS
#include <sm_atomic/orthogonals/or_timer.h>
#include <sm_atomic/orthogonals/or_odd_pub.h>

namespace sm_atomic
{

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomic
    : public smacc::SmaccStateMachineBase<SmAtomic, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrOddPub>();
    }
};

} // namespace sm_atomic

#include <sm_atomic/states/st_state_1.h>
#include <sm_atomic/states/st_state_2.h>