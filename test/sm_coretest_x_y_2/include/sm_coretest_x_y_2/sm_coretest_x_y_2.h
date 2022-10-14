#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>

// ORTHOGONALS
#include <sm_coretest_x_y_2/orthogonals/or_timer.h>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

using namespace boost;
using namespace smacc;

namespace sm_coretest_x_y_2
{

struct AutomaticTransitionEvent: sc::event<AutomaticTransitionEvent>
{

};

static int counter = 0;
ros::Time startTime;

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmCoreTestXY2
    : public smacc::SmaccStateMachineBase<SmCoreTestXY2, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        startTime=ros::Time::now();
    }
};

} // namespace sm_coretest_x_y_2

#include <sm_coretest_x_y_2/states/st_state_1.h>
#include <sm_coretest_x_y_2/states/st_state_2.h>
