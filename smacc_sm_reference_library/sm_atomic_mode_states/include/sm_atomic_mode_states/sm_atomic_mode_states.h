#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>

// ORTHOGONALS
#include <sm_atomic_mode_states/orthogonals/or_timer.h>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>
#include <sm_atomic_mode_states/client_behaviors/cb_updatable_test.h>

using namespace boost;
using namespace smacc;

namespace sm_atomic_mode_states
{

//STATE
class State1;
class State2;
class MsState1;
class MsState2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomic
    : public smacc::SmaccStateMachineBase<SmAtomic, MsState1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
    }
};

} // namespace sm_atomic_mode_states

#include <sm_atomic_mode_states/states/ms_state_1.h>
#include <sm_atomic_mode_states/states/ms_state_2.h>

#include <sm_atomic_mode_states/states/st_state_1.h>
#include <sm_atomic_mode_states/states/st_state_2.h>
