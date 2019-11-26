#include <smacc/smacc.h>

// CLIENTS
#include <smacc_navigation_plugin/move_base_action_client.h>

// ORTHOGONALS
#include <sm_atomic/orthogonals/navigation_orthogonal.h>

//SUBSTATE BEHAVIORS
#include <sm_atomic/substate_behaviors/sb_state_1.h>

namespace sm_atomic
{

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct BaseStateMachine
    : public smacc::SmaccStateMachineBase<BaseStateMachine, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<NavigationOrthogonal>();
    }
};

} // namespace sm_atomic

#include <sm_atomic/states/st_state_1.h>
#include <sm_atomic/states/st_state_2.h>