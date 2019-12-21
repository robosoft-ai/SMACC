#include <smacc/smacc.h>

// CLIENTS
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

// ORTHOGONALS
#include <sm_atomic/orthogonals/or_navigation.h>

//CLIENT BEHAVIORS
#include <sm_atomic/client_behaviors/cb_state_1.h>

using namespace boost;

namespace sm_atomic
{

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicStateMachine
    : public smacc::SmaccStateMachineBase<SmAtomicStateMachine, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrNavigation>();
    }
};

} // namespace sm_atomic

#include <sm_atomic/states/st_state_1.h>
#include <sm_atomic/states/st_state_2.h>