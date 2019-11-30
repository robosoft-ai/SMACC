#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <sm_three_some/clients/client_1.h>
#include <sm_three_some/clients/client_2.h>

// ORTHOGONALS
#include <sm_three_some/orthogonals/orthogonal_1.h>
#include <sm_three_some/orthogonals/orthogonal_2.h>

//SUBSTATE BEHAVIORS
#include <sm_three_some/substate_behaviors/client_1/substate_behavior_1.h>
#include <sm_three_some/substate_behaviors/client_1/substate_behavior_1b.h>
#include <sm_three_some/substate_behaviors/client_2/substate_behavior_2.h>
#include <sm_three_some/substate_behaviors/client_2/substate_behavior_2b.h>

//LOGIC UNITS
#include <all_events_go/lu_all_events_go.h>

using namespace smacc;

namespace sm_three_some
{
//SUPERSTATES
namespace SS1
{
class Ss1;
}

//STATES
class StState1; // first state specially needs a forward declaration
class StState2;
class StState3;

// STATE MACHINE
struct SmThreeSome
    : public smacc::SmaccStateMachineBase<SmThreeSome, StState1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<Orthogonal1>();
        this->createOrthogonal<Orthogonal2>();
    }
};
} // namespace sm_three_some

//STATES
#include <sm_three_some/states/st_state_1.h>
#include <sm_three_some/states/st_state_2.h>
#include <sm_three_some/states/st_state_3.h>

#include <sm_three_some/superstates/ss_superstate_1.h>