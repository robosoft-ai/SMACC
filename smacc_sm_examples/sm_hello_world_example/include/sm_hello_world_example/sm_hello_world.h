#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <sm_hello_world_example/clients/client_1.h>
#include <sm_hello_world_example/clients/client_2.h>

// ORTHOGONALS
#include <sm_hello_world_example/orthogonals/orthogonal_1.h>
#include <sm_hello_world_example/orthogonals/orthogonal_2.h>

//SUBSTATE BEHAVIORS
#include <sm_hello_world_example/substate_behaviors/client_1/substate_behavior_1.h>
#include <sm_hello_world_example/substate_behaviors/client_1/substate_behavior_1b.h>
#include <sm_hello_world_example/substate_behaviors/client_2/substate_behavior_2.h>
#include <sm_hello_world_example/substate_behaviors/client_2/substate_behavior_2b.h>

//LOGIC UNITS
#include <event_aggregator/logic_units/lu_event_all.h>

namespace hello_world_example
{
//STATES
class StState1; // first state specially needs a forward declaration
class StState2;
class StState3;

// STATE MACHINE
struct SmHelloWorld
    : public smacc::SmaccStateMachineBase<SmHelloWorld, StState1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<Orthogonal1>();
        this->createOrthogonal<Orthogonal2>();
    }
};
} // namespace hello_world_example

//STATES
#include <sm_hello_world_example/states/st_state_1.h>
#include <sm_hello_world_example/states/st_state_2.h>
#include <sm_hello_world_example/states/st_state_3.h>