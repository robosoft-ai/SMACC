#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <sm_hello_world_example/clients/client_1.h>
#include <sm_hello_world_example/clients/client_2.h>

// ORTHOGONALS
#include <sm_hello_world_example/orthogonals/orthogonal_1.h>
#include <sm_hello_world_example/orthogonals/orthogonal_2.h>

//SUBSTATE BEHAVIORS

//LOGIC UNITS
#include <event_aggregator/logic_units/lu_event_all.h>

namespace hello_world_example
{
//STATES
class StState1;
class StState2;

// STATE MACHINE
struct SmHelloWorld
    : public smacc::SmaccStateMachineBase<SmHelloWorld, StState1>
{
    SmHelloWorld(my_context ctx, smacc::SignalDetector *signalDetector)
        : SmaccStateMachineBase<SmHelloWorld, StState1>(ctx, signalDetector)
    {
        this->createOrthogonal<Orthogonal1>();
        this->createOrthogonal<Orthogonal2>();
    }
};
} // namespace hello_world_example

//STATES
#include <sm_hello_world_example/states/st_state_1.h>
#include <sm_hello_world_example/states/st_state_2.h>