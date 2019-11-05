#pragma once
namespace hello_world_example
{
namespace SS1
{

//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class Ssr1;
class Ssr2;
class Ssr3;

struct Ss1 : smacc::SmaccState<Ss1, hello_world_example::SmHelloWorld,Ssr1>
{
public:
    using SmaccState::SmaccState;

    typedef smacc::transition<EvLoopContinue<Ssr1>, StState1> reactions;


    static void onDefinition()
    {
    }

    void onInitialize()
    {
        
    }
}; // namespace SS4

//forward declaration for the superstate
using SS = SS1::Ss1;
#include <sm_hello_world_example/superstate_routines/ss_superstate_1/ssr_1.h>
#include <sm_hello_world_example/superstate_routines/ss_superstate_1/ssr_2.h>
#include <sm_hello_world_example/superstate_routines/ss_superstate_1/ssr_3.h>
} // namespace SS4
}