#pragma once
namespace sm_three_some
{
namespace SS1
{

//HERE WE MAKE FORWARD DECLARATIONS OF ALL SUBSTATE ROUTINES
class Ssr1;
class Ssr2;
class Ssr3;

struct Ss1 : smacc::SmaccState<Ss1, SmThreeSome, Ssr1>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<
        // smacc::transition<EvSuperstateFinish<Ssr3>, StState1>

        // Keyboard events
        smacc::transition<EvLoopEnd<Ssr1>, StState1>>
        reactions;

    static constexpr int total_iterations() { return 3; }
    int iteration_count = 0;

    static void onDefinition()
    {
    }

    void onInitialize()
    {
    }
}; // namespace SS4

//forward declaration for the superstate
using SS = SS1::Ss1;

#include <sm_three_some/superstate_routines/ss_superstate_1/ssr_1.h>
#include <sm_three_some/superstate_routines/ss_superstate_1/ssr_2.h>
#include <sm_three_some/superstate_routines/ss_superstate_1/ssr_3.h>

} // namespace SS1
} // namespace sm_three_some

