namespace SS1
{
//forward declaration for initial ssr
class ssr_radial_rotate;
class ssr_radial_return;

struct ss_radial_pattern_1 : smacc::SmaccState<ss_radial_pattern_1, sm_dance_bot, ssr_radial_rotate>
{
    public:

    using SmaccState::SmaccState;

    typedef mpl::list<sc::custom_reaction<smacc::EvStateFinished<ssr_radial_return>>,
                      sc::transition<EvFinish, st_rotate_degrees_1>> reactions;

    int iteration_count;

    void onEntry()
    {
        iteration_count = 0;
    }

    sc::result react(const smacc::EvStateFinished<ssr_radial_return> &ev) 
    {
        if(iteration_count++ == 4)
        {
            this->throwFinishEvent();
        }
    }
};

//forward declaration for the superstate
using SS = ss_radial_pattern_1;
#include <sm_dance_bot/superstate_routines/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/ssr_radial_rotate.h>
} // namespace SS1
