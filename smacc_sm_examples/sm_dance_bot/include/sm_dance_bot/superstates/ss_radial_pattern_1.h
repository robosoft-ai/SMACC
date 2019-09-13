
namespace SS1
{
//forward declaration for initial ssr
class ssr_radial_rotate;
class ssr_radial_return;

struct ss_radial_pattern_1 : smacc::SmaccState<ss_radial_pattern_1, sm_dance_bot, ssr_radial_rotate>
{
public:
    using SmaccState::SmaccState;

    typedef mpl::list<sc::custom_reaction<smacc::EvStateFinish<ssr_radial_return>>,
                      sc::transition<EvStateFinish<ss_radial_pattern_1>, st_rotate_degrees_1>>
        reactions;

    
    int iteration_count;

    void onEntry()
    {
        iteration_count = 0;
    }

    sc::result react(const smacc::EvStateFinish<ssr_radial_return> &ev)
    {
        ROS_INFO("Superstate count: %d", iteration_count);
        if (++iteration_count == 2) // 1 == two times
        {
            this->throwFinishEvent();
        }
    }
};
using SS = ss_radial_pattern_1;
//forward declaration for the superstate
#include <sm_dance_bot/superstate_routines/ssr_radial_end_point.h>
#include <sm_dance_bot/superstate_routines/ssr_radial_return.h>
#include <sm_dance_bot/superstate_routines/ssr_radial_rotate.h>
}