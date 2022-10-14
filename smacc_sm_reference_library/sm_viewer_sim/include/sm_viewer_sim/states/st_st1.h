#include <smacc/smacc.h>

namespace sm_viewer_sim
{
using namespace smacc::default_transition_tags;
struct St1 : SmaccState<St1, MsRunMode>
{
    typedef mpl::list<
        Transition<Ev1, St2, SUCCESS>,
        Transition<EvFail, MsRecoveryMode, ABORT>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St1");
        delayedPostEvent<Ev1>(this, 7);
    }
};
} // namespace sm_viewer_sim
