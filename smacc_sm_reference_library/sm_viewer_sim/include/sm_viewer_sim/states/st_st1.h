#include <smacc/smacc.h>

namespace sm_viewer_sim
{
using namespace smacc::default_transition_tags;
struct St1 : SmaccState<St1, MsRunMode>
{
    typedef mpl::list<smacc::transition<Ev1, St2, SUCCESS>,
                      smacc::transition<EvFail, MsRecoveryMode, ABORT>>
        reactions;
    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St1");
        delayedPostEvent<Ev1>(this, 7);
    }
};
} // namespace sm_viewer_sim