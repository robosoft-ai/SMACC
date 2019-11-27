#include <smacc/smacc.h>

namespace sm_viewer_sim
{
struct St1 : SmaccState<St1, MsRunMode>
{
    typedef mpl::list<smacc::transition<Ev1, St2, smacc::SUCCESS>,
                      smacc::transition<EvFail, MsRecoveryMode, smacc::ABORT>>
        reactions;
    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St1");
        delayedPostEvent<Ev1>(this, 7);
    }
};
}