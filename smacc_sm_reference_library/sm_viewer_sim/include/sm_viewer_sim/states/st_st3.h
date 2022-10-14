
#include <smacc/smacc.h>

namespace sm_viewer_sim
{
struct St3 : SmaccState<St3, MsRunMode>
{
    typedef mpl::list<
        Transition<Ev3, St1, smacc::SUCCESS>,
        Transition<EvFail, MsRecoveryMode, smacc::ABORT>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("St3");
        delayedPostEvent<Ev3>(this, 3);
    }
};
}
