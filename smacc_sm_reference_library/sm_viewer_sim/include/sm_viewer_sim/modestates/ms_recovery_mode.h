#include <smacc/smacc.h>

namespace sm_viewer_sim
{
struct MsRecoveryMode : SmaccState<MsRecoveryMode, SmViewerSim>
{
    typedef mpl::list<
        smacc::Transition<EvToDeep, sc::deep_history<St1>, SUCCESS>>
        reactions;

    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("Recovery");
        delayedPostEvent<EvToDeep>(this, 3);
    }
};
}