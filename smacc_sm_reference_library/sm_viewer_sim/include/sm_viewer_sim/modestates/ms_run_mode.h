#include <smacc/smacc.h>

namespace sm_viewer_sim
{
struct MsRunMode : SmaccState<MsRunMode, SmViewerSim, St1, sc::has_full_history>
{
    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("MS RUN MODE");
    }
};
}