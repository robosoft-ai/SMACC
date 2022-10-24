#include <smacc/smacc.h>

namespace sm_viewer_sim
{
struct MsRunMode : SmaccState<MsRunMode, SmViewerSim, St1>
{
    using SmaccState::SmaccState;
    void onEntry()
    {
        ROS_INFO("MS RUN MODE");
    }
};
}
