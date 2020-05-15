#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StInitialForward : smacc::SmaccState<StInitialForward, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        // Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotate180, SUCCESS>
        Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS1::SsPickObject, SUCCESS>,
        Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StInitialForward, ABORT>
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
        configure_orthogonal<OrNavigation, CbNavigateForward>(1.2);
    }

    void runtimeConfigure()
    {
        ROS_INFO("runtime");
    }

    void OnEntry()
    {
        ROS_INFO("state on entry");
    }
};
}