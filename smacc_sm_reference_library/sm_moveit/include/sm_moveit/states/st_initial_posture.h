#include <smacc/smacc.h>
namespace sm_moveit
{
// STATE DECLARATION
struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StInitialForward>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrNavigation, CbMoveKnownState>("sm_moveit", "config/manipulation/known_states/initial_posture.yaml");
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_moveit