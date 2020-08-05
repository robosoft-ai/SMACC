#include <smacc/smacc.h>
namespace sm_fetch_two_table_pick_n_place_1
{
// STATE DECLARATION
struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmFetchTwoTablePickNPlace1>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StInitialForward>,
        Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrNavigation, CbMoveKnownState>("sm_fetch_two_table_pick_n_place_1", "config/manipulation/known_states/initial_posture.yaml");
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_fetch_two_table_pick_n_place_1