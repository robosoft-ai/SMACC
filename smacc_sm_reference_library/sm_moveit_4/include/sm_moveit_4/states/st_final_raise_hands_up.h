#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_4
{
// STATE DECLARATION
struct StFinalRaiseHandsUp : smacc::SmaccState<StFinalRaiseHandsUp, SmMoveIt4>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        
        Transition<MoveGroupMotionExecutionFailed<StFinalRaiseHandsUp, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbMoveKnownState>("sm_moveit_4", "config/manipulation/known_states/final_raise_hands_up.yaml");
    }

    void runtimeConfigure()
    {
        // to avoid looping very fast if it aborts
        ros::Duration(1).sleep();
    }
};
} // namespace sm_moveit_4