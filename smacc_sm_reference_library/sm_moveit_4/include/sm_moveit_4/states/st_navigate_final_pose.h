#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_4
{
// STATE DECLARATION
struct StNavigateFinalPose : smacc::SmaccState<StNavigateFinalPose, SmMoveIt4>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StFinalRaiseHandsUp, SUCCESS>,
        Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateFinalPose,ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.0F,0.0F,(float)-M_PI/2.0);
    }

    void onExit()
    {
        // to avoid looping very fast if it aborts
        ros::Duration(1).sleep();
    }
};
} // namespace sm_moveit_4