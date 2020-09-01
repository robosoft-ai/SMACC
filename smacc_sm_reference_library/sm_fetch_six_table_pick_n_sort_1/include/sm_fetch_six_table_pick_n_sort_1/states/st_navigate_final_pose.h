#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_six_table_pick_n_sort_1
{
// STATE DECLARATION
struct StNavigateFinalPose : smacc::SmaccState<StNavigateFinalPose, SmFetchSixTablePickNSort1>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StFinalRaiseHandsUp, SUCCESS>,
        Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateFinalPose,ABORT> /*retry on failure*/
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
} // namespace sm_fetch_six_table_pick_n_sort_1