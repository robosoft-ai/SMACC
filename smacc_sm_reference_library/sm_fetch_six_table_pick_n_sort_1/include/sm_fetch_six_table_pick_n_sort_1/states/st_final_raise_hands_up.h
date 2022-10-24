#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_six_table_pick_n_sort_1
{
// STATE DECLARATION
struct StFinalRaiseHandsUp : smacc::SmaccState<StFinalRaiseHandsUp, SmFetchSixTablePickNSort1>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<EvCbFailure<CbMoveKnownState, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_six_table_pick_n_sort_1", "config/manipulation/known_states/final_raise_hands_up.yaml");
    }

    void runtimeConfigure()
    {
        // to avoid looping very fast if it aborts
        ros::Duration(1).sleep();
    }

    void onExit(ABORT)
    {
        ros::Duration(1).sleep();
    }
};
} // namespace sm_fetch_six_table_pick_n_sort_1
