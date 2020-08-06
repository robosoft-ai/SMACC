#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_six_table_pick_n_sort_1
{
    // STATE DECLARATION
    struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            // Transition<Finished<CbMoveKnownState, OrNavigation>, StNavigateToSourceTable, SUCCESS>,

            // Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StNavigateToSourceTable, SUCCESS>,
            // Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
            
            Transition<EvCbSuccess<CbMoveKnownState, OrArm>, StNavigateToSourceTable, SUCCESS>,
            Transition<EvCbFailure<CbMoveKnownState, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_six_table_pick_n_sort_1", "config/manipulation/known_states/initial_posture.yaml");
        }

        void runtimeConfigure()
        {
        }

        void onExit(SUCCESS)
        {
        }

        void onExit(ABORT)
        {
            // to avoid looping very fast if it aborts
            ros::Duration(1).sleep();
        }

        void onExit()
        {
        }
    };
} // namespace sm_fetch_six_table_pick_n_sort_1