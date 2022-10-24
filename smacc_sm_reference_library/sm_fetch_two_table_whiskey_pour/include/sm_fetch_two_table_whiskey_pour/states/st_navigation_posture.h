#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_two_table_whiskey_pour
{
    struct StNavigationPosture : smacc::SmaccState<StNavigationPosture, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbUndoLastTrajectory, OrArm>, StNavigateToSourceTable, SUCCESS>
            /*Transition<EvCbSuccess<CbUndoLastTrajectory, OrArm>, StNavigateToSourceTable>*/
            /*Transition<EvCbFailure<CbUndoLastTrajectory, OrArm>, StNavigationPosture, ABORT>*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_two_table_whiskey_pour", "config/manipulation/known_states/nav_posture.yaml");
            configure_orthogonal<OrArm, CbUndoLastTrajectory>(1);
            configure_orthogonal<OrNavigation, CbNavigateBackwards>(0.4);
        }

        void runtimeConfigure()
        {
            ros::WallDuration(2).sleep();
        }

        void onExit()
        {
            ClPerceptionSystem *perceptionSystem;
            this->requiresClient(perceptionSystem);
            perceptionSystem->nextObject();
        }
    };
} // namespace sm_fetch_two_table_whiskey_pour
