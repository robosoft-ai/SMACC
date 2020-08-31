#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_screw_loop_1
{
    // STATE DECLARATION
    struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmFetchScrewLoop1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            // Transition<Finished<CbMoveKnownState, OrNavigation>, StNavigateToSourceTable, SUCCESS>,

            // Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StNavigateToSourceTable, SUCCESS>,
            // Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/

            //Transition<EvCbSuccess<CbMoveKnownState, OrArm>, StNavigateToSourceTable, SUCCESS>,
            //Transition<EvCbFailure<CbMoveKnownState, OrArm>, StInitialPosture, ABORT>,/*retry on failure*/

            Transition<EvCbFailure<CbMoveEndEffector, OrArm>, StFinalRaiseHandsUp, ABORT>,/*retry on failure*/
            Transition<EvCbSuccess<CbMoveEndEffector, OrArm>, StSecondPosture, SUCCESS>>
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_screw_loop_1", "config/manipulation/known_states/initial_posture.yaml");

            geometry_msgs::PoseStamped pivot;
            pivot.pose.position.x = 0.6;
            pivot.pose.position.y = 0;
            pivot.pose.position.z = 0.6;
            pivot.pose.orientation.w = 1;
            pivot.header.frame_id = "map";
            configure_orthogonal<OrArm, CbMoveEndEffector>(pivot, "gripper_link");
            configure_orthogonal<OrGripper, CbOpenGripper>();
        }

        void runtimeConfigure()
        {
        }

        void onExit()
        {
            ros::Duration(2.0).sleep();
        }
    };
} // namespace sm_fetch_screw_loop_1