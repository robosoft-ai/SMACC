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

            // Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StNavigateToSourceTable, SUCCESS>,
            // Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
            
            Transition<EvCbSuccess<CbMoveKnownState, OrArm>, StNavigateToSourceTable, SUCCESS>,
            Transition<EvCbFailure<CbMoveKnownState, OrArm>, StInitialPosture, ABORT> /*retry on failure*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_six_table_pick_n_sort_1", "config/manipulation/known_states/initial_posture.yaml");

            std::vector<geometry_msgs::PoseStamped> endEffectorTrajectory;

            for(int i=0;i< 100;i++)
            {
                geometry_msgs::PoseStamped p;
                p.header.frame_id = "map";
                p.pose.position.x = 0.4;
                p.pose.position.y = -0.2 + 0.005* i;
                p.pose.position.z = 1 ;
                p.pose.orientation.w = 1;

                endEffectorTrajectory.push_back(p);
            }

            configure_orthogonal<OrArm, CbMoveEndEffectorTrajectory>(endEffectorTrajectory);
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