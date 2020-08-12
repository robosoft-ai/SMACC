#pragma once 

#include <smacc/smacc.h>
#include <ros/spinner.h>

namespace sm_moveit_wine_serve
{
    struct EvFinishDemo : sc::event<EvFinishDemo> {};

    // STATE DECLARATION
    struct StNavigateToSourceTable : smacc::SmaccState<StNavigateToSourceTable, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            // Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotate180, SUCCESS>
            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToSourceTable, ABORT>,
            Transition<EvFinishDemo, StInitialPosture>
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
        }

        void runtimeConfigure()
        {
            auto* navigateGlobalPosition = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateGlobalPosition>();
            
            ClPerceptionSystem *perceptionSystem;
            navigateGlobalPosition->requiresClient(perceptionSystem);

            auto tablePose = perceptionSystem->getMainTablePose().pose;
            tablePose.position.x -= 0.85;

            geometry_msgs::PoseStamped nextCubePose;
            if (perceptionSystem->decidePickCubePose(nextCubePose))
            {
                // align with the cube in the y axis
                tablePose.position.y = nextCubePose.pose.position.y;

                navigateGlobalPosition->setGoal(tablePose);
            }
            else
            {
                this->postEvent<EvFinishDemo>();
            }
        }

        void OnEntry()
        {
            ROS_INFO("state on entry");
        }
    };
} // namespace sm_fetch_six_table_pick_n_sort_1