#pragma once

#include <smacc/smacc.h>
#include <ros/spinner.h>

namespace sm_fetch_six_table_pick_n_sort_1
{
    struct EvFinishDemo : sc::event<EvFinishDemo>
    {
    };

    // STATE DECLARATION
    struct StNavigateToSourceTable : smacc::SmaccState<StNavigateToSourceTable, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToSourceTable, ABORT>,
            Transition<EvFinishDemo, StNavigateFinalPose>>
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
        }

        void runtimeConfigure()
        {
            auto *navigateGlobalPosition = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateGlobalPosition>();

            ClPerceptionSystem *perceptionSystem;
            navigateGlobalPosition->requiresClient(perceptionSystem);

            auto mainTablePose = perceptionSystem->getMainTablePose().pose;
            mainTablePose.position.x -= 0.85;

            geometry_msgs::PoseStamped nextCubePose;
            if (perceptionSystem->decidePickCubePose(nextCubePose))
            {
                // align with the cube in the y axis
                mainTablePose.position.y = nextCubePose.pose.position.y;

                navigateGlobalPosition->setGoal(mainTablePose);
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
