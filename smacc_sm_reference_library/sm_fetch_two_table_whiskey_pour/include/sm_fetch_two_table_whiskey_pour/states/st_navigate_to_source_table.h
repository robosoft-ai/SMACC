#pragma once

#include <smacc/smacc.h>
#include <ros/spinner.h>

namespace sm_fetch_two_table_whiskey_pour
{
    struct EvFinishDemo : sc::event<EvFinishDemo>
    {
    };

    // STATE DECLARATION
    struct StNavigateToSourceTable : smacc::SmaccState<StNavigateToSourceTable, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            // Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StRotate180, SUCCESS>
            Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToSourceTable, ABORT>,
            Transition<EvFinishDemo, StInitialPosture>>
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
            //perceptionSystem->nextObject();

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
} // namespace sm_fetch_two_table_whiskey_pour