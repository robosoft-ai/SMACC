#pragma once 

#include <smacc/smacc.h>
#include <ros/spinner.h>

namespace sm_moveit_4
{
    struct EvFinishDemo : sc::event<EvFinishDemo> {};

    // STATE DECLARATION
    struct StNavigateToSourceTable : smacc::SmaccState<StNavigateToSourceTable, SmMoveIt4>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            // Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StRotate180, SUCCESS>
            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, SS1::SsPickObject, SUCCESS>,
            Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToSourceTable, ABORT>,
            Transition<EvFinishDemo, StNavigateFinalPose>
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
} // namespace sm_moveit_4