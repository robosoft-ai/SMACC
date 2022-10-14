#pragma once

#include <smacc/smacc.h>
#include <tf/tf.h>
namespace sm_fetch_six_table_pick_n_sort_1
{
    // STATE DECLARATION
    struct StNavigateToDestinyTable : smacc::SmaccState<StNavigateToDestinyTable, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS2::SsPlaceObject, SUCCESS>,
            Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToDestinyTable, ABORT> /*retry*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>();
        }

        void runtimeConfigure()
        {
            ROS_INFO("getting navigate global position behavior");
            auto navigateGlobalPosition = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateGlobalPosition>();
            ClPerceptionSystem *perceptionSystem;

            ROS_INFO("getting perception system client");
            navigateGlobalPosition->requiresClient(perceptionSystem);

            geometry_msgs::PoseStamped nextCubePose;
            ROS_INFO("deciding pick cube pose");
            perceptionSystem->decidePickCubePose(nextCubePose);

            ROS_INFO("getting table pose");
            auto targetTablePose = perceptionSystem->getTargetTablePose().pose;

            perceptionSystem->printCubesState();
            ROS_INFO("filtering side");
            if (targetTablePose.position.x > 0)
            {
                targetTablePose.position.x -= 0.85;
            }
            else
            {
                targetTablePose.position.x += 0.85;
                auto quat = tf::createQuaternionFromYaw(M_PI);
                tf::quaternionTFToMsg(quat, targetTablePose.orientation);
            }

            navigateGlobalPosition->setGoal(targetTablePose);
        }
    };
} // namespace sm_fetch_six_table_pick_n_sort_1
