#pragma once

#include <smacc/smacc.h>

namespace sm_fetch_six_table_pick_n_sort_1
{
    // STATE DECLARATION
    struct StNavigationTableRetreat : smacc::SmaccState<StNavigationTableRetreat, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
                Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StNavigateToSourceTable, SUCCESS>
                >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal_runtime<OrNavigation, CbNavigateGlobalPosition>(
                                                                [](auto& navigateGlobalPosition)
                                                                {
                                                                    ClMoveBaseZ* moveBase;
                                                                    navigateGlobalPosition.requiresClient(moveBase);

                                                                    auto currentPose = moveBase->getComponent<cl_move_base_z::Pose>()->toPoseMsg();
                                                                    geometry_msgs::Pose targetPose = currentPose;

                                                                    if(currentPose.position.x < 0)
                                                                    {
                                                                        targetPose.position.x += 0.6;
                                                                        auto quat = tf::createQuaternionFromYaw(0);
                                                                        tf::quaternionTFToMsg(quat, targetPose.orientation);
                                                                    }
                                                                    else
                                                                    {
                                                                        targetPose.position.x -= 0.6;
                                                                        auto quat = tf::createQuaternionFromYaw(M_PI);
                                                                        tf::quaternionTFToMsg(quat, targetPose.orientation);
                                                                    }

                                                                    navigateGlobalPosition.setGoal(targetPose);
                                                                });
        }
    };
}
