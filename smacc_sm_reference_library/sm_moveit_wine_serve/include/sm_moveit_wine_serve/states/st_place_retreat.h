#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_wine_serve
{
    // STATE DECLARATION
    struct StPlaceRetreat : smacc::SmaccState<StPlaceRetreat, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // STATE FUNCTIONS
        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbUndoLastTrajectory, OrArm>, StFinalRaiseHandsUp, SUCCESS>,
            Transition<EvCbFailure<CbUndoLastTrajectory, OrArm>, StPlaceRetreat, ABORT> /*retry on failure*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            geometry_msgs::Vector3 offset;
            offset.x = -0.08;
            configure_orthogonal<OrArm, CbUndoLastTrajectory>(2);
        }

        void runtimeConfigure()
        {
        }

        void onExit(SUCCESS)
        {
            ClMoveGroup *moveGroup_;
            this->requiresClient(moveGroup_);
            moveGroup_->getComponent<CpConstraintTableWorkspaces>()->setSmallTableCollisionVolume();

            auto currentCubeCollision = moveGroup_->getComponent<cl_move_group_interface::CpConstraintVirtualSideWall>("cube_1");
            currentCubeCollision->enable();
        }

        void onExit(ABORT)
        {
            ros::Duration(2).sleep();
        }
    };
} // namespace sm_moveit_wine_serve