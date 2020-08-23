#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_two_table_whiskey_pour
{
    // STATE DECLARATION
    struct StPlaceRetreat : smacc::SmaccState<StPlaceRetreat, SmFetchTwoTableWhiskeyPour>
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
            configure_orthogonal<OrNavigation, CbNavigateBackwards>(0.6);
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
} // namespace sm_fetch_two_table_whiskey_pour