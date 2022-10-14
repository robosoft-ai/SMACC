
#pragma once
namespace sm_fetch_two_table_whiskey_pour
{
    namespace pick_states
    {
        // STATE DECLARATION
        struct StGraspApproach : smacc::SmaccState<StGraspApproach, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvCbSuccess<CbMoveCartesianRelative2, OrArm>, StCloseGripper, SUCCESS>,
                Transition<EvCbFailure<CbMoveCartesianRelative2, OrArm>, StGraspApproach, ABORT> /*retry on failure*/
                >
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbMoveCartesianRelative2>("map", "gripper_link");
            }

            void runtimeConfigure()
            {
                ClMoveGroup *moveGroup_;
                this->requiresClient(moveGroup_);
                moveGroup_->getComponent<CpConstraintTableWorkspaces>()->setSmallTableCollisionVolume();

                auto moveRelative = this->getOrthogonal<OrArm>()->getClientBehavior<CbMoveCartesianRelative2>();

                geometry_msgs::Vector3 offset;
                offset.x = 0.11;

                moveRelative->offset_ = offset;

                auto glassCollision = moveGroup_->getComponent<cl_move_group_interface::CpConstraintVirtualSideWall>("cube_1");
                glassCollision->disable();

                auto wineBottleCollision = moveGroup_->getComponent<cl_move_group_interface::CpConstraintVirtualSideWall>("cube_2");
                wineBottleCollision->disable();
            }

            void onExit(ABORT)
            {
                ros::Duration(2).sleep();
            }
        };
    } // namespace pick_states
} // namespace sm_fetch_two_table_whiskey_pour
