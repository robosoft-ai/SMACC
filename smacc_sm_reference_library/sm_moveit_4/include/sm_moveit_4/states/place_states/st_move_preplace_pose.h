#pragma once
namespace sm_moveit_4
{
    namespace place_states
    {
        // STATE DECLARATION
        struct StMovePrePlacePose : smacc::SmaccState<StMovePrePlacePose, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StPlaceApproach>,
                Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StMovePrePlacePose, ABORT> /*retry on failure*/
                >
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbMoveEndEffector>();
            }

            void runtimeConfigure()
            {
                ClPerceptionSystem *perceptionSystem;
                this->requiresClient(perceptionSystem);

                ClMoveGroup* moveGroup;
                this->requiresClient(moveGroup);
                moveGroup->getComponent<CpConstraintTableWorkspaces>()->setBigTableCollisionVolume();

                geometry_msgs::PoseStamped placingPose;

                if (perceptionSystem->decidePrePlacePose(placingPose))
                {
                    auto moveAbsolute = this->getOrthogonal<OrArm>()
                                            ->getClientBehavior<CbMoveEndEffector>();

                    moveAbsolute->targetPose = placingPose;
                }
            }

            void onExit()
            {
                ClMoveGroup *moveGroup;
                this->requiresClient(moveGroup);

                moveGroup->getComponent<CpConstraintTableWorkspaces>()->setSmallTableCollisionVolume();
            }
        };
    } // namespace place_states
} // namespace sm_moveit_4