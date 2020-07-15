#pragma once
namespace sm_moveit_4
{
    namespace pick_states
    {
        // STATE DECLARATION
        struct StMovePregraspPose : smacc::SmaccState<StMovePregraspPose, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StGraspApproach, SUCCESS>,
                Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StMovePregraspPose, ABORT> /*retry on failure*/
                >
                reactions;

            // State member variables
            ClMoveGroup *moveGroup_;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbMoveEndEffector>();
            }

            void runtimeConfigure()
            {
                ROS_INFO("Pre grasp pose initialization.");
                ros::WallDuration(1).sleep();

                // --------------------------------------
                this->requiresClient(moveGroup_);
                
                ClPerceptionSystem *perceptionSystem;
                this->requiresClient(perceptionSystem);

                auto cbMoveEndEffector = this->getOrthogonal<OrArm>()->getClientBehavior<CbMoveEndEffector>();
                // --------------------------------------
                moveGroup_->getComponent<CpConstraintTableWorkspaces>()->setBigTableCollisionVolume();

                geometry_msgs::PoseStamped targetCubePose;
                if (perceptionSystem->decidePickCubePose(targetCubePose))
                {
                    geometry_msgs::PoseStamped pregraspPose = targetCubePose;
                    perceptionSystem->computePregraspPoseFromCubePose(pregraspPose);

                    cbMoveEndEffector->targetPose = pregraspPose;
                }
            }

            void onExit()
            {
                moveGroup_->getComponent<CpConstraintTableWorkspaces>()->setSmallTableCollisionVolume();
            }
        };
    } // namespace pick_states
} // namespace sm_moveit_4