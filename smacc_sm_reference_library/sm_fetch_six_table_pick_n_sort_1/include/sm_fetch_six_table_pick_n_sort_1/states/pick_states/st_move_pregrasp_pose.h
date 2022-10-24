#pragma once
namespace sm_fetch_six_table_pick_n_sort_1
{
    namespace pick_states
    {
        // STATE DECLARATION
        struct StMovePregraspPose : smacc::SmaccState<StMovePregraspPose, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                Transition<EvCbSuccess<CbMoveEndEffector, OrArm>, StGraspApproach, SUCCESS>,
                Transition<EvCbFailure<CbMoveEndEffector, OrArm>, StMovePregraspPose, ABORT> /*retry on failure*/
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
        };
    } // namespace pick_states
} // namespace sm_fetch_six_table_pick_n_sort_1
