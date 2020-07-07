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

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal_fn<OrArm, CbMoveEndEffector>(
                    [](auto &cbMoveEndEffector) {
                        ROS_INFO("Pre grasp pose initialization.");
                        ros::WallDuration(1).sleep();

                        ClPerceptionSystem *perceptionSystem;
                        cbMoveEndEffector.requiresClient(perceptionSystem);

                        geometry_msgs::PoseStamped targetCubePose;

                        perceptionSystem->setSafeArmMotionToAvoidCubeCollisions();

                        if (perceptionSystem->decidePickCubePose(targetCubePose))
                        {
                            geometry_msgs::PoseStamped pregraspPose = targetCubePose;
                            perceptionSystem->computePregraspPoseFromCubePose(pregraspPose);

                            cbMoveEndEffector.targetPose = pregraspPose;
                        }
                    });
            }

            void runtimeConfigure()
            {
            }

            void onExit()
            {
                ClPerceptionSystem *perceptionSystem;
                this->requiresClient(perceptionSystem);

                perceptionSystem->unsetSafeArmMotionToAvoidCubeCollisions();
            }
        };
    } // namespace pick_states
} // namespace sm_moveit_4