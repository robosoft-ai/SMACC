#pragma once
namespace sm_moveit_4
{
    namespace place_states
    {

        struct StNavigationPosture : smacc::SmaccState<StNavigationPosture, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                //Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StCloseGripper>
                Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StNavigationPosture, ABORT>>
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbMoveKnownState>("sm_moveit_4", "config/manipulation/known_states/nav_posture.yaml");
            }

            void runtimeConfigure()
            {
                ros::WallDuration(2).sleep();
                ClMoveGroup *moveGroupClient;
                this->requiresClient(moveGroupClient);
                moveGroupClient->onMotionExecutionSuccedded(&StNavigationPosture::onSuccessFullExit, this);
            }

            void onSuccessFullExit()
            {
                ClPerceptionSystem *perceptionSystem;
                this->requiresClient(perceptionSystem);

                perceptionSystem->nextCube();
                this->throwSequenceFinishedEvent();
            }
        };
    } // namespace place_states
} // namespace sm_moveit_4