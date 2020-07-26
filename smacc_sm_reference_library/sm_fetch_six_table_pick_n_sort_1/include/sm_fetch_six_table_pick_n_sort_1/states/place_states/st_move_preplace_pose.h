#pragma once
namespace sm_fetch_six_table_pick_n_sort_1
{
    namespace place_states
    {
        // STATE DECLARATION
        struct StMovePrePlacePose : smacc::SmaccState<StMovePrePlacePose, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<
                Transition<EvCbSuccess<CbMoveEndEffector, OrArm>, StPlaceApproach>,
                Transition<EvCbFailure<CbMoveEndEffector, OrArm>, StMovePrePlacePose, ABORT> /*retry on failure*/
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

                ClMoveGroup *moveGroup;
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
} // namespace sm_fetch_six_table_pick_n_sort_1