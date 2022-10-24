#pragma once
namespace sm_fetch_six_table_pick_n_sort_1
{
    namespace place_states
    {
        // STATE DECLARATION
        struct StPlaceRetreat : smacc::SmaccState<StPlaceRetreat, SS>
        {
            using SmaccState::SmaccState;

            // TRANSITION TABLE
            typedef mpl::list<

                Transition<EvCbSuccess<CbMoveCartesianRelative, OrArm>, StNavigationPosture, SUCCESS>,
                Transition<EvCbFailure<CbMoveCartesianRelative, OrArm>, StPlaceRetreat, ABORT> /*retry on failure*/
                >
                reactions;

            // STATE FUNCTIONS
            static void staticConfigure()
            {
                configure_orthogonal<OrArm, CbMoveCartesianRelative>();
            }

            void runtimeConfigure()
            {
                /*for the case of abor/retry cartesian retreat --*/
                ClMoveGroup *moveGroup;
                this->requiresClient(moveGroup);
                moveGroup->getComponent<CpConstraintTableWorkspaces>()->disableTableCollisionVolume();
                ros::Duration(1).sleep();

                auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                                 ->getClientBehavior<CbMoveCartesianRelative>();

                moveCartesianRelative->offset_.z = 0.15;
            }

            void onExit()
            {
                ClMoveGroup *moveGroup;
                this->requiresClient(moveGroup);

                moveGroup->getComponent<CpConstraintTableWorkspaces>()->setBigTableCollisionVolume();
            }
        };
    } // namespace place_states
} // namespace sm_fetch_six_table_pick_n_sort_1
