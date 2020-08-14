#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_wine_serve
{
    // STATE DECLARATION
    struct StBringBottleBackBackwardNavigation : smacc::SmaccState<StBringBottleBackBackwardNavigation, SmMoveitWineFetch>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StBringBottleBackNavigateSourceTable, SUCCESS>
            //Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StNavigateToDestinyTable, SUCCESS>
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateBackwards>(0.4);
        }

        void runtimeConfigure()
        {
            /*for the case of abor/retry cartesian retreat --*/
            /*ClMoveGroup *moveGroup;
            this->requiresClient(moveGroup);
            moveGroup->getComponent<CpConstraintTableWorkspaces>()->disableTableCollisionVolume();
            ros::Duration(1).sleep();

            auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                             ->getClientBehavior<CbMoveCartesianRelative>();

            moveCartesianRelative->offset_.z = -0.15;*/
        }

        void onExit()
        {
            ros::Duration(1.0).sleep();
        }
    };
} // namespace sm_moveit_wine_serve