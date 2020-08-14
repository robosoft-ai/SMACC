#pragma once

#include <smacc/smacc.h>
#include <ros/spinner.h>

namespace sm_moveit_wine_serve
{

    // STATE DECLARATION
    struct StRetreatBackwards : smacc::SmaccState<StRetreatBackwards, SmMoveitWineFetch>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvActionSucceeded<ClMoveBaseZ, OrNavigation>, StNavigateToDestinyTable, SUCCESS>
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
        }

        void OnEntry()
        {
            ROS_INFO("state on entry");
        }
    };
} // namespace sm_moveit_wine_serve