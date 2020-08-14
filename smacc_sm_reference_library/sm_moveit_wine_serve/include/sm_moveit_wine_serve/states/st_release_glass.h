#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_wine_serve
{
    // STATE DECLARATION
    struct StReleaseGlass : smacc::SmaccState<StReleaseGlass, SmMoveitWineFetch>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvActionSucceeded<ClGripper, OrGripper>, StNavigationPosture , SUCCESS>
           >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            //configure_orthogonal<OrArm, CbMoveCartesianRelative>();
            configure_orthogonal<OrGripper, CbOpenGripper>();
            configure_orthogonal<OrGripper, CbDetachObject>();
        }

        void runtimeConfigure()
        {
          
        }

        void onExit()
        {
            ros::Duration(1.0).sleep();
        }
    };
} // namespace sm_moveit_wine_serve