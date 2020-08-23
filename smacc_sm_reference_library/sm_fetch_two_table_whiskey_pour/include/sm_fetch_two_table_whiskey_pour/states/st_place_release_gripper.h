#pragma once

#include <smacc/smacc.h>
namespace sm_fetch_two_table_whiskey_pour
{
    // STATE DECLARATION
    struct StReleaseGripper : smacc::SmaccState<StReleaseGripper, SmFetchTwoTableWhiskeyPour>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvActionSucceeded<ClGripper, OrGripper>, StPlaceRetreat, SUCCESS>
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
} // namespace sm_fetch_two_table_whiskey_pour