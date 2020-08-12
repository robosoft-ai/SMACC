#pragma once

#include <smacc/smacc.h>
namespace sm_moveit_wine_serve
{
    // STATE DECLARATION
    struct StMoveCartesianBottleToContainer : smacc::SmaccState<StMoveCartesianBottleToContainer, SmFetchSixTablePickNSort1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
            Transition<EvCbSuccess<CbMoveCartesianRelative2, OrArm>, StInitialPosture, SUCCESS>,
            Transition<EvCbFailure<CbMoveCartesianRelative2, OrArm>, StMoveCartesianBottleToContainer, ABORT>

            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {

            geometry_msgs::Vector3 offset;
            offset.y = 0.15;
            configure_orthogonal<OrArm, CbMoveCartesianRelative2>("map", "gripper_link", offset);
        }

        void runtimeConfigure()
        {
        }

        void onEntry()
        {
        }

        void onExit()
        {
            ros::Duration(1.0).sleep();
        }
    };
} // namespace sm_moveit_wine_serve