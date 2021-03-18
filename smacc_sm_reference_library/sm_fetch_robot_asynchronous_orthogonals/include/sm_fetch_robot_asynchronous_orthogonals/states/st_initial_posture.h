#pragma once 

#include <smacc/smacc.h>
#include <tf/tf.h>
namespace sm_fetch_robot_asynchronous_orthogonals
{
    // STATE DECLARATION
    struct StInitialPosture : smacc::SmaccState<StInitialPosture, SmFetchRobotAsynchronousOrthogonals>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<

            Transition<EvCbSuccess<CbMoveKnownState, OrArm>, StNavigateForwardsAndHandsUp, SUCCESS>,
            Transition<EvCbFailure<CbMoveKnownState, OrArm>, StInitialPosture, ABORT> /*retry*/
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0,0,0);
            configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_robot_asynchronous_orthogonals", "config/manipulation/known_states/initial_posture.yaml");
        }

        void runtimeConfigure()
        {
        }
    };
} // namespace sm_fetch_robot_asynchronous_orthogonals