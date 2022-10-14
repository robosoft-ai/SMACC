#pragma once

#include <smacc/smacc.h>

namespace sm_fetch_robot_asynchronous_orthogonals
{
    // STATE DECLARATION
    struct StNavigateOriginAndHandsDown : smacc::SmaccState<StNavigateOriginAndHandsDown, SmFetchRobotAsynchronousOrthogonals>
    {
        using SmaccState::SmaccState;

            // TRANSITION TABLE
        typedef mpl::list<
                Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StNavigateForwardsAndHandsUp, SUCCESS>,
                Transition<EvCbFailure<CbNavigateBackwards, OrNavigation>, StNavigateOriginAndHandsDown, ABORT> /*retry*/
                >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateBackwards>(8);
            configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_robot_asynchronous_orthogonals", "config/manipulation/known_states/initial_posture.yaml");
        }
    };
}
