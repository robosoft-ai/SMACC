#pragma once

#include <smacc/smacc.h>

namespace sm_fetch_robot_asynchronous_orthogonals
{
    // STATE DECLARATION
    struct StNavigateForwardsAndHandsUp : smacc::SmaccState<StNavigateForwardsAndHandsUp, SmFetchRobotAsynchronousOrthogonals>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
                Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StNavigateOriginAndHandsDown, SUCCESS>,
                Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateForwardsAndHandsUp, ABORT> /*retry*/
                >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrNavigation, CbNavigateForward>(8);
            configure_orthogonal<OrArm, CbMoveKnownState>("sm_fetch_robot_asynchronous_orthogonals", "config/manipulation/known_states/final_raise_hands_up.yaml");
        }

        void runtimeConfigure()
        {
            auto cbFw = getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateForward>();
            cbFw->forwardSpeed= 0.1;
        }

    };
}
