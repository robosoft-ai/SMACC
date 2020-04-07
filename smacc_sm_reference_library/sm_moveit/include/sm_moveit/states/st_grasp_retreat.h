#pragma once

namespace sm_moveit
{
// STATE DECLARATION
struct StGraspRetreat : smacc::SmaccState<StGraspRetreat, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
    }

    void runtimeConfigure()
    {
    }
};
} // namespace sm_moveit