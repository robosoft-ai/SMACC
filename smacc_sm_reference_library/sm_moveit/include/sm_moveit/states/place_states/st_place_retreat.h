#pragma once
namespace sm_moveit
{
namespace place_states
{
// STATE DECLARATION
struct StPlaceRetreat : smacc::SmaccState<StPlaceRetreat, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<>
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        geometry_msgs::Vector3 offset;
        offset.z = 0.12;
        configure_orthogonal<OrArm, CbMoveCartesianRelative>(offset);
    }

    void runtimeConfigure()
    {
        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);

        moveGroupClient->onMotionExecutionSuccedded(&StPlaceRetreat::throwSequenceFinishedEvent, this);
    }
};
} // namespace place_states
} // namespace sm_moveit