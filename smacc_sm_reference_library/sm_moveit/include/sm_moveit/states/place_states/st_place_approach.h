#pragma once
namespace sm_moveit
{
namespace place_states
{
// STATE DECLARATION
struct StPlaceApproach : smacc::SmaccState<StPlaceApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
            Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StOpenGripper>
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        geometry_msgs::Vector3 offset;
        offset.z = -0.12;
        configure_orthogonal<OrArm, CbMoveCartesianRelative>(offset);
    }

    void runtimeConfigure()
    {
    }
};
}
}