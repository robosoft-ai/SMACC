
#pragma once
namespace sm_fetch_two_table_pick_n_place_1
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspApproach : smacc::SmaccState<StGraspApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StCloseGripper, SUCCESS>,
        Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StGraspApproach, ABORT>/*retry on failure*/
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
} // namespace pick_states
} // namespace sm_fetch_two_table_pick_n_place_1