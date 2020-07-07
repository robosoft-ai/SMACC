
#pragma once
namespace sm_moveit_4
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspApproach : smacc::SmaccState<StGraspApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StCloseGripper, SUCCESS>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StGraspApproach, ABORT>/*retry on failure*/
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
        ros::Duration(2).sleep();
    }
};
} // namespace pick_states
} // namespace sm_moveit_4