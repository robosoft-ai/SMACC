
#pragma once
namespace sm_moveit
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
        configure_orthogonal<OrArm, CbMoveCartesianRelative>();
    }

    void runtimeConfigure()
    {
        auto moveCartesian = this->getOrthogonal<OrArm>()
                                ->getClientBehavior<CbMoveCartesianRelative>();

        geometry_msgs::Vector3 offset;
        offset.z = -0.12;

        moveCartesian->offset_= offset;
        moveCartesian->group_ = "arm";

    }
};
} // namespace pick_states
} // namespace sm_moveit