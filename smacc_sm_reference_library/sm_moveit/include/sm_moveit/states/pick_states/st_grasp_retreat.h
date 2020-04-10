#pragma once

namespace sm_moveit 
{
namespace pick_states 
{
// STATE DECLARATION
struct StGraspRetreat : smacc::SmaccState<StGraspRetreat, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        //Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StCloseGripper>
        >
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
        ClMoveGroup* moveGroupClient;
        this->requiresClient(moveGroupClient);

        moveGroupClient->onMotionExecutionSuccedded(&StGraspRetreat::throwSequenceFinishedEvent, this);
    }
};

} // namespace sm_moveit
}