#pragma once

namespace sm_moveit_4
{
namespace pick_states
{
// STATE DECLARATION
struct StNavigationPosture : smacc::SmaccState<StNavigationPosture, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        //Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StCloseGripper>
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StNavigationPosture, ABORT>>
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbMoveKnownState>("sm_moveit_4", "config/manipulation/known_states/nav_posture.yaml");
    }

    void runtimeConfigure()
    {
        ros::WallDuration(1).sleep();
        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);
        moveGroupClient->onMotionExecutionSuccedded(&StNavigationPosture::throwSequenceFinishedEvent, this);
        this->getOrthogonal<OrArm>()->getClientBehavior<CbMoveJoints>()->scalingFactor_ = 1;
    }
};

} // namespace pick_states
} // namespace sm_moveit_4