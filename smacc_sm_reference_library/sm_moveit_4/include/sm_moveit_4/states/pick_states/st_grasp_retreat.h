#pragma once

namespace sm_moveit_4
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspRetreat : smacc::SmaccState<StGraspRetreat, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StNavigationPosture>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>,  StNavigationPosture> /* not retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbMoveCartesianRelative>();
    }

    void runtimeConfigure()
    {
        auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                         ->getClientBehavior<CbMoveCartesianRelative>();

        moveCartesianRelative->offset_.z = 0.15;
    }

    void onExit()
    {
        ClMoveGroup *moveGroup;
        this->requiresClient(moveGroup);
        
        moveGroup->getComponent<CpConstraintTableWorkspaces>()->setSafeArmMotionToAvoidCubeCollisions();
    }
};

} // namespace pick_states
} // namespace sm_moveit_4