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
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StNavigationPosture>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StGraspRetreat, ABORT> /*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {

        configure_orthogonal<OrArm, CbMoveCartesianRelative>();
    }

    void runtimeConfigure()
    {
        ClPerceptionSystem *perceptionSystem;
        this->requiresClient(perceptionSystem);
        auto currentTable = perceptionSystem->getCurrentTable();

        auto moveCartesianRelative = this->getOrthogonal<OrArm>()->getClientBehavior<CbMoveCartesianRelative>();

        moveCartesianRelative->offset_.z = 0.15;
        if (currentTable == RobotProcessStatus::TABLE0)
        {
            moveCartesianRelative->offset_.x = -0.4;
        }
        else if (currentTable == RobotProcessStatus::TABLE1)
        {
            moveCartesianRelative->offset_.x == 0.4;
        }
    }
};

} // namespace pick_states
} // namespace sm_moveit