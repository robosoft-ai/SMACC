#pragma once

namespace sm_fetch_two_table_pick_n_place_1
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspRetreat : smacc::SmaccState<StGraspRetreat, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StNavigationPosture>,
        Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>,  StNavigationPosture> /* not retry on failure*/
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

        auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                         ->getClientBehavior<CbMoveCartesianRelative>();

        moveCartesianRelative->offset_.z = 0.1;
        if (currentTable == RobotProcessStatus::TABLE0)
        {
            moveCartesianRelative->offset_.x = -0.1;
        }
        else if (currentTable == RobotProcessStatus::TABLE1)
        {
            moveCartesianRelative->offset_.x = 0.1;
        }
    }
};

} // namespace pick_states
} // namespace sm_fetch_two_table_pick_n_place_1
