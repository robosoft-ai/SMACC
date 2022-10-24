#pragma once

namespace sm_fetch_two_table_whiskey_pour
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspRetreat : smacc::SmaccState<StGraspRetreat, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvCbSuccess<CbMoveCartesianRelative2, OrArm>, StNavigationPosture, SUCCESS>,
        Transition<EvCbFailure<CbMoveCartesianRelative2, OrArm>, StGraspRetreat, ABORT> /* not retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbMoveCartesianRelative2>("map", "gripper_link");
    }

    void runtimeConfigure()
    {
        /*for the case of abor/retry cartesian retreat --*/
        ClMoveGroup *moveGroup;
        this->requiresClient(moveGroup);
        moveGroup->getComponent<CpConstraintTableWorkspaces>()->disableTableCollisionVolume();
        ros::Duration(1).sleep();

        auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                         ->getClientBehavior<CbMoveCartesianRelative2>();

        moveCartesianRelative->offset_.z = 0.15;


        moveGroup->onMotionExecutionSuccedded(&StGraspRetreat::throwSequenceFinishedEvent, this);
    }

    void onEntry()
    {

    }

    void onExit(SUCCESS)
    {
        ClMoveGroup *moveGroup;
        this->requiresClient(moveGroup);

        moveGroup->getComponent<CpConstraintTableWorkspaces>()->setBigTableCollisionVolume();
    }
};

} // namespace pick_states
} // namespace sm_fetch_six_table_pick_n_sort_1
