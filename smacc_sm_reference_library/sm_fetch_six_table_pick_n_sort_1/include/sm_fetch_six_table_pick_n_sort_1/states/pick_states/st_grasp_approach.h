
#pragma once
namespace sm_fetch_six_table_pick_n_sort_1
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspApproach : smacc::SmaccState<StGraspApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvCbSuccess<CbMoveCartesianRelative, OrArm>, StCloseGripper, SUCCESS>,
        Transition<EvCbFailure<CbMoveCartesianRelative, OrArm>, StGraspApproach, ABORT>/*retry on failure*/
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
        ClMoveGroup* moveGroup_;
        this->requiresClient(moveGroup_);
        moveGroup_->getComponent<CpConstraintTableWorkspaces>()->setSmallTableCollisionVolume();
    }

    void onExit(ABORT)
    {
        ros::Duration(2).sleep();
    }
};
} // namespace pick_states
} // namespace sm_fetch_six_table_pick_n_sort_1
