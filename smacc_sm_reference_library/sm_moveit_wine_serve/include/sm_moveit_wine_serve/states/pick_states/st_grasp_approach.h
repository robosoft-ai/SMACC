
#pragma once
namespace sm_moveit_wine_serve
{
namespace pick_states
{
// STATE DECLARATION
struct StGraspApproach : smacc::SmaccState<StGraspApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvCbSuccess<CbMoveCartesianRelative2, OrArm>, StCloseGripper, SUCCESS>,
        Transition<EvCbFailure<CbMoveCartesianRelative2, OrArm>, StGraspApproach, ABORT>/*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        geometry_msgs::Vector3 offset;
        offset.x = 0.1;
        configure_orthogonal<OrArm, CbMoveCartesianRelative2>("map", "gripper_link", offset);
    }

    void runtimeConfigure()
    {
        ClMoveGroup* moveGroup_;
        this->requiresClient(moveGroup_);
        moveGroup_->getComponent<CpConstraintTableWorkspaces>()->setSmallTableCollisionVolume();

        auto currentCubeCollision = moveGroup_->getComponent<cl_move_group_interface::CpConstraintVirtualSideWall>("cube_1");
        currentCubeCollision->disable();
    }

    void onExit(ABORT)
    {
        ros::Duration(2).sleep();        
    }
};
} // namespace pick_states
} // namespace sm_fetch_six_table_pick_n_sort_1