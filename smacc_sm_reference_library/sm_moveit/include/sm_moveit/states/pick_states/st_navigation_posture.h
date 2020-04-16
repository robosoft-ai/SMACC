#pragma once

namespace sm_moveit
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
        configure_orthogonal<OrArm, CbMoveCartesianRelative>();
    }

    void runtimeConfigure()
    {
        ros::WallDuration(3).sleep();

        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);

        ClPerceptionSystem *perceptionSystem;
        this->requiresClient(perceptionSystem);

        auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                         ->getClientBehavior<CbMoveCartesianRelative>();

        /*
        //geometry_msgs::Transform transform;
        //transform.rotation.w=1;
        auto moveCartesianRelative = this->getOrthogonal<OrArm>()->getClientBehavior<CbMoveRelative>();

        auto quat = tf::createQuaternionFromRPY(M_PI, 0, 0);
        moveCartesianRelative->transform_.translation.z = 0.05;
        tf::quaternionTFToMsg(quat, moveCartesianRelative->transform_.rotation);
*/
        moveCartesianRelative->offset_.z = -0.35;
        auto currentTable = perceptionSystem->getCurrentTable();
        if (currentTable == RobotProcessStatus::TABLE0)
        {
            moveCartesianRelative->offset_.x = -0.25;
        }
        else if (currentTable == RobotProcessStatus::TABLE1)
        {
            moveCartesianRelative->offset_.x = 0.25;
        }

        moveGroupClient->onMotionExecutionSuccedded(&StNavigationPosture::throwSequenceFinishedEvent, this);
    }
};

} // namespace pick_states
} // namespace sm_moveit