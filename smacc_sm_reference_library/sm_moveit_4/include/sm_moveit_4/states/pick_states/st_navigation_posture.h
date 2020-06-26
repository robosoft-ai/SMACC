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
        configure_orthogonal<OrNavigation, CbMoveKnownState>("sm_moveit_4", "config/manipulation/known_states/nav_posture.yaml");
    }

    void runtimeConfigure()
    {
        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);
        moveGroupClient->onMotionExecutionSuccedded(&StNavigationPosture::throwSequenceFinishedEvent, this);
        this->getOrthogonal<OrNavigation>()->getClientBehavior<CbMoveJoints>()->scalingFactor_ = 1;

        /*
        ros::WallDuration(3).sleep();

        ClPerceptionSystem *perceptionSystem;
        this->requiresClient(perceptionSystem);

        auto moveCartesianRelative = this->getOrthogonal<OrArm>()
                                         ->getClientBehavior<CbMoveCartesianRelative>();

        std::vector<std::string> namesposes = moveGroupClient->moveGroupClientInterface.getNamedTargets();

        ROS_INFO("--- named poses: ");
        for (auto &posename : namesposes)
        {
            ROS_INFO_STREAM(" - named pose: " << posename);
        }
        ROS_INFO("--- named poses");

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
    */
    }
};

} // namespace pick_states
} // namespace sm_moveit_4