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
        std::map<std::string, double> jointValues{
            {"l_wheel_joint", 0.2032989263112741},
            {"r_wheel_joint", -0.25798452342087597},
            {"torso_lift_joint", -1.2577603815717127e-05},
            {"bellows_joint", 0.006785193726937263},
            {"head_pan_joint", 0.0053190193943049024},
            {"head_tilt_joint", 0.0037173708013753526},
            {"shoulder_pan_joint", 1.3199986235290062},
            {"shoulder_lift_joint", 0.7000699976533715},
            {"upperarm_roll_joint", -7.398079828480064e-05},
            {"elbow_flex_joint", -1.99994794099468},
            {"forearm_roll_joint", 0.00021030168417901507},
            {"wrist_flex_joint", -0.5700976747076689},
            {"wrist_roll_joint", -0.00023467601597193521},
            {"l_gripper_finger_joint", 0.05001998415439018},
            {"r_gripper_finger_joint", 0.050039698895249535}};

        configure_orthogonal<OrNavigation, CbMoveJoints>(jointValues);
    }

    void runtimeConfigure()
    {
        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);
        moveGroupClient->onMotionExecutionSuccedded(&StNavigationPosture::throwSequenceFinishedEvent, this);

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
} // namespace sm_moveit