#pragma once
namespace sm_moveit_4
{
namespace pick_states
{
// STATE DECLARATION
struct StMovePregraspPose : smacc::SmaccState<StMovePregraspPose, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StGraspApproach, SUCCESS>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StMovePregraspPose, ABORT>/*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbMoveEndEffector>();

        //   configure_orthogonal<OrNavigation, CbNavigateBackwards>(2);
        //   configure_orthogonal<OrLED, CbLEDOff>();
        //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
        ROS_INFO("Pre grasp pose initialization.");
        ros::WallDuration(5).sleep();

        ClPerceptionSystem *perceptionSystem;
        this->requiresClient(perceptionSystem);
        
        auto pregraspPose = perceptionSystem->decidePickCubePose();

        this->computeCubeGraspingOrientation(pregraspPose);

        auto moveAbsolute = this->getOrthogonal<OrArm>()
                                ->getClientBehavior<CbMoveEndEffector>();

        moveAbsolute->targetPose = pregraspPose;
    }

    void computeCubeGraspingOrientation(geometry_msgs::PoseStamped &objectPose)
    {
        objectPose.pose.position.x -= 0;
        objectPose.pose.position.z += 0.3;

        // ------ cube grasping orientation -------
        // grasp the object with the gripper using top-to-bottom direction
        auto cubeYawOnTable = tf::getYaw(objectPose.pose.orientation);

        const double degrees90 = M_PI / 2;
        while (cubeYawOnTable > degrees90)
        {
            cubeYawOnTable -= degrees90;
        }

        while (cubeYawOnTable < -degrees90)
        {
            cubeYawOnTable += degrees90;
        }

        ROS_INFO("cube yaw: %lf", cubeYawOnTable);
        auto quat = tf::createQuaternionFromRPY(0, M_PI / 2, cubeYawOnTable);
        tf::quaternionTFToMsg(quat, objectPose.pose.orientation);
    }
};
} // namespace pick_states
} // namespace sm_moveit_4