#pragma once
namespace sm_fetch_two_table_pick_n_place_1
{
namespace place_states
{
// STATE DECLARATION
struct StMovePrePlacePose : smacc::SmaccState<StMovePrePlacePose, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StPlaceApproach>,
        Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StMovePrePlacePose, ABORT>/*retry on failure*/
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
        ClPerceptionSystem *perceptionSystem;
        this->requiresClient(perceptionSystem);

        geometry_msgs::PoseStamped placingPose = perceptionSystem->decidePlacePose();

        ROS_INFO_STREAM("[StMovePreplacePose] Decided place pose for cube: " << placingPose);

        placingPose.pose.position.x -= 0;
        placingPose.pose.position.z += 0.3;

        auto moveAbsolute = this->getOrthogonal<OrArm>()
                                ->getClientBehavior<CbMoveEndEffector>();

        computeCubeGraspingOrientation(placingPose);
        moveAbsolute->targetPose = placingPose;
    }

    void computeCubeGraspingOrientation(geometry_msgs::PoseStamped &objectPose)
    {
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
} // namespace place_states
} // namespace sm_fetch_two_table_pick_n_place_1