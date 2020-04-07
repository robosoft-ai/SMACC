#pragma once
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace sm_moveit
{

template <typename TSource, typename TObjectTag>
struct MoveitAbsouteMotionFinished : sc::event<MoveitAbsouteMotionFinished<TSource, TObjectTag>>
{
};

// STATE DECLARATION
struct StMovePregraspPose : smacc::SmaccState<StMovePregraspPose, SmMoveIt>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<

        Transition<MoveitAbsouteMotionFinished<ClMoveGroup, OrArm>, StGraspRetreat>
        //    Transition<EvActionAborted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>

        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrArm, CbGoToCube>();

        //   configure_orthogonal<OrNavigation, CbNavigateBackwards>(2);
        //   configure_orthogonal<OrLED, CbLEDOff>();
        //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    void runtimeConfigure()
    {
        cl_move_base_z::Pose targetObjectPose("/cube_0", "/map");

        targetObjectPose.waitTransformUpdate();

        ROS_INFO("------- TESTING PLAN --------");
        geometry_msgs::PoseStamped pregrasPose;
        pregrasPose.header.frame_id = targetObjectPose.getReferenceFrame();
        pregrasPose.header.stamp = ros::Time::now();
        pregrasPose.pose = targetObjectPose.get();

        pregrasPose.pose.position.x -= 0;
        //pregrasPose.position.y = -pregrasPose.position.y;
        pregrasPose.pose.position.z += 0.3;

        auto cubeYawOnTable = tf::getYaw(pregrasPose.pose.orientation);
        auto degrees90 = M_PI / 2;

        while (cubeYawOnTable > degrees90)
        {
            cubeYawOnTable -= degrees90;
        }

        while (cubeYawOnTable < -degrees90)
        {
            cubeYawOnTable += degrees90;
        }

        ROS_INFO("picking yaw: %lf", cubeYawOnTable);
        auto quat = tf::createQuaternionFromRPY(0, M_PI / 2, cubeYawOnTable);
        tf::quaternionTFToMsg(quat, pregrasPose.pose.orientation);

        auto moveAbsolute = this->getOrthogonal<OrArm>()
                                    ->getClientBehavior<CbGoToCube>();

        moveAbsolute->targetPose = pregrasPose;
        
    }
};
} // namespace sm_moveit