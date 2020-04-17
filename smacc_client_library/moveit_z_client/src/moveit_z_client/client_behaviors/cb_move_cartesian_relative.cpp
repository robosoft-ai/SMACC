#include <moveit_z_client/client_behaviors/cb_move_cartesian_relative.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveCartesianRelative::CbMoveCartesianRelative()
{
}

CbMoveCartesianRelative::CbMoveCartesianRelative(geometry_msgs::Vector3 offset) : offset_(offset)
{
}

void CbMoveCartesianRelative::onEntry()
{
    this->moveRelativeCartesian(offset_);
}

void CbMoveCartesianRelative::onExit()
{
}

// keeps the end efector orientation fixed
void CbMoveCartesianRelative::moveRelativeCartesian(geometry_msgs::Vector3 &offset)
{
    ClMoveGroup *movegroupClient;
    this->requiresClient(movegroupClient);

    std::vector<geometry_msgs::Pose> waypoints;

    auto referenceStartPose = movegroupClient->moveGroupClientInterface.getPoseTarget();
    //auto referenceStartPose = this->moveGroupClientInterface.getCurrentPose();
    ROS_INFO_STREAM("[CbMoveCartesianRelative] RELATIVE MOTION, SOURCE POSE: " << referenceStartPose);
    ROS_INFO_STREAM("[CbMoveCartesianRelative] Offset: " << offset);

    waypoints.push_back(referenceStartPose.pose); // up and out

    auto targetObjectPose = movegroupClient->moveGroupClientInterface.getPoseTarget();

    auto endPose = referenceStartPose.pose;

    endPose.position.x += offset.x;
    endPose.position.y += offset.y;
    endPose.position.z += offset.z;

    ROS_INFO_STREAM("[CbMoveCartesianRelative] DESTINY POSE: " << endPose);

    //target_pose2.position.x -= 0.15;
    waypoints.push_back(endPose); // left

    movegroupClient->moveGroupClientInterface.setPoseTarget(endPose);

    movegroupClient->moveGroupClientInterface.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = movegroupClient->moveGroupClientInterface.computeCartesianPath(waypoints,
                                                                                     0.01, // eef_step
                                                                                     0.0,  // jump_threshold
                                                                                     trajectory);

    if (fraction == -1)
    {
        movegroupClient->postEventMotionExecutionFailed();
        ROS_INFO("[CbMoveCartesianRelative] Absolute motion planning failed. Skipping execution.");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan grasp_pose_plan;

    //grasp_pose_plan.start_state_ = *(moveGroupInterface.getCurrentState());
    grasp_pose_plan.trajectory_ = trajectory;
    auto executionResult = movegroupClient->moveGroupClientInterface.execute(grasp_pose_plan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_INFO("[CbMoveCartesianRelative] relative motion execution succedded: fraction %lf", fraction);
        movegroupClient->postEventMotionExecutionSucceded();
    }
    else
    {
        ROS_INFO("[CbMoveCartesianRelative] relative motion execution failed");
        movegroupClient->postEventMotionExecutionFailed();
    }

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
}
} // namespace cl_movegroup
} // namespace sm_moveit