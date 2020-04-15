#include <moveit_z_client/cl_movegroup.h>

namespace sm_moveit
{
namespace cl_movegroup
{

ClMoveGroup::ClMoveGroup(std::string groupName)
    : moveGroupClientInterface(groupName)
{
    ros::WallDuration(10.0).sleep();
}

ClMoveGroup::~ClMoveGroup()
{
}

void ClMoveGroup::moveToAbsolutePose(geometry_msgs::PoseStamped &targetPose)
{
    bool success = moveEndEfectorToPose(moveGroupClientInterface,
                                        planningSceneInterface,
                                        targetPose);
}

bool ClMoveGroup::moveEndEfectorToPose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                                       moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                                       geometry_msgs::PoseStamped &targetObjectPose)
{

    moveGroupInterface.setPlanningTime(1.0);

    ROS_INFO_STREAM("Target End efector Pose: " << targetObjectPose);

    moveGroupInterface.setPoseTarget(targetObjectPose);
    moveGroupInterface.setPoseReferenceFrame("/map");

    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
    bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
    {
        auto executionResult = moveGroupInterface.execute(computedMotionPlan);

        if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("[ClMoveGroup] relative motion execution succedded");
            this->postEventMotionExecutionSucceded();
        }
        else
        {
            ROS_INFO("[ClMoveGroup] motion execution failed");
            this->postEventMotionExecutionFailed();
        }
    }
    else
    {
        ROS_INFO("[ClMoveGroup] motion execution failed");
        this->postEventMotionExecutionFailed();
    }

    return success;
}

void ClMoveGroup::moveRelative(geometry_msgs::Transform &transformOffset)
{
    auto referenceStartPose = this->moveGroupClientInterface.getCurrentPose();
    tf::Quaternion currentOrientation;
    tf::quaternionMsgToTF(referenceStartPose.pose.orientation, currentOrientation);
    tf::Quaternion desiredRelativeRotation;

    tf::quaternionMsgToTF(transformOffset.rotation, desiredRelativeRotation);

    auto targetOrientation = desiredRelativeRotation * currentOrientation;

    auto targetPose = referenceStartPose;

    tf::quaternionTFToMsg(targetOrientation, targetPose.pose.orientation);
    targetPose.pose.position.x += transformOffset.translation.x;
    targetPose.pose.position.y += transformOffset.translation.y;
    targetPose.pose.position.z += transformOffset.translation.z;

    bool success = moveEndEfectorToPose(moveGroupClientInterface,
                                        planningSceneInterface,
                                        targetPose);
}

// keeps the end efector orientation fixed
void ClMoveGroup::moveRelativeCartesian(geometry_msgs::Vector3 &offset)
{
    std::vector<geometry_msgs::Pose> waypoints;

    auto referenceStartPose = this->moveGroupClientInterface.getPoseTarget();
    //auto referenceStartPose = this->moveGroupClientInterface.getCurrentPose();
    ROS_INFO_STREAM("RELATIVE MOTION, SOURCE POSE: " << referenceStartPose);
    waypoints.push_back(referenceStartPose.pose); // up and out

    auto targetObjectPose = this->moveGroupClientInterface.getPoseTarget();

    auto endPose = referenceStartPose.pose;

    endPose.position.x += offset.x;
    endPose.position.y += offset.y;
    endPose.position.z += offset.z;

    //target_pose2.position.x -= 0.15;
    waypoints.push_back(endPose); // left

    this->moveGroupClientInterface.setPoseTarget(endPose);

    this->moveGroupClientInterface.setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = this->moveGroupClientInterface.computeCartesianPath(waypoints,
                                                                          0.01, // eef_step
                                                                          0.0,  // jump_threshold
                                                                          trajectory);

    if (fraction == -1)
    {
        this->postEventMotionExecutionFailed();
        ROS_INFO("[ClMoveGroup] Absolute motion planning failed. Skipping execution.");
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan grasp_pose_plan;

    //grasp_pose_plan.start_state_ = *(moveGroupInterface.getCurrentState());
    grasp_pose_plan.trajectory_ = trajectory;
    auto executionResult = this->moveGroupClientInterface.execute(grasp_pose_plan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        ROS_INFO("[ClMoveGroup] relative motion execution succedded: fraction %lf", fraction);
        this->postEventMotionExecutionSucceded();
    }
    else
    {
        ROS_INFO("[ClMoveGroup] relative motion execution failed");
        this->postEventMotionExecutionFailed();
    }

    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
}
} // namespace cl_movegroup
} // namespace sm_moveit