#include <moveit_z_client/client_behaviors/cb_move_absolute.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveAbsolute::CbMoveAbsolute()
{
}

CbMoveAbsolute::CbMoveAbsolute(geometry_msgs::PoseStamped target_pose)
    : targetPose(target_pose)
{
}

void CbMoveAbsolute::onEntry()
{
    ros::WallDuration(4).sleep();
    this->requiresClient(movegroupClient_);

    this->moveToAbsolutePose(movegroupClient_->moveGroupClientInterface, movegroupClient_->planningSceneInterface, targetPose);

    ros::WallDuration(4).sleep();
}

void CbMoveAbsolute::onExit()
{
}

bool CbMoveAbsolute::moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                                          moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                                          geometry_msgs::PoseStamped &targetObjectPose)
{
    moveGroupInterface.setPlanningTime(1.0);

    ROS_INFO_STREAM("[CbMoveAbsolute] Target End efector Pose: " << targetObjectPose);

    moveGroupInterface.setPoseTarget(targetObjectPose);
    moveGroupInterface.setPoseReferenceFrame("/map");

    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
    bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("CbMoveAbsolute", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
    {
        auto executionResult = moveGroupInterface.execute(computedMotionPlan);

        if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("[CbMoveAbsolute] motion execution succedded");
            movegroupClient_->postEventMotionExecutionSucceded();
        }
        else
        {
            ROS_INFO("[CbMoveAbsolute] motion execution failed");
            movegroupClient_->postEventMotionExecutionFailed();
        }
    }
    else
    {
        ROS_INFO("[CbMoveAbsolute] motion execution failed");
        movegroupClient_->postEventMotionExecutionFailed();
    }

    return success;
}

} // namespace cl_movegroup
} // namespace sm_moveit
