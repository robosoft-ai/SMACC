#include <moveit_z_client/client_behaviors/cb_move_relative.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveRelative::CbMoveRelative()
{
    transform_.rotation.w = 1;
}

CbMoveRelative::CbMoveRelative(geometry_msgs::Transform transform)
{
}

void CbMoveRelative::onEntry()
{
    ROS_INFO_STREAM("[CbMoveRelative] Transform end effector pose relative: " << transform_);
    this->requiresClient(movegroupClient_);
    moveRelative(transform_);
}

void CbMoveRelative::onExit()
{
}

void CbMoveRelative::moveRelative(geometry_msgs::Transform &transformOffset)
{
    auto &moveGroupInterface = movegroupClient_->moveGroupClientInterface;

    auto referenceStartPose = moveGroupInterface.getCurrentPose();
    tf::Quaternion currentOrientation;
    tf::quaternionMsgToTF(referenceStartPose.pose.orientation, currentOrientation);
    tf::Quaternion desiredRelativeRotation;

    tf::quaternionMsgToTF(transformOffset.rotation, desiredRelativeRotation);

    auto targetOrientation = desiredRelativeRotation * currentOrientation;

    geometry_msgs::PoseStamped targetObjectPose = referenceStartPose;

    tf::quaternionTFToMsg(targetOrientation, targetObjectPose.pose.orientation);
    targetObjectPose.pose.position.x += transformOffset.translation.x;
    targetObjectPose.pose.position.y += transformOffset.translation.y;
    targetObjectPose.pose.position.z += transformOffset.translation.z;

    moveGroupInterface.setPlanningTime(1.0);

    ROS_INFO_STREAM("[CbMoveRelative] Target End efector Pose: " << targetObjectPose);

    moveGroupInterface.setPoseTarget(targetObjectPose);
    moveGroupInterface.setPoseReferenceFrame("/map");

    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
    bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("CbMoveRelative", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    if (success)
    {
        auto executionResult = moveGroupInterface.execute(computedMotionPlan);

        if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_INFO("[CbMoveRelative] motion execution succedded");
            movegroupClient_->postEventMotionExecutionSucceded();
        }
        else
        {
            ROS_INFO("[CbMoveRelative] motion execution failed");
            movegroupClient_->postEventMotionExecutionFailed();
        }
    }
    else
    {
        ROS_INFO("[CbMoveRelative] motion execution failed");
        movegroupClient_->postEventMotionExecutionFailed();
    }
}
} // namespace cl_movegroup
} // namespace sm_moveit