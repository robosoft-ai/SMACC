/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <moveit_z_client/client_behaviors/cb_move_joints.h>
#include <future>

namespace moveit_z_client
{
CbMoveJoints::CbMoveJoints(const std::map<std::string, double>& jointValueTarget) : jointValueTarget_(jointValueTarget)
{
}

CbMoveJoints::CbMoveJoints()
{
}

void CbMoveJoints::onEntry()
{
  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
    auto res = std::async(std::launch::async, [=] {
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveJoints(move_group);
    });
  }
  else
  {
    auto res =
        std::async(std::launch::async, [=] { this->moveJoints(movegroupClient_->moveGroupClientInterface); });
  }
}

void CbMoveJoints::moveJoints(moveit::planning_interface::MoveGroupInterface& moveGroupInterface)
{
  if (scalingFactor_)
    moveGroupInterface.setMaxVelocityScalingFactor(*scalingFactor_);

  moveGroupInterface.setJointValueTarget(jointValueTarget_);

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

  bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("CbMoveJoints", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("[CbMoveJoints] motion execution succedded");
      movegroupClient_->postEventMotionExecutionSucceded();
    }
    else
    {
      ROS_INFO("[CbMoveJoints] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
    }
  }
  else
  {
    ROS_INFO("[CbMoveJoints] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
  }
}

void CbMoveJoints::onExit()
{
}
}  // namespace moveit_z_client