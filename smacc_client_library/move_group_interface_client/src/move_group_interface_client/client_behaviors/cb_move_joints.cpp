/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_joints.h>
#include <future>

namespace move_group_interface_client
{
  CbMoveJoints::CbMoveJoints(const std::map<std::string, double> &jointValueTarget) : jointValueTarget_(jointValueTarget)
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
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveJoints(move_group);
    }
    else
    {
      this->moveJoints(movegroupClient_->moveGroupClientInterface);
    }
  }

  void CbMoveJoints::moveJoints(moveit::planning_interface::MoveGroupInterface &moveGroupInterface)
  {
    if (scalingFactor_)
      moveGroupInterface.setMaxVelocityScalingFactor(*scalingFactor_);

    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

    if (jointValueTarget_.size() == 0)
    {
      ROS_WARN("[CbMoveJoints] No joint was value specified. Skipping planning call.");
      success = false;
    }
    else
    {
      moveGroupInterface.setJointValueTarget(jointValueTarget_);
      success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("CbMoveJoints", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    }

    if (success)
    {
      auto executionResult = moveGroupInterface.execute(computedMotionPlan);

      if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_INFO("[CbMoveJoints] motion execution succedded. Throwing success event.");
        movegroupClient_->postEventMotionExecutionSucceded();
      }
      else
      {
        ROS_WARN("[CbMoveJoints] motion execution failed. Throwing fail event.");
        movegroupClient_->postEventMotionExecutionFailed();
      }
    }
    else
    {
      ROS_WARN("[CbMoveJoints] motion execution failed. Throwing fail event.");
      movegroupClient_->postEventMotionExecutionFailed();
    }
  }

  void CbMoveJoints::onExit()
  {
  }
} // namespace move_group_interface_client