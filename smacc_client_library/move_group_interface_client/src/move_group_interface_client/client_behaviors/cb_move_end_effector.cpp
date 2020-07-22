/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector.h>
// #include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <future>

namespace move_group_interface_client
{
CbMoveEndEffector::CbMoveEndEffector()
{
}

CbMoveEndEffector::CbMoveEndEffector(geometry_msgs::PoseStamped target_pose, std::string tip_link)
  : targetPose(target_pose)
{
  tip_link_ = tip_link;
}

void CbMoveEndEffector::onEntry()
{
  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
      ROS_DEBUG("[CbMoveEndEfector] new thread started to move absolute end effector");
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveToAbsolutePose(move_group, targetPose);
      ROS_DEBUG("[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
  else
  {
      ROS_DEBUG("[CbMoveEndEfector] new thread started to move absolute end effector");
      this->moveToAbsolutePose(movegroupClient_->moveGroupClientInterface, targetPose);
      ROS_DEBUG("[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
}

void CbMoveEndEffector::onExit()
{
}

bool CbMoveEndEffector::moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                                           geometry_msgs::PoseStamped &targetObjectPose)
{
  auto& planningSceneInterface = movegroupClient_->planningSceneInterface;
  ROS_DEBUG("[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  ros::Duration(0.5).sleep();

  moveGroupInterface.setPlanningTime(1.0);

  ROS_INFO_STREAM("[CbMoveEndEffector] Target End efector Pose: " << targetObjectPose);

  moveGroupInterface.setPoseTarget(targetObjectPose, tip_link_);
  moveGroupInterface.setPoseReferenceFrame(targetObjectPose.header.frame_id);

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
  bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("CbMoveEndEffector", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("[CbMoveEndEffector] motion execution succedded");
      movegroupClient_->postEventMotionExecutionSucceded();
    }
    else
    {
      ROS_INFO("[CbMoveEndEffector] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
    }
  }
  else
  {
    ROS_INFO("[CbMoveEndEffector] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
  }

  ROS_DEBUG("[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  ros::Duration(0.5).sleep();

  return success;
}

}  // namespace move_group_interface_client