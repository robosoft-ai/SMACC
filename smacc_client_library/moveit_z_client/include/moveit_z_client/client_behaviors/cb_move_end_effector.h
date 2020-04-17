/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveEndEffector : public smacc::SmaccClientBehavior
{

private:
  ClMoveGroup *movegroupClient_;

public:
  geometry_msgs::PoseStamped targetPose;
  CbMoveEndEffector();
  CbMoveEndEffector(geometry_msgs::PoseStamped target_pose);
  virtual void onEntry() override;
  virtual void onExit() override;

private:
  bool moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                          moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                          geometry_msgs::PoseStamped &targetObjectPose);
};
} // namespace cl_movegroup
} // namespace sm_moveit
