/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace moveit_z_client
{
class CbMoveEndEffector : public smacc::SmaccClientBehavior, public smacc::ISmaccUpdatable
{
private:
  ClMoveGroup *movegroupClient_;

public:
  geometry_msgs::PoseStamped targetPose;
  std::string tip_link_;
  boost::optional<std::string> group_;

  CbMoveEndEffector();
  CbMoveEndEffector(geometry_msgs::PoseStamped target_pose, std::string tip_link = "");
  virtual void onEntry() override;
  virtual void onExit() override;
  virtual void update() override;

private:
  bool moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                          moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                          geometry_msgs::PoseStamped &targetObjectPose);
};
}  // namespace moveit_z_client
