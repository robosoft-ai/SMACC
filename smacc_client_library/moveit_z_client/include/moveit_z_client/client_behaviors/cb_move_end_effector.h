/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>
#include <future>
namespace moveit_z_client
{
class CbMoveEndEffector : public smacc::SmaccClientBehavior, public smacc::ISmaccUpdatable
{
public:
  geometry_msgs::PoseStamped targetPose;
  std::string tip_link_;
  boost::optional<std::string> group_;

  CbMoveEndEffector();
  CbMoveEndEffector(geometry_msgs::PoseStamped target_pose, std::string tip_link = "");
  virtual void onEntry() override;
  virtual void onExit() override;
  virtual void update() override;

  std::future<moveit::planning_interface::MoveItErrorCode> planAndExecuteAsync();

protected:
  bool moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                          geometry_msgs::PoseStamped &targetObjectPose);

  ClMoveGroup *movegroupClient_;
};
}  // namespace moveit_z_client