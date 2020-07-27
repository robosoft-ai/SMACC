/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <future>
namespace cl_move_group_interface
{
class CbMoveEndEffectorTrajectory : public smacc::SmaccAsyncClientBehavior
{
public:
  
  // std::string tip_link_;
  boost::optional<std::string> group_;

  CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::PoseStamped>& endEffectorTrajectory);

  virtual void onEntry() override;

  std::future<moveit::planning_interface::MoveItErrorCode> planAndExecuteAsync();

protected:

  std::vector<geometry_msgs::PoseStamped> endEffectorTrajectory_;

  ClMoveGroup *movegroupClient_;
};
}  // namespace cl_move_group_interface