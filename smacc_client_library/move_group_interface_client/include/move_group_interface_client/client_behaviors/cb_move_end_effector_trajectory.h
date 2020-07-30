/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <move_group_interface_client/cl_movegroup.h>
#include <smacc/smacc_asynchronous_client_behavior.h>
#include <future>
#include <visualization_msgs/MarkerArray.h>

namespace cl_move_group_interface
{
class CbMoveEndEffectorTrajectory : public smacc::SmaccAsyncClientBehavior, public smacc::ISmaccUpdatable
{
public:
  
  // std::string tip_link_;
  boost::optional<std::string> group_;

  CbMoveEndEffectorTrajectory();
  CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::PoseStamped>& endEffectorTrajectory);

  virtual void onEntry() override;

  virtual void update() override;

protected:
  virtual void generateTrajectory();  
  void publishTrajectoryMarkers();

  std::vector<geometry_msgs::PoseStamped> endEffectorTrajectory_;

  ClMoveGroup *movegroupClient_;

  private:
  void initializeROS();
  ros::Publisher markersPub_;
  visualization_msgs::MarkerArray ma;
  ros::ServiceClient iksrv_;
};
}  // namespace cl_move_group_interface