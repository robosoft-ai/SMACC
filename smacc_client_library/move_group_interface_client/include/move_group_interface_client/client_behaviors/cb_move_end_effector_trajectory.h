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
  boost::optional<std::string> tipLink_;

  CbMoveEndEffectorTrajectory(std::string tipLink="");
  CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::PoseStamped>& endEffectorTrajectory, std::string tipLink="");

  virtual void onEntry() override;

  virtual void onExit() override;

  virtual void update() override;

protected:
  virtual void generateTrajectory();  
  virtual void createMarkers();

  std::vector<geometry_msgs::PoseStamped> endEffectorTrajectory_;

  ClMoveGroup *movegroupClient_;
  visualization_msgs::MarkerArray beahiorMarkers_;

  private:
  void initializeROS();
  ros::Publisher markersPub_;
  std::atomic<bool> markersInitialized_ = false;
  
  ros::ServiceClient iksrv_;
  std::mutex m_mutex_;
};
}  // namespace cl_move_group_interface