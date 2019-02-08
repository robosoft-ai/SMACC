/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_action_client.h>
#include <ros/ros.h>
#include <functional>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace smacc_planner_switcher
{
class PlannerSwitcher: public smacc::ISmaccComponent
{
public:
  virtual void init(ros::NodeHandle& nh) override;
  void setBackwardPlanner();
  void setForwardPlanner();

  // sets ROS defaults local and global planners
  void setDefaultPlanners();

private:
  std::string desired_global_planner_;
  std::string desired_local_planner_;
  ros::Subscriber dynrecofSub_;
  bool set_planners_mode_flag;

  void updatePlanners(bool subscribecallback=true);
  void dynreconfCallback(const dynamic_reconfigure::Config::ConstPtr& configuration_update);
};
}
