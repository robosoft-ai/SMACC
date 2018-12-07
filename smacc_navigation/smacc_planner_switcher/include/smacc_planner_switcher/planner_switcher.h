/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <ros/ros.h>
#include <functional>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <smacc/smacc.h>

namespace smacc_planner_switcher
{
class PlannerSwitcher: public smacc::ISmaccComponent
{
private:
    std::string desired_global_planner_;
    std::string desired_local_planner_;
    ros::Subscriber dynrecofSub_;
    

public:

virtual void init(ros::NodeHandle& nh) override
{
  dynrecofSub_ = nh.subscribe<dynamic_reconfigure::Config>("/move_base/parameter_updates" , 1, boost::bind(&PlannerSwitcher::dynreconfCallback, this, _1));
}

void setBackwardPlanner()
{
  desired_global_planner_ = "backward_global_planner/BackwardGlobalPlanner";
  desired_local_planner_ = "backward_local_planner/BackwardLocalPlanner";
  updatePlanners();
}

void setForwardPlanner()
{
  desired_global_planner_ = "forward_global_planner/ForwardGlobalPlanner";
  desired_local_planner_ = "forward_local_planner/ForwardLocalPlanner";
  //setPlanners("navfn/NavfnROS", "base_local_planner/TrajectoryPlannerROS");
  updatePlanners();
}

private:
void updatePlanners(bool subscribecallback=true)
{
  ROS_INFO_STREAM("Setting global planner: " << desired_global_planner_);
  ROS_INFO_STREAM("Setting local planner: " << desired_local_planner_);

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::StrParameter local_planner, global_planner;
  dynamic_reconfigure::Config conf;

  local_planner.name = "base_local_planner";
  local_planner.value =  desired_local_planner_;
  conf.strs.push_back(local_planner);

  global_planner.name= "base_global_planner";
  global_planner.value = desired_global_planner_;
  conf.strs.push_back(global_planner);
  
  srv_req.config = conf;
  ROS_INFO("seting values of the dynamic reconfigure server");
  ros::service::call("/move_base/set_parameters", srv_req, srv_resp);
  ros::spinOnce();
  ROS_INFO_STREAM("Response: "<< srv_resp);
}

bool set_planners_mode_flag;
void dynreconfCallback(const dynamic_reconfigure::Config::ConstPtr& configuration_update)
{
  auto gp = std::find_if(configuration_update->strs.begin(),configuration_update->strs.begin() ,
              [&](const dynamic_reconfigure::StrParameter& p)
              {
                  return p.name == "base_global_planner" &&  p.value == desired_global_planner_;
              });

  auto lp = std::find_if(configuration_update->strs.begin(),configuration_update->strs.begin() ,
              [&](const dynamic_reconfigure::StrParameter& p)
              {
                return p.name == "base_local_planner" &&  p.value == desired_local_planner_;
              });

  if(gp== configuration_update->strs.end() ||  lp == configuration_update->strs.end())
    {
      ROS_INFO("After planner update it is noticed an incorrect move_base planner configuration. Resending request.");
      set_planners_mode_flag =false;
      updatePlanners(false);
    }
    else
    {
      ROS_INFO("Planners correctly configured according to parameter update callback");
      set_planners_mode_flag= true;
    }
}
};
}