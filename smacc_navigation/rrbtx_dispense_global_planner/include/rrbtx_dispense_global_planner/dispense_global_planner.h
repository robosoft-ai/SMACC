/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace rrbtx_dispense_global_planner 
{
class DispenseGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    DispenseGlobalPlanner();

    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;

    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
        double& cost) override;

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros_) override;

private:

    ros::NodeHandle nh_;

    ros::Publisher planPub_;

    /// stored but almost not used
    costmap_2d::Costmap2DROS* costmap_ros_;

    double skip_straight_motion_distance_; //meters
    
    double puresSpinningRadStep_; // rads
};
}
