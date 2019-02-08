/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace forward_global_planner 
{
class ForwardGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    ForwardGlobalPlanner();

    bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
        double& cost);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros_) ;

private:

    ros::NodeHandle nh_;

    ros::Publisher planPub_;

    /// stored but almost not used
    costmap_2d::Costmap2DROS* costmap_ros_;

    double skip_straight_motion_distance_; //meters
    
    double puresSpinningRadStep_; // rads
};
}
