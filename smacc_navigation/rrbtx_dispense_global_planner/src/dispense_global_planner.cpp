/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <rrbtx_dispense_global_planner/dispense_global_planner.h>
#include <rrbtx_dispense_global_planner/reel_path_tools.h>
#include <fstream>
#include <streambuf>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

namespace rrbtx_dispense_global_planner 
{
DispenseGlobalPlanner::DispenseGlobalPlanner()
    :nh_("~/DispenseGlobalPlanner")
{
    skip_straight_motion_distance_ = 0.2; //meters
    puresSpinningRadStep_ = 1000; // rads
}

void DispenseGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros_) 
{
    planPub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1);
    skip_straight_motion_distance_ = 0.2; //meters
    puresSpinningRadStep_ = 1000; // rads
}

bool DispenseGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
    double& cost) 
{
    cost = 0;
    makePlan(start, goal, plan);
}

bool DispenseGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
{
    //ROS_WARN_STREAM("Dispense global plan goal: " << goal);

    //three stages: 1 - heading to goal position, 2 - going forward keep orientation, 3 - heading to goal orientation

    // 1 - heading to goal position
    // orientation direction

    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;

    double lenght = sqrt(dx*dx + dy*dy);

    geometry_msgs::PoseStamped prevState;
    if (lenght > skip_straight_motion_distance_) 
    {   
        // skip initial pure spinning and initial straight motion
        //ROS_INFO("1 - heading to goal position pure spinning");
        double heading_direction = atan2(dy, dx);
        prevState = reel_path_tools::makePureSpinningSubPlan(start,heading_direction,plan,puresSpinningRadStep_);
        //ROS_INFO("2 - going forward keep orientation pure straight");
        prevState = reel_path_tools::makePureStraightSubPlan(prevState, goal.pose.position,  lenght, plan);
    }
    else
    {
        prevState = start;
    }

    //ROS_INFO("3 - heading to goal orientation");
    double goalOrientation = angles::normalize_angle(tf::getYaw(goal.pose.orientation));
    reel_path_tools::makePureSpinningSubPlan(prevState,goalOrientation,plan,puresSpinningRadStep_);
    
    nav_msgs::Path planMsg;
    planMsg.poses = plan;
    planMsg.header.stamp = ros::Time::now();
    planMsg.header.frame_id="/odom";
    planPub_.publish(planMsg);
    //ROS_INFO_STREAM("global dispense plan: " << planMsg);

    return true;
}

//register this planner as a BaseGlobalPlanner plugin

PLUGINLIB_EXPORT_CLASS(rrbtx_dispense_global_planner::DispenseGlobalPlanner, nav_core::BaseGlobalPlanner);
};