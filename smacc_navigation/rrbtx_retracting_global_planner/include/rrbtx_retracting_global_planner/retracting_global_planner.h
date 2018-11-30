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
#include <rrbtx_retracting_global_planner/command.h>

namespace rrbtx_retracting_global_planner {
class RetractingGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    RetractingGlobalPlanner();

    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;

    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
        double& cost) override;

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros_) override;

private:
    ros::NodeHandle nh_;

    ros::Subscriber dispensedCordPathSub_;

    ros::Publisher planPub_;

    ros::Publisher markersPub_;

    ///
    /// \brief lastCordTrailMsg_: the last message received with the cord trail points
    ///
    nav_msgs::Path lastDispensedCordPathMsg_;

    /// stored but almost not used
    costmap_2d::Costmap2DROS* costmap_ros_;

    void onDispensedCordTrailMsg(const nav_msgs::Path::ConstPtr& trailMessage);

    void publishGoalMarker(const geometry_msgs::Pose& pose, double r, double g, double b);

    ros::ServiceServer cmd_server_;

    bool commandServiceCall(rrbtx_retracting_global_planner::command::Request  &req, rrbtx_retracting_global_planner::command::Response  &res);
    
    double skip_straight_motion_distance_; //meters
    
    double puresSpinningRadStep_; // rads
};
}
