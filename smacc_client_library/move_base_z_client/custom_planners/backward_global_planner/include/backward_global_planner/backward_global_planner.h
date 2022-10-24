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

namespace cl_move_base_z
{
namespace backward_global_planner
{
class BackwardGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
    BackwardGlobalPlanner();

    virtual ~BackwardGlobalPlanner();

    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan,
                  double &cost);

    virtual bool createDefaultBackwardPath(const geometry_msgs::PoseStamped &start,
                                           const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

    virtual void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros_) override;

private:
    ros::NodeHandle nh_;

    ros::Publisher planPub_;

    ros::Publisher markersPub_;

    costmap_2d::Costmap2DROS *costmap_ros_;

    void onForwardTrailMsg(const nav_msgs::Path::ConstPtr &trailMessage);

    void publishGoalMarker(const geometry_msgs::Pose &pose, double r, double g, double b);

    double skip_straight_motion_distance_; //meters

    double puresSpinningRadStep_; // rads
};
} // namespace backward_global_planner
} // namespace cl_move_base_z
