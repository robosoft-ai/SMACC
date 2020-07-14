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
#include <backward_global_planner/backward_global_planner.h>
#include <fstream>
#include <streambuf>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <forward_global_planner/move_base_z_client_tools.h>
          
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::backward_global_planner::BackwardGlobalPlanner, nav_core::BaseGlobalPlanner);

namespace cl_move_base_z
{
namespace backward_global_planner
{
/**
******************************************************************************************************************
* Constructor()
******************************************************************************************************************
*/
BackwardGlobalPlanner::BackwardGlobalPlanner()
{
    skip_straight_motion_distance_ = 0.2;
}

BackwardGlobalPlanner::~BackwardGlobalPlanner()
{
    //clear
    nav_msgs::Path planMsg;
    planMsg.header.stamp = ros::Time::now();
    planPub_.publish(planMsg);
}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    //ROS_INFO_NAMED("Backwards", "BackwardGlobalPlanner initialize");
    costmap_ros_ = costmap_ros;
    //ROS_WARN_NAMED("Backwards", "initializating global planner, costmap address: %ld", (long)costmap_ros);

    ros::NodeHandle nh;
    planPub_ = nh.advertise<nav_msgs::Path>("backward_planner/global_plan", 1);
    markersPub_ = nh.advertise<visualization_msgs::MarkerArray>("backward_planner/markers", 1);
}

/**
******************************************************************************************************************
* onForwardTrailMsg()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::onForwardTrailMsg(const nav_msgs::Path::ConstPtr &trailMessage)
{
    lastForwardPathMsg_ = *trailMessage;
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::publishGoalMarker(const geometry_msgs::Pose &pose, double r, double g, double b)
{
    double phi = tf::getYaw(pose.orientation);
    visualization_msgs::Marker marker;
    marker.header.frame_id = this->costmap_ros_->getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "my_namespace2";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.3;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    geometry_msgs::Point start, end;
    start.x = pose.position.x;
    start.y = pose.position.y;

    end.x = pose.position.x + 0.5 * cos(phi);
    end.y = pose.position.y + 0.5 * sin(phi);

    marker.points.push_back(start);
    marker.points.push_back(end);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);

    markersPub_.publish(ma);
}

/**
******************************************************************************************************************
* defaultBackwardPath()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::createDefaultBackwardPath(const geometry_msgs::PoseStamped &start,
                                                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    auto q = start.pose.orientation;

    geometry_msgs::PoseStamped pose;
    pose = start;
    
    double dx = start.pose.position.x - goal.pose.position.x;
    double dy = start.pose.position.y - goal.pose.position.y;

    double lenght = sqrt(dx * dx + dy * dy);

    geometry_msgs::PoseStamped prevState;
    if (lenght > skip_straight_motion_distance_)
    {
        // skip initial pure spinning and initial straight motion
        //ROS_INFO("1 - heading to goal position pure spinning");
        double heading_direction = atan2(dy, dx);
        double startyaw = tf::getYaw(q);
        double offset = angles::shortest_angular_distance(startyaw, heading_direction);
        heading_direction = startyaw + offset;

        prevState = cl_move_base_z::makePureSpinningSubPlan(start, heading_direction, plan, puresSpinningRadStep_);
        //ROS_INFO("2 - going forward keep orientation pure straight");

        prevState = cl_move_base_z::makePureStraightSubPlan(prevState, goal.pose.position, lenght, plan);
    }
    else
    {
        prevState = start;
    }

    ROS_WARN_STREAM( "[BackwardGlobalPlanner ] backward global plan size:  " <<plan.size());
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    //ROS_WARN_NAMED("Backwards", "Backwards global planner: Generating global plan ");
    //ROS_WARN_NAMED("Backwards", "Clearing...");

    plan.clear();

    this->createDefaultBackwardPath(start, goal, plan);
    //this->createPureSpiningAndStragihtLineBackwardPath(start, goal, plan);

    //ROS_INFO_STREAM(" start - " << start);
    //ROS_INFO_STREAM(" end - " << goal.pose.position);

    //ROS_INFO("3 - heading to goal orientation");
    //double goalOrientation = angles::normalize_angle(tf::getYaw(goal.pose.orientation));
    //cl_move_base_z::makePureSpinningSubPlan(prevState,goalOrientation,plan);

    //ROS_WARN_STREAM( "MAKE PLAN INVOKED, plan size:"<< plan.size());
    publishGoalMarker(goal.pose, 1.0, 0, 1.0);

    if (plan.size() <= 1)
    {
        // if the backward plan is too much small, we create one artificial path
        // with two poses (the current pose) so that we do not reject the plan request        
        plan.clear();
        plan.push_back(start);
        plan.push_back(start);
        ROS_INFO("[BackwardGlobalPlanner] Artificial backward plan on current pose.");
    }

    nav_msgs::Path planMsg;
    planMsg.poses = plan;
    planMsg.header.frame_id = this->costmap_ros_->getGlobalFrameID();

        // check plan rejection
    bool acceptedGlobalPlan = true;

    // static const unsigned char NO_INFORMATION = 255;
    // static const unsigned char LETHAL_OBSTACLE = 254;
    // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    // static const unsigned char FREE_SPACE = 0;

    costmap_2d::Costmap2D *costmap2d = this->costmap_ros_->getCostmap();
    for (auto &p : plan)
    {
        unsigned int mx, my;
        costmap2d->worldToMap(p.pose.position.x, p.pose.position.y, mx, my);
        auto cost = costmap2d->getCost(mx, my);

        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            acceptedGlobalPlan = false;
            break;
        }
    }
    
    planPub_.publish(planMsg);

    // this was previously set to size() <= 1, but a plan with a single point is also a valid plan (the goal)
    return true;
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
bool BackwardGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan,
                                     double &cost)
{
    cost = 0;
    makePlan(start, goal, plan);
    return true;
}

} // namespace backward_global_planner
} // namespace cl_move_base_z