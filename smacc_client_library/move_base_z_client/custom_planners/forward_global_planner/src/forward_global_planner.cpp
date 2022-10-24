/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <pluginlib/class_list_macros.h>
#include <forward_global_planner/forward_global_planner.h>
#include <forward_global_planner/move_base_z_client_tools.h>
#include <fstream>
#include <streambuf>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace cl_move_base_z
{
namespace forward_global_planner
{
ForwardGlobalPlanner::ForwardGlobalPlanner()
    : nh_("~/ForwardGlobalPlanner")
{
    skip_straight_motion_distance_ = 0.2; //meters
    puresSpinningRadStep_ = 1000;         // rads
}

void ForwardGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    ROS_INFO("[Forward Global Planner] initializing");
    planPub_ = nh_.advertise<nav_msgs::Path>("global_plan", 1);
    skip_straight_motion_distance_ = 0.2; //meters
    puresSpinningRadStep_ = 1000;         // rads
    this->costmap_ros_ = costmap_ros;
}

bool ForwardGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                    const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan,
                                    double &cost)
{
    ROS_INFO("[Forward Global Planner] planning");
    cost = 0;
    return makePlan(start, goal, plan);
}

bool ForwardGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                    const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    //ROS_WARN_STREAM("Forward global plan goal: " << goal);

    //three stages: 1 - heading to goal position, 2 - going forward keep orientation, 3 - heading to goal orientation

    // 1 - heading to goal position
    // orientation direction

    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;

    double length = sqrt(dx * dx + dy * dy);

    geometry_msgs::PoseStamped prevState;
    if (length > skip_straight_motion_distance_)
    {
        // skip initial pure spinning and initial straight motion
        //ROS_INFO("1 - heading to goal position pure spinning");
        double heading_direction = atan2(dy, dx);
        prevState = cl_move_base_z::makePureSpinningSubPlan(start, heading_direction, plan, puresSpinningRadStep_);
        //ROS_INFO("2 - going forward keep orientation pure straight");
        prevState = cl_move_base_z::makePureStraightSubPlan(prevState, goal.pose.position, length, plan);
    }
    else
    {
        prevState = start;
    }

    //ROS_INFO("3 - heading to goal orientation");
    double goalOrientation = angles::normalize_angle(tf::getYaw(goal.pose.orientation));
    cl_move_base_z::makePureSpinningSubPlan(prevState, goalOrientation, plan, puresSpinningRadStep_);

    nav_msgs::Path planMsg;
    planMsg.poses = plan;
    planMsg.header.stamp = ros::Time::now();

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

    if (acceptedGlobalPlan)
    {
        planPub_.publish(planMsg);
        //ROS_INFO_STREAM("global forward plan: " << planMsg);
        return true;
    }
    else
    {
        return false;
    }
}

}; // namespace forward_global_planner
} // namespace cl_move_base_z

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cl_move_base_z::forward_global_planner::ForwardGlobalPlanner, nav_core::BaseGlobalPlanner);
