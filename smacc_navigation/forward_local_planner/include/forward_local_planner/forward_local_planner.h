/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_local_planner.h>

typedef double meter;
typedef double rad;

namespace forward_local_planner {
class ForwardLocalPlanner : public nav_core::BaseLocalPlanner {

public:
    ForwardLocalPlanner();

    virtual ~ForwardLocalPlanner();

    /**
   * @brief  Given the current position, orientation, and velocity of the robot: compute velocity commands to send to the robot mobile base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid velocity command was found, false otherwise
   */
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

    /**
   * @brief  Check if the goal pose has been achieved by the local planner
   * @return True if achieved, false otherwise
   */
    virtual bool isGoalReached() override;

    /**
   * @brief  Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

    /**
   * @brief  Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
    virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmapRos_) override;

private:

    void publishGoalMarker(double x, double y, double phi);

    costmap_2d::Costmap2DROS* costmapRos_;

    ros::Publisher goalMarkerPublisher_;

    double k_rho_;
    double k_alpha_;
    double k_betta_;
    bool goalReached_;

    double alpha_offset_;
    double betta_offset_;

    meter carrot_distance_;
    rad carrot_angular_distance_;

    double yaw_goal_tolerance_; // radians
    double xy_goal_tolerance_; // meters

    // references the current point inside the backwardsPlanPath were the robot is located
    int currentPoseIndex_;

    std::vector<geometry_msgs::PoseStamped> plan_;
};
}
