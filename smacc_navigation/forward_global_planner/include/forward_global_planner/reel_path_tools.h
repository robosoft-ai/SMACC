/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

   
namespace reel_path_tools
{
    geometry_msgs::PoseStamped makePureSpinningSubPlan(const geometry_msgs::PoseStamped& start, double dstRads, std::vector<geometry_msgs::PoseStamped>& plan, double puresSpinningRadStep=1000);

    geometry_msgs::PoseStamped makePureStraightSubPlan(const geometry_msgs::PoseStamped& startOrientedPose, 
                                                        const geometry_msgs::Point& goal, 
                                                        double lenght, 
                                                        std::vector<geometry_msgs::PoseStamped>& plan);
}