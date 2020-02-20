/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>
#include <smacc/smacc_updatable.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace move_base_z_client
{
class Pose : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
{
public:
    Pose(std::string targetFrame = "/base_link", std::string referenceFrame = "/odom");

    virtual void update() override;

    inline geometry_msgs::Pose get()
    {
        return this->pose_;
    }

private:
    geometry_msgs::Pose pose_;
    tf::TransformListener tfListener_;
    std::string targetFrame_;
    std::string referenceFrame_;
};
}
