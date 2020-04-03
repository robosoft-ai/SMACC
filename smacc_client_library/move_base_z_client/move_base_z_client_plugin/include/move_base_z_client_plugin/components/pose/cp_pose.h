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
#include <mutex>

namespace cl_move_base_z
{
class Pose : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
{
public:
    Pose(std::string poseFrameName = "/base_link", std::string referenceFrame = "/odom");

    virtual void update() override;

    void waitTransformUpdate(ros::Rate r = ros::Rate(20));
    
    inline geometry_msgs::Pose get()
    {
        std::lock_guard<std::mutex> guard(m_mutex_);
        return this->pose_;
    }

    inline const std::string &getReferenceFrame() const
    {
        return referenceFrame_;
    }

private:
    geometry_msgs::Pose pose_;
    tf::TransformListener tfListener_;
    std::string poseFrameName_;
    std::string referenceFrame_;

    std::mutex m_mutex_;
};
} // namespace cl_move_base_z
