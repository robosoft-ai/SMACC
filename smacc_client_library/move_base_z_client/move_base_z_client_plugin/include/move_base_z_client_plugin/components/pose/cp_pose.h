/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>
#include <smacc/smacc_updatable.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <mutex>
#include <boost/thread.hpp>

namespace cl_move_base_z
{
class Pose : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
{
public:
    Pose(std::string poseFrameName = "base_link", std::string referenceFrame = "odom");

    virtual void update() override;

    void waitTransformUpdate(ros::Rate r = ros::Rate(20));
    
    geometry_msgs::Pose toPoseMsg();

    geometry_msgs::PoseStamped toPoseStampedMsg();

    inline const std::string &getReferenceFrame() const
    {
        return referenceFrame_;
    }

    inline const std::string &getFrameId() const
    {
        return poseFrameName_;
    }

    bool isInitialized;

private:
    geometry_msgs::PoseStamped pose_;
    
    static std::shared_ptr<tf::TransformListener> tfListener_;
    static boost::mutex listenerMutex_;

    std::string poseFrameName_;
    std::string referenceFrame_;

    boost::mutex m_mutex_;
};
} // namespace cl_move_base_z
