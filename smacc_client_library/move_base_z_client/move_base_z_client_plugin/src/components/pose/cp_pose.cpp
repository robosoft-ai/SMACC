/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{

Pose::Pose(std::string targetFrame, std::string referenceFrame)
    : targetFrame_(targetFrame), referenceFrame_(referenceFrame)
{
}

void Pose::update()
{
    tf::StampedTransform transform;
    try
    {
        this->tfListener_.lookupTransform(targetFrame_, referenceFrame_,
                                          ros::Time(0), transform);

        tf::poseTFToMsg(transform, this->pose_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR_THROTTLE(1, "Component pose is failing on pose update: %s", ex.what());
    }
}
}
