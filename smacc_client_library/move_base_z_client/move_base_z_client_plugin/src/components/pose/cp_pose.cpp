/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{

Pose::Pose(std::string targetFrame, std::string referenceFrame)
    : poseFrameName_(targetFrame)
    , referenceFrame_(referenceFrame)
    , isInitialized(false)
{
    this->pose_.header.frame_id = referenceFrame_;
    ROS_INFO("[Pose] Creating Pose tracker component to track %s in the reference frame %s", targetFrame.c_str(), referenceFrame.c_str());
}

void Pose::waitTransformUpdate(ros::Rate r)
{
    bool found = false;
    while (ros::ok() && !found)
    {

        tf::StampedTransform transform;
        try
        {
            this->tfListener_.lookupTransform(referenceFrame_, poseFrameName_,
                                              ros::Time(0), transform);

            {
                std::lock_guard<std::mutex> guard(m_mutex_);
                tf::poseTFToMsg(transform, this->pose_.pose);
                this->pose_.header.stamp = transform.stamp_;
                found = true;
                this->isInitialized = true;
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR_THROTTLE(1, "[Component pose] is failing on pose update: %s", ex.what());
        }

        r.sleep();
        ros::spinOnce();
    }
}

void Pose::update()
{
    tf::StampedTransform transform;
    try
    {
        this->tfListener_.lookupTransform(referenceFrame_, poseFrameName_,
                                          ros::Time(0), transform);

        {
            std::lock_guard<std::mutex> guard(m_mutex_);
            tf::poseTFToMsg(transform, this->pose_.pose);
            this->pose_.header.stamp = transform.stamp_;
            this->isInitialized = true;
        }
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR_THROTTLE(1, "[Component pose] is failing on pose update: %s", ex.what());
    }
}
} // namespace cl_move_base_z
