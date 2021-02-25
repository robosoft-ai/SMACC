/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <boost/thread.hpp>

namespace cl_move_base_z
{
    std::shared_ptr<tf::TransformListener> Pose::tfListener_;
    boost::mutex Pose::listenerMutex_;

    Pose::Pose(std::string targetFrame, std::string referenceFrame)
        : poseFrameName_(targetFrame),
          referenceFrame_(referenceFrame),
          isInitialized(false),
          m_mutex_()
    {
        this->pose_.header.frame_id = referenceFrame_;
        ROS_INFO("[Pose] Creating Pose tracker component to track %s in the reference frame %s", targetFrame.c_str(), referenceFrame.c_str());

        {
            //singleton
            boost::lock_guard<boost::mutex> guard(listenerMutex_);
            tfListener_ = std::make_shared<tf::TransformListener>();
        }
    }

    void Pose::waitTransformUpdate(ros::Rate r)
    {
        bool found = false;
        while (ros::ok() && !found)
        {
            tf::StampedTransform transform;
            try
            {
                {
                    boost::lock_guard<boost::mutex> lock(listenerMutex_);
                    tfListener_->lookupTransform(referenceFrame_, poseFrameName_,
                                                      ros::Time(0), transform);
                }

                {
                    boost::lock_guard<boost::mutex> guard(m_mutex_);
                    tf::poseTFToMsg(transform, this->pose_.pose);
                    this->pose_.header.stamp = transform.stamp_;
                    found = true;
                    this->isInitialized = true;
                }
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR_STREAM_THROTTLE(1, "[Component pose] (" << poseFrameName_ << "/[" << referenceFrame_ << "] ) is failing on pose update : " << ex.what());
            }

            r.sleep();
            ros::spinOnce();
        }
    }

geometry_msgs::Pose Pose::toPoseMsg()
  {
    boost::lock_guard<boost::mutex> guard(m_mutex_);
    return this->pose_.pose;
  }

  geometry_msgs::PoseStamped Pose::toPoseStampedMsg()
  {
    boost::lock_guard<boost::mutex> guard(m_mutex_);
    return this->pose_;
  }

    void Pose::update()
    {
        tf::StampedTransform transform;
        try
        {
            {
                boost::lock_guard<boost::mutex> lock(listenerMutex_);
                tfListener_->lookupTransform(referenceFrame_, poseFrameName_,
                                                  ros::Time(0), transform);
            }

            {
                boost::lock_guard<boost::mutex> guard(m_mutex_);
                tf::poseTFToMsg(transform, this->pose_.pose);
                this->pose_.header.stamp = transform.stamp_;
                this->isInitialized = true;
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR_STREAM_THROTTLE(1, "[Component pose] (" << poseFrameName_ << "/[" << referenceFrame_ << "] ) is failing on pose update : " << ex.what());
        }
    }
} // namespace cl_move_base_z
