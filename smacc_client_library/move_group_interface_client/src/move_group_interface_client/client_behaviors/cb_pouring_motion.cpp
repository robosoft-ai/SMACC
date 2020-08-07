#include <move_group_interface_client/client_behaviors/cb_pouring_motion.h>

/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_circular_pivot_motion.h>

namespace cl_move_group_interface
{
    CbCircularPouringMotion::CbCircularPouringMotion(const geometry_msgs::PointStamped &planePivotPoint, double deltaHeight, std::string tipLink)
        : pivotPoint_(planePivotPoint)
        , deltaHeight_(deltaHeight)
        , CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    void CbCircularPouringMotion::getCurrentEndEffectorPose(tf::StampedTransform& currentEndEffectorTransform)
    {
        tf::TransformListener tfListener;

        try
        {
            if (!tipLink_ || *tipLink_ == "")
            {
                tipLink_ = this->movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
            }

            tfListener.waitForTransform(this->pivotPoint_.header.frame_id, *tipLink_, ros::Time(0), ros::Duration(10));
            tfListener.lookupTransform(this->pivotPoint_.header.frame_id, *tipLink_, ros::Time(0), currentEndEffectorTransform);

            // we define here the global frame as the pivot frame id
            // tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, ros::Time(0), ros::Duration(10));
            // tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, planePivotPose_.header.frame_id, ros::Time(0), globalBaseLink);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void CbCircularPouringMotion::generateTrajectory()
    {
        // at least 1 sample per centimeter (average)        
        const double METERS_PER_SAMPLE = 0.01;

        float dist_meters =0;
        float totallineardist = fabs(this->deltaHeight_);

        int totalSamplesCount = totallineardist / METERS_PER_SAMPLE;
        int steps = totallineardist/ METERS_PER_SAMPLE;

        float interpolation_factor_step = 1.0/totalSamplesCount;

        double secondsPerSample;

        if (linearSpeed_m_s_)
        {
            secondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
        }
        else
        {
            secondsPerSample = std::numeric_limits<double>::max();
        }

        tf::Vector3 v0, v1;
        tf::StampedTransform currentEndEffectorTransform;

        this->getCurrentEndEffectorPose(currentEndEffectorTransform);

        tf::Vector3 pivot;
        tf::pointMsgToTF(pivotPoint_.point,pivot);
        v0 = currentEndEffectorTransform.getOrigin() - pivot ;
        v1 = v0;
        v1.setZ(v1.z()- this->deltaHeight_);

        tf::Quaternion initialOrientation = currentEndEffectorTransform.getRotation();
        tf::Vector3 vp1(v1);
        tf::Vector3 vp0(v0);
        tf::Quaternion rotation = tf::shortestArcQuatNormalize2(vp1,vp0);
        auto finalOrientation = initialOrientation*rotation ;

        auto shortestAngle = tf::angleShortestPath(initialOrientation, finalOrientation);

        v0+=pivot;
        v1+=pivot;
        
        float linc = deltaHeight_/steps;// METERS_PER_SAMPLE with sign
        float interpolation_factor = 0;
        tf::Vector3 vi = v0;
        for (float i =0; i < steps; i++)
        {
            auto q = tf::slerp(initialOrientation, finalOrientation, interpolation_factor);

            interpolation_factor += interpolation_factor_step;
            dist_meters += linc;

            vi = v0;
            vi.setZ(vi.getZ() + dist_meters);

            tf::Transform pose;
            pose.setOrigin(vi);
            pose.setRotation(q);

            geometry_msgs::PoseStamped globalPose;
            tf::poseTFToMsg(pose, globalPose.pose);
            globalPose.header.frame_id = pivotPoint_.header.frame_id;
            globalPose.header.stamp = pivotPoint_.header.stamp + ros::Duration(i * secondsPerSample);

            this->endEffectorTrajectory_.push_back(globalPose);
        }
    }

    void CbCircularPouringMotion::createMarkers()
    {
        CbMoveEndEffectorTrajectory::createMarkers();

        visualization_msgs::Marker marker;

        marker.ns = "trajectory";
        marker.id = beahiorMarkers_.markers.size();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0;
        marker.color.b = 1.0;

        marker.pose.position = this->pivotPoint_.point;
        marker.header.frame_id = this->pivotPoint_.header.frame_id;
        marker.header.stamp = ros::Time::now();

        beahiorMarkers_.markers.push_back(marker);
    }

} // namespace cl_move_group_interface