#include <move_group_interface_client/client_behaviors/cb_pouring_motion.h>

/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_circular_pivot_motion.h>

namespace cl_move_group_interface
{
    CbCircularPouringMotion::CbCircularPouringMotion(std::string tipLink)
        : CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    CbCircularPouringMotion::CbCircularPouringMotion(const geometry_msgs::PoseStamped &planePivotPose, double deltaHeight, std::string tipLink)
        : pivotPose_(planePivotPose), deltaHeight_(deltaHeight), CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    CbCircularPouringMotion::CbCircularPouringMotion(const geometry_msgs::PoseStamped &planePivotPose, const geometry_msgs::Pose &relativeInitialPose, double deltaHeight, std::string tipLink)
        : pivotPose_(planePivotPose), relativeInitialPose_(relativeInitialPose), deltaHeight_(deltaHeight), CbMoveEndEffectorTrajectory(tipLink)
    {
    }

    void CbCircularPouringMotion::generateTrajectory()
    {
        if (!relativeInitialPose_)
        {
            this->computeCurrentEndEffectorPoseRelativeToPivot();
        }

        // project offset into the xy-plane
        // get the radius
        double radius = sqrt(relativeInitialPose_->position.z * relativeInitialPose_->position.z + relativeInitialPose_->position.y * relativeInitialPose_->position.y);
        double initialAngle = atan2(relativeInitialPose_->position.z, relativeInitialPose_->position.y);

        double totallineardist = radius * deltaHeight_;
        double totalangulardist = deltaHeight_;

        // at least 1 sample per centimeter (average)
        // at least 1 sample per ~1.1 degrees (average)

        const double RADS_PER_SAMPLE = 0.02;
        const double METERS_PER_SAMPLE = 0.01;

        int totalSamplesCount = std::max(totallineardist / METERS_PER_SAMPLE, totalangulardist / RADS_PER_SAMPLE);

        double linearSecondsPerSample;
        double angularSecondsPerSamples;
        double secondsPerSample;

        if (linearSpeed_m_s_)
        {
            linearSecondsPerSample = METERS_PER_SAMPLE / (*linearSpeed_m_s_);
        }
        else
        {
            linearSecondsPerSample = std::numeric_limits<double>::max();
        }

        if (angularSpeed_rad_s_)
        {
            angularSecondsPerSamples = RADS_PER_SAMPLE / (*angularSpeed_rad_s_);
        }
        else
        {
            angularSecondsPerSamples = std::numeric_limits<double>::max();
        }

        if (!linearSpeed_m_s_ && !angularSpeed_rad_s_)
        {
            secondsPerSample = 0.5;
        }
        else
        {
            secondsPerSample = std::min(linearSecondsPerSample, angularSecondsPerSamples);
        }

        double currentAngle = initialAngle;

        double angleStep = deltaHeight_ / (double)totalSamplesCount;

        tf::Transform tfBasePose;
        tf::poseMsgToTF(pivotPose_.pose, tfBasePose);

        for (int i = 0; i < totalSamplesCount; i++)
        {
            // relativePose i
            currentAngle += angleStep;
            double y = radius * cos(currentAngle);
            double z = radius * sin(currentAngle);

            geometry_msgs::Pose relativeCurrentPose;

            relativeCurrentPose.position.x = relativeInitialPose_->position.x;
            relativeCurrentPose.position.y = y;
            relativeCurrentPose.position.z = z;

            auto localquat = tf::createQuaternionFromRPY(currentAngle, 0, 0);
            //relativeCurrentPose.orientation = relativeInitialPose_.orientation;
            //tf::quaternionTFToMsg(localquat, relativeCurrentPose.orientation);
            relativeCurrentPose.orientation.w = 1;

            tf::Transform tfRelativeCurrentPose;
            tf::poseMsgToTF(relativeCurrentPose, tfRelativeCurrentPose);

            tf::Transform tfGlobalPose = tfRelativeCurrentPose * tfBasePose;

            tfGlobalPose.setRotation(tfGlobalPose.getRotation() * localquat);

            geometry_msgs::PoseStamped globalPose;
            tf::poseTFToMsg(tfGlobalPose, globalPose.pose);
            globalPose.header.frame_id = pivotPose_.header.frame_id;
            globalPose.header.stamp = pivotPose_.header.stamp + ros::Duration(i * secondsPerSample);

            this->endEffectorTrajectory_.push_back(globalPose);
        }
    }

    void CbCircularPouringMotion::computeCurrentEndEffectorPoseRelativeToPivot()
    {
        //auto currentRobotEndEffectorPose = this->movegroupClient_->moveGroupClientInterface.getCurrentPose();

        tf::TransformListener tfListener;
        // tf::StampedTransform globalBaseLink;
        tf::StampedTransform endEffectorInPivotFrame;

        try
        {
            if (!tipLink_ || *tipLink_ == "")
            {
                tipLink_ = this->movegroupClient_->moveGroupClientInterface.getEndEffectorLink();
            }

            tfListener.waitForTransform(this->pivotPose_.header.frame_id, *tipLink_, ros::Time(0), ros::Duration(10));
            tfListener.lookupTransform(this->pivotPose_.header.frame_id, *tipLink_, ros::Time(0), endEffectorInPivotFrame);

            // we define here the global frame as the pivot frame id
            // tfListener.waitForTransform(currentRobotEndEffectorPose.header.frame_id, this->pivotPose_.header.frame_id, ros::Time(0), ros::Duration(10));
            // tfListener.lookupTransform(currentRobotEndEffectorPose.header.frame_id, this->pivotPose_.header.frame_id, ros::Time(0), globalBaseLink);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // tf::Transform endEffectorInBaseLinkFrame;
        // tf::poseMsgToTF(currentRobotEndEffectorPose.pose, endEffectorInBaseLinkFrame);

        // tf::Transform endEffectorInPivotFrame = globalBaseLink * endEffectorInBaseLinkFrame; // pose composition

        // now pivot and EndEffector share a common reference frame (let say map)
        // now get the current pose from the pivot reference frame with inverse composition
        tf::Transform pivotTransform;
        tf::poseMsgToTF(this->pivotPose_.pose, pivotTransform);
        tf::Transform invertedNewReferenceFrame = pivotTransform.inverse();

        tf::Transform currentPoseRelativeToPivot = invertedNewReferenceFrame * endEffectorInPivotFrame;

        geometry_msgs::Pose finalEndEffectorRelativePose;
        tf::poseTFToMsg(currentPoseRelativeToPivot, finalEndEffectorRelativePose);
        relativeInitialPose_ = finalEndEffectorRelativePose;
    }

    void CbCircularPouringMotion::createMarkers()
    {
        CbMoveEndEffectorTrajectory::createMarkers();

        tf::Transform localdirection;
        localdirection.setIdentity();
        localdirection.setOrigin(tf::Vector3(0.12, 0, 0));
        auto frameid = this->pivotPose_.header.frame_id;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frameid;
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = beahiorMarkers_.markers.size();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0;
        marker.color.b = 1.0;

        geometry_msgs::Point start, end;
        start.x = this->pivotPose_.pose.position.x;
        start.y = this->pivotPose_.pose.position.y;
        start.z = this->pivotPose_.pose.position.z;

        tf::Transform basetransform;
        tf::poseMsgToTF(this->pivotPose_.pose, basetransform);
        tf::Transform endarrow = localdirection * basetransform;

        end.x = endarrow.getOrigin().x();
        end.y = endarrow.getOrigin().y();
        end.z = endarrow.getOrigin().z();

        marker.pose.orientation.w = 1;
        marker.points.push_back(start);
        marker.points.push_back(end);

        beahiorMarkers_.markers.push_back(marker);
    }

} // namespace cl_move_group_interface