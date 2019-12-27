#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <planner_switcher/planner_switcher.h>

#include <boost/optional.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>
namespace sm_dance_bot
{
namespace move_base_z_client
{
class CbRotate : public smacc::SmaccClientBehavior
{
public:
    tf::TransformListener listener;

    smacc::ClMoveBaseZ *moveBaseClient_;

    boost::optional<float> rotateDegree;

    CbRotate()
    {
    }

    CbRotate(float rotate_degree)
    {
        rotateDegree = rotate_degree;
    }

    virtual void onEntry() override
    {
        double angle_increment_degree;

        if (!rotateDegree)
        {
            this->currentState->param("angle_increment_degree", angle_increment_degree, 45.0);
        }
        else
        {
            angle_increment_degree = *rotateDegree;
        }

        this->requiresClient(moveBaseClient_);

        //this should work better with a coroutine and await
        //this->plannerSwitcher_->setForwardPlanner();
        moveBaseClient_->plannerSwitcher_->setDefaultPlanners();

        ros::Rate rate(10.0);
        geometry_msgs::Pose currentPoseMsg;
        while (ros::ok())
        {
            tf::StampedTransform currentPose;
            try
            {
                listener.lookupTransform("/odom", "/base_link",
                                         ros::Time(0), currentPose);

                tf::poseTFToMsg(currentPose, currentPoseMsg);
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        smacc::ClMoveBaseZ::Goal goal;
        goal.target_pose.header.frame_id = "/odom";
        goal.target_pose.header.stamp = ros::Time::now();

        auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
        auto targetAngle = currentAngle + angle_increment_degree * M_PI / 180.0;
        goal.target_pose.pose.position = currentPoseMsg.position;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetAngle);

        ROS_INFO_STREAM("current pose: " << currentPoseMsg);
        ROS_INFO_STREAM("goal pose: " << goal.target_pose.pose);
        moveBaseClient_->sendGoal(goal);
    }
};
} // namespace move_base_z_client
} // namespace sm_dance_bot