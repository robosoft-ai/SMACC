#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <planner_switcher/planner_switcher.h>

#include <boost/optional.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace move_base_z_client
{
class CbAbsoluteRotate : public smacc::SmaccClientBehavior
{
public:
    tf::TransformListener listener;

    ClMoveBaseZ *moveBaseClient_;

    boost::optional<float> absoluteGoalAngleDegree;

    CbAbsoluteRotate()
    {
    }

    CbAbsoluteRotate(float absoluteGoalAngleDegree)
    {
        this->absoluteGoalAngleDegree = absoluteGoalAngleDegree;
    }

    virtual void onEntry() override
    {
        double goal_angle;

        if (!this->absoluteGoalAngleDegree)
        {
            this->currentState->param("goal_angle", goal_angle, 45.0);
        }
        else
        {
            goal_angle = *this->absoluteGoalAngleDegree;
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

        ClMoveBaseZ::Goal goal;
        goal.target_pose.header.frame_id = "/odom";
        goal.target_pose.header.stamp = ros::Time::now();

        auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
        auto targetAngle = goal_angle * M_PI / 180.0;
        goal.target_pose.pose.position = currentPoseMsg.position;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetAngle);

        ROS_INFO_STREAM("current pose: " << currentPoseMsg);
        ROS_INFO_STREAM("goal pose: " << goal.target_pose.pose);
        moveBaseClient_->sendGoal(goal);
    }
};
} // namespace move_base_z_client
