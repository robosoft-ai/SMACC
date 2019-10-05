#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <boost/optional.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>


class Rotate : public smacc::SmaccSubStateBehavior
{
 public:
  tf::TransformListener listener;

  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  boost::optional<float> rotateDegree;

  Rotate()
  {

  }

  Rotate(float rotate_degree)
  {
    rotateDegree = rotate_degree;
  }

  virtual void onEntry() override
  {
    double  angle_increment_degree;

    if(!rotateDegree)
    {
        this->currentState->param("angle_increment_degree", angle_increment_degree, 90.0);
    }
    else
    {
       angle_increment_degree = *rotateDegree;
    }

    this->requiresClient(moveBaseClient_ );
    
    //this->plannerSwitcher_->setDefaultPlanners();
    moveBaseClient_->plannerSwitcher_->setForwardPlanner();

    //this should work better with a coroutine and await
    ros::Rate rate(10.0);
    geometry_msgs::Pose currentPoseMsg;
    while (ros::ok())
    {
        tf::StampedTransform currentPose;
        try{
          listener.lookupTransform("/odom", "/base_link",  
                                ros::Time(0), currentPose);

          tf::poseTFToMsg (currentPose, currentPoseMsg);
        break;
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }
    }   

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();

    auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
    auto targetAngle = currentAngle + angle_increment_degree *M_PI/180.0;
    goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(targetAngle);
    goal.target_pose.pose.position = currentPoseMsg.position;
    ROS_WARN_STREAM("rotate behavior current_state : " << currentPoseMsg);
    ROS_WARN_STREAM("rotate behavior target : " << goal.target_pose.pose);

    ROS_INFO("forward behavior: sending forward goal");
    moveBaseClient_->sendGoal(goal);
  }    
};
