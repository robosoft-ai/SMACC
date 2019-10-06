#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <boost/optional.hpp>
#include <geometry_msgs/Point.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_odom_tracker/odom_tracker.h>
#include <smacc_planner_switcher/planner_switcher.h>

class NavigateGlobalPosition : public smacc::SmaccSubStateBehavior
{
public:
  
  boost::optional<geometry_msgs::Point> initialPoint;

  NavigateGlobalPosition()
  {

  }

  NavigateGlobalPosition(float x, float y)
  {
    auto p =  geometry_msgs::Point();
    p.x = x;
    p.y =y;
    initialPoint = p;
  }

  virtual void onEntry()
  {
    ROS_INFO("Entering Navigate Global position");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresClient(moveBaseClient_ );
    this->requiresComponent(odomTracker_);
    
    ROS_INFO("Component requirements completed");

    moveBaseClient_->plannerSwitcher_->setForwardPlanner();
    this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::RECORD_PATH_FORWARD);

    goToRadialStart(); 
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void goToRadialStart() 
  {
    ROS_INFO("Sending Goal to MoveBase");
    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();
    readStartPoseFromParameterServer(goal);

    // store the start pose on the state machine storage so that it can
    // be referenced from other states (for example return to radial start)
    this->stateMachine->setGlobalSMData("radial_start_pose", goal.target_pose);

    moveBaseClient_->sendGoal(goal);
  }

  void readStartPoseFromParameterServer(smacc::SmaccMoveBaseActionClient::Goal& goal)
  {
    if(! initialPoint)
    {
      this->currentState->getParam("start_position_x", goal.target_pose.pose.position.x);
      this->currentState->getParam("start_position_y", goal.target_pose.pose.position.y);
    }
    else
    {
      goal.target_pose.pose.position= *initialPoint;
    }
    
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO_STREAM("start position read from parameter server: " << goal.target_pose.pose.position);
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  virtual void onExit() override
  { 
    ROS_INFO("Exiting move goal Action Client"); 
  }

private:
  // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server). 
  // this resource can be used from any method in this state
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  smacc_odom_tracker::OdomTracker* odomTracker_;
};