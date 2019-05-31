#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace NavigateToEndPoint 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// NavigateToEndPoint State
struct NavigateToEndPoint
    : SmaccState<NavigateToEndPoint, RadialMotionStateMachine,
                 mpl::list<NavigationOrthogonalLine,ToolOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  // when this state is finished move to the ReturnToRadialStart state
  typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>, ReturnToRadialStart::ReturnToRadialStart> reactions; 

public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("-------");
    ROS_INFO("Initializating Navigate to endpoint state");
  }

  void onExit()
  {
  }
};

//------------------------------------------------------------------------------
// orthogonal line 0
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, NavigateToEndPoint::orthogonal<0>,
                 Navigate> {
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
  }
};


    // this is the navigate substate inside the navigation orthogonal line of the RotateDegreess State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> 
{
public:
  // the angle of the current radial motion
  double yaw;

  // distance parameter of the motion
  double dist;

  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresComponent(moveBaseClient_ ,ros::NodeHandle("move_base"));
    this->requiresComponent(odomTracker_);
    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    // read from the state machine yaw "global variable"
    this->getGlobalSMData("current_yaw", yaw);
    ROS_INFO_STREAM("[NavigateToEndPoint/Navigate] current yaw: " << yaw);

    // straight motion distance
    this->param("straight_motion_distance", dist, 3.5);
    ROS_INFO_STREAM("Straight motion distance: " << dist);

    goToEndPoint();
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void goToEndPoint() {
    geometry_msgs::PoseStamped radialStartPose;
    this->getGlobalSMData("radial_start_pose", radialStartPose);

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.stamp = ros::Time::now();

    // in order to find the goal we create a virtual line from the origin to
    // some yaw direction and some distance
    goal.target_pose = radialStartPose;
    goal.target_pose.pose.position.x += cos(yaw) * dist;
    goal.target_pose.pose.position.y += sin(yaw) * dist;
    goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

    moveBaseClient_->sendGoal(goal);

    this->odomTracker_->clearPath();
    ROS_WARN_STREAM("SETTING STARTING POINT" << radialStartPose);
    this->odomTracker_->setStartPoint(radialStartPose);
  }

  void onExit() 
  { 
    ROS_INFO("Exiting move goal Action Client"); 
  }

private:
  // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server). 
  // this resource can be used from any method in this state
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  smacc_odom_tracker::OdomTracker* odomTracker_;

  smacc_planner_switcher::PlannerSwitcher* plannerSwitcher_;  
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToEndPoint::orthogonal<1>, ToolSubstate> {
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  void onExit()
  { 
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};

struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:
    using SmaccState::SmaccState;
    SMACC_STATE_BEHAVIOR(ToolBehaviorKeyName);
};

}
