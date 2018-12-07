#pragma once

#include <waypoints_machine.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace NavigateToEvenWaypoint 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// NavigateToEvenWaypoint State
struct NavigateToEvenWaypoint
    : SmaccState<NavigateToEvenWaypoint, WayPointsStateMachine,
                 mpl::list<NavigationOrthogonalLine,ToolOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  // when this state is finished move to the NavigateToOddWaypoint state
  typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>, NavigateToOddWaypoint::NavigateToOddWaypoint> reactions; 

public:
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  NavigateToEvenWaypoint(my_context ctx)
      : SmaccState<NavigateToEvenWaypoint, WayPointsStateMachine, 
                    mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("-------");
    ROS_INFO("Initializating Navigate to endpoint state");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~NavigateToEvenWaypoint() 
  {
  }
};

//------------------------------------------------------------------------------
// orthogonal line 0
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, NavigateToEvenWaypoint::orthogonal<0>,
                 Navigate> {
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  NavigationOrthogonalLine(my_context ctx)
      : SmaccState<NavigationOrthogonalLine, NavigateToEvenWaypoint::orthogonal<0>,Navigate>(ctx) // call the SmaccState base constructor
  {
  }
};

//------------------------------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the RotateDegreess State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> 
{

private:
  // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server). 
  // this resource can be used from any method in this state
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  smacc_odom_tracker::OdomTracker* odomTracker_;

  smacc_planner_switcher::PlannerSwitcher* plannerSwitcher_;  

  std::shared_ptr<std::vector<geometry_msgs::Point>> waypoints_;

  int currentWayPointIndex_;

public:
  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx):
    SmaccState<Navigate, NavigationOrthogonalLine> (ctx)
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresComponent(moveBaseClient_ ,ros::NodeHandle("move_base"));
    this->requiresComponent(odomTracker_);
    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    this->getGlobalSMData("waypoints", waypoints_);
    this->getGlobalSMData("waypoint_index", currentWayPointIndex_);

    if (currentWayPointIndex_ >= waypoints_->size())
    {
      this->terminate();
    }
    else
    {
      gotoNextPoint();
    }
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void gotoNextPoint() 
  {  
    // setup new goal pose
    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "odom";

    goal.target_pose.pose.position = (*waypoints_)[currentWayPointIndex_];
    goal.target_pose.pose.orientation.w = 1;

    // update waypoint
    currentWayPointIndex_++;
    this->setGlobalSMData("waypoint_index", currentWayPointIndex_);
    
    // send request
    moveBaseClient_->sendGoal(goal);
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  ~Navigate() 
  { 
    ROS_INFO("Exiting move goal Action Client"); 
  }
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToEvenWaypoint::orthogonal<1>, ToolSubstate> {
public:
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, NavigateToEvenWaypoint::orthogonal<1>, ToolSubstate>(ctx) // call the SmaccState base constructor                 
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  ~ToolOrthogonalLine() 
  { 
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};

//-----------------------------------------------------1----------------------------------------------------
struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
   ToolSubstate(my_context ctx) 
    : SmaccState<ToolSubstate, ToolOrthogonalLine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering ToolSubstate");

    this->requiresComponent(toolActionClient_ ,ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_START;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
}
