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
    : SmaccState<NavigationOrthogonalLine, NavigateToEvenWaypoint::orthogonal<0>,Navigate> 
{
public:
  using SmaccState::SmaccState;

  void onEntry()
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
  using SmaccState::SmaccState;

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  void onEntry()
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresComponent(moveBaseClient_ ,ros::NodeHandle("move_base"));
    this->requiresComponent(odomTracker_);
    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    ROS_WARN("Getting global waypoints data...");

    this->getGlobalSMData("waypoints", waypoints_);
    
    ROS_INFO("1");
    this->getGlobalSMData("waypoint_index", currentWayPointIndex_);
    ROS_INFO("2");

    ROS_WARN("set default ROS Planner");

    //this->plannerSwitcher_->setForwardPlanner();
    this->plannerSwitcher_->setDefaultPlanners();
    this->odomTracker_->clearPath();
    this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::RECORD_PATH_FORWARD);

    ROS_WARN("current waypoint index: %d", currentWayPointIndex_);
    if (currentWayPointIndex_ >= waypoints_->size())
    {
      ROS_WARN("Terminate");
      this->terminate();
    }
    else
    {
      ROS_WARN("Go to next point");
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
    //currentWayPointIndex_++;
    this->setGlobalSMData("waypoint_index", currentWayPointIndex_);
    
    // send request
    moveBaseClient_->sendGoal(goal);
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  void onExit()
  { 
    ROS_INFO("Exiting move goal Action Client"); 
  }
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToEvenWaypoint::orthogonal<1>, ToolSubstate> 
{
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

//-----------------------------------------------------1----------------------------------------------------
struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:
  using SmaccState::SmaccState;
  
  void onEntry()
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