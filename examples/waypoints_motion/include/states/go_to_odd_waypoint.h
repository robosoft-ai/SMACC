#pragma once

#include <waypoints_machine.h>
#include <thread>

namespace NavigateToOddWaypoint 
{
using namespace smacc;

//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;
struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// State NavigateToOddWaypoint
struct NavigateToOddWaypoint
    : SmaccState<NavigateToOddWaypoint, WayPointsStateMachine,
                 mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>, NavigateToEvenWaypoint::NavigateToEvenWaypoint> reactions; 

public:

  // reference the parent context parameter constructor
  using SmaccState::SmaccState;

  void onEntry()
  {
      ROS_INFO("-------");
      ROS_INFO("Entering in NavigateToOddWaypoint State");
  }

  void onExit()
  {
    ROS_INFO("Finishing NavigateToOddWaypoint state");
  }
};

//--------------------------------------------------
// orthogonal line 0
struct NavigationOrthogonalLine
    : public SmaccState<NavigationOrthogonalLine,
                        NavigateToOddWaypoint::orthogonal<0>, Navigate> 
  {
public:
  // reference the parent context parameter constructor (required)
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering in move_base orthogonal line");
  }

  void onExit()
  {
    ROS_INFO("Finishing move base orthogonal line");
  }
};

//--------------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the NavigateToOddWaypoint State
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

  void onEntry()
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresComponent(moveBaseClient_ ,ros::NodeHandle("move_base"));
    this->requiresComponent(odomTracker_);
    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    this->getGlobalSMData("waypoints", waypoints_);
    this->getGlobalSMData("waypoint_index", currentWayPointIndex_);

    ros::spinOnce();

    ROS_WARN("Setting backward planner");
    this->plannerSwitcher_->setBackwardPlanner();
    ROS_WARN("coinfiguring odom tracker");
    this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::CLEAR_PATH_BACKWARD);

    ros::spinOnce();
    //ros::Duration(2).sleep();

    if (currentWayPointIndex_ >= waypoints_->size())
    {
      ROS_WARN("Terminate, currentWayPointIndex: %d", currentWayPointIndex_);
      this->terminate();
    }
    else
    {
      ROS_INFO("Go to next Point");
      // pose = (*waypoints_)[currentWayPointIndex_]
      // goal.target_pose.pose.orientation.w = 1;
      geometry_msgs::Pose pose = this->odomTracker_->getPath().poses.front().pose;
      gotoNextPoint(pose);
    }
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void gotoNextPoint(const geometry_msgs::Pose& pose) 
  {  
    // setup new goal pose
    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.pose= pose;
    
    ROS_WARN_STREAM("Undo trajectory: " << goal);
    
    // update waypoint
    currentWayPointIndex_++;
    this->setGlobalSMData("waypoint_index", currentWayPointIndex_);
    
    // send request
    moveBaseClient_->sendGoal(goal);
  }

  void onExit()
  { 
    ROS_INFO("Exiting move goal Action Client"); 
  }
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToOddWaypoint::orthogonal<1>, ToolSubstate> {
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

//---------------------------------------------------------------------------------------------------------
struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering ToolSubstate");
    this->requiresComponent(toolActionClient_ , ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

private:
  smacc::SmaccToolActionClient* toolActionClient_;
};
}
