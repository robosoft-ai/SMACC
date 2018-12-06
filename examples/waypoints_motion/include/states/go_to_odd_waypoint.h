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
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  NavigateToOddWaypoint(my_context ctx)
      : SmaccState<NavigateToOddWaypoint, WayPointsStateMachine,
                   mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>>(ctx) 
    {
       ROS_INFO("-------");
       ROS_INFO("Entering in NavigateToOddWaypoint State");
    }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~NavigateToOddWaypoint() 
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
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  NavigationOrthogonalLine(my_context ctx)
      : SmaccState<NavigationOrthogonalLine,
                   NavigateToOddWaypoint::orthogonal<0>, Navigate>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering in move_base orthogonal line");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~NavigationOrthogonalLine() 
  {
    ROS_INFO("Finishing move base orthogonal line");
  }
};

//--------------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the NavigateToOddWaypoint State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> {
public:
  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx) 
    : SmaccState<Navigate, NavigationOrthogonalLine>(ctx) // call the SmaccState base constructor 
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresComponent(moveBaseClient_ , ros::NodeHandle("move_base"));
    this->requiresComponent(odomTracker_);
    this->requiresComponent(plannerSwitcher_ , ros::NodeHandle("move_base"));   

    this->plannerSwitcher_->setForwardPlanner();
    this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::RECORD_PATH_FORWARD);

    gotoNextPoint(); 
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void gotoNextPoint() 
  {
    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();
    
    moveBaseClient_->sendGoal(goal);
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  ~Navigate() 
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
    : SmaccState<ToolOrthogonalLine, NavigateToOddWaypoint::orthogonal<1>, ToolSubstate> {
public:
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, NavigateToOddWaypoint::orthogonal<1>, ToolSubstate>(ctx) // call the SmaccState base constructor                 
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  ~ToolOrthogonalLine() 
  { 
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};
//---------------------------------------------------------------------------------------------------------
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
    this->requiresComponent(toolActionClient_ , ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
}
