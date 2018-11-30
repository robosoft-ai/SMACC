#pragma once

#include <radial_motion.h>
#include <thread>

namespace NavigateToRadialStart 
{
using namespace smacc;

//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// State NavigateToRadialStart
struct NavigateToRadialStart
    : SmaccState<NavigateToRadialStart, RadialMotionStateMachine,
                 mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  // when this state is finished then move to the RotateDegress state
  typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>, RotateDegress::RotateDegress> reactions; 

public:
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  NavigateToRadialStart(my_context ctx)
      : SmaccState<NavigateToRadialStart, RadialMotionStateMachine,
                   mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>>(ctx) 
    {
       ROS_INFO("-------");
       ROS_INFO("Entering in NavigateToRadialStart State");
    }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~NavigateToRadialStart() 
  {
    ROS_INFO("Finishing NavigateToRadialStart state");
  }
};

//--------------------------------------------------
// orthogonal line 0
struct NavigationOrthogonalLine
    : public SmaccState<NavigationOrthogonalLine,
                        NavigateToRadialStart::orthogonal<0>, Navigate> 
  {
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  NavigationOrthogonalLine(my_context ctx)
      : SmaccState<NavigationOrthogonalLine,
                   NavigateToRadialStart::orthogonal<0>, Navigate>(ctx) // call the SmaccState base constructor
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
// this is the navigate substate inside the navigation orthogonal line of the NavigateToRadialStart State
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

    goToRadialStart(); 
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void goToRadialStart() 
  {
    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();
    readStartPoseFromParameterServer(goal);

    // store the start pose on the state machine storage so that it can
    // be referenced from other states (for example return to radial start)
    this->setGlobalData("radial_start_pose", goal.target_pose);

    moveBaseClient_->sendGoal(goal);
  }

  void readStartPoseFromParameterServer(smacc::SmaccMoveBaseActionClient::Goal& goal)
  {
    getParam("start_position_x", goal.target_pose.pose.position.x);
    getParam("start_position_y", goal.target_pose.pose.position.y);
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO_STREAM("start position read from parameter server: " << goal.target_pose.pose.position);
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

  smacc_odom_tracker::PlannerSwitcher* plannerSwitcher_;
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToRadialStart::orthogonal<1>, ToolSubstate> {
public:
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, NavigateToRadialStart::orthogonal<1>, ToolSubstate>(ctx) // call the SmaccState base constructor                 
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
