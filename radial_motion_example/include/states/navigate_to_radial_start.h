#pragma once

#include <radial_motion.h>

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
  //typedef sc::transition<EvStateFinished, RotateDegress::RotateDegress> reactions;
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
  typedef mpl::list</*sc::custom_reaction<EvActionResultBase<smacc::SmaccMoveBaseActionClient::Result>>,*/
                    sc::custom_reaction<EvReelInitialized>> reactions;

public:
  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx) 
    : SmaccState<Navigate, NavigationOrthogonalLine>(ctx) // call the SmaccState base constructor 
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
        context<RadialMotionStateMachine>()
            .requiresComponent<smacc::SmaccMoveBaseActionClient>(moveBaseClient_ , ros::NodeHandle(
                "move_base"));
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
    context<RadialMotionStateMachine>().setData("radial_start_pose",
                                                goal.target_pose);

    moveBaseClient_->sendGoal(goal);
  }

  void readStartPoseFromParameterServer(smacc::SmaccMoveBaseActionClient::Goal& goal)
  {
    getParam("start_position_x", goal.target_pose.pose.position.x);
    getParam("start_position_y", goal.target_pose.pose.position.y);
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO_STREAM("start position read from parameter server: " << goal.target_pose.pose.position);
  }

  // when the reel substate is finished we will react starting the motion
  sc::result react(const EvReelInitialized &ev) 
  { 
    goToRadialStart(); 
  }

  // this is the callback when the navigate action of this state is finished
  // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
  /*
  sc::result react(const EvActionResultBase<smacc::SmaccMoveBaseActionClient::Result> &ev) 
  {
    if (ev.client == moveBaseClient_) 
    {
      if (ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
        ROS_INFO("move base, goal position reached.");

        // notify the parent State to finish via event (the current parent state reacts to this event)
        post_event(EvStateFinished());
        
        // declare this substate as finished
        //return terminate();
        return discard_event();
      } 
      else if (ev.getResult() == actionlib::SimpleClientGoalState::ABORTED) 
      {
        // repeat the navigate action request to the move base node if we get ABORT as response
        // It may work if try again. Move base sometime rejects the request because it is busy.
        goToRadialStart();

        // this event was for us. We have used it without moving to any other state. Do not let others consume it.
        return discard_event();
      }
    } 
    else 
    {
      // the action client event success is not for this substate. Let others process this event.
      ROS_INFO("navigate substate lets other process the EvActionResultEv");
      return forward_event();
    }
  }
  */

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
    : SmaccState<ToolSubstate, ToolOrthogonalLine> {
  
public:

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  ToolSubstate(my_context ctx) 
    : SmaccState<ToolSubstate, ToolOrthogonalLine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering ToolSubstate");
    context<RadialMotionStateMachine>().requiresComponent<smacc::SmaccToolActionClient>(toolActionClient_ , ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
}
