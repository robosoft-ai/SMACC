#pragma once

#include "radial_motion.h"
#include <angles/angles.h>
#include <tf/tf.h>

//--------------------------------------------
namespace RotateDegress 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;
struct Navigate;
struct ReelDispense;
struct ReelOrthogonalLine;
struct ToolSubstate;

/// State NavigateToRadialStart
struct RotateDegress
    : SmaccState<RotateDegress, RadialMotionStateMachine, mpl::list<NavigationOrthogonalLine, ReelOrthogonalLine, ToolOrthogonalLine>> // <- these are the orthogonal lines of this State 
{
  // when this state is finished then move to the NavigateToEndPoint state
  typedef sc::transition<EvStateFinished, NavigateToEndPoint::NavigateToEndPoint> reactions;

public:
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  RotateDegress(my_context ctx)
      : SmaccState<RotateDegress, RadialMotionStateMachine,
                   mpl::list<NavigationOrthogonalLine, ReelOrthogonalLine, ToolOrthogonalLine>>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~RotateDegress() { ROS_INFO("Exiting in ROTATE TEN DEGREES STATE"); }
};

//------------------------------------------------------------------
// orthogonal line 0Reel_ActionClient
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, RotateDegress::orthogonal<0>,
                 Navigate> 
{
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  NavigationOrthogonalLine(my_context ctx)
      : SmaccState<NavigationOrthogonalLine, RotateDegress::orthogonal<0>,
                   Navigate>(ctx) // call the SmaccState base constructor
    {
    }
};
//------------------------------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the RotateDegreess State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> {
  typedef mpl::list<sc::custom_reaction<EvActionResult>,
                    sc::custom_reaction<EvReelInitialized>> reactions;

public:
  // the angle of the current radial motion
  double yaw;

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx) : SmaccState<Navigate, NavigationOrthogonalLine>(ctx) {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    moveBaseClient_ =
        context<RadialMotionStateMachine>()
            .requiresActionClient<smacc::SmaccMoveBaseActionClient>(
                "move_base");
  }

  // when the reel substate is finished we will react starting the motion
  sc::result react(const EvReelInitialized &ev) {
    int i;
    if (!context<RadialMotionStateMachine>().getData("angle_index", i)) {
      // this is the first radial motion, initialize the index at -2 (-20 degrees)
      i = -2;
    } else {
      i += 1; // this is not the first time, increment the degress with 10 degreess
    }

    // read from the state machine i "global variable" to know the current orientation
    context<RadialMotionStateMachine>().setData("angle_index", i);
    ROS_INFO("Radial angle index: %d", i);

    // get the angle according to the angle index
    yaw = i * angles::from_degrees(10);

    rotateTenDegrees();
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void rotateTenDegrees() {
    geometry_msgs::PoseStamped radialStart;
    context<RadialMotionStateMachine>().getData("radial_start_pose",
                                                radialStart);

    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose = radialStart;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    moveBaseClient_->sendGoal(goal);
  }

  // this is the callback when the navigate action of this state is finished
  // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
  sc::result react(const EvActionResult &ev) {
    if (ev.client == moveBaseClient_) {
      if (ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Received event to movebase: %s",ev.getResult().toString().c_str());
        
        // notify the parent State to finish via event (the current parent state reacts to this event)
        post_event(EvStateFinished());
        
        // declare this substate as finished
        return discard_event();
        //return terminate();
      }
      else 
      {
        // repeat the navigate action request to the move base node if we get ABORT as response
        // It may work if try again. Move base sometime rejects the request because it is busy.
        rotateTenDegrees();

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
};

//------------------------------------------------------------------
// orthogonal line 1
struct ReelOrthogonalLine
    : SmaccState<ReelOrthogonalLine, RotateDegress::orthogonal<1>, ReelDispense> {
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  ReelOrthogonalLine(my_context ctx)
      : SmaccState<ReelOrthogonalLine, RotateDegress::orthogonal<1>, ReelDispense>(ctx) // call the SmaccState base constructor 
      {

      }
};

//------------------------------------------------------------------
// this is the reel substate inside the reel orthogonal line of the RotateDegreess State
struct ReelDispense : SmaccState<ReelDispense, ReelOrthogonalLine> {
  typedef boost::mpl::list< sc::custom_reaction< EvActionFeedback>,
                            sc::custom_reaction< EvActionResult >> reactions;
public:
  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  ReelDispense(my_context ctx) 
    : SmaccState<ReelDispense, ReelOrthogonalLine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering ReelDispense");

    // this substate will need access to the "Reel" resource or plugin. In this line
    // you get the reference to this resource.
    reelActionClient_ = context<RadialMotionStateMachine>()
                            .requiresActionClient<smacc::SmaccReelActionClient>(
                                "non_rt_helper");

    // use the reel resource to request dispense (request to the non_rt_helper)
    dispenseMotion();
  }

  // this is the "main method of this state" that calls to the action server
  // is is called form the constructor (at the entrance to this substate) and
  // also from the retry attempt (when we get some abort response)
  void dispenseMotion()
  {
    smacc::SmaccReelActionClient::Goal goal;
    goal.command = smacc::SmaccReelActionClient::Goal::DISPENSE;
    reelActionClient_->sendGoal(goal);
  }

  // subscribe to resource feedback event
  sc::result react( const EvActionFeedback &  ev)
  {
      // if the reel request is finished and success, then notify the event to the move base substate
      // and finish this substate
      if (ev.client == reelActionClient_) 
      {
        ROS_INFO("Received event for reel client");
        smacc::SmaccReelActionClient::Feedback feedback = reelActionClient_->getFeedbackMessage(ev);
        if(feedback.dispensing_state == smacc::SmaccReelActionClient::Goal::DISPENSE)
        {          
          ROS_INFO("Correct dispense mode. Let's notify others we are ready");
          // notify the navigate substate that we finished
          post_event(EvReelInitialized());

          // declare the reel substate as finished
          //return terminate();
          return forward_event();
        }
        else
        {
          ROS_INFO("Received feedback message from reel but, the dispensing state is not the expected. skipping");
          return forward_event();
        }
      }
      else
      {
          // the action client event success is not for this substate. Let others process this event.
          ROS_INFO("this feedback event is not for this resource");
          return forward_event();
      }
  }

  // subscribe to resource action result event
  sc::result react(const EvActionResult &ev) {
        ROS_INFO("Reel substate: Received event for reel client");
        if (ev.client == reelActionClient_)
        {
            if(ev.getResult() == actionlib::SimpleClientGoalState::ABORTED)
            {
                // retry again
                ROS_INFO("Retry reel retract");
                dispenseMotion();
                return discard_event();
            }
            else
            {
                ROS_INFO("Not handled result");
                return forward_event();
            }
        }
        else
        {
            // the action client event success is not for this substate. Let others process this event.
            ROS_INFO("navigate substate lets other process the EvActionResultEv");
            return forward_event();
        }
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  ~ReelDispense() 
  { 
    ROS_INFO("Exiting Reel_Action Client"); 
  }

private:
    // keeps the reference to the reel resorce or plugin (to connect to the non_rt_helper)
    smacc::SmaccReelActionClient *reelActionClient_;
};

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, RotateDegress::orthogonal<2>, ToolSubstate> {
public:
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, RotateDegress::orthogonal<2>, ToolSubstate>(ctx) // call the SmaccState base constructor                 
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
  }
};
}
