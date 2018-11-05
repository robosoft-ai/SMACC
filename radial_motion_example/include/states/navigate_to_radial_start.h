#pragma once

#include <radial_motion.h>

namespace NavigateToRadialStart 
{
using namespace smacc;

//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ReelOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ReelStartAndDispense;
struct ToolSubstate;

//--------------------------------------------
/// State NavigateToRadialStart
struct NavigateToRadialStart
    : SmaccState<NavigateToRadialStart, RadialMotionStateMachine,
                 mpl::list<NavigationOrthogonalLine, ReelOrthogonalLine, ToolOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  // when this state is finished then move to the RotateDegress state
  typedef sc::transition<EvStateFinished, RotateDegress::RotateDegress>
      reactions;

public:
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  NavigateToRadialStart(my_context ctx)
      : SmaccState<NavigateToRadialStart, RadialMotionStateMachine,
                   mpl::list<NavigationOrthogonalLine, ReelOrthogonalLine, ToolOrthogonalLine>>(ctx) 
    {
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
  typedef mpl::list<sc::custom_reaction<EvActionResult>,
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
    moveBaseClient_ =
        context<RadialMotionStateMachine>()
            .requiresActionClient<smacc::SmaccMoveBaseActionClient>(
                "move_base");
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void goToRadialStart() {
    smacc::SmaccMoveBaseActionClient::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 3;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.pose.orientation.w = 1;
    context<RadialMotionStateMachine>().setData("radial_start_pose",
                                                goal.target_pose);

    moveBaseClient_->sendGoal(goal);
  }

  // when the reel substate is finished we will react starting the motion
  sc::result react(const EvReelInitialized &ev) 
  { 
    goToRadialStart(); 
  }

  // this is the callback when the navigate action of this state is finished
  // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
  sc::result react(const EvActionResult &ev) {

    if (ev.client == moveBaseClient_) {
      if (ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED) {
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

//--------------------------------------------------
// orthogonal line 1
struct ReelOrthogonalLine
    : SmaccState<ReelOrthogonalLine, NavigateToRadialStart::orthogonal<1>,
                 ReelStartAndDispense> {
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  ReelOrthogonalLine(my_context ctx)
      : SmaccState<ReelOrthogonalLine, NavigateToRadialStart::orthogonal<1>,
                   ReelStartAndDispense>(ctx) // call the SmaccState base constructor                 
  {
    ROS_INFO("Entering in reel orthogonal line");
  }

  ~ReelOrthogonalLine() 
  { 
    ROS_INFO("Finishing reel orthogonal line"); 
  }
};

//--------------------------------------------------
// this is the reel substate inside the reel orthogonal line of the NavigateToRadialStart State
struct ReelStartAndDispense
    : SmaccState<ReelStartAndDispense, ReelOrthogonalLine> {
  
  typedef boost::mpl::list< sc::custom_reaction< EvActionFeedback>,
                            sc::custom_reaction< EvActionResult >> reactions;

public:

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  ReelStartAndDispense(my_context ctx) 
    : SmaccState<ReelStartAndDispense, ReelOrthogonalLine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering ReelStartAndDispense");

    // this substate will need access to the "Reel" resource or plugin. In this line
    // you get the reference to this resource.
    reelActionClient_ = context<RadialMotionStateMachine>()
                            .requiresActionClient<smacc::SmaccReelActionClient>(
                                "non_rt_helper");

    // make the motion
    dispenseMotion();
  }

  // this is the "main method of this state" that calls to the action server
  // is is called form the constructor (at the entrance to this substate) and
  // also from the retry attempt (when we get some abort response)
  void dispenseMotion()
  {
    // use the reel resource to request dispense (request to the non_rt_helper)
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

          // keep this substate alive
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
              // and consume 
              return discard_event();
          }
          else
          {
              ROS_INFO("Not handled result. Finishing reel substate.");
              return terminate();
          }
      }
      else
      {
          // the action client event success is not for this substate. Let others process this event.
          ROS_INFO("this EvActionResultEv is not for the reel.");
          return forward_event();
      }
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  ~ReelStartAndDispense() 
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
    : SmaccState<ToolOrthogonalLine, NavigateToRadialStart::orthogonal<2>,
                 ToolSubstate> {
public:
  ToolOrthogonalLine(my_context ctx)
      : SmaccState<ToolOrthogonalLine, NavigateToRadialStart::orthogonal<2>,
                   ToolSubstate>(ctx) // call the SmaccState base constructor                 
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
