#pragma once
#include "radial_motion.h"
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace NavigateToEndPoint 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ReelOrthogonalLine;
struct Navigate;

//--------------------------------------------
/// NavigateToEndPoint State
struct NavigateToEndPoint
    : SmaccState<NavigateToEndPoint, RadialMotionStateMachine,
                 mpl::list<NavigationOrthogonalLine, ReelOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  // when this state is finished move to the ReturnToRadialStart state
  typedef sc::transition<EvStateFinished, ReturnToRadialStart::ReturnToRadialStart> reactions;

public:
  // This is the state constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  // after this, its orthogonal lines are created (see orthogonal line classes).
  NavigateToEndPoint(my_context ctx)
      : SmaccState<NavigateToEndPoint, RadialMotionStateMachine, 
                    mpl::list<NavigationOrthogonalLine, ReelOrthogonalLine>>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Initializating Navigate to endpoint state");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  ~NavigateToEndPoint() {}
};

//------------------------------------------------------------------------------
// orthogonal line 0
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, NavigateToEndPoint::orthogonal<0>,
                 Navigate> {
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  NavigationOrthogonalLine(my_context ctx)
      : SmaccState<NavigationOrthogonalLine, NavigateToEndPoint::orthogonal<0>,Navigate>(ctx) // call the SmaccState base constructor
  {
  }
};
//------------------------------------------------------------------
// this is the navigate substate inside the navigation orthogonal line of the RotateDegreess State
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> {
  typedef mpl::list<sc::custom_reaction<EvActionClientSuccess>> reactions;

public:
  double yaw;
  double dist;

  // This is the substate constructor. This code will be executed when the
  // workflow enters in this substate (that is according to statechart the moment when this object is created)
  Navigate(my_context ctx)
      : SmaccState<Navigate, NavigationOrthogonalLine>(ctx) // call the SmaccState base constructor
  {
    ROS_INFO("Entering Navigate");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    moveBaseClient_ =
        context<RadialMotionStateMachine>().requiresActionClient<smacc::SmaccMoveBaseActionClient>("move_base");

    // read from the state machine i "global variable" to know the current orientation
    int i;
    context<RadialMotionStateMachine>().getData("angle_index", i);
    yaw = i * angles::from_degrees(10);
    dist = 3.5;

    goToEndPoint();
  }

  // auxiliar function that defines the motion that is requested to the move_base action server
  void goToEndPoint() {
    geometry_msgs::PoseStamped radialStartPose;
    context<RadialMotionStateMachine>().getData("radial_start_pose",
                                                radialStartPose);

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
  }

  // this is the callback when the navigate action of this state is finished
  // if it succeeded we will notify to the parent State to finish sending a EvStateFinishedEvent
  sc::result react(const EvActionClientSuccess &ev) {

    if (ev.client == moveBaseClient_) {
      if (ev.getResult() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      {
        ROS_INFO("Received event to movebase: %s",ev.getResult().toString().c_str());

        // notify the parent State to finish via event (the current parent state reacts to this event)
        post_event(EvStateFinished());
        
        // declare this substate as finished
        return terminate();
      } 
      else if (ev.getResult() == actionlib::SimpleClientGoalState::ABORTED) 
      {
        // repeat the navigate action request to the move base node if we get ABORT as response
        // It may work if try again. Move base sometime rejects the request because it is busy.
        goToEndPoint();

        // this event was for us. We have used it without moving to any other state. Do not let others consume it.
        return discard_event();
      }
    } 
    else 
    {
      // the action client event success is not for this substate. Let others process this event.
      ROS_INFO("navigate substate lets other process the EvActionClientSuccessEv");
      return forward_event();
    }
  }

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  ~Navigate() { ROS_INFO("Exiting move goal Action Client"); }

private:
  // keeps the reference to the move_base resorce or plugin (to connect to the move_base action server). 
  // this resource can be used from any method in this state
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;
};

//------------------------------------------------------------------------------
// orthogonal line 1
struct ReelOrthogonalLine
    : SmaccState<ReelOrthogonalLine, NavigateToEndPoint::orthogonal<1>> {
public:
  // This is the orthogonal line constructor. This code will be executed when the
  // workflow enters in this orthogonal line (that is according to statechart the moment when this object is created)
  ReelOrthogonalLine(my_context ctx)
      : SmaccState<ReelOrthogonalLine, NavigateToEndPoint::orthogonal<1>>(ctx) // call the SmaccState base constructor 
      {

      }
};
}
