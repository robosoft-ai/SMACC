/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <thread>

//--------------------------------------------
namespace RotateDegress 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;
struct Navigate;
struct ToolSubstate;

/// State NavigateToRadialStart
struct RotateDegress
    : SmaccState<RotateDegress, RadialMotionStateMachine, mpl::list<NavigationOrthogonalLine, ToolOrthogonalLine>> // <- these are the orthogonal lines of this State 
{
  // when this state is finished then move to the NavigateToEndPoint state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  NavigateToEndPoint::NavigateToEndPoint> reactions; 

public:

  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("-------");
    ROS_INFO("Entering in ROTATE TEN DEGREES STATE");
  }

  // This is the state destructor. This code will be executed when the
  // workflow exits from this state (that is according to statechart the moment when this object is destroyed)
  void onExit()
  { 
    ROS_INFO("Exiting in ROTATE TEN DEGREES STATE"); 
  }
};

//------------------------------------------------------------------
// orthogonal line 0Reel_ActionClient
struct NavigationOrthogonalLine
    : SmaccState<NavigationOrthogonalLine, RotateDegress::orthogonal<0>,
                 Navigate> 
{
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
  }
};
//--------------------------------------------
struct Navigate : SmaccState<Navigate, NavigationOrthogonalLine> 
{
public:
    SMACC_STATE_BEHAVIOR
    using SmaccState::SmaccState;

};
//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, RotateDegress::orthogonal<1>, ToolSubstate> {
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
    SMACC_STATE_BEHAVIOR  
    using SmaccState::SmaccState;
};

}
