/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <radial_motion.h>

#include <orthogonals/navigation_orthogonal.h>
#include <orthogonals/tool_orthogonal.h>
#include <substates_behaviors/navigation/rotate.h>
#include <substates_behaviors/tool/tool_stop.h>


/// State NavigateToRadialStart
struct RotateDegress: SmaccState<RotateDegress, RadialMotionStateMachine> // <- these are the orthogonal lines of this State 
{
  // when this state is finished then move to the NavigateToEndPoint state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>,  NavigateToEndPoint> reactions; 

public:

  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new Rotate(10));
    this->configure<ToolOrthogonal>(new ToolStop());
  }

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
