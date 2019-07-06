#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace NavigateToEndPoint 
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// NavigateToEndPoint State
struct NavigateToEndPoint
    : SmaccState<NavigateToEndPoint, RadialMotionStateMachine,
                 mpl::list<NavigationOrthogonalLine,ToolOrthogonalLine>> // <- these are the orthogonal lines of this State
{
  // when this state is finished move to the ReturnToRadialStart state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, ReturnToRadialStart::ReturnToRadialStart> reactions; 

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
    : SmaccState<NavigationOrthogonalLine, NavigateToEndPoint::orthogonal<0>,
                 Navigate> {
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
  }
};
//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToEndPoint::orthogonal<1>, ToolSubstate> {
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

}
