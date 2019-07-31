#pragma once

#include <radial_motion.h>

#include <orthogonals/navigation_orthogonal.h>
#include <orthogonals/tool_orthogonal.h>
#include <substates_behaviors/navigation/navigate_global_position.h>
#include <substates_behaviors/tool/tool_start.h>


using namespace smacc;

/// State NavigateToRadialStart
struct NavigateToRadialStart: smacc::SmaccState<NavigateToRadialStart, RadialMotionStateMachine> // <- these are the orthogonal lines of this State
{
  // when this state is finished then move to the RotateDegress state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, RotateDegress> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
     ROS_INFO("ON INITIALIZEEEE");
     this->configure<NavigationOrthogonal>(new NavigateGlobalPosition(1, 0));
     this->configure<ToolOrthogonal>(new ToolStart());
  }

  void onEntry()
  {
    ROS_INFO("-------");
    ROS_INFO("Entering in NavigateToRadialStart State");
  }

  void onExit() 
  {
    ROS_INFO("Finishing NavigateToRadialStart state");
  }
};
