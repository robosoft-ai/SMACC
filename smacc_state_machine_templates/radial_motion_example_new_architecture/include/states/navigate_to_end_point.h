#pragma once

#include <radial_motion.h>
#include <orthogonals/navigation_orthogonal.h>
#include <orthogonals/tool_orthogonal.h>

#include <substates_behaviors/navigation/navigate_forward.h>
#include <substates_behaviors/tool/tool_start.h>

//--------------------------------------------
/// NavigateToEndPoint State
struct NavigateToEndPoint: SmaccState<NavigateToEndPoint, RadialMotionStateMachine>
{
  // when this state is finished move to the ReturnToRadialStart state
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, ReturnToRadialStart> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(new NavigateForward(3));
    this->configure<ToolOrthogonal>(new ToolStart());
  }

  void onEntry()
  {
  }

  void onExit()
  {
  }
};
