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
  typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, RotateDegress::RotateDegress> reactions; 

public:
    using SmaccState::SmaccState;

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

//--------------------------------------------------
// orthogonal line 0
struct NavigationOrthogonalLine
    : public SmaccState<NavigationOrthogonalLine,
                        NavigateToRadialStart::orthogonal<0>, Navigate> 
  {
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering in move_base orthogonal line");
  }

  void onExit()
  {
    ROS_INFO("Finishing move base orthogonal line");
  }
};


//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, NavigateToRadialStart::orthogonal<1>, ToolSubstate> {
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
//---------------------------------------------------------------------------------------------------------

}
