#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>

namespace ReturnToRadialStart
{
//forward declarations of subcomponents of this state
struct NavigationOrthogonalLine;
struct ToolOrthogonalLine;

struct Navigate;
struct ToolSubstate;

//--------------------------------------------
/// ReturnToRadialStart State
struct ReturnToRadialStart : SmaccState< ReturnToRadialStart, RadialMotionStateMachine,
                               mpl::list< NavigationOrthogonalLine, ToolOrthogonalLine > > // <- these are the orthogonal lines of this State
{
    // when this state is finished then move to the RotateDegress state
    typedef sc::transition<EvActionResult<smacc::SmaccMoveBaseActionClient::Result>, RotateDegress::RotateDegress> reactions; 

public:
    using SmaccState::SmaccState;

    void onEntry()
    {
        ROS_INFO("-------");
        ROS_INFO("Entering State: ReturnToRadialStart");
    }
    
    void onExit()
    {
        ROS_INFO("Exiting State: ReturnToRadialStart");
    }
};

//--------------------------------------------
struct NavigationOrthogonalLine: SmaccState<NavigationOrthogonalLine, ReturnToRadialStart::orthogonal< 0 > , Navigate>
{
public:
    using SmaccState::SmaccState;

    void onEntry()
    {
    }
};

//--------------------------------------------

//---------------------------------------------------------------------------------------------------------
// orthogonal line 2
struct ToolOrthogonalLine
    : SmaccState<ToolOrthogonalLine, ReturnToRadialStart::orthogonal<1>, ToolSubstate> {
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering in the tool orthogonal line");
  }

  ~ToolOrthogonalLine() 
  { 
    ROS_INFO("Finishing the tool orthogonal line"); 
  }
};
}