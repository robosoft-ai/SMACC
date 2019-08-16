#pragma once

#include <orthogonals/navigation_orthogonal.h>
#include <orthogonals/tool_orthogonal.h>
#include <substates_behaviors/navigation/undo_path_backwards.h>
#include <substates_behaviors/tool/tool_stop.h>

//--------------------------------------------
/// ReturnToRadialStart State
struct ReturnToRadialStart : SmaccState< ReturnToRadialStart, Superstate > // <- these are the orthogonal lines of this State
{
    // when this state is finished then move to the RotateDegress state
    typedef sc::transition<EvActionSucceded<smacc::SmaccMoveBaseActionClient::Result>, RotateDegress> reactions; 

public:
  using SmaccState::SmaccState;

   void onInitialize()
   {
      this->configure<NavigationOrthogonal>(new UndoPathBackwards());
      this->configure<ToolOrthogonal>(new ToolStop());
   }

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