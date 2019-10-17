#pragma once

#include <timer_state_machine.h>
#include <orthogonals/navigation_orthogonal.h>
#include <substates_behaviors/navigation/timer_substate.h>

//--------------------------------------------
struct TimerState: smacc::SmaccState<TimerState, TimerStateMachine>
{
  //typedef smacc::transition<smacc::Timer::TickEvent, TimerState> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<NavigationOrthogonal>(std::make_shared<smacc::Timer>(ros::Duration(3)));
  }

  void onEntry()
  {
    ROS_INFO("timer subatate, time: %lf", ros::Time::now().toSec() );
  }

  void onExit()
  {
  }
};
