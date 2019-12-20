#pragma once

#include <timer_state_machine.h>
#include <orthogonals/or_navigation.h>
#include <client_behaviors/navigation/cb_timer.h>

//--------------------------------------------
struct TimerState: smacc::SmaccState<TimerState, TimerStateMachine>
{
  //typedef smacc::transition<smacc::Timer::TickEvent, TimerState> reactions; 

public:
  using SmaccState::SmaccState;

  void onInitialize()
  {
    this->configure<OrNavigation, smacc::Timer>(ros::Duration(3));
  }

  void onEntry()
  {
    ROS_INFO("timer subatate, time: %lf", ros::Time::now().toSec() );
  }

  void onExit()
  {
  }
};
