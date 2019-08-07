#pragma once

#include <smacc/smacc.h>
class TimerState;

struct TimerStateMachine
    : public smacc::SmaccStateMachineBase<TimerStateMachine,TimerState> 
{
    TimerStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<TimerStateMachine,TimerState>(ctx, signalDetector) 
      {
      }      

};

#include <states/timer_state.h>

