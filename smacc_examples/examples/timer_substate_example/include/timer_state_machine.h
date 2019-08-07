#pragma once

#include <smacc/smacc.h>
class TimerState;

// ----- Radial Motion State Machine --------------

// create the RadialMotion State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct TimerStateMachine
    : public smacc::SmaccStateMachineBase<TimerStateMachine,TimerState> 
{
    TimerStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<TimerStateMachine,TimerState>(ctx, signalDetector) 
      {
      }      

};

#include <states/timer_state.h>

