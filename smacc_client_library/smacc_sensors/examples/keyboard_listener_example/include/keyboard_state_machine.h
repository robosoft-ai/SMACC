#pragma once

#include <smacc/smacc.h>

struct KeyboardState;

struct KeyboardStateMachine
    : public smacc::SmaccStateMachineBase<KeyboardStateMachine,KeyboardState> 
{
    KeyboardStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<KeyboardStateMachine,KeyboardState>(ctx, signalDetector) 
      {
      }      

};

#include <states/keyboard_state.h>


