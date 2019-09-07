#pragma once

class RadialMotionSuperState;
// ----- Radial Motion State Machine --------------

// create the RadialMotion State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct RadialMotionStateMachine
    : public smacc::SmaccStateMachineBase<RadialMotionStateMachine,RadialMotionSuperState> 
{
    RadialMotionStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<RadialMotionStateMachine,RadialMotionSuperState>(ctx, signalDetector) 
      {
      }      

};
