#pragma once

#include <smacc/smacc.h>

class NavigateToRadialStart;
class RotateDegress;
class NavigateToEndPoint;
class ReturnToRadialStart;

// ----- Radial Motion State Machine --------------

// create the RadialMotion State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct RadialMotionStateMachine
    : public smacc::SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart> 
{
    RadialMotionStateMachine(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart>(ctx, signalDetector) 
      {
      }      

};

