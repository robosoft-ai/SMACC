#pragma once

#include <smacc/smacc.h>

class Superstate;
class NavigateToRadialStart;
class RotateDegress;
class NavigateToEndPoint;
class ReturnToRadialStart;

// ----- Radial Motion State Machine --------------

// create the RadialMotion State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct RadialMotionStateMachineReplacement
    : public smacc::SmaccStateMachineBase<RadialMotionStateMachineReplacement,Superstate> 
{
    RadialMotionStateMachineReplacement(my_context ctx, smacc::SignalDetector *signalDetector)
      : SmaccStateMachineBase<RadialMotionStateMachineReplacement,Superstate>(ctx, signalDetector) 
      {
      }      

};

