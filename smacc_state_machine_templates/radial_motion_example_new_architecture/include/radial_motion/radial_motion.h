#pragma once

#include <smacc/smacc.h>
#include <radial_motion/orthogonals/navigation_orthogonal.h>
#include <radial_motion/orthogonals/tool_orthogonal.h>

#include <radial_motion/state_names.h>
#include <radial_motion/substate_behaviors.h>


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

#include <radial_motion/states.h>

