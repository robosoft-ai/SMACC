#pragma once

#include <smacc/smacc_state_machine_base.h>
#include <smacc_reelrbtx_plugin/reel.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_tool_plugin_template/smacc_tool_plugin.h>
#include <smacc/smacc_state.h>

using namespace smacc;

// this event is launched by any substate to make the "parent state" know that 
// it must finish and move to the next state. This kind of event may exist in any application:
// "some of the substates must throw this event so that the parent event is not alive forever"
struct EvStateFinished : sc::event<EvStateFinished> {};

// this event is launched by any substate to make the "parent state" know that 
// it must finish and move to the next state. This event is typically launched for
// this radial motion example by Reel substates. It is used to declare the "order dependency"
// between the reel and the navigation substates. In this example, the navigation substates happen
// always after the reel substates. Navigation substates notice that they have to start when they
// detect this event (that was launched by the reel substate at its end)
struct EvReelInitialized : sc::event<EvReelInitialized> {};

// ----- STATES FORWARD DECLARATIONS ---
namespace NavigateToRadialStart {
struct NavigateToRadialStart;
};

namespace RotateDegress {
struct RotateDegress;
}

namespace NavigateToEndPoint {
struct NavigateToEndPoint;
}

namespace ReturnToRadialStart {
struct ReturnToRadialStart;
}

// ----- Radial Motion State Machine --------------

// create the RadialMotion State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct RadialMotionStateMachine
    : public SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::NavigateToRadialStart> 
{
  RadialMotionStateMachine(my_context ctx, SignalDetector *signalDetector)
      : SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::NavigateToRadialStart>(ctx, signalDetector) {}
};
