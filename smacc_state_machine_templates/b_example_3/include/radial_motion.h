#pragma once

#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_tool_plugin_template/smacc_tool_plugin.h>

#include <ros/ros.h>
#include <smacc_odom_tracker/odom_tracker.h>
#include <smacc_planner_switcher/planner_switcher.h>

using namespace smacc;

// ----- STATES FORWARD DECLARATIONS ---
namespace NavigateToRadialStart {
struct NavigateToRadialStart;
struct ToolSubstate;
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
      : SmaccStateMachineBase<RadialMotionStateMachine,NavigateToRadialStart::NavigateToRadialStart>(ctx, signalDetector) 
      {
      }      
};
