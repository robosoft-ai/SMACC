#pragma once

#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_tool_plugin_template/smacc_tool_plugin.h>

#include <ros/ros.h>
#include <smacc_odom_tracker/odom_tracker.h>
#include <smacc_planner_switcher/planner_switcher.h>

#include <substates/tool/tool_start.h>
#include <substates/tool/tool_stop.h>


using namespace smacc;

// ----- STATES FORWARD DECLARATIONS ---
namespace NavigateToRadialStart {
struct NavigateToRadialStart;
const char* ToolBehaviorKeyName = "NavigateToRadialStart_Tool";
};

namespace RotateDegress {
struct RotateDegress;
const char* ToolBehaviorKeyName = "RotateDegress_Tool";
}

namespace NavigateToEndPoint {
struct NavigateToEndPoint;
const char* ToolBehaviorKeyName = "NavigateToEndPoint_Tool";
}

namespace ReturnToRadialStart {
struct ReturnToRadialStart;
const char* ToolBehaviorKeyName = "ReturnToRadialStart_Tool";
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
          mapSmaccStateBehaviors();
      }      

    void mapSmaccStateBehaviors();
};
