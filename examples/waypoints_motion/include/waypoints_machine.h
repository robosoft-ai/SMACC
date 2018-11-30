#pragma once

#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_tool_plugin_template/smacc_tool_plugin.h>
#include <smacc_odom_tracker/odom_tracker.h>
#include <smacc_planner_switcher/planner_switcher.h>

using namespace smacc;

// ----- STATES FORWARD DECLARATIONS ---
namespace NavigateToOddWaypoint {
struct NavigateToOddWaypoint;
};

namespace NavigateToEvenWaypoint {
struct NavigateToOddWaypoint;
}

// ----- WaypointsStateMachine State Machine --------------

// create the WaypointsStateMachine State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct WaypointsStateMachine
    : public SmaccStateMachineBase<WaypointsStateMachine,NavigateToEvenWaypoint::NavigateToEvenWaypoint> 
{
  WaypointsStateMachine(my_context ctx, SignalDetector *signalDetector)
      : SmaccStateMachineBase<WaypointsStateMachine,NavigateToEvenWaypoint::NavigateToEvenWaypoint>(ctx, signalDetector) {}
};
