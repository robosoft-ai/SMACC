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
struct NavigateToEvenWaypoint;
}

// ----- WayPointsStateMachine State Machine --------------

// create the WayPointsStateMachine State Machine example class that inherits from the 
// SmaccStateMachineBase. You only have to declare it, the most of the funcionality is inhterited.
struct WayPointsStateMachine
    : public SmaccStateMachineBase<WayPointsStateMachine,NavigateToEvenWaypoint::NavigateToEvenWaypoint> 
{
public:
WayPointsStateMachine(my_context ctx, SignalDetector *signalDetector)
    : SmaccStateMachineBase<WayPointsStateMachine,NavigateToEvenWaypoint::NavigateToEvenWaypoint>(ctx, signalDetector) 
    {
        XmlRpc::XmlRpcValue waypointsList;
        this->getParam("waypoints", waypointsList);
        ROS_ASSERT(waypointsList.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int32_t i = 0; i < waypointsList.size(); ++i) 
        {
            ROS_ASSERT(waypointsList[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            //sum += static_cast<double>(my_list[i]);
        }
    }
};
