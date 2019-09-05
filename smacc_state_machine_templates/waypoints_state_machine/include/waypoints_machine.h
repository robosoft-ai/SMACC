#pragma once

#include <smacc_core/smacc.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_interface_components/smacc_tool_plugin_template/smacc_tool_plugin.h>
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
    loadWayPointsFromParameterServer();

    // we will use this global index variable to iterate on waypoints
    int currentWayPointIndex=0;
    this->setGlobalSMData("waypoint_index", currentWayPointIndex);
}

 void loadWayPointsFromParameterServer()
 {
    XmlRpc::XmlRpcValue waypointsList;
    this->getParam("waypoints", waypointsList);
    ROS_ASSERT(waypointsList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    
    auto waypoints = std::make_shared<std::vector<geometry_msgs::Point>>();
    for (int32_t i = 0; i < waypointsList.size(); ++i) 
    {
        ROS_ASSERT(waypointsList[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(waypointsList[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        
        double x = static_cast<double>(waypointsList[i][0]);
        double y = static_cast<double>(waypointsList[i][1]);
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;

        //sum += static_cast<double>(my_list[i]);
        ROS_INFO("waypoint %d, (%lf, %lf)", i, x, y);
        waypoints->push_back(p);
    }

    this->setGlobalSMData("waypoints", waypoints );
 }
};