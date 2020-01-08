#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include   <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include   <move_base_z_client_plugin/components/waypoints_navigator/waypoints_navigator.h>

namespace sm_dance_bot
{
using namespace move_base_z_client;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        auto node_namespace = "move_base";
        auto movebaseClient = this->createClient<ClMoveBaseZ>();
        movebaseClient->name_ = node_namespace;
        movebaseClient->initialize();

        // create planner switcher
        movebaseClient->createComponent<PlannerSwitcher>(node_namespace);    

        // create odom tracker
        movebaseClient->createComponent<OdomTracker>("/");       

        // create waypoints navigator component
        auto waypointsNavigator = movebaseClient->createComponent<WaypointNavigator>();
        loadWaypointsFromYaml(waypointsNavigator);
    }

    void loadWaypointsFromYaml(WaypointNavigator *waypointsNavigator)
    {
        // if it is the first time and the waypoints navigator is not configured

        std::string planfilepath;
        ros::NodeHandle nh("~");
        if (nh.getParam("/sm_dance_bot/waypoints_plan", planfilepath))
        {
            waypointsNavigator->loadWayPointsFromFile(planfilepath);
        }
    }
};
} // namespace sm_dance_bot