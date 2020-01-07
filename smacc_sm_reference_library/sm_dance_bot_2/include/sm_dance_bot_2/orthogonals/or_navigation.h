#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <odom_tracker/odom_tracker.h>
#include <waypoints_navigator/waypoints_navigator.h>

namespace sm_dance_bot_2
{
using namespace move_base_z_client;
using namespace move_base_z_client::odom_tracker;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        auto movebaseClient = this->createClient<ClMoveBaseZ>();
        movebaseClient->name_ = "move_base";
        movebaseClient->initialize();

        // create odom tracker
        movebaseClient->createComponent<OdomTracker>("/");

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
        if (nh.getParam("/sm_dance_bot_2/waypoints_plan", planfilepath))
        {
            waypointsNavigator->loadWayPointsFromFile(planfilepath);
        }
    }
};
} // namespace sm_dance_bot_2