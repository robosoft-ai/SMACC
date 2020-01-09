#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include   <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include   <move_base_z_client_plugin/components/waypoints_navigator/waypoints_navigator.h>

namespace sm_dance_bot_2
{
using namespace move_base_z_client;
using namespace move_base_z_client::odom_tracker;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        std::string node_namespace = "move_base";
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
        std::string filePathParam = "/sm_dance_bot_2/waypoints_plan";
        if (nh.getParam(filePathParam, planfilepath))
        {
            ROS_INFO("Loading waypoints from file %s", planfilepath.c_str());
            waypointsNavigator->loadWayPointsFromFile(planfilepath);
        }
        else
        {
            ROS_ERROR("waypoints file is not properly set in the parameter '%s' ", filePathParam.c_str());
        }
    }
};
} // namespace sm_dance_bot_2