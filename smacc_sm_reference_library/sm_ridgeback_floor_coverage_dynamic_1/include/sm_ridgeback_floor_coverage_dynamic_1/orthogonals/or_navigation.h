#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_navigator.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace sm_ridgeback_floor_coverage_dynamic_1
{
using namespace cl_move_base_z;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        auto movebaseClient = this->createClient<ClMoveBaseZ>();
        movebaseClient->initialize();

        movebaseClient->createComponent<Pose>();

        // create planner switcher
        movebaseClient->createComponent<PlannerSwitcher>();

        // create odom tracker
        movebaseClient->createComponent<cl_move_base_z::odom_tracker::OdomTracker>();

        // create waypoints navigator component
        auto waypointsNavigator = movebaseClient->createComponent<WaypointNavigator>();
        loadWaypointsFromYaml(waypointsNavigator);

        // change this to skip some points of the yaml file, default = 0
        waypointsNavigator->currentWaypoint_ = 0;
    }

    void loadWaypointsFromYaml(WaypointNavigator *waypointsNavigator)
    {
        // if it is the first time and the waypoints navigator is not configured

        std::string planfilepath;
        ros::NodeHandle nh("~");
        if (nh.getParam("/sm_ridgeback_floor_coverage_dynamic_1/waypoints_plan", planfilepath))
        {
            waypointsNavigator->loadWayPointsFromFile(planfilepath);
        }
    }
};
} // namespace sm_ridgeback_floor_coverage_dynamic_1