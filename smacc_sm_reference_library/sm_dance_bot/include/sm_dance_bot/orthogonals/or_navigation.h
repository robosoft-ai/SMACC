#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <odom_tracker/odom_tracker.h>
#include <waypoints_navigator/waypoints_navigator.h>

namespace sm_dance_bot
{
class OrNavigation : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto movebaseClient = this->createClient<OrNavigation, smacc::ClMoveBaseZ>();
        movebaseClient->name_ = "move_base";
        movebaseClient->initialize();

        auto odomtracker = movebaseClient->createComponent<odom_tracker::OdomTracker>();
        odomtracker->initialize("/");

        auto waypointsNavigator_ = movebaseClient->createComponent<smacc::WaypointNavigator>();
        waypointsNavigator_->configureEventSourceTypes<OrNavigation, smacc::ClMoveBaseZ>();
    }
};
} // namespace sm_dance_bot