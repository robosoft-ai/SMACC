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

        // create odom tracker
        movebaseClient->createComponent<odom_tracker::OdomTracker>("/");
        
        // create waypoints navigator component
        auto waypointsNavigator_ = movebaseClient->createComponent<smacc::WaypointNavigator>();
        waypointsNavigator_->configureEventSourceTypes<smacc::ClMoveBaseZ, OrNavigation>();
    }
};
} // namespace sm_dance_bot