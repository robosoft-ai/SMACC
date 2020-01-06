#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <odom_tracker/odom_tracker.h>
//#include <waypoints_navigator/waypoints_navigator.h>

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
    }
};
} // namespace sm_dance_bot