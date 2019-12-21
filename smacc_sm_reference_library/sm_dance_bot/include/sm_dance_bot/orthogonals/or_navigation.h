#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <odom_tracker/odom_tracker.h>

namespace sm_dance_bot
{
class OrNavigation : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        //what we had
        //auto client = this->createClient<smacc::ClMoveBaseZ>(23,24);

        auto movebaseClient = this->createClient<OrNavigation, smacc::ClMoveBaseZ>();
        //auto client = createClient<smacc::ClMoveBaseZ>(this, 23, 234);
        //auto client = CREATE_CLIENT(smacc::ClMoveBaseZ, 23, 234);

        movebaseClient->name_ = "move_base";
        movebaseClient->initialize();

        odom_tracker::OdomTracker *odomtracker;
        this->requiresComponent(odomtracker);
        odomtracker->initialize("/");

        // auto odomtracker = movebaseClient->createComponent<OdomTracker>();
        // odomtracker->initialize("/");
    }
};
} // namespace sm_dance_bot