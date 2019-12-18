#pragma once

#include <smacc/smacc_orthogonal.h>
#include <smacc_navigation_plugin/move_base_action_client.h>
#include <smacc_odom_tracker/odom_tracker.h>

namespace sm_dance_bot
{
class NavigationOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        //what we had
        //auto client = this->createClient<smacc::SmaccMoveBaseActionClient>(23,24);

        auto movebaseClient = this->createClient<NavigationOrthogonal, smacc::SmaccMoveBaseActionClient>();
        //auto client = createClient<smacc::SmaccMoveBaseActionClient>(this, 23, 234);
        //auto client = CREATE_CLIENT(smacc::SmaccMoveBaseActionClient, 23, 234);

        movebaseClient->name_ = "move_base";
        movebaseClient->initialize();

        smacc_odom_tracker::OdomTracker *odomtracker;
        this->requiresComponent(odomtracker);
        odomtracker->initialize("/");

        // auto odomtracker = movebaseClient->createComponent<OdomTracker>();
        // odomtracker->initialize("/");
    }
};
} // namespace sm_dance_bot