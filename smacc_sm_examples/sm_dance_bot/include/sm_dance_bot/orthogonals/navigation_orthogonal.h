#pragma once

#include <smacc/orthogonal.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_odom_tracker/odom_tracker.h>

class NavigationOrthogonal : public smacc::Orthogonal
{
    public:
    virtual void onInitialize() override
    {
        auto* client = this->createClient<smacc::SmaccMoveBaseActionClient>();       
        client->name_ = "move_base";
        client->initialize();

        smacc_odom_tracker::OdomTracker* odomtracker;
        this->requiresComponent(odomtracker);
        odomtracker->initialize("/");
    }
};