#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace sm_fetch_two_table_whiskey_pour
{
using namespace cl_move_base_z;
using namespace cl_move_base_z::odom_tracker;

class OrNavigation : public smacc::Orthogonal<OrNavigation>
{
public:
    virtual void onInitialize() override
    {
        auto movebaseClient = this->createClient<ClMoveBaseZ>();
        movebaseClient->initialize();

        movebaseClient->createComponent<Pose>("base_link", "map");

        // create planner switcher
        movebaseClient->createComponent<PlannerSwitcher>();

        // create odom tracker
        movebaseClient->createComponent<OdomTracker>();
    }
};
} // namespace sm_fetch_two_table_whiskey_pour