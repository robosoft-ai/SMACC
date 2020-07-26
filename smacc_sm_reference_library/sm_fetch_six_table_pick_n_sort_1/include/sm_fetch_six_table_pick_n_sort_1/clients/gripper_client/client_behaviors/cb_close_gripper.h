#pragma once

#include <smacc/smacc.h>
#include <sm_fetch_six_table_pick_n_sort_1/clients/gripper_client/cl_gripper.h>
#include <control_msgs/GripperCommandActionGoal.h>

namespace sm_fetch_six_table_pick_n_sort_1
{
namespace cl_gripper
{
class CbCloseGripper : public smacc::SmaccClientBehavior
{
public:
    virtual void onEntry() override
    {
        ClGripper *gripper;
        this->requiresClient(gripper);

        control_msgs::GripperCommandGoal gripperGoal;
        gripperGoal.command.position = 0;
        gripperGoal.command.max_effort = 100;
        gripper->sendGoal(gripperGoal);
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_gripper
} // namespace sm_fetch_six_table_pick_n_sort_1
