#pragma once

#include <smacc/smacc.h>
#include <sm_moveit_4/clients/gripper_client/cl_gripper.h>

namespace sm_moveit_4
{
namespace cl_gripper
{
class CbOpenGripper : public smacc::SmaccClientBehavior
{
public:
    virtual void onEntry() override
    {
        ClGripper *gripper;
        this->requiresClient(gripper);

        control_msgs::GripperCommandGoal gripperGoal;
        gripperGoal.command.position = 100;
        gripperGoal.command.max_effort = 100;
        gripper->sendGoal(gripperGoal);
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_gripper
} // namespace sm_moveit_4
