#pragma once

#include <smacc/smacc.h>
#include <sm_moveit/clients/gripper_client/cl_gripper.h>

namespace sm_moveit
{
namespace cl_gripper
{
class CbCloseGripper : public smacc::SmaccClientBehavior
{
public:
    virtual void onEntry() override
    {
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_gripper
} // namespace sm_moveit
