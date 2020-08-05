#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sm_moveit_screw_loop/clients/gripper_client/cl_gripper.h>

namespace sm_moveit_screw_loop
{
using namespace cl_gripper;
class OrGripper : public smacc::Orthogonal<OrGripper>
{
public:
    virtual void onInitialize() override
    {
        auto gripperActionClient = this->createClient<ClGripper>("/gripper_controller/gripper_action/");
        gripperActionClient->initialize();
    }
};
} // namespace sm_moveit_screw_loop