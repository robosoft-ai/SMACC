#pragma once

#include <smacc/smacc_orthogonal.h>

namespace sm_moveit
{
class OrPerception : public smacc::Orthogonal<OrPerception>
{
public:
    virtual void onInitialize() override
    {
        // auto gripperActionClient = this->createClient<ClGripper>("/gripper_controller/gripper_action/");
        // actionClient->initialize();
    }
};
} // namespace sm_moveit