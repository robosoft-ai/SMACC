#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace sm_moveit_3
{
using namespace move_group_interface_client;
class OrArm : public smacc::Orthogonal<OrArm>
{
public:
    virtual void onInitialize() override
    {
        auto moveGroupClient = this->createClient<ClMoveGroup>("arm_with_torso");
        moveGroupClient->initialize();
    }
};
} // namespace sm_moveit_3