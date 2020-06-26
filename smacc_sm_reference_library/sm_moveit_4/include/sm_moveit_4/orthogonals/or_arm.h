#pragma once

#include <smacc/smacc_orthogonal.h>
#include <moveit_z_client/cl_movegroup.h>

namespace sm_moveit_4
{
using namespace moveit_z_client;
class OrArm : public smacc::Orthogonal<OrArm>
{
public:
    virtual void onInitialize() override
    {
        auto moveGroupClient = this->createClient<ClMoveGroup>("arm_with_torso");
        moveGroupClient->initialize();
    }
};
} // namespace sm_moveit_4