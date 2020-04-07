#pragma once

#include <smacc/smacc_orthogonal.h>
#include <sm_moveit/clients/movegroup_client/cl_movegroup.h>

namespace sm_moveit
{
using namespace cl_movegroup;
class OrArm : public smacc::Orthogonal<OrArm>
{
public:
    virtual void onInitialize() override
    {
        auto moveGroupClient = this->createClient<ClMoveGroup>("arm_with_torso");
        moveGroupClient->initialize();
    }
};
} // namespace sm_moveit