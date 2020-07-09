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

            // moveGroupClient->moveGroupClientInterface.setWorkspace(-0., 2.5, -0.45, 0.45, -0.35, 1.75);
        }
    };
} // namespace sm_moveit_4