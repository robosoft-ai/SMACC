#pragma once

#include <smacc/smacc_orthogonal.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace sm_fetch_two_table_pick_n_place_1
{
using namespace cl_move_group_interface;
class OrArm : public smacc::Orthogonal<OrArm>
{
public:
    virtual void onInitialize() override
    {
        auto moveGroupClient = this->createClient<ClMoveGroup>("arm_with_torso");
        moveGroupClient->initialize();
    }
};
} // namespace sm_fetch_two_table_pick_n_place_1