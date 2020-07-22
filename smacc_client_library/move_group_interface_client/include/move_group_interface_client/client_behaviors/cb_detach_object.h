#pragma once

#include <smacc/smacc.h>
#include <move_group_interface_client/components/cp_grasping_objects.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace move_group_interface_client
{
    class CbDetachObject : public smacc::SmaccClientBehavior
    {
    public:
        virtual void onEntry() override;

        virtual void onExit() override;
    };
} // namespace move_group_interface_client
