#pragma once

#include <smacc/smacc.h>
#include <moveit_z_client/components/cp_grasping_objects.h>
#include <moveit_z_client/cl_movegroup.h>

namespace moveit_z_client
{
    class CbDetachObject : public smacc::SmaccClientBehavior
    {
    public:
        virtual void onEntry() override;

        virtual void onExit() override;
    };
} // namespace moveit_z_client
