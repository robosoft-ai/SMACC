#pragma once

#include <smacc/smacc.h>
#include <moveit_z_client/cl_movegroup.h>

namespace moveit_z_client
{
    class CbAttachObject : public smacc::SmaccClientBehavior
    {
    public:
        CbAttachObject(std::string targetObjectName);

        CbAttachObject();

        virtual void onEntry() override;

        virtual void onExit() override;

        std::string targetObjectName_;

    private:
    };
} // namespace moveit_z_client
