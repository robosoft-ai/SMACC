#pragma once

#include <smacc/smacc.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace move_group_interface_client
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
} // namespace move_group_interface_client
