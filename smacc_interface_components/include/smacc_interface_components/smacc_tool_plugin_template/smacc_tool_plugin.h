#pragma once

#include <smacc/smacc_action_client_base.h>
#include <smacc_interface_components/ToolControlAction.h>

namespace smacc
{
class SmaccToolActionClient: public SmaccActionClientBase<smacc_interface_components::ToolControlAction>
{
    typedef SmaccActionClientBase<smacc_interface_components::ToolControlAction> Base;

    public:
        SmaccToolActionClient();
        virtual std::string getName() const override;
        virtual ~SmaccToolActionClient();
};
}