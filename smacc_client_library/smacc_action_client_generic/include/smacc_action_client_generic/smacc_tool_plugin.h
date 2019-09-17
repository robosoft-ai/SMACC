#pragma once

#include <smacc/interface_components/smacc_action_client_base.h>
#include <smacc_action_client_generic/ToolControlAction.h>

namespace smacc
{
class SmaccToolActionClient: public SmaccActionClientBase<smacc_action_client_generic::ToolControlAction>
{
    typedef SmaccActionClientBase<smacc_action_client_generic::ToolControlAction> Base;

    public:
        SmaccToolActionClient();
        virtual std::string getName() const override;
        virtual ~SmaccToolActionClient();
};
}