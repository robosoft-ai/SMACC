#pragma once

#include <smacc/smacc_action_client_base.h>
#include <smacc_tool_plugin_template/ToolControlAction.h>

namespace smacc
{
class SmaccToolActionClient: public SmaccActionClientBase<smacc_tool_plugin_template::ToolControlAction>
{
    typedef SmaccActionClientBase<smacc_tool_plugin_template::ToolControlAction> Base;

    public:
        SmaccToolActionClient();
        SmaccToolActionClient(std::string action_server_namespace);
        virtual std::string getName() const override;
        virtual ~SmaccToolActionClient();
};
}