#pragma once

#include <smacc/orthogonal.h>
#include <smacc_action_client_generic/smacc_tool_plugin.h>

class ToolOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *actionclient = this->createClient<smacc::SmaccToolActionClient>();
        actionclient->name_ = "tool_action_server";
        actionclient->initialize();
    }
};