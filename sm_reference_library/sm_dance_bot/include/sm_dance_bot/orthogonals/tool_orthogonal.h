#pragma once
#include <smacc/smacc_orthogonal.h>
#include <smacc_action_client_generic/smacc_tool_plugin.h>

namespace sm_dancebot
{
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
}