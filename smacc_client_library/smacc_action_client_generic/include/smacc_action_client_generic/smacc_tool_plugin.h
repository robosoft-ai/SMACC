#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <smacc_action_client_generic/ToolControlAction.h>

namespace smacc
{
class SmaccToolActionClient : public SmaccActionClientBase<smacc_action_client_generic::ToolControlAction>
{
public:
    // for any action client you develop you need to call the ros action client type definition macro
    ACTION_DEFINITION(smacc_action_client_generic::ToolControlAction);

    SmaccToolActionClient();
    virtual std::string getName() const override;
    virtual ~SmaccToolActionClient();
};
} // namespace smacc