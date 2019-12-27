#pragma once

#include <smacc/smacc.h>
#include <smacc_action_client_generic/smacc_tool_plugin.h>

namespace sm_dance_bot
{
namespace tool_client
{
class CbToolStop : public smacc::SmaccClientBehavior
{
public:
  smacc::SmaccToolActionClient *toolActionClient_;

  virtual void onEntry() override
  {
    this->requiresClient(toolActionClient_);

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

  virtual void onExit() override
  {
    //ROS_INFO("Entering ToolClientBehavior");
  }
};
} // namespace tool_client
} // namespace sm_dance_bot
