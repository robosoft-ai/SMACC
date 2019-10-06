#pragma once

#include <smacc_action_client_generic/smacc_tool_plugin.h>

class ToolStop: public smacc::SmaccSubStateBehavior
{ 
public:
    
    smacc::SmaccToolActionClient* toolActionClient_;

    virtual void onEntry() override
    {
      this->requiresClient(toolActionClient_ );

      smacc::SmaccToolActionClient::Goal goal;
      goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
      toolActionClient_->sendGoal(goal);
    }

    virtual void onExit() override
    {
      //ROS_INFO("Entering ToolSubstateBehavior");
    }
};
