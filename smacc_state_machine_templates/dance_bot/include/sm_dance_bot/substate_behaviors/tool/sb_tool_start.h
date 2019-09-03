#pragma once

#include <thread>
#include <smacc_interface_components/smacc_tool_plugin_template/smacc_tool_plugin.h>

class sb_tool_start: public smacc::SmaccStateBehavior
{ 
public:
    
    smacc::SmaccToolActionClient* toolActionClient_;

    virtual void onEntry() override
    {
      this->requiresComponent(toolActionClient_ , ros::NodeHandle("tool_action_server"));

      smacc::SmaccToolActionClient::Goal goal;
      goal.command = smacc::SmaccToolActionClient::Goal::CMD_START;
      toolActionClient_->sendGoal(goal);
    }

    virtual bool onExit() override
    {
      //ROS_INFO("Entering ToolSubstateBehavior");
      return true;
    }
};
