#pragma once

#include <thread>

class SbToolStop: public smacc::SmaccSubStateBehavior
{ 
public:
    
    smacc::SmaccToolActionClient* toolActionClient_;

    virtual void onEntry() override
    {
      this->requiresComponent(toolActionClient_ , ros::NodeHandle("tool_action_server"));

      smacc::SmaccToolActionClient::Goal goal;
      goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
      toolActionClient_->sendGoal(goal);
    }

    virtual bool onExit() override
    {
      //ROS_INFO("Entering ToolSubstateBehavior");
      return true;
    }
};
