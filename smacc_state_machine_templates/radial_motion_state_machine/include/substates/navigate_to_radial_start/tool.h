#pragma once

#include <radial_motion.h>
#include <thread>

namespace NavigateToRadialStart 
{
using namespace smacc;


class ToolBehavior: public smacc::SmaccStateBehavior
{ 
    public:
    
    smacc::SmaccToolActionClient* toolActionClient_;

    virtual void onEntry() override
    {
      ROS_INFO("Entering ToolSubstateBehavior");
      
      ROS_INFO("Entering ToolSubstate");
      this->requiresComponent(toolActionClient_ , ros::NodeHandle("tool_action_server"));

      smacc::SmaccToolActionClient::Goal goal;
      goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
      toolActionClient_->sendGoal(goal);
    }

    virtual bool onExit() override
    {
      ROS_INFO("Entering ToolSubstateBehavior");
      return true;
    }
};

struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:
    using SmaccState::SmaccState;
  
    SMACC_STATE_BEHAVIOR(ToolBehavior);
  
  /*
  void onEntry()
  {
    ROS_INFO("Entering ToolSubstate");
    this->requiresComponent(toolActionClient_ , ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }
  
  smacc::SmaccToolActionClient* toolActionClient_;
  */
};

}