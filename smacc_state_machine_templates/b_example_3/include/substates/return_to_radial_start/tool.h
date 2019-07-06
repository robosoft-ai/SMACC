#pragma once

#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>

namespace ReturnToRadialStart
{
    struct ToolSubstate
    : SmaccState<ToolSubstate, ToolOrthogonalLine> {
  
public:

  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering ToolSubstate");
    this->requiresComponent(toolActionClient_, ros::NodeHandle("tool_action_server"));

    smacc::SmaccToolActionClient::Goal goal;
    goal.command = smacc::SmaccToolActionClient::Goal::CMD_STOP;
    toolActionClient_->sendGoal(goal);
  }

  smacc::SmaccToolActionClient* toolActionClient_;
};
}