
#pragma once

#include <smacc/client_bases/smacc_action_client_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <planner_switcher/planner_switcher.h>

namespace sm_three_some
{
class ClClient2 : public smacc::SmaccActionClientBase<move_base_msgs::MoveBaseAction>
{
};

} // namespace sm_three_some