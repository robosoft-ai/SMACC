
#pragma once

#include <smacc/interface_components/smacc_action_client_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <smacc_planner_switcher/planner_switcher.h>

namespace hello_world_example
{
class Client2 : public smacc::SmaccActionClientBase < Client2, move_base_msgs::MoveBaseAction>
{
};

} // namespace hello_world_example