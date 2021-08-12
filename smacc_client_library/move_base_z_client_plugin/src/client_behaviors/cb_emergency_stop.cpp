
#include <move_base_z_client_plugin/client_behaviors/cb_emergency_stop.h>

namespace cl_move_base_z
{
CbEmergencyStop::CbEmergencyStop()
{
}

void CbEmergencyStop::onExit()
{
}

void CbEmergencyStop::onEntry()
{
  ROS_INFO_STREAM("[CbEmergencyStop] :");

  ClMoveBaseZ::Goal goal;

  goal.target_pose.header.stamp = ros::Time::now();

  moveBaseClient_->sendGoal(goal);
}
}  // namespace cl_move_base_z
