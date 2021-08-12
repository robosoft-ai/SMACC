#include <move_base_z_client_plugin/client_behaviors/cb_safe_stop.h>

namespace cl_move_base_z
{
CbSafeStop::CbSafeStop()
{
}

void CbSafeStop::onEntry()
{
  ClMoveBaseZ::Goal goal;
  goal.target_pose.header.stamp = ros::Time::now();

  moveBaseClient_->sendGoal(goal);
}

}  // namespace cl_move_base_z
