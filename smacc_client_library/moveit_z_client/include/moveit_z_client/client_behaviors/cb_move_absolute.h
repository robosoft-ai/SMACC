#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveAbsolute : public smacc::SmaccClientBehavior
{

public:
  geometry_msgs::PoseStamped targetPose;
  CbMoveAbsolute();
  CbMoveAbsolute(geometry_msgs::PoseStamped target_pose);
  virtual void onEntry() override;
  virtual void onExit() override;
};
} // namespace cl_movegroup
} // namespace sm_moveit
