#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveAbsolute : public smacc::SmaccClientBehavior
{

private:
  ClMoveGroup *movegroupClient_;

public:
  geometry_msgs::PoseStamped targetPose;
  CbMoveAbsolute();
  CbMoveAbsolute(geometry_msgs::PoseStamped target_pose);
  virtual void onEntry() override;
  virtual void onExit() override;

private:
  bool moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                          moveit::planning_interface::PlanningSceneInterface &planningSceneInterface,
                          geometry_msgs::PoseStamped &targetObjectPose);
};
} // namespace cl_movegroup
} // namespace sm_moveit
