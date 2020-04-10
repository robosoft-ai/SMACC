#pragma once

#include <smacc/smacc.h>
#include <sm_moveit/clients/movegroup_client/cl_movegroup.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveAbsolute : public smacc::SmaccClientBehavior
{

public:
  geometry_msgs::PoseStamped targetPose;

  CbMoveAbsolute()
  {
  }

  CbMoveAbsolute(geometry_msgs::PoseStamped target_pose)
      : targetPose(target_pose)
  {
  }

  virtual void onEntry() override
  {
    ros::WallDuration(4).sleep();

    ClMoveGroup *movegroupClient;
    this->requiresClient(movegroupClient);
    movegroupClient->moveToAbsolutePose(targetPose);

    ros::WallDuration(4).sleep();
  }

  virtual void onExit() override
  {
  }
};
} // namespace cl_movegroup
} // namespace sm_moveit
