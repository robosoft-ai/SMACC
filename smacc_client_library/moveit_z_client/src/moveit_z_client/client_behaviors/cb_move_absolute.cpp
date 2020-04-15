#include <moveit_z_client/client_behaviors/cb_move_absolute.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveAbsolute::CbMoveAbsolute()
{
}

CbMoveAbsolute::CbMoveAbsolute(geometry_msgs::PoseStamped target_pose)
    : targetPose(target_pose)
{
}

void CbMoveAbsolute::onEntry()
{
    ros::WallDuration(4).sleep();

    ClMoveGroup *movegroupClient;
    this->requiresClient(movegroupClient);
    movegroupClient->moveToAbsolutePose(targetPose);

    ros::WallDuration(4).sleep();
}

void CbMoveAbsolute::onExit()
{
}
} // namespace cl_movegroup
} // namespace sm_moveit
