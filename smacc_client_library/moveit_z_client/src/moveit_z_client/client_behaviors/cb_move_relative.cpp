#include <moveit_z_client/client_behaviors/cb_move_relative.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveRelative::CbMoveRelative()
{
    transform_.rotation.w = 1;
}

CbMoveRelative::CbMoveRelative(geometry_msgs::Transform transform)
{
}

void CbMoveRelative::onEntry()
{
    ROS_INFO_STREAM("[CbMoveRelative] Transform end effector pose relative: " << transform_);

    ClMoveGroup *movegroupClient;
    this->requiresClient(movegroupClient);

    movegroupClient->moveRelative(transform_);
}

void CbMoveRelative::onExit()
{
}
} // namespace cl_movegroup
} // namespace sm_moveit