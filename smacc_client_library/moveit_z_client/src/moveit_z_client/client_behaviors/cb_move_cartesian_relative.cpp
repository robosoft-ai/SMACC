#include <moveit_z_client/client_behaviors/cb_move_cartesian_relative.h>

namespace sm_moveit
{
namespace cl_movegroup
{
CbMoveCartesianRelative::CbMoveCartesianRelative()
{
}

CbMoveCartesianRelative::CbMoveCartesianRelative(geometry_msgs::Vector3 offset) : offset_(offset)
{
}

void CbMoveCartesianRelative::onEntry()
{
    ClMoveGroup *movegroupClient;
    this->requiresClient(movegroupClient);
    movegroupClient->moveRelativeCartesian(offset_);
}

void CbMoveCartesianRelative::onExit()
{
}
} // namespace cl_movegroup
} // namespace sm_moveit