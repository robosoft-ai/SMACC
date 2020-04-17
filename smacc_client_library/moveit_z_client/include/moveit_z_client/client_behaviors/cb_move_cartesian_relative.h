#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveCartesianRelative : public smacc::SmaccClientBehavior
{
public:
    geometry_msgs::Vector3 offset_;

    CbMoveCartesianRelative();

    CbMoveCartesianRelative(geometry_msgs::Vector3 offset);

    virtual void onEntry() override;

    virtual void onExit() override;

    void moveRelativeCartesian(geometry_msgs::Vector3 &offset);
};
} // namespace cl_movegroup
} // namespace sm_moveit