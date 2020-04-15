#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveRelative : public smacc::SmaccClientBehavior
{
public:
    geometry_msgs::Transform transform_;

    CbMoveRelative();

    CbMoveRelative(geometry_msgs::Transform transform);

    virtual void onEntry() override;

    virtual void onExit() override;
};
} // namespace cl_movegroup
} // namespace sm_moveit