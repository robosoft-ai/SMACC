#pragma once
#include <geometry_msgs/Transform.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveRelative : public smacc::SmaccClientBehavior
{
public:
    geometry_msgs::Transform transform_;

    CbMoveRelative()
    {
        transform_.rotation.w = 1;
    }

    CbMoveRelative(geometry_msgs::Transform transform) : transform_(transform)
    {
    }

    virtual void onEntry() override
    {
        ROS_INFO_STREAM("[CbMoveRelative] Transform end effector pose relative: " << transform_);

        ClMoveGroup *movegroupClient;
        this->requiresClient(movegroupClient);

        movegroupClient->moveRelative(transform_);
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_movegroup
} // namespace sm_moveit