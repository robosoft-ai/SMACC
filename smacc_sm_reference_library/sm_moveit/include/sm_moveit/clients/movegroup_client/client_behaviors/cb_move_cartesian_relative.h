#pragma once

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveCartesianRelative : public smacc::SmaccClientBehavior
{
public:
    geometry_msgs::Vector3 offset_;
    
    CbMoveCartesianRelative(geometry_msgs::Vector3 offset) : offset_(offset)
    {
    }

    virtual void onEntry() override
    {
        ClMoveGroup *movegroupClient;
        this->requiresClient(movegroupClient);
        movegroupClient->moveRelativeCartesian(offset_);
    }

    virtual void onExit() override
    {
    }
};
} // namespace cl_movegroup
} // namespace sm_moveit