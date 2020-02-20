
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <ros_publisher_client/cl_ros_publisher.h>

namespace ros_publisher_client
{
template <typename RosMsgType>
class CbMutedBehavior : public smacc::SmaccClientBehavior
{
public:
    virtual void onEntry() override
    {
    }
    virtual void onExit() override
    {
    }
};
} // namespace ros_publisher_client
