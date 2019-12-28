
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <ros_publisher_client/cl_ros_publisher.h>

namespace ros_publisher_client
{
template <typename RosMsgType>
class CbMutedBehavior : public smacc::SmaccClientBehavior
{
public:
    void onEntry()
    {
    }
};
} // namespace ros_publisher_client
