
#pragma once
#include <smacc/smacc_client_behavior.h>
#include <ros_publisher_client/cl_ros_publisher.h>

namespace ros_publisher_client
{
class CbDefaultPublishLoop : public smacc::SmaccClientBehavior,
                             public smacc::ISmaccUpdatable
{
public:
    void onEntry()
    {
    }

    virtual void update()
    {
    }
};
} // namespace ros_publisher_client