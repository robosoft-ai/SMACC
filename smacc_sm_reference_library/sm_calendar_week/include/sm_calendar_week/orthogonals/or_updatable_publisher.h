#pragma once

#include <ros_publisher_client/cl_ros_publisher.h>
#include <smacc/smacc_orthogonal.h>
#include <std_msgs/String.h>

namespace sm_calendar_week
{
using namespace ros_publisher_client;

class OrUpdatablePublisher : public smacc::Orthogonal<OrUpdatablePublisher>
{
public:
    virtual void onInitialize() override
    {
        auto ros_publisher_client = this->createClient<ClRosPublisher>();
        ros_publisher_client->initialize();
    }
};
} // namespace sm_calendar_week