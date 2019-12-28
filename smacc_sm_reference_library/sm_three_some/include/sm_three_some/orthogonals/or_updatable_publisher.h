#pragma once

#include <ros_publisher_client/cl_ros_publisher.h>
#include <smacc/smacc_orthogonal.h>

namespace sm_three_some
{
using namespace ros_publisher_client;

class OrUpdatablePublisher : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto ros_publisher_client = this->createClient<OrUpdatablePublisher, ClRosPublisher>();
        ros_publisher_client->initialize();
    }
};
} // namespace sm_three_some