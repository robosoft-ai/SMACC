
#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>

namespace ros_publisher_client
{

class ClRosPublisher : public smacc::SmaccPublisherClient
{
public:
    ClRosPublisher()
    {
    }
};
} // namespace ros_publisher_client