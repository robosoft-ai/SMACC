
#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace ros_publisher_client
{
class ClRosPublisher : public smacc::SmaccPublisherClient<std_msgs::String>
{
public:
    ClRosPublisher() : smacc::SmaccPublisherClient<std_msgs::String>()
    {
    }
};
} // namespace ros_publisher_client