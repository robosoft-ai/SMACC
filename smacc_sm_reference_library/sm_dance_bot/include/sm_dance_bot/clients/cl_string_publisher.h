#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
class ClStringPublisher : public smacc::SmaccPublisherClient<std_msgs::String>
{
public:
    ClStringPublisher() : smacc::SmaccPublisherClient<std_msgs::String>()
    {
    }
};
} // namespace dance_bot