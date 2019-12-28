#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
namespace string_publisher_client
{
class ClStringPublisher : public smacc::SmaccPublisherClient
{
public:
    ClStringPublisher() : smacc::SmaccPublisherClient()
    {
        this->configureMessageType<std_msgs::String>();
    }
};
} // namespace string_publisher_client
} // namespace sm_dance_bot