#pragma once

#include <smacc/client_bases/smacc_topic_publisher.h>
#include <std_msgs/String.h>

namespace sm_dance_bot
{
class StringPublisherClient : public smacc::SmaccTopicPublisherClient<std_msgs::String>
{
public:
    StringPublisherClient() : smacc::SmaccTopicPublisherClient<std_msgs::String>()
    {
    }
};
} // namespace dance_bot