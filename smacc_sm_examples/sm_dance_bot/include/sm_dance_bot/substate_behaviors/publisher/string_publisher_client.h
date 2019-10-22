#pragma once

#include <smacc/interface_components/smacc_topic_publisher.h>
#include <std_msgs/String.h>

namespace dance_bot
{
class StringPublisherClient : public smacc::SmaccTopicPublisherClient<std_msgs::String>
{
public:
    StringPublisherClient() : smacc::SmaccTopicPublisherClient<std_msgs::String>()
    {
    }
};
} // namespace dance_bot