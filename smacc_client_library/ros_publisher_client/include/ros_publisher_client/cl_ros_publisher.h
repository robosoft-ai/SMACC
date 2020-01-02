
#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>

namespace ros_publisher_client
{

class ClRosPublisher : public smacc::client_bases::SmaccPublisherClient
{
public:
    ClRosPublisher()
    {
    }

    template <typename MessageType>
    void configure(std::string topicName)
    {
        SmaccPublisherClient::configure<MessageType>(topicName);
    }

    template <typename MessageType>
    void publish(const MessageType &msg)
    {
        SmaccPublisherClient::publish(msg);
    }
};
} // namespace ros_publisher_client