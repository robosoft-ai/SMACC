#pragma once

#include <smacc/client_bases/smacc_publisher_client.h>
#include <std_msgs/String.h>

namespace sm_atomic
{
namespace cl_odd_pub
{
class ClOddPub : public smacc::client_bases::SmaccPublisherClient
{
public:
    ClOddPub(std::string topicName)
        : smacc::client_bases::SmaccPublisherClient()
    {
        this->configure<std_msgs::String>(topicName);
    }
};
} // namespace cl_odd_pub
} // namespace sm_atomic